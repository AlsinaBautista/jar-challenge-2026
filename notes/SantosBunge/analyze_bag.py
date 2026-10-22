"""Offline evaluation of detection and fusion from a session recording (MCAP).

Usage (inside the dev container, with install/setup.bash sourced):
    python3 analyze_bag.py <bag.mcap> <out_dir>

Reads /jar/observaciones, /jar/candidatas, /odom, /ground_truth/odom and /map.
pruebas.launch.py fixes map = odom, so every observation carries the odometry drift.
Ground truth lets us remove it: an observation is moved to base_footprint with the odom
pose at its time, then back to map with the TRUE pose at that time. The remaining error is
detection's own (plus the LiDAR/camera noise). Ground truth is used here ONLY for offline
evaluation; it never enters src/.
"""
import json
import math
import sys
from bisect import bisect_left

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RED = {'rojo_1': (-0.1698, -2.0228), 'rojo_2': (2.0734, -2.0864)}
BLUE = {'azul_1': (2.0594, -1.0174), 'azul_2': (3.9811, 0.9197)}
TOPICS = ['/jar/observaciones', '/jar/candidatas', '/odom', '/ground_truth/odom', '/map']


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    out = {t: [] for t in TOPICS}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        out[topic].append((t_ns * 1e-9, deserialize_message(data, types[topic])))
    return out


class PoseTrack:
    """Nearest-time lookup of (x, y, yaw) from an Odometry stream."""

    def __init__(self, msgs):
        self.t = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in msgs]
        self.p = [(m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
                  for _, m in msgs]

    def at(self, t):
        i = bisect_left(self.t, t)
        if i <= 0:
            return self.p[0], abs(self.t[0] - t)
        if i >= len(self.t):
            return self.p[-1], abs(self.t[-1] - t)
        a, b = i - 1, i
        j = a if abs(self.t[a] - t) <= abs(self.t[b] - t) else b
        return self.p[j], abs(self.t[j] - t)


def to_base(p, pose):
    x, y, yaw = pose
    dx, dy = p[0] - x, p[1] - y
    c, s = math.cos(-yaw), math.sin(-yaw)
    return (c * dx - s * dy, s * dx + c * dy)


def to_map(pb, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * pb[0] - s * pb[1], y + s * pb[0] + c * pb[1])


def nearest(p, boxes):
    name, d = min(((n, math.hypot(p[0] - b[0], p[1] - b[1])) for n, b in boxes.items()), key=lambda x: x[1])
    return name, d


def main():
    bag, out_dir = sys.argv[1], sys.argv[2]
    data = read(bag)
    print(f"messages: " + ", ".join(f"{t} {len(v)}" for t, v in data.items()))
    odom = PoseTrack(data['/odom'])
    truth = PoseTrack(data['/ground_truth/odom'])

    rows = []
    for t_rx, m in data['/jar/observaciones']:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        z = (m.pose.pose.position.x, m.pose.pose.position.y)
        sigma = math.sqrt(m.pose.covariance[0])
        po, dto = odom.at(t)
        pt, dtt = truth.at(t)
        pb = to_base(z, po)                     # observation relative to the robot
        zc = to_map(pb, pt)                     # same, placed with the true pose
        dist = math.hypot(*pb)
        n_raw, e_raw = nearest(z, RED)
        n_cor, e_cor = nearest(zc, RED)
        box = RED[n_cor]
        # Decompose the corrected error along the line of sight (radial) and across it.
        rob = (pt[0], pt[1])
        los = np.array([box[0] - rob[0], box[1] - rob[1]])
        los /= max(np.linalg.norm(los), 1e-9)
        err = np.array([zc[0] - box[0], zc[1] - box[1]])
        radial = float(err @ los)               # + = estimate beyond the box center
        lateral = float(err @ np.array([-los[1], los[0]]))
        drift = math.hypot(pt[0] - po[0], pt[1] - po[1])
        _, d_blue = nearest(zc, BLUE)
        rows.append(dict(t=t, x=z[0], y=z[1], xc=zc[0], yc=zc[1], sigma=sigma, dist=dist,
                         box=n_cor, e_raw=e_raw, e_cor=e_cor, radial=radial, lateral=lateral,
                         drift=drift, d_blue=d_blue, dt_odom=dto, dt_truth=dtt))

    with open(f'{out_dir}/observations.json', 'w') as f:
        json.dump(rows, f)

    print("\n=== OBSERVATIONS ===")
    print(f"total {len(rows)}; time span {rows[0]['t']:.1f} .. {rows[-1]['t']:.1f} s" if rows else "none")
    for name in RED:
        r = [x for x in rows if x['box'] == name]
        if not r:
            print(f"{name}: no observations")
            continue
        e_cor = np.array([x['e_cor'] for x in r]); e_raw = np.array([x['e_raw'] for x in r])
        rad = np.array([x['radial'] for x in r]); lat = np.array([x['lateral'] for x in r])
        dist = np.array([x['dist'] for x in r]); sig = np.array([x['sigma'] for x in r])
        drift = np.array([x['drift'] for x in r])
        print(f"\n{name}: {len(r)} obs, distance {dist.min():.2f}..{dist.max():.2f} m, odom drift {drift.mean():.2f} m (mean)")
        print(f"  error vs true center, drift removed: median {np.median(e_cor):.3f}  p90 {np.percentile(e_cor, 90):.3f}  max {e_cor.max():.3f} m")
        print(f"  error as published (with drift):    median {np.median(e_raw):.3f}  p90 {np.percentile(e_raw, 90):.3f} m")
        print(f"  radial bias (+ = beyond center) {rad.mean():+.3f} m (sd {rad.std():.3f}); lateral bias {lat.mean():+.3f} m (sd {lat.std():.3f})")
        print(f"  sigma reported: mean {sig.mean():.3f}; |error|/sigma median {np.median(e_cor / sig):.2f} (≈1 means calibrated)")
        print(f"  within 0.25 m: {(e_cor <= 0.25).mean() * 100:.0f} %   within 0.50 m: {(e_cor <= 0.50).mean() * 100:.0f} %")
        for lo, hi in [(0, 1), (1, 2), (2, 3), (3, 9)]:
            sel = (dist >= lo) & (dist < hi)
            if sel.any():
                print(f"    {lo}-{hi} m: n={sel.sum():3d}  median err {np.median(e_cor[sel]):.3f}  radial {rad[sel].mean():+.3f}")
    far = [x for x in rows if x['e_cor'] > 0.5]
    print(f"\nobservations farther than 0.5 m from any red box (drift removed): {len(far)}")
    near_blue = [x for x in rows if x['d_blue'] < 0.5]
    print(f"observations within 0.5 m of a BLUE box: {len(near_blue)}")

    print("\n=== CANDIDATES (last message) ===")
    if data['/jar/candidatas']:
        _, last = data['/jar/candidatas'][-1]
        for c in last.candidatas:
            cov = np.array(c.covarianza_xy).reshape(2, 2)
            std = math.sqrt(max(np.linalg.eigvalsh(cov)))
            name, e = nearest((c.centro.x, c.centro.y), RED)
            print(f"  id {c.id}: ({c.centro.x:.3f}, {c.centro.y:.3f}) n={c.observaciones} max_std={std:.3f} "
                  f"nearest {name} at {e:.3f} m (as published, with drift)")
        # when did each id first become reportable by mission's rule (n>=5, std<=0.10)?
        first_ok = {}
        for t_rx, msg in data['/jar/candidatas']:
            for c in msg.candidatas:
                cov = np.array(c.covarianza_xy).reshape(2, 2)
                std = math.sqrt(max(np.linalg.eigvalsh(cov)))
                if c.observaciones >= 5 and std <= 0.10 and c.id not in first_ok:
                    first_ok[c.id] = (t_rx, c.observaciones)
        ids_seen = sorted({c.id for _, m in data['/jar/candidatas'] for c in m.candidatas})
        print(f"  ids ever seen: {ids_seen}")
        for i, (t, n) in sorted(first_ok.items()):
            print(f"  id {i} reportable (n>=5, std<=0.10) at t={t:.1f} s with n={n}")

    # ---- picture: map, boxes, observations, candidates, true path
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 7))
        if data['/map']:
            _, mp = data['/map'][-1]
            g = np.array(mp.data, dtype=np.int16).reshape(mp.info.height, mp.info.width)
            ox, oy, res = mp.info.origin.position.x, mp.info.origin.position.y, mp.info.resolution
            ax.imshow(g >= 65, cmap='Greys', origin='lower',
                      extent=[ox, ox + g.shape[1] * res, oy, oy + g.shape[0] * res], alpha=0.8)
        tp = np.array(truth.p)
        ax.plot(tp[:, 0], tp[:, 1], color='tab:green', lw=1, label='true robot path')
        if rows:
            ax.scatter([r['x'] for r in rows], [r['y'] for r in rows], s=6, color='grey', alpha=0.5, label='observations as published')
            ax.scatter([r['xc'] for r in rows], [r['yc'] for r in rows], s=6, color='tab:red', alpha=0.6, label='observations, drift removed')
        for n, (x, y) in RED.items():
            ax.add_patch(plt.Circle((x, y), 0.25, fill=False, color='red', lw=1.5))
            ax.plot(x, y, 'r*', ms=12)
        for n, (x, y) in BLUE.items():
            ax.plot(x, y, 'b*', ms=12)
        if data['/jar/candidatas']:
            for c in last.candidatas:
                ax.plot(c.centro.x, c.centro.y, 'kx', ms=10, mew=2)
                ax.annotate(f'id {c.id} n={c.observaciones}', (c.centro.x, c.centro.y), xytext=(5, 5), textcoords='offset points', fontsize=8)
        ax.set_aspect('equal'); ax.legend(loc='upper right', fontsize=8); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.set_title('Session 20261022-140046: detection vs true box centers (0.25 m circles)')
        fig.savefig(f'{out_dir}/analysis.png', dpi=130, bbox_inches='tight')
        print(f"\nfigure: {out_dir}/analysis.png")
    except Exception as e:  # noqa: BLE001
        print(f"no figure: {e}")


if __name__ == '__main__':
    main()
