import math
from equipo_jar.mapa import is_free, line_is_clear
from equipo_jar.confirmacion import near_any

def viewpoint_for(grid, cx, cy, robot_x, robot_y, distance, clearance, n_angles, min_shift, box_margin):
    best = None
    best_dist = None
    for k in range(n_angles):
        angle = 2 * math.pi * k / n_angles
        c = math.cos(angle)
        s = math.sin(angle)
        vx = cx + distance * c  
        vy = cy + distance * s
        ex = cx + box_margin * c
        ey = cy + box_margin * s
        if not is_free(grid, vx, vy, clearance):
            continue
        if not line_is_clear(grid, vx, vy, ex, ey):
            continue
        d = math.hypot(vx - robot_x, vy - robot_y)
        if d < min_shift:
            continue
        if best is None or d < best_dist:
            yaw = math.atan2(cy - vy, cx - vx)
            best = (vx, vy, yaw)
            best_dist = d
    return best

def pick_doubtful(cands, reported_ids, reported_points, inspected_points, min_obs, dist_min, robot_x, robot_y):
    best = None
    best_dist = None
    remaining = []
    for c in cands:
        if c.id in reported_ids or near_any(c.centro.x, c.centro.y, reported_points, dist_min):
            continue
        if near_any(c.centro.x, c.centro.y, inspected_points, dist_min):
            continue
        if not math.isfinite(c.centro.x) or not math.isfinite(c.centro.y):
            continue
        if c.observaciones < min_obs:
            continue
        remaining.append(c)
    for c in remaining:
        dist = math.hypot(c.centro.x - robot_x, c.centro.y - robot_y)
        if best is None or dist < best_dist:
            best = c
            best_dist = dist
    return best
