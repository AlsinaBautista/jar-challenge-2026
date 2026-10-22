import math
from types import SimpleNamespace

import pytest

from equipo_jar.inspeccion import pick_doubtful, viewpoint_for

RES = 0.05

# Valores de params.yaml (visit_distance_m, visit_clearance_m, visit_angles,
# visit_min_shift_m, visit_box_margin_m).
VIEW = dict(distance=0.8, clearance=0.25, n_angles=12, min_shift=0.4, box_margin=0.25)


def make_grid(walls=()):
    """OccupancyGrid falso de 4 x 4 m, origen (0, 0). `walls`: celdas (i, j) ocupadas."""
    width = height = 80
    data = [0] * (width * height)
    for i, j in walls:
        data[j * width + i] = 100
    info = SimpleNamespace(
        resolution=RES, width=width, height=height,
        origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)))
    return SimpleNamespace(info=info, data=data)


def faces(view, cx, cy):
    """¿El yaw del lugar elegido mira hacia la caja?"""
    x, y, yaw = view
    return yaw == pytest.approx(math.atan2(cy - y, cx - x))


# Pared vertical en x de 2.00 a 2.05 (columna 40), de punta a punta.
WALL = [(40, j) for j in range(80)]


# --- viewpoint_for ---

def test_free_map_picks_the_spot_on_the_robot_side():
    # Caja en (2, 2), robot a la izquierda: el punto del círculo más cercano es (1.2, 2).
    x, y, yaw = viewpoint_for(make_grid(), 2.0, 2.0, 0.5, 2.0, **VIEW)
    assert (x, y) == (pytest.approx(1.2), pytest.approx(2.0))
    assert yaw == pytest.approx(0.0)


def test_skips_the_spot_where_the_robot_already_is():
    # Mirarla desde donde ya está no le da una vista nueva a fusión.
    view = viewpoint_for(make_grid(), 2.0, 2.0, 1.2, 2.0, **VIEW)
    assert math.hypot(view[0] - 1.2, view[1] - 2.0) >= VIEW['min_shift']
    assert faces(view, 2.0, 2.0)


def test_does_not_look_through_a_wall():
    # Caja a la derecha de la pared, robot a la izquierda: el lugar tiene que quedar a la derecha.
    view = viewpoint_for(make_grid(WALL), 2.6, 2.0, 0.5, 2.0, **VIEW)
    assert view[0] > 2.05 + VIEW['clearance']
    assert faces(view, 2.6, 2.0)


def test_box_near_the_map_edge_gets_a_spot_inside():
    view = viewpoint_for(make_grid(), 2.0, 0.3, 2.0, 3.0, **VIEW)
    assert view[1] > 0.3
    assert faces(view, 2.0, 0.3)


def test_no_spot_returns_none():
    # Tan cerca de la pared que el robot no cabe en ningún punto del círculo.
    assert viewpoint_for(make_grid(WALL), 2.1, 2.0, 0.5, 2.0, **{**VIEW, 'distance': 0.1}) is None


def test_box_drawn_in_the_map_is_not_taken_as_a_wall():
    # Si el /map dibuja la caja (0.25 x 0.25 m alrededor de (2, 2)), box_margin hace que
    # la línea termine antes de la caja. Sin ese margen, la caja tapa todas las líneas.
    box = [(i, j) for i in range(37, 43) for j in range(37, 43)]
    grid = make_grid(box)
    assert viewpoint_for(grid, 2.0, 2.0, 0.5, 2.0, **VIEW) is not None
    assert viewpoint_for(grid, 2.0, 2.0, 0.5, 2.0, **{**VIEW, 'box_margin': 0.0}) is None


# --- pick_doubtful ---

MIN_OBS = 2
DIST_MIN = 0.4


def cand(id_, x, y, observaciones=3):
    """Candidata falsa con los mismos campos que equipo_jar_msgs/Candidata."""
    return SimpleNamespace(id=id_, centro=SimpleNamespace(x=x, y=y), observaciones=observaciones)


def pick(cands, reported_ids=None, reported_points=None, inspected_points=None):
    # El robot está en (0, 0).
    return pick_doubtful(cands, reported_ids or set(), reported_points or [],
                         inspected_points or [], MIN_OBS, DIST_MIN, 0.0, 0.0)


TWO = [cand(1, 1.0, 0.0), cand(2, 3.0, 0.0)]


def test_picks_the_nearest():
    assert pick(TWO).id == 1


def test_skips_a_reported_id():
    assert pick(TWO, reported_ids={1}).id == 2


def test_skips_the_same_box_reported_with_another_id():
    assert pick(TWO, reported_points=[(1.1, 0.1)]).id == 2


def test_skips_one_already_visited():
    assert pick(TWO, inspected_points=[(0.9, 0.0)]).id == 2


def test_skips_one_with_few_observations():
    assert pick([cand(1, 1.0, 0.0, observaciones=1), cand(2, 3.0, 0.0)]).id == 2


def test_skips_a_non_finite_center():
    assert pick([cand(1, float('nan'), 0.0), cand(2, 3.0, 0.0)]).id == 2


def test_nothing_left_returns_none():
    assert pick(TWO, reported_ids={1, 2}) is None
    assert pick([]) is None


def test_does_not_modify_the_lists_it_gets():
    reported_ids, reported_points, inspected_points = {9}, [(5.0, 5.0)], [(6.0, 6.0)]
    pick(TWO, reported_ids, reported_points, inspected_points)
    assert reported_ids == {9}
    assert reported_points == [(5.0, 5.0)]
    assert inspected_points == [(6.0, 6.0)]
