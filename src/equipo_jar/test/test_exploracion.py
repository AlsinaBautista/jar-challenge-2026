import math
from types import SimpleNamespace

import pytest

from equipo_jar.exploracion import (
    angle_difference, first_pending, already_visited, yaw_from_quaternion)

DIST = 0.30
ANG = 0.5


def quaternion(yaw):
    """Cuaternión de un giro `yaw` sobre el eje vertical."""
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def pose(x, y, yaw):
    """PoseStamped falso: mismos campos que usa first_pending."""
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y), orientation=quaternion(yaw)))


# --- yaw_from_quaternion ---

def test_yaw_no_rotation():
    assert yaw_from_quaternion(quaternion(0.0)) == pytest.approx(0.0)


def test_yaw_quarter_turn():
    assert yaw_from_quaternion(quaternion(math.pi / 2)) == pytest.approx(math.pi / 2)


def test_yaw_negative():
    assert yaw_from_quaternion(quaternion(-1.0)) == pytest.approx(-1.0)


# --- angle_difference ---

def test_difference_simple():
    assert angle_difference(0.1, 0.3) == pytest.approx(0.2)


def test_difference_does_not_depend_on_order():
    assert angle_difference(0.3, 0.1) == pytest.approx(0.2)


def test_difference_wraps_around():
    # 179° y -179° están a 2°, no a 358°.
    assert angle_difference(math.radians(179), math.radians(-179)) == pytest.approx(math.radians(2))


def test_difference_full_turn_is_zero():
    assert angle_difference(0.0, 2 * math.pi) == pytest.approx(0.0, abs=1e-9)


# --- already_visited ---

def test_no_visited():
    assert not already_visited(1.0, 1.0, 0.0, [], DIST, ANG)


def test_same_point_far_from_origin():
    assert already_visited(3.0, 3.0, 0.0, [(3.0, 3.0, 0.0)], DIST, ANG)


def test_near_origin_not_visited():
    assert not already_visited(0.1, 0.1, 0.0, [(5.0, 5.0, 0.0)], DIST, ANG)


def test_almost_equal_counts_as_visited():
    assert already_visited(1.1, 1.0, 0.1, [(1.0, 1.0, 0.0)], DIST, ANG)


def test_same_place_facing_other_way_is_not_visited():
    assert not already_visited(1.0, 1.0, math.pi / 2, [(1.0, 1.0, 0.0)], DIST, ANG)


def test_same_heading_elsewhere_is_not_visited():
    assert not already_visited(2.0, 1.0, 0.0, [(1.0, 1.0, 0.0)], DIST, ANG)


# --- first_pending ---

def test_empty_plan():
    assert first_pending([], [], DIST, ANG) is None


def test_no_visited_returns_first():
    poses = [pose(1.0, 0.5, 0.0), pose(2.0, 0.5, 0.0)]
    assert first_pending(poses, [], DIST, ANG) is poses[0]


def test_skips_visited_and_keeps_order():
    poses = [pose(1.0, 0.5, 0.0), pose(2.0, 0.5, 0.0), pose(3.0, 0.5, 0.0)]
    assert first_pending(poses, [(1.0, 0.5, 0.0)], DIST, ANG) is poses[1]


def test_all_visited():
    poses = [pose(1.0, 0.5, 0.0), pose(2.0, 0.5, 0.0)]
    visited = [(1.0, 0.5, 0.0), (2.0, 0.5, 0.0)]
    assert first_pending(poses, visited, DIST, ANG) is None


def test_same_place_other_heading_is_still_pending():
    # El plan de la prueba del paso 8: los dos últimos puntos están en el mismo lugar.
    poses = [pose(1.0, 0.5, 0.0), pose(2.0, 0.5, 0.0), pose(2.0, 0.5, math.pi / 2)]
    visited = [(1.0, 0.5, 0.0), (2.0, 0.5, 0.0)]
    assert first_pending(poses, visited, DIST, ANG) is poses[2]
