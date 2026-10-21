import math
from types import SimpleNamespace

import pytest

from equipo_jar.exploracion import (
    diferencia_angulo, primer_pendiente, ya_visitado, yaw_de_cuaternion)

DIST = 0.30
ANG = 0.5


def cuaternion(yaw):
    """Cuaternión de un giro `yaw` sobre el eje vertical."""
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def pose(x, y, yaw):
    """PoseStamped falso: mismos campos que usa primer_pendiente."""
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y), orientation=cuaternion(yaw)))


# --- yaw_de_cuaternion ---

def test_yaw_sin_giro():
    assert yaw_de_cuaternion(cuaternion(0.0)) == pytest.approx(0.0)


def test_yaw_cuarto_de_vuelta():
    assert yaw_de_cuaternion(cuaternion(math.pi / 2)) == pytest.approx(math.pi / 2)


def test_yaw_negativo():
    assert yaw_de_cuaternion(cuaternion(-1.0)) == pytest.approx(-1.0)


# --- diferencia_angulo ---

def test_diferencia_simple():
    assert diferencia_angulo(0.1, 0.3) == pytest.approx(0.2)


def test_diferencia_no_depende_del_orden():
    assert diferencia_angulo(0.3, 0.1) == pytest.approx(0.2)


def test_diferencia_da_la_vuelta():
    # 179° y -179° están a 2°, no a 358°.
    assert diferencia_angulo(math.radians(179), math.radians(-179)) == pytest.approx(math.radians(2))


def test_diferencia_vuelta_completa_es_cero():
    assert diferencia_angulo(0.0, 2 * math.pi) == pytest.approx(0.0, abs=1e-9)


# --- ya_visitado ---

def test_sin_visitados():
    assert not ya_visitado(1.0, 1.0, 0.0, [], DIST, ANG)


def test_mismo_punto_lejos_del_origen():
    assert ya_visitado(3.0, 3.0, 0.0, [(3.0, 3.0, 0.0)], DIST, ANG)


def test_cerca_del_origen_sin_visitar():
    assert not ya_visitado(0.1, 0.1, 0.0, [(5.0, 5.0, 0.0)], DIST, ANG)


def test_casi_igual_cuenta_como_visitado():
    assert ya_visitado(1.1, 1.0, 0.1, [(1.0, 1.0, 0.0)], DIST, ANG)


def test_mismo_lugar_mirando_a_otro_lado_no_es_visitado():
    assert not ya_visitado(1.0, 1.0, math.pi / 2, [(1.0, 1.0, 0.0)], DIST, ANG)


def test_misma_orientacion_en_otro_lugar_no_es_visitado():
    assert not ya_visitado(2.0, 1.0, 0.0, [(1.0, 1.0, 0.0)], DIST, ANG)


# --- primer_pendiente ---

def test_plan_vacio():
    assert primer_pendiente([], [], DIST, ANG) is None


def test_sin_visitados_devuelve_el_primero():
    poses = [pose(1.0, 0.5, 0.0), pose(2.0, 0.5, 0.0)]
    assert primer_pendiente(poses, [], DIST, ANG) is poses[0]


def test_saltea_los_visitados_y_respeta_el_orden():
    poses = [pose(1.0, 0.5, 0.0), pose(2.0, 0.5, 0.0), pose(3.0, 0.5, 0.0)]
    assert primer_pendiente(poses, [(1.0, 0.5, 0.0)], DIST, ANG) is poses[1]


def test_todos_visitados():
    poses = [pose(1.0, 0.5, 0.0), pose(2.0, 0.5, 0.0)]
    visitados = [(1.0, 0.5, 0.0), (2.0, 0.5, 0.0)]
    assert primer_pendiente(poses, visitados, DIST, ANG) is None


def test_mismo_lugar_otra_orientacion_sigue_pendiente():
    # El plan de la prueba del paso 8: los dos últimos puntos están en el mismo lugar.
    poses = [pose(1.0, 0.5, 0.0), pose(2.0, 0.5, 0.0), pose(2.0, 0.5, math.pi / 2)]
    visitados = [(1.0, 0.5, 0.0), (2.0, 0.5, 0.0)]
    assert primer_pendiente(poses, visitados, DIST, ANG) is poses[2]
