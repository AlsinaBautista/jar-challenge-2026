import math
from types import SimpleNamespace

import pytest

from equipo_jar.confirmacion import (
    cerca_de_alguno, desvio_maximo, elegir_para_reportar, esta_confirmada)


def test_desvio_elipse_redonda():
    assert desvio_maximo([0.01, 0.0, 0.0, 0.01]) == pytest.approx(0.1)


def test_desvio_elipse_alargada_en_x():
    assert desvio_maximo([0.04, 0.0, 0.0, 0.0025]) == pytest.approx(0.2)


def test_desvio_elipse_inclinada_es_mayor_que_en_x_o_y():
    # Mirando solo a y d daría 0.1; en la diagonal (45°) el desvío es mayor.
    assert desvio_maximo([0.01, 0.009, 0.009, 0.01]) == pytest.approx(math.sqrt(0.019))


def test_desvio_con_varianzas_negativas_no_se_cae():
    # Un bug en fusión no tiene que tirar abajo el nodo de misión.
    assert desvio_maximo([-0.01, 0.0, 0.0, -0.01]) == 0.0


def test_no_confirma_con_pocas_observaciones():
    assert not esta_confirmada(2, [0.0001, 0.0, 0.0, 0.0001], 5, 0.1)


def test_no_confirma_si_es_muy_incierta():
    assert not esta_confirmada(20, [0.04, 0.0, 0.0, 0.04], 5, 0.1)


def test_confirma_con_observaciones_y_precision_suficientes():
    assert esta_confirmada(20, [0.0025, 0.0, 0.0, 0.0025], 5, 0.1)


def test_confirma_justo_con_el_minimo_de_observaciones():
    assert esta_confirmada(5, [0.0025, 0.0, 0.0, 0.0025], 5, 0.1)


# --- Paso 4: cerca_de_alguno y elegir_para_reportar ---

# Umbrales fijos para los tests de elegir_para_reportar.
MIN_OBS = 5
MAX_DESVIO = 0.1
DIST_MIN = 0.4
PRECISA = [0.0025, 0.0, 0.0, 0.0025]   # desvío 0.05 m: pasa MAX_DESVIO


def cand(id_, x, y, observaciones=20, cov=PRECISA):
    """Candidata falsa con los mismos campos que equipo_jar_msgs/Candidata."""
    return SimpleNamespace(
        id=id_, centro=SimpleNamespace(x=x, y=y),
        observaciones=observaciones, covarianza_xy=cov)


def elegir(cands, ids_reportados=None, puntos_reportados=None, n=5):
    return elegir_para_reportar(
        cands, ids_reportados or set(), puntos_reportados or [], n,
        MIN_OBS, MAX_DESVIO, DIST_MIN)


def ids(candidatas):
    return [c.id for c in candidatas]


def test_cerca_sin_puntos_es_falso():
    assert not cerca_de_alguno(1.0, 1.0, [], 0.4)


def test_cerca_dentro_de_la_distancia():
    assert cerca_de_alguno(1.0, 1.0, [(5.0, 5.0), (1.3, 1.0)], 0.4)


def test_lejos_fuera_de_la_distancia():
    assert not cerca_de_alguno(1.0, 1.0, [(1.5, 1.0)], 0.4)


def test_elige_una_confirmada():
    assert ids(elegir([cand(1, 1.0, 1.0)])) == [1]


def test_no_elige_una_sin_confirmar():
    assert elegir([cand(1, 1.0, 1.0, observaciones=2)]) == []


def test_nunca_mas_de_n():
    cands = [cand(1, 0.0, 0.0), cand(2, 2.0, 0.0), cand(3, 4.0, 0.0)]
    assert len(elegir(cands, n=2)) == 2


def test_cuenta_los_reportes_ya_gastados():
    cands = [cand(1, 0.0, 0.0), cand(2, 2.0, 0.0), cand(3, 4.0, 0.0)]
    assert len(elegir(cands, puntos_reportados=[(9.0, 9.0)], n=3)) == 2


def test_sin_reportes_disponibles_no_elige_nada():
    assert elegir([cand(1, 0.0, 0.0)], puntos_reportados=[(9.0, 9.0)], n=1) == []


def test_saltea_id_ya_reportado():
    assert elegir([cand(1, 1.0, 1.0)], ids_reportados={1}, puntos_reportados=[(9.0, 9.0)]) == []


def test_saltea_la_misma_caja_con_otro_id():
    # Fusión la recreó con id nuevo, a 10 cm de un punto ya reportado.
    assert elegir([cand(7, 1.1, 1.0)], ids_reportados={1}, puntos_reportados=[(1.0, 1.0)]) == []


def test_no_elige_dos_veces_la_misma_caja_en_una_llamada():
    assert len(elegir([cand(1, 1.0, 1.0), cand(2, 1.1, 1.0)])) == 1


def test_con_un_reporte_elige_la_mas_precisa():
    menos_precisa = cand(1, 0.0, 0.0, cov=[0.0064, 0.0, 0.0, 0.0064])   # desvío 0.08
    mas_precisa = cand(2, 3.0, 0.0, cov=[0.0001, 0.0, 0.0, 0.0001])     # desvío 0.01
    assert ids(elegir([menos_precisa, mas_precisa], n=1)) == [2]


def test_no_modifica_las_listas_que_recibe():
    ids_reportados = {9}
    puntos_reportados = [(9.0, 9.0)]
    elegir([cand(1, 0.0, 0.0), cand(2, 2.0, 0.0)], ids_reportados, puntos_reportados)
    assert ids_reportados == {9}
    assert puntos_reportados == [(9.0, 9.0)]
