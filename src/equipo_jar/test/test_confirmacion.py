import math

import pytest

from equipo_jar.confirmacion import desvio_maximo, esta_confirmada


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
