"""Cobertura: desde qué puntos, y mirando hacia dónde, conviene mirar el laberinto.

Recibe /map y publica en /jar/plan_cobertura los puntos de observación
pendientes, en orden, en el marco map (docs/team/INTERFACES.md).
Esqueleto: todavía no publica ningún plan.
"""
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node

from equipo_jar.comun import QOS_ESTADO, QOS_MAPA, correr, parametros_en_vivo


class Cobertura(Node):
    def __init__(self):
        super().__init__('cobertura')
        self.p = parametros_en_vivo(self, {})  # parámetros ajustables
        self.mapa = None
        self.create_subscription(OccupancyGrid, '/map', self.al_recibir_mapa, QOS_MAPA)
        self.pub_plan = self.create_publisher(Path, '/jar/plan_cobertura', QOS_ESTADO)

    def al_recibir_mapa(self, mapa):
        self.mapa = mapa
        # TODO: calcular los puntos de observación y publicarlos en self.pub_plan.


def main(args=None):
    correr(Cobertura, args)
