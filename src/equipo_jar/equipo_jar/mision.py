"""Misión: qué hace el robot ahora, y cuándo reportar y terminar.

Recibe /map, el plan de cobertura y las candidatas; manda objetivos a
navegación con la acción NavigateToPose y publica /victimas y /done
(docs/team/INTERFACES.md). Es el único que reporta y el único cliente de
navegación. No publica /cmd_vel: eso es solo de navegación.

Esqueleto: espera el mapa y no hace nada más. Hasta recibir /map no se puede
mover el robot ni reportar (§5).
"""
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty

from equipo_jar.comun import QOS_ESTADO, QOS_MAPA, correr, parametros_en_vivo
from equipo_jar.reportes import armar_reporte
from equipo_jar_msgs.msg import Candidatas


class Mision(Node):
    def __init__(self):
        super().__init__('mision')
        # Cantidad de víctimas a buscar (N), que llega como argumento del launch (victimas:=N).
        self.declare_parameter('cantidad_victimas', 5)
        self.cantidad_victimas = self.get_parameter('cantidad_victimas').value
        self.p = parametros_en_vivo(self, {})  # parámetros ajustables

        self.mapa = None
        self.plan = None
        self.candidatas = None

        self.create_subscription(OccupancyGrid, '/map', self.al_recibir_mapa, QOS_MAPA)
        self.create_subscription(Path, '/jar/plan_cobertura', self.al_recibir_plan, QOS_ESTADO)
        self.create_subscription(
            Candidatas, '/jar/candidatas', self.al_recibir_candidatas, QOS_ESTADO)
        self.navegacion = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub_victimas = self.create_publisher(PointStamped, '/victimas', 10)
        self.pub_done = self.create_publisher(Empty, '/done', 10)
        self.get_logger().info(
            f'Hay que encontrar {self.cantidad_victimas} víctimas. Esperando el mapa en /map...')

    def al_recibir_mapa(self, mapa):
        if self.mapa is None:
            self.get_logger().info(
                f'Mapa recibido: {mapa.info.width}x{mapa.info.height} celdas, '
                f'resolución {mapa.info.resolution:.3f} m')
        self.mapa = mapa

    def al_recibir_plan(self, plan):
        self.plan = plan

    def al_recibir_candidatas(self, msg):
        self.candidatas = msg.candidatas

    def reportar_victima(self, x, y):
        """Publica un reporte. Cada llamada gasta un reporte (§7.2)."""
        self.pub_victimas.publish(armar_reporte(x, y, self.get_clock().now().to_msg()))

    def terminar(self):
        """Publica en /done: termina la corrida (§5)."""
        self.pub_done.publish(Empty())


def main(args=None):
    correr(Mision, args)
