"""Fusión: qué observaciones son la misma caja y qué tan seguros estamos.

Publica en /jar/candidatas la lista completa de candidatas cada vez que cambia
algo y al menos una vez por segundo (docs/team/INTERFACES.md).
Esqueleto: publica la lista vacía una vez por segundo.
"""
from rclpy.node import Node

from equipo_jar.comun import QOS_ESTADO, correr, parametros_en_vivo
from equipo_jar_msgs.msg import Candidatas


class Fusion(Node):
    def __init__(self):
        super().__init__('fusion')
        self.p = parametros_en_vivo(self, {})  # parámetros ajustables
        self.candidatas = []  # equipo_jar_msgs/Candidata
        self.pub_candidatas = self.create_publisher(Candidatas, '/jar/candidatas', QOS_ESTADO)
        self.create_timer(1.0, self.publicar_candidatas)

    def publicar_candidatas(self):
        msg = Candidatas()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.candidatas = self.candidatas
        self.pub_candidatas.publish(msg)


def main(args=None):
    correr(Fusion, args)
