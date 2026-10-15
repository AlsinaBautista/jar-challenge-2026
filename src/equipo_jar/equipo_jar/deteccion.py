"""Detección: qué se ve rojo y dónde queda en el mapa.

Recibe la cámara y el LiDAR y, con TF, pasa cada detección al marco map. Lo
que le entrega a fusión lo definen los dos responsables de percepción
(docs/team/INTERFACES.md). Esqueleto: solo guarda los últimos mensajes.
"""
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformListener

from equipo_jar.comun import correr, parametros_en_vivo


class Deteccion(Node):
    def __init__(self):
        super().__init__('deteccion')
        self.p = parametros_en_vivo(self, {})  # parámetros ajustables
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.imagen = None
        self.info_camara = None
        self.scan = None
        # Sensores: best effort, por eso qos_profile_sensor_data.
        self.create_subscription(
            Image, '/cam_1/color/image_raw', self.al_recibir_imagen, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/cam_1/color/camera_info', self.al_recibir_info, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.al_recibir_scan, qos_profile_sensor_data)

    def al_recibir_imagen(self, imagen):
        self.imagen = imagen
        # TODO: buscar rojo, medir la distancia con el LiDAR y pasar a map con TF.

    def al_recibir_info(self, info):
        self.info_camara = info

    def al_recibir_scan(self, scan):
        self.scan = scan


def main(args=None):
    correr(Deteccion, args)
