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
from geometry_msgs.msg import PoseStamped

from equipo_jar.comun import QOS_ESTADO, QOS_MAPA, correr, parametros_en_vivo
from equipo_jar.reportes import armar_reporte
from equipo_jar_msgs.msg import Candidatas
from equipo_jar.confirmacion import elegir_para_reportar
from equipo_jar.exploracion import yaw_de_cuaternion, primer_pendiente

from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
import math
from action_msgs.msg import GoalStatus


class Mision(Node):
    def __init__(self):
        super().__init__('mision')
        # Cantidad de víctimas a buscar (N), que llega como argumento del launch (victimas:=N).
        self.declare_parameter('cantidad_victimas', 5)
        self.cantidad_victimas = self.get_parameter('cantidad_victimas').value
        self.p = parametros_en_vivo(self, {'min_observaciones': 5, 'max_desvio_m': 0.10, 
                                            'distancia_min_reportes_m': 0.40, 'distancia_mismo_punto_m': 0.30, 
                                            'angulo_mismo_punto_rad': 0.5, 'tiempo_volver_s': 480.0})  # parámetros ajustables

        self.mapa = None
        self.plan = None
        self.candidatas = None

        self.t0 = None
        self.create_timer(0.1, self.paso)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_inicio = None

        self.ids_reportados = set()
        self.puntos_reportados = []

        self.estado_nav = 'libre'
        self.estado = 'esperando_mapa'

        self.visitados = []
        self.punto_actual = None

        self.vuelta_enviada = False
        self.cancelando = False

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
            self.t0 = self.get_clock().now()
        self.mapa = mapa

    def segundos_desde_t0(self):
        if self.t0 is None:
            return None
        ahora = self.get_clock().now()
        transcurrido = ahora - self.t0
        segundos = transcurrido.nanoseconds / 1e9
        return float(segundos)

    def pose_actual(self):
        try:
            pose = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            x = pose.transform.translation.x
            y = pose.transform.translation.y
            q = pose.transform.rotation
            yaw = yaw_de_cuaternion(q)
            return (x, y, yaw)
        except TransformException:
            return None

    def revisar_reportes(self):
        if self.candidatas is None:
            return
        elegidas = elegir_para_reportar(self.candidatas, self.ids_reportados, self.puntos_reportados,
                                        self.cantidad_victimas, self.p['min_observaciones'], 
                                        self.p['max_desvio_m'], self.p['distancia_min_reportes_m'])
        for c in elegidas:
            self.reportar_victima(c.centro.x, c.centro.y)
            self.ids_reportados.add(c.id)
            self.puntos_reportados.append((c.centro.x, c.centro.y))
            i = len(self.puntos_reportados)
            self.get_logger().info(f'Reporte {i}. id={c.id} x={c.centro.x :.2f} y={c.centro.y:.2f}')

    def ir_a(self, pose):
        if not self.navegacion.server_is_ready():
            self.get_logger().warn('!!! El servidor de navegacion no esta listo !!!', throttle_duration_sec=5.0)
            return False
        objetivo = NavigateToPose.Goal()
        objetivo.pose = pose
        self.estado_nav = "enviando"
        futuro = self.navegacion.send_goal_async(objetivo)
        futuro.add_done_callback(self.al_responder_objetivo)
        return True

    def al_responder_objetivo(self, futuro):
        goal_handle = futuro.result()
        if not goal_handle.accepted:
            self.estado_nav = 'fallo'
            self.get_logger().warn('Fallo el objetivo')
            return
        self.objetivo_actual = goal_handle
        self.estado_nav = 'yendo'
        goal_handle.get_result_async().add_done_callback(self.al_terminar_objetivo)

    def al_terminar_objetivo(self, futuro):
        estado = futuro.result().status
        if estado == GoalStatus.STATUS_SUCCEEDED:
            self.estado_nav = 'llego'
            self.get_logger().info('Status succeded')
        else:
            self.estado_nav = 'fallo'
            self.get_logger().info('Status failed')

    def cambiar_estado(self, nuevo):
        self.get_logger().info(f'Estado: <{self.estado}> -> <{nuevo}>')
        self.estado = nuevo

    def paso(self):
        if self.estado == 'esperando_mapa':
            if self.t0 is not None:
                self.cambiar_estado('explorando')
            return
        if self.estado == 'terminado':
            return
        if self.pose_inicio is None:
            pose = self.pose_actual()
            if pose is not None:
                self.pose_inicio = pose
                x, y, yaw = pose
                self.get_logger().info(f'Pose de inicio: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
        self.revisar_reportes()
        if self.estado == 'explorando':
            motivo = self.motivo_para_volver()
            if motivo is not None:
                self.get_logger().info(f'Vuelvo: {motivo}')
                self.cambiar_estado('volviendo')
            else:
                self.explorar()
        if self.estado == 'volviendo':
            self.volver()

    def explorar(self):
        if self.estado_nav == 'enviando' or self.estado_nav == 'yendo':
            return
        if self.estado_nav == 'llego' or self.estado_nav == 'fallo':
            x = self.punto_actual.pose.position.x
            y = self.punto_actual.pose.position.y
            yaw = yaw_de_cuaternion(self.punto_actual.pose.orientation)
            self.visitados.append((x, y, yaw))
            self.punto_actual = None
            self.estado_nav = 'libre'
        if self.plan is None:
            return
        punto = primer_pendiente(self.plan.poses, self.visitados, self.p['distancia_mismo_punto_m'], self.p['angulo_mismo_punto_rad'])
        if punto is None:
            return
        enviado = self.ir_a(punto)
        if enviado:
            self.punto_actual = punto
            q = punto.pose.orientation
            yaw = yaw_de_cuaternion(q)
            self.get_logger().info(f'Punto actual: ({punto.pose.position.x}, {punto.pose.position.y}, {yaw})')

    def motivo_para_volver(self):
        if len(self.puntos_reportados) >= self.cantidad_victimas:
            return 'todas reportadas'
        elif self.segundos_desde_t0() >= self.p['tiempo_volver_s']:
            return 'se termina el tiempo'
        elif (self.plan is not None and self.estado_nav != 'enviando' and self.estado_nav != 'yendo' 
                and primer_pendiente(self.plan.poses, self.visitados, self.p['distancia_mismo_punto_m'], 
                    self.p['angulo_mismo_punto_rad']) is None):
            return 'no quedan puntos'
        return None

    def pose_de(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def volver(self):
        if self.estado_nav == 'enviando':
            return
        if self.estado_nav == 'yendo' and not self.vuelta_enviada:
            if not self.cancelando:
                self.objetivo_actual.cancel_goal_async()
                self.cancelando = True
                self.get_logger().info("Cancelando objetivo...")
            return
        if not self.vuelta_enviada:
            if self.pose_inicio is None:
                self.get_logger().warning('No hay pose de inicio, termino sin volver')
                self.terminar()
                return
            else:
                x, y, yaw = self.pose_inicio
                enviado = self.ir_a(self.pose_de(x, y, yaw))
                if enviado:
                    self.vuelta_enviada = True
                    return
        if self.vuelta_enviada and (self.estado_nav == 'llego' or self.estado_nav == 'fallo'):
            self.get_logger().info(f'Vuelta terminada: {self.estado_nav}')
            self.terminar()
            return

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
        self.cambiar_estado('terminado')


def main(args=None):
    correr(Mision, args)
