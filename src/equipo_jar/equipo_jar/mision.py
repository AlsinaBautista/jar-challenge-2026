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
from equipo_jar.confirmacion import select_to_report
from equipo_jar.exploracion import yaw_from_quaternion, first_pending

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
        self.p = parametros_en_vivo(self, {'min_observations': 5, 'max_deviation_m': 0.10, 
                                            'min_report_distance_m': 0.40, 'same_point_distance_m': 0.30, 
                                            'same_point_angle_rad': 0.5, 'return_time_s': 480.0,
                                            'min_observations_final': 1, 'max_deviation_final_m': 0.30, 
                                            'finish_time_s': 570.0, 'idle_spin_s': 10.0, 'idle_spin_rad': 1.57})  # parámetros ajustables

        self.mapa = None
        self.plan = None
        self.candidatas = None

        self.t0 = None
        self.create_timer(0.1, self.step)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_pose = None

        self.reported_ids = set()
        self.reported_points = []

        self.nav_state = 'idle'
        self.state = 'waiting_for_map'

        self.visited = []
        self.current_point = None
        self.current_goal = None

        self.return_sent = False
        self.canceling = False

        self.idle_since = None

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

    def seconds_since_t0(self):
        if self.t0 is None:
            return None
        now = self.get_clock().now()
        elapsed = now - self.t0
        seconds = elapsed.nanoseconds / 1e9
        return float(seconds)

    def current_pose(self):
        try:
            pose = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            x = pose.transform.translation.x
            y = pose.transform.translation.y
            q = pose.transform.rotation
            yaw = yaw_from_quaternion(q)
            return (x, y, yaw)
        except TransformException:
            return None

    def check_reports(self, min_obs, max_deviation):
        if self.candidatas is None:
            return
        selected = select_to_report(self.candidatas, self.reported_ids, self.reported_points,
                                        self.cantidad_victimas, min_obs, 
                                        max_deviation, self.p['min_report_distance_m'])
        for c in selected:
            self.reportar_victima(c.centro.x, c.centro.y)
            self.reported_ids.add(c.id)
            self.reported_points.append((c.centro.x, c.centro.y))
            i = len(self.reported_points)
            self.get_logger().info(f'Reporte {i}. id={c.id} x={c.centro.x :.2f} y={c.centro.y:.2f}')

    def go_to(self, pose):
        if not self.navegacion.server_is_ready():
            self.get_logger().warn('!!! El servidor de navegacion no esta listo !!!', throttle_duration_sec=5.0)
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav_state = "sending"
        future = self.navegacion.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)
        return True

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.nav_state = 'failed'
            self.get_logger().warn('Fallo el objetivo')
            return
        if self.state == 'finishing' or self.state == 'finished':
            goal_handle.cancel_goal_async()
            return
        self.current_goal = goal_handle
        self.nav_state = 'moving'
        goal_handle.get_result_async().add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav_state = 'arrived'
            self.get_logger().info('Status succeded')
        else:
            self.nav_state = 'failed'
            self.get_logger().info('Status failed')

    def change_state(self, new_state):
        self.get_logger().info(f'Estado: <{self.state}> -> <{new_state}>')
        self.state = new_state

    def step(self):
        if self.state == 'waiting_for_map':
            if self.t0 is not None:
                self.change_state('exploring')
            return
        if self.state == 'finished':
            return
        if self.state == 'finishing':
            self.pub_done.publish(Empty())
            self.change_state('finished')
            return
        if self.start_pose is None:
            pose = self.current_pose()
            if pose is not None:
                self.start_pose = pose
                x, y, yaw = pose
                self.get_logger().info(f'Pose de inicio: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
        if self.seconds_since_t0() >= self.p['finish_time_s']:
            self.get_logger().info('Se termina el tiempo de la corrida')
            if self.nav_state == 'moving':
                self.current_goal.cancel_goal_async()
            self.terminar()
            return
        self.check_reports(self.p['min_observations'], self.p['max_deviation_m'])
        if self.state == 'exploring':
            reason = self.reason_to_return()
            if reason is not None:
                self.get_logger().info(f'Vuelvo: {reason}')
                self.change_state('returning')
            else:
                self.explore()
        if self.state == 'returning':
            self.return_to_start()

    def explore(self):
        if self.nav_state == 'sending' or self.nav_state == 'moving':
            return
        if self.nav_state == 'arrived' or self.nav_state == 'failed':
            if self.current_point is not None:
                x = self.current_point.pose.position.x
                y = self.current_point.pose.position.y
                yaw = yaw_from_quaternion(self.current_point.pose.orientation)
                self.visited.append((x, y, yaw))
            self.current_point = None
            self.nav_state = 'idle'
        if self.plan is None:
            self.spin_if_idle()
            return
        point = first_pending(self.plan.poses, self.visited, self.p['same_point_distance_m'], self.p['same_point_angle_rad'])
        if point is None:
            self.spin_if_idle()
            return
        sent = self.go_to(point)
        if sent:
            self.current_point = point
            q = point.pose.orientation
            yaw = yaw_from_quaternion(q)
            self.get_logger().info(f'Punto actual: ({point.pose.position.x}, {point.pose.position.y}, {yaw})')

    def spin_if_idle(self):
        if self.idle_since is None:
            time = self.seconds_since_t0()
            self.idle_since = time
            return
        if self.seconds_since_t0() - self.idle_since < self.p['idle_spin_s']:
            return
        pose = self.current_pose()
        if pose is None:
            return
        x, y, yaw = pose
        if self.go_to(self.make_pose(x, y, yaw + self.p['idle_spin_rad'])):
            self.get_logger().info('Sin plan: giro en el lugar')
            self.idle_since = None


    def reason_to_return(self):
        if len(self.reported_points) >= self.cantidad_victimas:
            return 'todas reportadas'
        elif self.seconds_since_t0() >= self.p['return_time_s']:
            return 'se termina el tiempo'
        elif (self.plan is not None and self.nav_state != 'sending' and self.nav_state != 'moving' 
                and first_pending(self.plan.poses, self.visited, self.p['same_point_distance_m'], 
                    self.p['same_point_angle_rad']) is None):
            return 'no quedan puntos'
        return None

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def return_to_start(self):
        if self.nav_state == 'sending':
            return
        if self.nav_state == 'moving' and not self.return_sent:
            if not self.canceling:
                self.current_goal.cancel_goal_async()
                self.canceling = True
                self.get_logger().info("Cancelando objetivo...")
            return
        if not self.return_sent:
            if self.start_pose is None:
                self.get_logger().warning('No hay pose de inicio, termino sin volver')
                self.terminar()
                return
            else:
                x, y, yaw = self.start_pose
                sent = self.go_to(self.make_pose(x, y, yaw))
                if sent:
                    self.return_sent = True
                    return
        if self.return_sent and (self.nav_state == 'arrived' or self.nav_state == 'failed'):
            self.get_logger().info(f'Vuelta terminada: {self.nav_state}')
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
        self.check_reports(self.p['min_observations_final'], self.p['max_deviation_final_m'])
        self.change_state('finishing')


def main(args=None):
    correr(Mision, args)
