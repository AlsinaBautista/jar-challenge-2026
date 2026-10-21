"""Servidor falso de navigate_to_pose, para probar misión sin Nav2.

No mueve nada: acepta cada objetivo, espera `duracion` segundos y responde
que llegó, o que falló si se lanza con fallar:=true. Acepta cancelaciones.
Solo para pruebas locales; notes/ no va en la entrega.

    python3 notes/AlsinaBautista/nav_falso.py
    python3 notes/AlsinaBautista/nav_falso.py --ros-args -p duracion:=5.0 -p fallar:=true
"""
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class NavFalso(Node):
    def __init__(self):
        super().__init__('nav_falso')
        self.declare_parameter('duracion', 3.0)
        self.declare_parameter('fallar', False)
        # Reentrant + MultiThreadedExecutor: así un pedido de cancelación se
        # atiende mientras ejecutar() está esperando.
        self.servidor = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self.ejecutar,
            cancel_callback=lambda pedido: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info('Servidor falso de navigate_to_pose listo')

    def ejecutar(self, goal_handle):
        pose = goal_handle.request.pose
        p = pose.pose.position
        duracion = self.get_parameter('duracion').value
        if pose.header.frame_id != 'map':
            self.get_logger().warn(f"frame_id es '{pose.header.frame_id}', debería ser 'map'")
        self.get_logger().info(
            f'Objetivo recibido: x={p.x:.2f} y={p.y:.2f}; respondo en {duracion:.1f} s')
        fin = time.monotonic() + duracion
        while time.monotonic() < fin:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Objetivo cancelado')
                return NavigateToPose.Result()
            time.sleep(0.1)
        if self.get_parameter('fallar').value:
            goal_handle.abort()
            self.get_logger().info('Objetivo fallado (fallar:=true)')
        else:
            goal_handle.succeed()
            self.get_logger().info('Objetivo cumplido')
        return NavigateToPose.Result()


def main():
    rclpy.init()
    nodo = NavFalso()
    executor = MultiThreadedExecutor()
    executor.add_node(nodo)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
