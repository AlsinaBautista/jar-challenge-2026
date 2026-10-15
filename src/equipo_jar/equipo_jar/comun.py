"""Lo que comparten los nodos del equipo (docs/team/INTERFACES.md)."""
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# /map: el QoS que usa el juez.
QOS_MAPA = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# Topics /jar/ que guardan un estado (plan, candidatas): un nodo que arranca
# tarde recibe el último mensaje.
QOS_ESTADO = QOS_MAPA


def parametros_en_vivo(nodo, valores):
    """Declara los parámetros y devuelve un dict que se actualiza en vivo.

    `valores` son los nombres con su valor por defecto. Leé siempre del dict
    (por ejemplo `self.p['umbral']`), no guardes copias: así un cambio hecho
    durante la corrida se aplica sin reiniciar el nodo.
    """
    actuales = {n: nodo.declare_parameter(n, v).value for n, v in valores.items()}

    def al_cambiar(parametros):
        for p in parametros:
            if p.name in actuales:
                actuales[p.name] = p.value
        return SetParametersResult(successful=True)

    nodo.add_on_set_parameters_callback(al_cambiar)
    return actuales


def correr(clase_nodo, args=None):
    """main() de todos los nodos."""
    import rclpy
    rclpy.init(args=args)
    nodo = clase_nodo()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
