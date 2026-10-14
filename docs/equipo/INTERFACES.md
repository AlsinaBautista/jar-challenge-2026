# Interfaces entre módulos (propuesta)

Este documento define qué se pasan los módulos entre sí. Es una **propuesta
para revisar**: cada uno la lee desde su módulo y comenta en el PR lo que le
falte o le sobre. Cuando se apruebe, cada módulo puede trabajar por separado
contra estas interfaces, usando los nodos falsos de los demás.

Los módulos y la arquitectura están en la base de discusión del equipo. La
interfaz oficial con el robot y el juez está en el reglamento (§7.1) y en
[`docs/QOS.md`](../QOS.md).

## Resumen

| De | A | Qué | Topic o acción | Tipo |
| --- | --- | --- | --- | --- |
| Localización | Todos | Pose del robot en el mapa | TF `map → odom` | `tf2` |
| Localización | Misión | Pose con incertidumbre | `/equipo/pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| Cobertura | Misión | Puntos de observación pendientes, en orden | `/equipo/plan_cobertura` | `nav_msgs/Path` |
| Misión | Cobertura | Punto terminado o descartado | `/equipo/punto_terminado` | `geometry_msgs/PoseStamped` |
| Misión | Navegación | Ir a una pose | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |
| Percepción | Misión | Víctimas candidatas | `/equipo/candidatas` | `equipo_jar_msgs/Candidatas` (propio) |
| Misión | Juez | Reportes y fin | `/victimas`, `/done` | oficiales |
| Navegación | Robot | Velocidad | `/cmd_vel` | oficial |
| Misión | Grabaciones | Estado de la máquina de estados | `/equipo/estado_mision` | `std_msgs/String` |

## Reglas para todos

- **Topics internos con prefijo `/equipo/`.** El auditor del CI marca como error
  a cualquier nodo del equipo que use un topic que no es oficial y que también
  usa el simulador. Con el prefijo no hay choque posible. Los topics que Nav2 y
  AMCL crean solos (`/plan`, `/amcl_pose`, costmaps) se revisan corriendo el
  auditor; si alguno choca, se remapea.
- **QoS:** sensores (`/scan`, `/cam_1/*`) con `qos_profile_sensor_data`. Topics
  internos *reliable*. Los que representan un estado (plan, candidatas) además
  *transient local* con profundidad 1, para que un nodo que arranca tarde reciba
  el último mensaje.
- **Frames:** todo lo que es una posición en el laberinto va en `map`.
- **Un solo nodo publica en `/cmd_vel`:** el de Nav2 (`velocity_smoother`). El
  timer del template que publica velocidad cero se saca cuando entre Nav2.
- **Un solo nodo publica en `/victimas` y `/done`:** el de misión.
- **Parámetros compartidos** (por ejemplo `alcance_deteccion` o `lado_caja`) van
  en `params/params.yaml` bajo cada nodo que los use, con el mismo nombre. Los
  nombres no pueden contener `victim`, `coord`, `spawn` ni `posicion_`.

## Localización → todos

- **TF `map → odom`**, publicado por AMCL. Con eso cualquier nodo transforma a
  `map` lo que ve.
- **`/equipo/pose`** (`PoseWithCovarianceStamped`, `/amcl_pose` remapeado).
  Misión decide con la covarianza si la pose es confiable. El umbral es un
  parámetro.
- **Arranque sin intervención:** `verificar_params.py` permite una pose inicial
  en `params/`. AMCL la toma con `set_initial_pose: true` e
  `initial_pose: {x: 0.0, y: 0.0, yaw: 0.0}`. El robot no larga exactamente en
  el origen, así que la covarianza inicial tiene que cubrir ese error.

Abierto: ¿hace falta avisar algo más que la covarianza? (por ejemplo, "me perdí").

## Cobertura ↔ Misión

- **`/equipo/plan_cobertura`** (`nav_msgs/Path`, frame `map`): los puntos de
  observación **pendientes**, en el orden en que conviene visitarlos. Cada pose
  lleva orientación, porque con 61° de campo de visión importa hacia dónde mira
  el robot. Cada publicación reemplaza a la anterior.
- **`/equipo/punto_terminado`** (`PoseStamped`): misión avisa cada punto que
  alcanzó o que descartó (por ejemplo, porque Nav2 no llegó). Cobertura lo saca
  del plan y, si quiere, recalcula.

Abierto: ¿cobertura necesita saber si el punto se alcanzó o se descartó? ¿Recibe
también las candidatas para no volver a mirar zonas ya resueltas?

## Misión → Navegación

- **Acción `NavigateToPose`** del `bt_navigator` de Nav2. Solo misión es
  cliente: ningún otro módulo manda objetivos ni velocidades.
- En Humble el resultado no trae código de error: "falló" es el estado
  `ABORTED`. Misión pone su propio tiempo máximo por objetivo y cancela si se
  pasa.
- Para mirar alrededor sin moverse, Nav2 trae la acción `Spin`.

Abierto: ¿cómo asegura navegación la orientación final que pide misión? ¿Qué
tolerancias de llegada (posición y ángulo)?

## Percepción → Misión

Percepción agrupa las observaciones y entrega **candidatas**: una por caja que
cree haber visto. Misión solo decide cuándo reportar cada una.

**`/equipo/candidatas`** (`equipo_jar_msgs/Candidatas`): la lista completa
cada vez que cambia algo, y al menos una vez por segundo.

```text
# Candidatas.msg
std_msgs/Header header        # frame_id: map
Candidata[] candidatas

# Candidata.msg
uint32 id                     # estable mientras exista; si dos se fusionan, queda una
geometry_msgs/Point centro    # centro estimado de la caja, en map
float64[4] covarianza_xy      # matriz 2x2 por filas, en m²
uint32 observaciones          # cuántas detecciones la respaldan
builtin_interfaces/Time ultima_vez
```

Hace falta un paquete de mensajes propio (`src/equipo_jar_msgs`, con
`ament_cmake`). Lo arma el responsable de entorno; el Dockerfile lo compila con
el resto de `src/`.

Abierto: ¿alcanza con esto para decidir cuándo reportar? ¿Hace falta un puntaje
de color o de forma, aparte de la covarianza?

## Misión → Juez

- **`/victimas`** (`PointStamped`, frame `map`): un mensaje por reporte, con
  `armar_reporte()` del template. Hay N en total.
- **`/done`** (`std_msgs/Empty`): una vez, al terminar. Antes, misión cancela
  el objetivo de Nav2; sin velocidad nueva, el robot frena solo a los 0,5 s.

## Nodos falsos

Para que cada módulo se pruebe sin esperar a los demás. **Nunca** van en
`competencia.launch.py` ni publican en un topic oficial del robot: van en un
launch aparte (`pruebas.launch.py`).

| Reemplaza a | Qué hace |
| --- | --- |
| Localización | TF estático `map → odom` identidad. En el simulador la odometría alcanza al principio. |
| Cobertura | Publica un `Path` fijo leído de un archivo de escenario. |
| Percepción | Publica candidatas fijas leídas de un archivo de escenario, con ruido. |
| Misión | Recorre el `Path` mandando `NavigateToPose` uno por uno. |
| Cámara real | El degradador: lee la nube del simulador y publica en `/equipo/prueba/points` una versión con los defectos del robot real. Percepción se remapea a ese topic en las pruebas. |

Los archivos de escenario tienen coordenadas, así que van **fuera de `src/`**
(por ejemplo en `pruebas/escenarios/`), para no disparar las advertencias de
hardcodeo.

## Cómo se cambia esta interfaz

Mientras se revisa, los cambios se proponen como comentarios en el PR de la rama
`interfaces`. Una vez acordada, todo cambio a este archivo o a `equipo_jar_msgs`
entra por PR hacia `desarrollo` y lo aprueban los responsables de los módulos
afectados. Ver [`FLUJO.md`](FLUJO.md).
