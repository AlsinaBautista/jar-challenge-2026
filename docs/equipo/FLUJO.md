# Flujo de trabajo

## Ramas

| Rama | Para qué |
| --- | --- |
| `main` | La entrega. Igual al repo del torneo hasta que entreguemos. |
| `interfaces` | Acordar las interfaces entre módulos. Se revisa en su PR, que **no se mezcla**. |
| `desarrollo` | Se crea desde `interfaces` cuando las interfaces estén acordadas. Acá se juntan los módulos. |
| `<módulo>/<tema>` | El trabajo de cada uno, por ejemplo `percepcion/segmentacion-hsv`. Sale de `desarrollo` y vuelve por PR. |

Prefijos de las ramas de trabajo: `localizacion`, `navegacion`, `cobertura`,
`percepcion`, `mision`, `entorno`.

## Pasos

1. **Interfaces:** todos revisan [`INTERFACES.md`](INTERFACES.md) desde su
   módulo y comentan en el PR de `interfaces`.
2. **Desarrollo:** cuando estén acordadas, se crea `desarrollo` desde
   `interfaces` y el PR se cierra sin mezclar.
3. **Módulos:** cada uno trabaja en ramas cortas `<módulo>/<tema>` que vuelven
   a `desarrollo` por PR en pocos días, aunque el módulo no esté terminado.
4. **Entrega:** `desarrollo` se mezcla en `main` como **un solo commit
   (squash)** que además borra `notas/` y `docs/equipo/`. Así a la entrega no
   llegan las notas ni la historia interna.

## Notas de cada uno

Cada integrante tiene su carpeta en [`notas/`](../../notas/), con:

- `NOTAS.md`: las decisiones de su módulo. Para cada una, la pregunta, qué se
  probó y qué se decidió.
- `AGENTE.md`: el contexto para su agente de IA.

Cada uno escribe solo en su carpeta, así no hay conflictos.

Para que tu agente lo lea sin agregar archivos al repo: con Claude Code, creá
un `CLAUDE.local.md` en la raíz con

```text
@notas/<tu-usuario>/AGENTE.md
@docs/equipo/INTERFACES.md
@docs/equipo/FLUJO.md
```

y excluilo de git en tu copia:

```bash
echo CLAUDE.local.md >> .git/info/exclude
```

Con otro agente, lo mismo con su archivo de instrucciones.

## Reglas

- **Sin firmas de agentes:** ningún commit ni PR lleva `Co-Authored-By` de un
  agente ni líneas como "Generated with …".
- **`tools/` y `.github/` no se tocan.**
- **Lo que no está listo no se activa:** si un nodo todavía no funciona, se
  mezcla igual pero fuera de `competencia.launch.py`.
- **Antes de abrir un PR,** correr los mismos controles que el CI (necesita
  ROS 2 Humble: Linux, WSL o Docker):

  ```bash
  bash tools/verificar.sh --template
  ```

- **`equipo.yaml` se completa antes del PR de entrega** a la organización.

## Reglas del CI que conviene conocer

Rompen el CI:

- La palabra `ground_truth` en cualquier archivo de `src/`.
- Mapas o mundos en el repo: `.pgm`, `.world`, `.sdf`, o un `.yaml` de
  `map_server` (con `image:` y `resolution:`).
- En `params/`:
  - nombres de parámetros que contengan `victim`, `coord`, `spawn` o
    `posicion_`, o que terminen en `waypoint`/`waypoints`;
  - listas anidadas o de 4 o más números;
  - strings que contengan `maze` o `laberinto`, o que terminen en `.yaml`,
    `.png`, `.pgm`, `.world` o `.sdf`.
- Un nodo nuestro que publique en un topic del robot o del juez. Solo
  publicamos en `/victimas`, `/done`, `/cmd_vel`, `/tf` y `/tf_static`.
- Un topic interno que también use el simulador. Por eso los nuestros van con
  el prefijo `/equipo/`.

Generan advertencias para el juez técnico:

- Una variable cuyo nombre contenga `victim`, `waypoint`, `checkpoint` o `ruta`,
  inicializada con una lista o un diccionario literal. Ejemplo:
  `self.reportadas = []` y no `self.victimas_reportadas = []`.
- Tres o más pares de coordenadas escritas en el código.
- Nombres de mundos de práctica (`maze_…`, `laberinto_…`) en `src/`.
