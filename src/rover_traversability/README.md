# rover_traversability

Terrain traversability for the rover's 3D navigation stack (`rover_nav2/doc/3d_nav/SPEC.md`).

| Part | What it is |
|---|---|
| `traversability_core` | Pure C++ geometry shared by every consumer: disc dilation, physical parameter loading. Phase 3 adds pose evaluation (stability margin, clearance, articulation, utilisation). |
| `rover_traversability/TraversabilityLayer` | Nav2 costmap plugin. Turns a terrain feature map (`grid_map_msgs/GridMap`) into costs. |
| `config/rover_physical.yaml` | The only source of physical parameters. `# PLACEHOLDER` marks unmeasured values. |
| `rover_traversability.physical_params` | Python loader for the same file, same rules. |
| `layer_names.hpp` | Layer names of the map contract, used by every node and plugin. |

## TraversabilityLayer

Ported from the prototype's `mrt_traversability_layer` (github.com/ramsinghal08/3d-nav). Phase 0
keeps its cost blend. Phase 3 replaces it with the utilisation model of `SPEC.md`.

Changes from the prototype:

- `mark_stale_as_not_current` is gone. A map older than `map_timeout`, or no map at all, makes the
  layer not current, which stops the controller. Nothing makes it current without fresh data.
- Default `step_layer` is `step` and default `map_topic` is `/terrain/local_features`, per the
  `SPEC.md` contract.
- The disc dilation moved into `traversability_core` (`core/disc_dilation.hpp`), so the test runs
  against the real code.

### Cost value contract

| Value | Meaning |
|---|---|
| 0 to 252 | graded traversability |
| 253 | reserved, `INSCRIBED_INFLATED_OBSTACLE`, produced only by the inflation layer |
| 254 | `LETHAL_OBSTACLE`, at least one channel over its hard limit |
| 255 | `NO_INFORMATION`, cells outside the feature map only |

Unknown terrain inside the map is written as `unknown_cost` (default 200), never 255.
`updateWithMax` skips cells equal to `NO_INFORMATION`, so a penalty written as 255 would vanish
and the rover would treat occluded ground as free.

### Dilation

Lethal and graded costs are dilated by the inscribed radius with a true disc, decomposed into
vertical chords. A separable horizontal-then-vertical max filter dilates by the bounding square,
over-inflates by sqrt(2) on the diagonals and closes gaps the rover can drive through. The source
snapshot extends the radius past the update window, so the leading edge of a rolling costmap is
not under-inflated. `updateBounds` calls `resetMaps` every cycle, so the dilation never compounds.

## Tests

`colcon test --packages-select rover_traversability`: disc dilation against a brute-force
reference and hand-computed shapes, the physical parameter loaders (C++ and Python), and the
layer loaded through pluginlib with hand-computed costs and the staleness fail-safe.
