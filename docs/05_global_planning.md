# 5. Global planning

The global planner computes a raceline (path + speed profile) from the extracted track,
offline, and writes it as a CSV. `global_planner` publishes the CSV at the start of every run.

Full reference: [`code/planning_scripts/README.md`](../code/planning_scripts/README.md)
(optimiser) and [`code/global_planner/README.md`](../code/global_planner/README.md) (publisher).

## 5.1 Methods

The raceline is the centreline shifted sideways by an offset αᵢ at every point, bounded
by the track widths; each method is a quadratic programme in α (solved with OSQP).

| Method | Command | Objective |
|---|---|---|
| Shortest path | `--method shortest` | minimise Σ (first differences)² |
| Minimum curvature | `--method mincurv` | minimise Σ (second differences)² |
| Iterative minimum curvature | `--method mincurv --iters 10` | as above, re-linearised around the previous solution each pass until the offsets change by less than `--tol` |

All methods use the same speed profile: a friction-circle flying lap using `--grip` (0.7)
of μg, capped at `--v-max` (5 m/s).

## 5.2 Generate the racelines

```bash
cd ~/f1tenth_sim_ws/src/Autonomous-Racing-Platform/code/planning_scripts
for MAP in GT carto; do
  python3 optimise_raceline.py --map $MAP --method shortest
  python3 optimise_raceline.py --map $MAP --method mincurv
  python3 optimise_raceline.py --map $MAP --method mincurv --iters 10
done
```

Output: `data/<track>/<map>/raceline/<track>_<map>_<tag>.csv`, `tag` = `shortest`,
`mincurv`, `mincurv_iter` (plus `_<suffix>` if given, e.g. grip variants
`--grip 0.8 --suffix g08`). A plot of each line goes to `plots/`.

## 5.3 Wall clearance check

After optimising, the true (shortest) distance from every point to the walls is checked
against `car width / 2 + --margin` (0.155 + 0.15 m). Points that are too close get tighter
bounds and the QP is solved again, up to `--clear-rounds` times; a line that still fails is
not written.

## 5.4 Use a raceline

Set it in a scenario file (`code/simulation_bringup/config/*.yaml`):

```yaml
stack:
  raceline: Spielberg/carto/raceline/Spielberg_carto_shortest.csv
```

The raceline must be in the same frame as `stack.frame` (a Cartographer-map raceline:
`carto_map`). With `planner:=passthrough` the raceline is tracked without local planning.

## 5.5 Racelines in `data/`

| Map | Racelines in `data/Spielberg/<map>/raceline/` |
|---|---|
| `carto` | `shortest`, `mincurv`, `mincurv_iter` |
| `GT` | `mincurv_iter`, plus grip variants `mincurv_iter_g07` … `_g10` |
