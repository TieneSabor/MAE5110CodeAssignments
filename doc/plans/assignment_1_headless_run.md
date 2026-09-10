# Assignment 1 — headless runbook
Status: Brief — 2026-09-09

Headless (no display) runs of `assignments/assignment_1/assignment_1.py` from the workspace root. Setup once: `uv sync --python 3.14`. Always prefix `MPLBACKEND=Agg` so `plt.show()` never blocks; with a non-empty `fig_dir` outputs save as PNG plus `config_used.yaml`, with empty `fig_dir` nothing is written.

| Mode | Command | Artifacts in fig_dir |
|------|---------|----------------------|
| sanity_check | `MPLBACKEND=Agg uv run python assignments/assignment_1/assignment_1.py --config <cfg>` with `mode: sanity_check` | none (energy variation on stdout; warn if > 1e-2) |
| plot_sim | same, with `mode: plot_sim` | `rimless_wheel_simulation.png`, `config_used.yaml` |
| analysis | same, with `mode: analysis` | `rimless_wheel_RoA_analysis.png`, `rimless_wheel_theta_dot_jump_map.png`, `config_used.yaml` |

Default config `assignments/assignment_1/assignment_1.yaml` runs with no `--config` flag. Schema note: angles are floats (rad); `num_spokes`/`gamma_multiplier` derive `alpha`/`gamma`; `theta_dot_fixpoint_multiplier` derives `state_0[1]` from the analytical fix point; RoA/Poincare horizons stay hardcoded (`tf=10`), only `sampling_period` comes from YAML.
