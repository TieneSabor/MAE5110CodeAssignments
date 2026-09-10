# Assignment 1 YAML-driven runner
# Archived 2026-09-09 — finished; detail lives here, index in `doc/journal.md`.
## Exec — 2026-09-09 — phase edit (V1..V4)
Status: Done (archived) — 2026-09-09
Open questions: none.

## 1. Goal
### 1.a. Goal statements
- assignment_1.py selects behavior from a YAML file via run mode in {sanity_check, plot_sim, analysis}.
- YAML supplies integrator params, model params, sim initial condition, sampling period, final time, output controls, and analysis toggle.
- sanity_check reports energy conservation without writing plot files.
- plot_sim produces one PNG combining energy and phase portrait.
- analysis produces one RoA PNG and one theta-dot jump PNG.
- fig_dir outputs are auto-created before any save.
### 1.b. Success criteria
- Each mode runs from YAML with no hardcoded scenario selection in main().
- sanity_check prints energy error and writes no PNG.
- plot_sim writes exactly one energy/phase-portrait PNG when plotting is enabled.
- analysis writes RoA PNG plus theta-dot jump PNG when plotting is enabled.
- Missing fig_dir path is created automatically on save paths.
### 1.c. Constraints
- Edit scope is mostly main() in assignments/assignment_1/assignment_1.py.
- YAML file(s) live in assignments/assignment_1/.
- YAML schema is minimal, analysis grid and Poincare constants stay hardcoded.
- CLI uses --config flag defaulting to same-folder YAML.
- sanity_check simulates from analytical fix point, not YAML state_0.
- Save/show semantics use fig_dir plus if_plot.
### 1.d. Questions
- {answered} YAML breadth: minimal vs full — minimal chosen.
- {answered} CLI form: --config with same-folder default chosen.
- {answered} Output semantics: fig_dir plus if_plot chosen.
- {answered} sanity_check start state: analytical fix point chosen.
- {open} Default YAML filename and sample config count.
- {open} PyYAML availability and dependency declaration method.

## 2. High-Level Architecture
### 2.a. Interfaces
```python
# ========================
# Library: config (File: assignments/assignment_1/assignment_1.py, Section: config)
# ========================
# Load and normalize the minimal YAML into Config; applies defaults for missing optional keys.
def load_config(path: str) -> Config: ...

# Create fig_dir (mkdir -p, no-op on empty); called once in main() before dispatch.
def ensure_fig_dir(fig_dir: str) -> None: ...

# =========================
# Application: main (File: assignments/assignment_1/assignment_1.py, Section: main)
# =========================
# Existing callees reused unchanged: plot_sim(...), analysis(...), analytical_fix_point(...).
def main(config_path: str) -> None: ...
```
### 2.b. Data structures
```python
# Config: normalized YAML dict; all angles are floats (rad), no expression parsing.
class Config:
  # State: run mode; Invariance: one of {sanity_check, plot_sim, analysis}
  mode: str
  # State: integrator knobs; Interaction: passed through as param_integrator
  integrator_params: Dict = {}  # e.g. {min_step_size, max_step_size_during_jump}
  # State: wheel params; Interaction: passed through as param_model
  model_params: Dict = {}  # {mass, g, length, alpha, gamma}
  # State: plot_sim start state only; Interaction: ignored by sanity_check/analysis
  state_0: List[float] = [0.0, 0.0]
  # State: time base shared by plot_sim; analysis keeps hardcoded tf for RoA/Poincare
  sampling_period: float
  final_time: float
  # State: output controls; Invariance: empty fig_dir means show, non-empty means save
  fig_dir: str = ""
  if_plot: bool = True
  # State: analysis toggle; Interaction: False skips RoA block, always runs jump map
  if_RoA_analysis: bool = True
```
### 2.c. Pseudo code
```python
def main(config_path: str) -> None:
  # Part A: Config
  # A.1. Resolve default path to assignments/assignment_1/assignment_1.yaml if flag absent
  # A.2. cfg = load_config(config_path)

  # Part B: Preprocess
  # B.1. if cfg.if_plot and cfg.fig_dir != "": ensure_fig_dir(cfg.fig_dir)

  # Part C: Dispatch on cfg.mode
  # C.1. sanity_check: fix = analytical_fix_point(...); plot_sim(fix, ..., if_energy_test=True, if_plot=False)
  # C.2. plot_sim: plot_sim(state_0, ..., final_time, sampling_period, if_energy_test=True, if_plot, fig_dir)
  # C.3. analysis: analysis(sampling_period, ..., if_RoA_analysis, fig_dir)
```

## 3. Refined steps
### setup
- Add PyYAML to project dependencies and sync env so yaml import resolves.
- Create default YAML at assignments/assignment_1/assignment_1.yaml with minimal keys.
### edit
- Edit assignments/assignment_1/assignment_1.py main() to parse --config, load YAML, ensure fig_dir, dispatch by mode.
  - Modified assignments/assignment_1/assignment_1.py adds argparse, load_config, ensure_fig_dir, mode dispatch.
  - New assignments/assignment_1/assignment_1.yaml holds mode, integrator/model params, state_0, sampling_period, final_time, fig_dir, if_plot, if_RoA_analysis.
  - Expected impact: hardcoded scenario block removed, all runs driven by YAML with same plot outputs.
- Edit assignments/assignment_1/assignment_1.py save paths to use pathlib join and mkdir-parents behavior.
  - Modified assignments/assignment_1/assignment_1.py centralizes fig_dir creation in main().
  - Expected impact: missing output folders no longer error, save names unchanged.
### build
- Build tool uv sync covers Python deps only, no compiled packages and no worker-thread tuning needed.
- Build scope is workspace env plus YAML file presence check.
### verification
- Test 1 sanity_check: run with mode sanity_check headless, expect energy-variation log and no PNG written.
- Test 2 plot_sim: run with mode plot_sim and fig_dir set, expect one rimless_wheel_simulation.png written.
- Test 3 analysis: run with mode analysis and if_RoA_analysis true, expect RoA PNG plus theta-dot jump PNG written.

### Verification result — V1
- Command: `MPLBACKEND=Agg uv run python assignments/assignment_1/assignment_1.py --config /tmp/opencode/a1_sanity.yaml`
- Expectation: energy-variation log only, no PNG, variation below 1e-2 warning threshold.
- Actual: `Energy variation over the trajectory: 0.004081`, no warning, no file written.
- Log location: none (stdout only).
- Pass.

### Verification result — V2
- Command: `MPLBACKEND=Agg uv run python assignments/assignment_1/assignment_1.py --config assignments/assignment_1/assignment_1.yaml` (plus no-arg default-path run).
- Expectation: exactly one `rimless_wheel_simulation.png` in fig_dir, auto-created.
- Actual: `assignments/assignment_1/figs/rimless_wheel_simulation.png` (34 KB) written; no-arg run resolves same default with identical output. Perturbed start shows variation 3.76 with conservation warning (expected: plastic impacts dissipate).
- Log location: none (stdout only).
- Pass.

### Verification result — V3
- Command: `MPLBACKEND=Agg uv run python assignments/assignment_1/assignment_1.py --config /tmp/opencode/a1_analysis.yaml`
- Expectation: RoA PNG plus theta-dot jump PNG, Floquet ratios printed.
- Actual: `rimless_wheel_RoA_analysis.png` (54 KB) and `rimless_wheel_theta_dot_jump_map.png` (44 KB) written, `Jump 0..17` ratios printed.
- Log location: none (stdout only).
- Pass.
- Regression: `uv run pytest` 2 passed; invalid mode/dt/tf/state_0 all raise ValueError.

### Verification result — V4 (provenance copy)
- Command: `MPLBACKEND=Agg uv run python assignments/assignment_1/assignment_1.py --config /tmp/opencode/a1_prov.yaml`
- Expectation: `config_used.yaml` byte-identical to input lands in fig_dir next to the PNG; sanity_check (empty fig_dir) skips silently.
- Actual: `/tmp/opencode/a1_prov_figs/` holds `rimless_wheel_simulation.png` + identical `config_used.yaml` (`diff` clean); sanity run copies nothing, no crash. Independently confirmed in-repo by the concurrent sweep run (`figs/num_spokes_4/config_used.yaml`).
- Log location: none (stdout only).
- Pass.
- Note: tree now also holds the user's concurrent extensions (num_spokes/gamma_multiplier/theta_dot_fixpoint_multiplier schema, Floquet PNG, report.md) — interleaved with session edits, so no commit from this session.

## 4. Review
- Invalid mode raises ValueError with allowed set in message.
- Missing YAML path or required keys fail fast with clear error, optional keys use Config defaults.
- state_0 validated as length-2 floats before plot_sim dispatch.
- Non-positive sampling_period or final_time <= sampling_period rejected before time_trajectory build.
- ensure_fig_dir runs whenever fig_dir is non-empty, independent of if_plot, using pathlib mkdir parents.
- analysis final_time limitation documented: RoA and Poincare keep hardcoded tf, only sampling_period comes from YAML.
- analysis if_plot gap closed: if_plot false skips both PNG writes instead of falling through to show.
- Empty fig_dir with if_plot true means show, headless runs must set fig_dir or MPLBACKEND=Agg.
- Angles in YAML are floats only, no pi-expression parsing.
- PyYAML pyproject change is accepted as out-of-main setup scope, PNG overwrites are silent.

## 5. Documentation
- Journal: implementer appends run-mode and YAML-schema notes to doc/journal.md after execution per AGENTS.md Docs.
- Skill: no package skill behavior changes, so no skill-doc update planned.
- Documentation: implementer notes YAML copy/paste run commands for the assignment report deliverable.
