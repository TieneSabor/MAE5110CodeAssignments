# assignments/assignment_0/assignment_0.py update — align caller with current model/integrator contracts
## Exec — 2026-09-08 — phase edit (V1..V3)
### Verification result — V3 (signature smoke)
- Command: `uv run python -c` inspecting `IntegratorEuler.integrate`, `IntegratorRK4.integrate`, `ModelPendulum.calculate_energy`, `ModelBouncingBall.calculate_energy` signatures.
- Expectation: `model` object parameter present; no `dynamics_function`.
- Actual: both integrators show `(self, param_integrator, param_model, time_trajectory, initial_state, model, checkpoint_callback=None)`; both models show `(self, state, params=None)`. Pass.
### Verification result — V1 (bouncing-ball/RK4, default dt=1e-2)
- Command: `MPLBACKEND=Agg PYTHONPATH=. uv run python assignments/assignment_0/assignment_0.py`
- Expectation: completes without signature errors; energy change <1%.
- Actual: `Initial 9.81 / Final 9.8116 / ratio 1.00015951` → 0.016% change. Pass. (Note: plain run without `PYTHONPATH=.` fails with `ModuleNotFoundError: No module named 'integrators'` — consequence of the script move, see §4 finding.)
### Verification result — V2 (pendulum/RK4 at dt=1e-3, script params)
- Command: inline run with script params (`mass 0.2, damping 0.0`), `sim 5s`.
- Expectation: energy change <1%.
- Actual: `init 1.387344 / final 1.387344 / ratio 1.00000000`. Pass. (Model defaults carry `damping_coeff 0.1`, which bleeds ~86% over 5s — script's explicit `damping 0.0` params are what make this pass.)
- Log location: none (stdout only).
Status: Done — 2026-09-08
Open questions: none blocking; §1.d selections confirmed
Result: all edits applied and verified (V1 ball/RK4 0.016%, V2 pendulum/RK4 1.00000000, V3 signature smoke, V4 plain-run bootstrap ratio 1.00015951). Archived to doc/plans/.

## 1. Goal
### 1.a. Goal statements
- assignments/assignment_0/assignment_0.py runs end-to-end against the current model and integrator contracts.
- Pendulum and bouncing-ball selections both remain usable from the script.
- Energy sanity-check output and energy plot remain available for the selected model.

### 1.b. Success criteria
- Script starts without import errors for either model selection.
- Script completes a 5s simulation without signature/argument errors, and for RK5 and for time step no more than 1e-3, the total energy changes for either cases should be less than 1%. 
    - It should be very close to the assignments/assignment_0/report.md
- Script prints initial, final, and ratio total-energy values.
- Script displays the energy time-history plot for the selected model.

### 1.c. Constraints
- Only assignments/assignment_0/assignment_0.py is in scope; models/ and integrators/ contracts are treated as fixed.
- Existing selectable pendulum and bouncing-ball behavior is preserved.
- No new third-party dependencies beyond current numpy/matplotlib usage.
- Don't remove comment.  Edit those relevant to the signature updates.

### 1.d. Questions
- {open} Is scope caller-only, or should model/integrator defects also be fixed.
    - model/integrator defects in todo, except that the current line 21-24 in RK4 be replaced with latest checkpoint callback signature
- {open} Which default integrator and dynamic selections should the updated script keep.
    - Default bouncing ball/rk4 but keep comments consistent (so I can (un)comment some and test different things)
- {open} Is the phase-portrait TODO in scope or explicitly out of scope.
    - out of scope
- {answered} Plan deliverable is an update plan for assignment_0.py, not a research-only note.

## 2. High-Level Architecture
### 2.a. Interfaces
```python
# ========================
# Library: models (models/model_base.py, model_pendulum.py, model_bouncing_ball.py)
# ========================
# Section: Model instance — owns params; dynamics/energy take optional override.
class Model:
  # Return default param dict for this model
  def generate_params() -> dict: ...
  # Compute state derivative [M] from time, state [M], params
  def dynamics(t: float, state: Array[M], params: dict | None) -> Array[M]: ...
  # Compute energies over state [M] or trajectory [M, N]
  def calculate_energy(state: Array, params: dict | None) -> tuple[Array[N], Array[N]]: ...

# ========================
# Library: integrators (integrators/integrator_base.py, integrator_euler.py, integrator_rk4.py)
# ========================
# Section: Integrator — takes model object, never a bare dynamics function.
class Integrator:
  # Step time_trajectory [N] from initial_state [M] via model.dynamics
  def integrate(param_integrator: dict, param_model: dict, time_trajectory: Array[N], initial_state: Array[M], model: Model, checkpoint_callback=None) -> Array[M, N]: ...
  # checkpoint_callback contract: (t1, s1, t2, s2, model) -> None; both Euler and RK4 forward all 5 args.
  # Caller needing a different model in the callback wraps with lambda; integrator still passes its own model.

# =========================
# Application: assignments/assignment_0/assignment_0.py
# =========================
# Section: Selection + orchestration — picks integrator/model, builds time_traj, calls integrate, then energy check + plot.
def main() -> None: ...
```
### 2.b. Data structures
```python
# params_pendulum: dict with gravity, length, mass, damping_coeff
# params_ball: dict with gravity, mass, spring_constant_ground, damping_coefficient_ground, ground_height
# Invariance: model.param_ is source of truth when params arg is None; script passes explicit param_model so both agree.
# Interaction: only via model.set_params/get_params and generate_params; script never reaches into model internals.

# time_traj: Array[N] uniform grid from timestep/sim_time
# state_traj: Array[M, N], column 0 is initial_state
# Interaction: only Integrator.integrate writes state_traj; energy/plot only read it.
```
### 2.c. Pseudo code
```python
def main() -> None:
  # Part A: Initialization
  # A.1. Instantiate integrator (Euler or RK4) per integrator_type
  # A.2. Instantiate model (Pendulum or Ball) per dynamic_type; resolve params + initial_state

  # Part B: Simulation
  # B.1. Build uniform time_traj from timestep/sim_time
  # B.2. state_traj = integrator.integrate({}, params, time_traj, initial_state, model)
  # B.3. Optional commented demo callback (e.g. phase-change print) passed via lambda; integrators forward (t1, s1, t2, s2, model)

  # Part C: Check + report
  # C.1. ke, pe = model.calculate_energy(state_traj, params)
  # C.2. Print initial/final/ratio total energy; plot energy time-histories
```

## 3. Refined steps
### setup
- Confirm active script is assignments/assignment_0/assignment_0.py and root-level stale copy (if any) is out of scope.
- Confirm .venv matches pyproject via uv sync before any run.

### edit
- Edit 1 — update caller signatures in assignments/assignment_0/assignment_0.py to current model/integrator contracts.
  - Modified: assignments/assignment_0/assignment_0.py.
  - Replace stale from models import pendulum/bouncing_ball with ModelPendulum/ModelBouncingBall instantiation.
  - Replace dynamics_function=dynamic_module.dynamics call with model=<instance> in integrator.integrate.
  - Replace module-level calculate_energy call with model instance method.
  - Keep default bouncing-ball/RK4 selection and all unrelated comments intact, updating only signature-relevant comment lines.
- Edit 2 — align checkpoint paths in integrators/integrator_euler.py and integrators/integrator_rk4.py to 5-arg contract (sole §1.d exception, extended to Euler).
  - Modified: integrators/integrator_rk4.py (lines 21-28 region), integrators/integrator_euler.py (callback line).
  - Both integrators call checkpoint_callback(t1, s1, t2, s2, model) with all 5 args on every step.
  - Remove model-specific phase-change print from RK4; re-express it as a commented demo callback in assignments/assignment_0/assignment_0.py via lambda closure.

### build
- Tool uv with uv sync --python 3.14 at workspace root; no compiled packages and no parallel-worker/OOM tuning applies.

### verification
- Test 1 — bouncing-ball/RK4 at dt 1e-3: run uv run python assignments/assignment_0/assignment_0.py with default selection; expect completion and total-energy change below 1% (report.md expects <0.01%).
- Test 2 — pendulum/RK4 at dt 1e-3: switch dynamic_type to pendulum via comments, rerun same command; expect completion and total-energy change below 1% (report.md expects <0.01%).
- Test 3 — import/signature smoke: run uv run python -c to import ModelPendulum, ModelBouncingBall, IntegratorEuler, IntegratorRK4 and inspect integrate/calculate_energy signatures; expect model object parameter present and no dynamics_function parameter.

## 4. Review
- Boundary: dynamics receives single state (M,) while calculate_energy receives trajectory (M, N); caller must not swap shapes.
- Boundary: default dt 1e-2 is fine for default RK4 but Euler at same dt drifts per report.md; large Euler drift is expected, not an update failure.
- Side effect: removing the RK4 phase-change print changes console output; it returns as a commented demo callback in the assignment script, so no behavior is lost.
- Side effect: 2-arg (RK4) and 4-arg (Euler) callbacks break under the 5-arg contract; none exist in-repo, and a lambda wrapper can adapt any legacy callback.
- Boundary: forwarded model vs lambda-captured model may differ by design; the callback body decides which to use, so the demo must state which one it reads.
- Boundary: Model subclasses take no constructor params despite base signature; caller must use generate_params/set_params, not constructor args.
- Boundary: plt.show() blocks headless runs; verification may need a display or temporary non-blocking switch.
- Constraint guard: signature-relevant lines only; sweep notes, TODO, and selection comments stay intact.

## 5. Documentation
- Journal: implementer appends the plan entry (Context/Plan/Files-to-edit) to doc/journal.md, creating doc/ and the bootstrap header if missing; after execution appends Result/Files-changed/Notes per dev-journal format.
- Skill: no package SKILL.md exists for models/integrators in this repo, so no skill-doc update applies.
- Documentation: leave assignments/assignment_0/report.md unchanged unless verification diverges from its energy tables; any divergence gets a brief note in the journal, not a report rewrite.
