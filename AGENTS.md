# AGENTS.md

## Scope
- Only use files inside this workspace (`MAE5110CodeAssignments/`). Do not read parent directories, sibling courses, or anything outside the workspace unless the user explicitly tells you to.

## Build & Run
- Requires Python `>=3.14`. Env manager is `uv` (`pyproject.toml:6`, `uv.lock:3`).
- Setup: `uv sync --python 3.14` — creates `.venv` (gitignored).
- Run inside env: `uv run python assignments/assignment_0/assignment_0.py` from the workspace root (see `README.md:9-16` for env setup).
  - The script self-anchors the workspace root on `sys.path`, so no `PYTHONPATH` is needed when run from the root. Headless runs (no display / must not block on `plt.show()`): prefix `MPLBACKEND=Agg`.
- Tests (dev group): `uv run pytest` / `uv run pytest <path>::<test>` — `pytest` is the only dev dependency (`pyproject.toml:12-15`). No lint/typecheck/formatter config in repo.

## Structure
- `assignments/assignment_0/assignment_0.py` — entry script; selects `IntegratorEuler`/`IntegratorRK4` + `ModelPendulum`/`ModelBouncingBall` instance, runs `integrate()`, prints/plots energy. Includes a demo `checkpoint_callback` (phase-change print).
- `models/` — `model_base.py` (`ModelBase`: `dynamics(t, state, params)`, `calculate_energy(state, params)`, `generate_params()`), `model_pendulum.py` (`ModelPendulum`), `model_bouncing_ball.py` (`ModelBouncingBall`). State shape `(2,)` or `(2, N)` vectorized. Note: `ModelPendulum.generate_params()` defaults to `damping_coeff 0.1`; the assignment script overrides with `0.0` for the energy-conservation check.
- `integrators/` — `integrator_base.py:3` contract is `IntegratorBase.integrate(param_integrator, param_model, time_trajectory, initial_state, model, checkpoint_callback=None)`; both Euler and RK4 forward `checkpoint_callback(t1, s1, t2, s2, model)` (5 args). Import as `from integrators import integrator_euler` etc. (`integrators/__init__.py:4`).
- `assignments/assignment_0/` — `assignment_0.md` (spec for Euler/RK4 encapsulation and bouncing-ball task) + `report.md` (results/energy tables).

## Conventions
- Params are plain `dict`s (`gravity`, `length`, `mass`, `damping_coeff`) — see `models/model_pendulum.py` and `assignments/assignment_0/assignment_0.py` selection block.
- Follow existing style: `snake_case` for functions/vars, `PascalCase` for integrator classes.

## Docs
- Dev journal: `doc/journal.md`. Finished plans archive to `doc/plans/`.

## OpenCode
- Config at `.opencode/opencode.json:3` points `instructions` to this file (`../AGENTS.md` relative to `.opencode/`). Skills paths: `.opencode/skills` + `.opencode/workspace/skills`.
- Available skills: `agents-bootstrap` (bootstrap/sync this file from `.opencode/AGENTS.md.template`), `research-notes`, `dev-journal`. Load via `skill` tool, not by guessing.
- After changing `opencode.json`, skills, or this file, restart opencode.
