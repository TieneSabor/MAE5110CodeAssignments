# Energy-conservation pytest (bouncing ball, Euler + RK4)
# Archived 2026-09-09 — moved from `doc/journal.md`; index in `doc/journal.md`.
Status: Done — 2026-09-08

## Context
No tests existed (`pytest` dev dep, zero `test_*.py`); agreed layout A with a single energy test, scope confined to `tests/`, ball with zero ground damping, `dt=1e-5`, `sim 5s`, `<1%` loss for both integrators.

## Plan
1. Create `tests/test_energy_conservation.py` parametrized over `IntegratorEuler`/`IntegratorRK4`.
2. Run `uv run pytest tests/test_energy_conservation.py -v`.
3. Log result here; no source/config edits.

### Files to edit
| File | Change |
|------|--------|
| `tests/test_energy_conservation.py` | new parametrized energy-conservation test |

## Result
✅

2 passed in 5.76s (`test_energy_conservation[IntegratorEuler]`, `[IntegratorRK4]`).

### Files changed
| File | Change |
|------|--------|
| `tests/test_energy_conservation.py` | new file (params/initial state mirror `assignment_0.py` ball selection; `(2, N)` energy check) |

## Notes
- Pre-existing `discrete_jump` abstract-blocker already fixed by user (no-op overrides in both models), so tests-only scope held.
- `dt=1e-5 x 5s` = 500001 points runs in ~6s total; no speed-up needed.
