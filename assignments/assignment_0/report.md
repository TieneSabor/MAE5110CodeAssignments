# Assignment 0 Report
---

## Time step sweeps
### Pendulum
#### Solver: Explicit Euler
| time step (s) | Change in total energy (%) |
| --- | --- |
| $10^{-5}$ | 0.04% |
| $10^{-4}$ | 0.4% |
| $10^{-3}$ | 4% |
| $10^{-2}$ | 39% |

#### Solver: RK4
| time step (s) | Change in total energy (%) |
| --- | --- |
| $10^{-5}$ | < 0.01% |
| $10^{-3}$ | < 0.01% |
| $10^{-1}$ | 0.07% |
| $1$ | 97% |

### Bouncing ball
### Solver: Explicit Euler
| time step (s) | Change in total energy (%) |
| --- | --- |
| $10^{-5}$ | 0.17% |
| $10^{-4}$ | 1.7% |
| $10^{-3}$ | 18% |
| $10^{-2}$ | 324% |

### Solver: RK4
| time step (s) | Change in total energy (%) |
| --- | --- |
| $10^{-5}$ | < 0.01% |
| $10^{-3}$ | < 0.01% |
| $10^{-1}$ | 0.07% |
| $1$ | 97% |

## AI usage
### Agentic Coding
- Model used: Muse Spark 1.2 (opencode Go)
- Scopes
    - Relative import issues fix (for the integrators module)

### Auto-completion
- Model used: Github Autopilot Auto-select
- Scopes
    - All code edits
    - Help the comment after manual code edits
    - Help writing the warning messages