# lts-aero

Aero component package for the FSAE transient lap-time simulation: the
`+Aero` classes (`AeroManager`, `WholeCarAero`, wing and floor elements),
mounted into the main repository at `src/+lts/+components/+Aero`.

## Ownership

| | |
|---|---|
| Department | Aero |
| Maintainer | *add GitHub handle* |
| Term | *e.g. 2026/27* |

## Running the tests

Requires MATLAB R2019b+ (CI pins R2026a) and the `lts-kit` submodule:

    git submodule update --init --recursive

Then in MATLAB, from the repository root: `run_tests`

The runner assembles a temporary `+lts` package sandbox in `build/`
(gitignored) — this repository's classes plus kit's `+util` — and runs
`tests/`. Nothing is installed into the main repository.

## Branch model and workflow

- `staging` — where PRs from forks land. `main` — stable, release-only; it advances only via the release cascade from the main `lts` repository.
- All development is done on forks; see [CONTRIBUTING.md](CONTRIBUTING.md).

## Contract with the main repository

- Components construct from the `cfg.aero` struct fields (`ClA`, `CdA`,
  `xPosition`, `zPosition`, `pitchSensitivityClA`); SI units throughout.
- `computeForces` returns the axle-load struct the Simulator consumes
  (`Fz_front`, `Fz_rear`, `F_drag`, `dragHeight`, `dragXPosition`) —
  `tests/ConformanceTest.m` pins this shape and the `cfg.aero` schema
  (`validateConfig`).
- Renaming any pinned cfg field, state field, or `computeForces` field
  is a **contract change** — see "Changing the contract" on the
  [Contracts page](https://jyjh.github.io/lts/contracts/).
- Details: <https://jyjh.github.io/lts/repo-split/>
