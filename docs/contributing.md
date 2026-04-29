# Contributing

Thanks for your interest in Quad-SDK. This page covers how to set up a dev environment, the project conventions, and how to land a change.

## Quick links

- :material-bug: [Report a bug](https://github.com/robomechanics/quad-sdk/issues)
- :material-source-pull: [Open a PR](https://github.com/robomechanics/quad-sdk/pulls)
- :material-source-branch: Source: [`robomechanics/quad-sdk`](https://github.com/robomechanics/quad-sdk)

## Setting up

Follow [Installation](getting-started/installation.md). Then, for development:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` lets you edit Python launch files / configs without rebuilding.

## Project conventions

### Code

- **C++:** Google style + `clang-format` (see `.clang-format` — 80 cols, 2-space indent). Run `scripts/lint_soft.sh` before submitting.
- **Python:** PEP 8.
- **Tests:** `ament_add_gtest` for C++ tests under `<pkg>/test/`. Add a test next to every new behavior.
- **Launch:** prefer `OpaqueFunction` + `FindPackageShare`. See `quad_utils/launch/planning.py` for the canonical pattern.
- **Params:** use `/**:` namespace wildcards; YAML keys live under `ros__parameters`. Example: `quad_utils/config/go2.yaml`.

### Commits + PRs

- Keep commits focused. A change that touches three packages is fine; a change that touches three packages **and** reformats them is not.
- PR title: imperative mood, ≤ 70 characters. Example: `Add B2 hardware interface and per-robot config`.
- PR body: what + why. Link the issue if applicable.
- Run CI green before requesting review.

### Documentation

This site lives under `docs/`. **Per-package documentation lives in the package's own `README.md`** — the docs site includes those READMEs verbatim via `include-markdown`. Keep edits in one place:

| Edit this | When you're updating |
|---|---|
| `<package>/README.md` | Anything package-specific |
| `docs/architecture/*.md` | Cross-cutting design content |
| `docs/tutorials/*.md` | Step-by-step user guides |
| `docs/faq.md` | Common questions |

To preview the docs site locally:

```bash
python -m venv .venv-docs
source .venv-docs/bin/activate
pip install -r docs/requirements.txt
mkdocs serve
# Browse to http://127.0.0.1:8000
```

`mkdocs serve` live-reloads on file changes.

To verify all links / refs before pushing:

```bash
mkdocs build --strict
```

## Adding a new package

1. Create the package directory with `package.xml`, `CMakeLists.txt` (or `setup.py`), `README.md`, and a `LICENSE` link.
2. Add it to the top-level `README.md` package table.
3. Add a `docs/packages/<pkg>.md` that `include-markdown`s the README.
4. Add a nav entry in `mkdocs.yml`.

## Adding a new robot

Step-by-step: [Adding a New Robot](tutorials/adding-a-robot.md).

## Reporting issues

Useful issues include:

- ROS distro + Ubuntu version
- Robot type and platform (sim or hardware)
- Full launch command
- The first ~30 lines of the failing log
- A minimal repro (bag attachment if possible)

## Code of conduct

This project follows the [Contributor Covenant](https://www.contributor-covenant.org/version/2/1/code_of_conduct/) v2.1. By participating you agree to abide by it.

## License

Quad-SDK is released under the [MIT License](https://github.com/robomechanics/quad-sdk/blob/main/LICENSE). Contributions are accepted under the same terms.
