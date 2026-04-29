# Quad-SDK docs

This folder is the source for the Quad-SDK documentation site at
<https://robomechanics.github.io/quad-sdk/>.

## Local preview

```bash
python -m venv .venv-docs
source .venv-docs/bin/activate
pip install -r docs/requirements.txt

# Live-reload on file changes:
mkdocs serve

# Strict build (CI runs this — fails on broken refs):
mkdocs build --strict
```

Docs and the C++ build are completely separate. `colcon build` does not
touch this folder; this folder does not affect the runtime.

## Layout

```
docs/
├── index.md                    # Landing page
├── getting-started/            # Install, quickstart, hardware
├── architecture/               # Stack overview, ROS2 migration, Pinocchio
├── packages/                   # One page per package (include-markdown)
├── tutorials/                  # First run, logging, training, etc.
├── faq.md
├── contributing.md
├── requirements.txt            # Python deps for the docs build
├── assets/                     # Images
└── stylesheets/extra.css       # Theme overrides
```

## Source-of-truth conventions

- **Per-package documentation lives in `<package>/README.md`**. The
  `docs/packages/<pkg>.md` pages just `include-markdown` from those. Edit
  the package README, not the site copy.
- **Architecture / cross-cutting content lives in `docs/architecture/`**.
- **Tutorials live in `docs/tutorials/`**.

## Versioning (ROS1 / ROS2 dropdown)

The site uses `mike` for versioning. The dropdown in the top-right shows
labelled versions. Default labels:

| Label | What it is |
|---|---|
| `ros2` (= `latest`) | Current ROS 2 docs (this content) |
| `ros1` | Legacy ROS 1 docs (when published) |

Deploy a specific version manually:

```bash
mike deploy --push --update-aliases ros2 latest
mike set-default --push latest
```

The `Docs` GitHub Actions workflow (in `.github/workflows/docs.yml`)
runs the same commands automatically on `main`.
