#!/usr/bin/env bash
set -euo pipefail
conga_env=${1:-/opt/conga-venv}
conga_deps=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
/usr/bin/python3 -m venv "$conga_env"
"$conga_env/bin/python" -m pip install --upgrade pip
"$conga_env/bin/python" -m pip install --index-url https://download.pytorch.org/whl/cpu 'torch==2.15.0'
"$conga_env/bin/python" -m pip install -r "$conga_deps/requirements.txt"
"$conga_env/bin/python" -c 'import numpy, torch, onnxruntime, yaml; print("Conga CPU runtime installed", torch.__version__, onnxruntime.__version__)'
