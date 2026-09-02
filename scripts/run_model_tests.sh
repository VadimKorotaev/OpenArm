#!/usr/bin/env bash
set -euo pipefail

readonly project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly python="${project_root}/.venv/bin/python"

if [[ ! -x "${python}" ]]; then
  echo "ERROR: virtual environment is missing: ${python}" >&2
  exit 1
fi

export PYTHONPATH="${project_root}/backend"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
exec "${python}" -m pytest -q "${project_root}/backend/tests"
