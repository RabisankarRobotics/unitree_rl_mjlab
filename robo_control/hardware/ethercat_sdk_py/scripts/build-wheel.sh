#!/usr/bin/env bash

set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIST_DIR="${PKG_DIR}/dist"
WHEELHOUSE="${PKG_DIR}/wheelhouse"

cd "${PKG_DIR}"

if ! command -v uv >/dev/null 2>&1; then
  echo "error: uv not found. Install from https://docs.astral.sh/uv/" >&2
  exit 1
fi

ARCH="$(uname -m)"
if [[ "${ARCH}" != "x86_64" ]]; then
  echo "warning: building on ${ARCH}; this script targets x86_64." >&2
fi

echo "==> Cleaning previous build output"
rm -rf "${DIST_DIR}" "${WHEELHOUSE}" build

echo "==> Building wheel with uv (pinned to CPython 3.12 for abi3 compatibility)"
# Build against the oldest supported interpreter so the resulting cp312-abi3
# wheel loads on every Python >= 3.12. Building with a newer Python would
# produce an abi3 module that only loads on that Python or newer.
uv build --wheel --no-sources --python 3.12 --out-dir "${DIST_DIR}"

raw_wheel=$(ls "${DIST_DIR}"/*.whl | head -n1)
echo "==> Built: ${raw_wheel}"

echo "==> Repairing wheel with auditwheel (bundling native deps)"
mkdir -p "${WHEELHOUSE}"
uvx --from auditwheel auditwheel repair \
    --wheel-dir "${WHEELHOUSE}" \
    "${raw_wheel}"

repaired_wheel=$(ls "${WHEELHOUSE}"/*.whl | head -n1)
echo
echo "==> Done. Distributable wheel:"
echo "    ${repaired_wheel}"
echo
echo "    Install with:"
echo "        pip install ${repaired_wheel}"
