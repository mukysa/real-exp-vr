#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"

# These defaults reuse the SAPIEN installation already used by DexArena on the
# lab client. Both paths are explicit overrides so another workstation can use
# its own Python environment without editing this script.
steamvr_python="${STEAMVR_PYTHON:-/home/landau/.local/share/uv/python/cpython-3.11.15-linux-x86_64-gnu/bin/python3.11}"
sapien_site_packages="${SAPIEN_SITE_PACKAGES:-/home/landau/gzy/real-exp/.venvs/rmbench/lib/python3.11/site-packages}"

if [[ ! -x "${steamvr_python}" ]]; then
  echo "Error: SteamVR Python is not executable: ${steamvr_python}" >&2
  echo "Set STEAMVR_PYTHON to a Python interpreter compatible with SAPIEN." >&2
  exit 1
fi
if [[ ! -d "${sapien_site_packages}/sapien" ]]; then
  echo "Error: SAPIEN package not found below: ${sapien_site_packages}" >&2
  echo "Set SAPIEN_SITE_PACKAGES to the matching site-packages directory." >&2
  exit 1
fi

unset PYTHONHOME
export PYTHONPATH="${repository_root}:${sapien_site_packages}"

cd -- "${repository_root}"
exec "${steamvr_python}" -m data_collection.meta_quest.steamvr_bridge "$@"
