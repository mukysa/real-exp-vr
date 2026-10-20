#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"

# Reuse the standalone Python 3.11 interpreter and site-packages directory
# already used by the lab's SteamVR/SAPIEN tooling.  OpenVR itself is a pure
# Python binding; callers may override both paths on another workstation.
openvr_python="${OPENVR_PYTHON:-/home/landau/.local/share/uv/python/cpython-3.11.15-linux-x86_64-gnu/bin/python3.11}"
openvr_site_packages="${OPENVR_SITE_PACKAGES:-/home/landau/gzy/real-exp/.venvs/rmbench/lib/python3.11/site-packages}"
steamvr_runtime="${STEAMVR_RUNTIME:-/home/landau/.local/share/Steam/steamapps/common/SteamVR}"

if [[ ! -x "${openvr_python}" ]]; then
  echo "Error: OpenVR Python is not executable: ${openvr_python}" >&2
  echo "Set OPENVR_PYTHON to a Python 3 interpreter." >&2
  exit 1
fi
if [[ ! -d "${openvr_site_packages}/openvr" ]]; then
  echo "Error: Python OpenVR bindings are missing below: ${openvr_site_packages}" >&2
  echo "Install with: python -m pip install --target '${openvr_site_packages}' openvr==2.12.1401" >&2
  exit 1
fi
if [[ ! -f "${steamvr_runtime}/bin/linux64/libopenvr_api.so" ]]; then
  echo "Error: SteamVR OpenVR runtime is missing below: ${steamvr_runtime}" >&2
  exit 1
fi

unset PYTHONHOME
export PYTHONPATH="${repository_root}:${openvr_site_packages}${PYTHONPATH:+:${PYTHONPATH}}"
export LD_LIBRARY_PATH="${steamvr_runtime}/bin/linux64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

cd -- "${repository_root}"
exec "${openvr_python}" -m data_collection.meta_quest.openvr_bridge "$@"
