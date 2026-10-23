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

# OpenVR must use the server desktop, never SSH's localhost:10.x X11 proxy.
# Overrides affect this child process only; no desktop/system configuration changes.
export XDG_RUNTIME_DIR="${STEAMVR_XDG_RUNTIME_DIR:-${XDG_RUNTIME_DIR:-/run/user/$(id -u)}}"
if [[ -n "${STEAMVR_DISPLAY:-}" ]]; then
  export DISPLAY="${STEAMVR_DISPLAY}"
elif [[ -n "${SSH_CONNECTION:-}${SSH_TTY:-}" || "${DISPLAY:-}" == localhost:* || "${DISPLAY:-}" == 127.0.0.1:* ]]; then
  export DISPLAY=:0
  # The forwarded authority inherited over SSH belongs to MoTTY, not GNOME.
  unset XAUTHORITY
else
  export DISPLAY="${DISPLAY:-:0}"
fi
if [[ -n "${STEAMVR_XAUTHORITY:-}" ]]; then
  export XAUTHORITY="${STEAMVR_XAUTHORITY}"
elif [[ -z "${XAUTHORITY:-}" ]]; then
  shopt -s nullglob
  xauthority_candidates=("${XDG_RUNTIME_DIR}"/.mutter-Xwaylandauth.*)
  shopt -u nullglob
  if (( ${#xauthority_candidates[@]} == 1 )); then
    export XAUTHORITY="${xauthority_candidates[0]}"
  elif [[ -r "${HOME}/.Xauthority" ]]; then
    export XAUTHORITY="${HOME}/.Xauthority"
  else
    echo "Error: could not identify the server desktop XAUTHORITY file." >&2
    echo "Set STEAMVR_DISPLAY and STEAMVR_XAUTHORITY from the server desktop terminal." >&2
    exit 1
  fi
fi
if [[ ! -r "${XAUTHORITY}" ]]; then
  echo "Error: desktop XAUTHORITY is not readable: ${XAUTHORITY}" >&2
  exit 1
fi
echo "SteamVR desktop: DISPLAY=${DISPLAY} XAUTHORITY=${XAUTHORITY}"

unset PYTHONHOME
export PYTHONPATH="${repository_root}:${sapien_site_packages}"

cd -- "${repository_root}"
exec "${steamvr_python}" -m data_collection.meta_quest.steamvr_bridge "$@"
