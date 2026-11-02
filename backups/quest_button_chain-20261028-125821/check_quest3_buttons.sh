#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"

# Match the interpreter and SAPIEN environment used by the old SteamVR bridge.
steamvr_python="${STEAMVR_PYTHON:-/home/landau/.local/share/uv/python/cpython-3.11.15-linux-x86_64-gnu/bin/python3.11}"
sapien_site_packages="${SAPIEN_SITE_PACKAGES:-/home/landau/gzy/real-exp/.venvs/rmbench/lib/python3.11/site-packages}"

[[ -x "${steamvr_python}" ]] || { echo "Error: SteamVR Python is not executable: ${steamvr_python}" >&2; exit 1; }
[[ -d "${sapien_site_packages}/sapien" ]] || { echo "Error: SAPIEN package is missing: ${sapien_site_packages}" >&2; exit 1; }

export XDG_RUNTIME_DIR="${STEAMVR_XDG_RUNTIME_DIR:-${XDG_RUNTIME_DIR:-/run/user/$(id -u)}}"
export DISPLAY="${STEAMVR_DISPLAY:-${DISPLAY:-:0}}"
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
    exit 1
  fi
fi
[[ -r "${XAUTHORITY}" ]] || { echo "Error: desktop XAUTHORITY is not readable: ${XAUTHORITY}" >&2; exit 1; }

unset PYTHONHOME
export PYTHONPATH="${repository_root}:${sapien_site_packages}"

cd -- "${repository_root}"
exec "${steamvr_python}" -m data_collection.meta_quest.button_check "$@"
