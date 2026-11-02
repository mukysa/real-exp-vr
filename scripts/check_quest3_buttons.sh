#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
if [[ -z "${VR_DEBUG_RUN_ID:-}" ]]; then
  printf -v VR_DEBUG_RUN_ID '%(%Y%m%d-%H%M%S)T-%s' -1 "$$"
  export VR_DEBUG_RUN_ID
fi

# Input-only OpenVR avoids the SAPIEN Vulkan/SteamVR shutdown path that hung
# the shared Intel GPU. Retain --backend sapien for explicit renderer debugging.
backend=openvr
expect_backend=false
for argument in "$@"; do
  if [[ "${expect_backend}" == true ]]; then
    backend="${argument}"
    expect_backend=false
  elif [[ "${argument}" == --backend ]]; then
    expect_backend=true
  elif [[ "${argument}" == --backend=* ]]; then
    backend="${argument#--backend=}"
  fi
done
case "${backend}" in
  openvr|sapien) ;;
  *) echo "Error: --backend must be openvr or sapien." >&2; exit 2 ;;
esac
if [[ "${expect_backend}" == true ]]; then
  echo "Error: --backend requires openvr or sapien." >&2
  exit 2
fi
steamvr_python="${OPENVR_PYTHON:-${STEAMVR_PYTHON:-/home/landau/.local/share/uv/python/cpython-3.11.15-linux-x86_64-gnu/bin/python3.11}}"
input_site_packages="${OPENVR_SITE_PACKAGES:-${SAPIEN_SITE_PACKAGES:-/home/landau/gzy/real-exp/.venvs/rmbench/lib/python3.11/site-packages}}"
if [[ "${backend}" == sapien ]]; then
  steamvr_python="${STEAMVR_PYTHON:-/home/landau/.local/share/uv/python/cpython-3.11.15-linux-x86_64-gnu/bin/python3.11}"
  input_site_packages="${SAPIEN_SITE_PACKAGES:-/home/landau/gzy/real-exp/.venvs/rmbench/lib/python3.11/site-packages}"
  echo "Warning: SAPIEN backend renders VR frames and has caused Intel GPU hangs during shutdown on this host." >&2
fi

[[ -x "${steamvr_python}" ]] || { echo "Error: SteamVR Python is not executable: ${steamvr_python}" >&2; exit 1; }
[[ -d "${input_site_packages}/${backend}" ]] || { echo "Error: ${backend} package is missing: ${input_site_packages}" >&2; exit 1; }

export XDG_RUNTIME_DIR="${STEAMVR_XDG_RUNTIME_DIR:-${XDG_RUNTIME_DIR:-/run/user/$(id -u)}}"
export DISPLAY="${STEAMVR_DISPLAY:-${DISPLAY:-:0}}"
if [[ "${backend}" == sapien ]]; then
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
fi

unset PYTHONHOME
export PYTHONPATH="${repository_root}:${input_site_packages}"

cd -- "${repository_root}"
exec "${steamvr_python}" -m data_collection.meta_quest.button_check "$@"
