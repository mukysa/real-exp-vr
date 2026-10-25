#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for argument in "$@"; do
  case "${argument}" in
    --left|--right|--duo)
      exec bash "${script_dir}/start_quest3_hardware.sh" "$@"
      ;;
  esac
done
exec bash "${script_dir}/start_quest3_hardware.sh" --left "$@"