#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$project_root/build/libBaleLoaderHydraulics.so" ]]; then
  echo "Build the hydraulic plugin first:"
  echo "  cmake -S \"$project_root\" -B \"$project_root/build\""
  echo "  cmake --build \"$project_root/build\" -j 2"
  exit 1
fi
export GZ_SIM_SYSTEM_PLUGIN_PATH="$project_root/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
exec gz sim -r "$@" "$project_root/worlds/simple_tractor.sdf"
