#!/usr/bin/env bash
set -euo pipefail
stage_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
frozen_tree=/tmp/retain_gains_20261008_geometry_build
frozen_artifacts="$stage_dir/../baseline"
for variant in v2 v4 combined; do
  if [[ -e "$stage_dir/$variant/paper_replay" || -e "$stage_dir/$variant/geometry_differential.csv" ]]; then
    printf '%s\n' "Refusing to overwrite variant $variant" >&2
    exit 1
  fi
  g++ -O3 -DNDEBUG -Wall -Wextra -Wpedantic -fopenmp -I"$frozen_tree/src/pc_gvf/include" -isystem /usr/include/eigen3 -c "$stage_dir/$variant/paper_guidance.cpp" -o "$frozen_tree/paper_guidance_$variant.o"
  g++ -O3 -DNDEBUG -Wall -Wextra -Wpedantic -fopenmp -I"$frozen_tree/src/pc_gvf/include" -isystem /usr/include/eigen3 "$frozen_tree/src/pc_gvf/tools/paper_replay.cpp" "$frozen_tree/paper_guidance_$variant.o" "$frozen_artifacts/libpc_gvf_depth_angular_core.a" -lcrypto -o "$stage_dir/$variant/paper_replay"
  g++ -O3 -DNDEBUG -Wall -Wextra -Wpedantic -I"$frozen_tree/src/pc_gvf/include" -isystem /usr/include/eigen3 "$stage_dir/$variant/geometry_differential.cpp" -o "$frozen_tree/geometry_${variant}_differential"
  "$frozen_tree/geometry_${variant}_differential" > "$stage_dir/$variant/geometry_differential.csv" 2> "$stage_dir/$variant/geometry_differential.stderr"
done
