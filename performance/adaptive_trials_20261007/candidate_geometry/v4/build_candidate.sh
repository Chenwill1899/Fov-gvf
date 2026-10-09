#!/usr/bin/env bash
set -euo pipefail
stage_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
frozen_tree=/tmp/p5_adaptive_profile
frozen_artifacts="$stage_dir/../../baseline"
if [[ -e "$stage_dir/paper_replay" || -e "$stage_dir/geometry_differential.csv" ]]; then
  printf '%s\n' 'Refusing to overwrite an existing candidate or result.' >&2
  exit 1
fi
g++ -O3 -DNDEBUG -Wall -Wextra -Wpedantic -fopenmp -std=c++17 -I"$frozen_tree/src/pc_gvf/include" -isystem /usr/include/eigen3 -c "$stage_dir/paper_guidance.cpp" -o /tmp/p5_adaptive_profile/paper_guidance_v4.o
g++ -O3 -DNDEBUG -Wall -Wextra -Wpedantic -fopenmp -std=c++17 -I"$frozen_tree/src/pc_gvf/include" -isystem /usr/include/eigen3 "$frozen_tree/src/pc_gvf/tools/paper_replay.cpp" /tmp/p5_adaptive_profile/paper_guidance_v4.o "$frozen_artifacts/libpc_gvf_depth_angular_core.a" -lcrypto -o "$stage_dir/paper_replay"
g++ -O3 -DNDEBUG -Wall -Wextra -Wpedantic -std=c++17 -I"$frozen_tree/src/pc_gvf/include" -isystem /usr/include/eigen3 "$stage_dir/geometry_differential.cpp" -o /tmp/p5_adaptive_profile/geometry_v4_differential
