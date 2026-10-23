#!/bin/bash
# Build the host simulation of whisker_daq.ino (see sim.cpp) into $OUT/sim.
#   msd/esp32/host_sim/run_sim.sh [OUT_DIR]      (default /tmp/whisker_sim)
# Needs arduino-cli with the esp32 core (for --preprocess) and a g++ (CXX).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:-/tmp/whisker_sim}
CXX=${CXX:-g++}
mkdir -p "$OUT"
arduino-cli compile --fqbn esp32:esp32:esp32 --preprocess "$HERE/../whisker_daq" > "$OUT/whisker_daq_pp.cpp"
# One translation unit: the preprocessed .ino, then the sketch's .cpp files
# (sim.cpp includes them), so the sim's hooks (LK_TRACE) reach the reader too.
SK=$(cd "$HERE/../whisker_daq" && pwd)
for f in "$SK"/*.cpp; do echo "#include \"$f\""; done > "$OUT/sketch_cpp.inc"
"$CXX" -std=gnu++17 -O2 -w -I"$HERE/mock" -I"$SK" \
    -DSKETCH="\"$OUT/whisker_daq_pp.cpp\"" -DSKETCH_CPP="\"$OUT/sketch_cpp.inc\"" \
    "$HERE/sim.cpp" -o "$OUT/sim"
echo "$OUT/sim"
