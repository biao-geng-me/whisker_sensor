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
"$CXX" -std=gnu++17 -O2 -w -I"$HERE/mock" -DSKETCH="\"$OUT/whisker_daq_pp.cpp\"" \
    "$HERE/sim.cpp" -o "$OUT/sim"
echo "$OUT/sim"
