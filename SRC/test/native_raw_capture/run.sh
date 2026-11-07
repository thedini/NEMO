#!/usr/bin/env bash
# Native validation of NEMO's raw CAN capture path.
#
#   SRC/test/native_raw_capture/run.sh [path/to/candump_t_a_slice.txt]
#
# Builds the Raw_Capture core and the NMEA2000 library (the exact sources
# PlatformIO fetched into SRC/.pio/libdeps/teensy40, i.e. the Soups71 fork the
# firmware links) with the host g++, runs the unit/integration tests, the
# Python round-trip test, and - if a slice of a real SocketCAN capture is
# given - the differential replay check.
#
# Requires: g++ (C++11), python3 (pandas optional; used if present).
# Run `pio run -e teensy40` (or `pio pkg install`) once first so libdeps exist.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$(cd "$HERE/../.." && pwd)"
N2K="${NMEA2000_LIB:-$SRC/.pio/libdeps/teensy40/NMEA2000-library/src}"
CAP="$SRC/lib/Raw_Capture"
OUT="${OUT_DIR:-$HERE/build}"
mkdir -p "$OUT"

if [ ! -f "$N2K/NMEA2000.cpp" ]; then
  echo "NMEA2000 library not found at $N2K (run 'pio run -e teensy40' first or set NMEA2000_LIB)" >&2
  exit 2
fi

echo "== NMEA2000 library: $N2K ($(git -C "$N2K/.." rev-parse --short HEAD 2>/dev/null || echo unknown))"

# Library sources: warnings are the library's business; build with -w.
LIBSRCS="N2kMsg N2kStream N2kMessages N2kDeviceList N2kGroupFunction N2kGroupFunctionDefaultHandlers NMEA2000"
OBJS=()
for f in $LIBSRCS; do
  g++ -std=c++11 -O1 -g -w -I"$N2K" -c "$N2K/$f.cpp" -o "$OUT/$f.o"
  OBJS+=("$OUT/$f.o")
done
# N2kTimer: use the generic millis() branch (not clock_gettime) so the test drives time.
g++ -std=c++11 -O1 -g -w -U__linux__ -U__linux -Ulinux -I"$N2K" -c "$N2K/N2kTimer.cpp" -o "$OUT/N2kTimer.o"
OBJS+=("$OUT/N2kTimer.o")

# Our code: strict.
g++ -std=c++11 -O1 -g -Wall -Wextra -Werror -I"$CAP" -c "$CAP/RawCapture.cpp" -o "$OUT/RawCapture.o"
g++ -std=c++11 -O1 -g -Wall -Wextra -Werror -Wno-unused-parameter -I"$CAP" -I"$N2K" \
    -c "$HERE/test_raw_capture.cpp" -o "$OUT/test_raw_capture.o"
g++ -o "$OUT/test_raw_capture" "$OUT/test_raw_capture.o" "$OUT/RawCapture.o" "${OBJS[@]}"

echo "== unit + library-integrated scenario tests"
"$OUT/test_raw_capture"

echo "== round-trip through the analysis repo's parse_can_log"
python3 "$HERE/roundtrip_check.py" selftest "$OUT/test_raw_capture" "$OUT"

if [ "${1:-}" != "" ]; then
  echo "== differential replay of $1"
  # Start micros() 60 s before its 2^32 wrap so the replay also crosses a wrap.
  "$OUT/test_raw_capture" replay "$1" "$OUT/replay_raw.txt" "$OUT/replay_legacy.txt" 0xFC6AC480
  python3 "$HERE/roundtrip_check.py" diff "$1" "$OUT/replay_raw.txt" "$OUT/replay_legacy.txt"
  echo "== hardware-comparison tool (tools/compare_nemo_candump.py) on replay output + injected faults"
  python3 "$HERE/roundtrip_check.py" perturb "$SRC/../tools/compare_nemo_candump.py" "$OUT/replay_raw.txt" "$1" "$OUT"
fi
echo "== ALL NATIVE CHECKS PASSED"
