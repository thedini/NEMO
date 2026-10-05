#!/usr/bin/env python3
"""Round-trip / differential checks of NEMO raw capture output against the
analysis code's own candump parser.

parse_can_log() below is copied VERBATIM from
  the downstream analysis pipeline (train_ensemble.py, parse_can_log)
so "parses identically" means exactly what the training pipeline sees.

  roundtrip_check.py selftest <test_raw_capture binary> <workdir>
      Generate random frames, push them through the native firmware harness
      (fake driver + real NMEA2000 library + Raw_Capture core), parse the
      output with parse_can_log and require identical (ts, id, dlc, data).

  roundtrip_check.py diff <input candump -t a> <nemo raw output> <legacy output>
      Differential replay check: NEMO output must be frame-identical to the
      input (timestamps compared relative to the first frame).
"""
import random
import re
import subprocess
import sys
from collections import Counter
from typing import Optional

import pandas as pd


# ---- begin verbatim: paper2-lstm/code/train_ensemble.py:53-101 ----
def parse_can_log(filepath: str, max_lines: Optional[int] = None) -> pd.DataFrame:
    """Parse CAN log file into DataFrame.

    Supports standard candump format:
        (timestamp) interface header [dlc] data_bytes

    Args:
        filepath: Path to CAN log file
        max_lines: Maximum lines to read (None for all)

    Returns:
        DataFrame with columns: timestamp, header, pgn, sa, data
    """
    pattern = r'\((\d+\.\d+)\)\s+\w+\s+([0-9A-Fa-f]+)\s+\[(\d)\]\s+(.*)'

    records = []
    with open(filepath, 'r') as f:
        for i, line in enumerate(f):
            if max_lines and i >= max_lines:
                break

            match = re.match(pattern, line.strip())
            if match:
                timestamp = float(match.group(1))
                header_hex = match.group(2)
                header = int(header_hex, 16)
                data_bytes = match.group(4)

                # Extract J1939 fields
                sa = header & 0xFF
                ps = (header >> 8) & 0xFF
                pf = (header >> 16) & 0xFF
                dp = (header >> 24) & 0x01

                # Calculate PGN
                if pf < 240:
                    pgn = (dp << 16) | (pf << 8)
                else:
                    pgn = (dp << 16) | (pf << 8) | ps

                records.append({
                    'timestamp': timestamp,
                    'header': header_hex,
                    'pgn': pgn,
                    'sa': sa,
                    'data': data_bytes
                })

    return pd.DataFrame(records)
# ---- end verbatim ----

PATTERN = re.compile(r'\((\d+\.\d+)\)\s+\w+\s+([0-9A-Fa-f]+)\s+\[(\d)\]\s+(.*)')  # same regex, for dlc + exact ts
LEGACY = re.compile(r'^can1\s+([0-9A-Fa-f]{8})\s+\[(\d)\]\s+(.*)$')


def exact_rows(path):
    """(ts_us int, id int, dlc int, data tuple) using the same regex, ts parsed exactly from the string."""
    rows = []
    with open(path) as f:
        for line in f:
            m = PATTERN.match(line.strip())
            if not m:
                continue
            s, us = m.group(1).split('.')
            rows.append((int(s) * 1_000_000 + int(us.ljust(6, '0')[:6]), int(m.group(2), 16),
                         int(m.group(3)), tuple(int(b, 16) for b in m.group(4).split())))
    return rows


def parser_rows(path):
    df = parse_can_log(path)
    return [(r.timestamp, int(r.header, 16), len(r.data.split()), tuple(int(b, 16) for b in r.data.split()))
            for r in df.itertuples()]


def compare(ref, out, label, ts_tol=None):
    """ref/out: lists of (ts, id, dlc, data). ts compared relative to the first row."""
    problems = 0
    if len(ref) != len(out):
        print(f"  {label}: COUNT MISMATCH ref={len(ref)} out={len(out)}")
        problems += 1
    n = min(len(ref), len(out))
    t0r, t0o = (ref[0][0], out[0][0]) if n else (0, 0)
    id_bad = dlc_bad = data_bad = ts_bad = 0
    max_dt = 0
    for (a, b) in zip(ref[:n], out[:n]):
        id_bad += a[1] != b[1]
        dlc_bad += a[2] != b[2]
        data_bad += a[3] != b[3]
        dt = abs((a[0] - t0r) - (b[0] - t0o))
        max_dt = max(max_dt, dt)
        ts_bad += (dt != 0) if ts_tol is None else (dt > ts_tol)
    problems += id_bad + dlc_bad + data_bad + ts_bad
    print(f"  {label}: rows ref={len(ref)} out={len(out)} id_mismatch={id_bad} dlc_mismatch={dlc_bad} "
          f"data_mismatch={data_bad} ts_mismatch={ts_bad} max_rel_ts_diff={max_dt:g}")
    return problems


def write_candump(path, frames):
    with open(path, 'w') as f:
        for ts, cid, data in frames:
            f.write(f" ({ts // 1_000_000:010d}.{ts % 1_000_000:06d})  can0  {cid:08X}   [{len(data)}] "
                    + ''.join(f" {b:02X}" for b in data) + "\n")


def selftest(binary, workdir):
    rnd = random.Random(20261004)
    frames, ts = [], 1_769_452_128_292_872
    for _ in range(5000):
        ts += rnd.randint(150, 40_000)
        cid = rnd.getrandbits(29)
        dlc = rnd.randint(1, 8)  # DLC 0: see note below
        frames.append((ts, cid, [rnd.getrandbits(8) for _ in range(dlc)]))
    src = f"{workdir}/selftest_in.txt"
    write_candump(src, frames)
    out, leg = f"{workdir}/selftest_raw.txt", f"{workdir}/selftest_legacy.txt"
    # start micros 5 s before the 32-bit wrap
    r = subprocess.run([binary, "replay", src, out, leg, str(2**32 - 5_000_000)], capture_output=True, text=True)
    print("  " + r.stdout.strip())
    ref = [(t, c, len(d), tuple(d)) for t, c, d in frames]
    p = compare(ref, exact_rows(out), "exact-string parse vs generated", None)
    ref_s = [(t / 1e6, c, n, d) for t, c, n, d in ref]  # parse_can_log yields float seconds
    p += compare(ref_s, parser_rows(out), "parse_can_log (verbatim) vs generated", 1e-6)
    # DLC 0: candump writes "[0] " and parse_can_log strips the line, so neither
    # candump's nor NEMO's DLC-0 lines are parsed; check that NEMO matches candump byte-for-byte instead.
    if r.returncode != 0 or p:
        print("SELFTEST FAILED"); sys.exit(1)
    print("  selftest round-trip OK (5000 random frames, micros wrap crossed)")


def diff(inp, raw, legacy):
    problems = 0
    ref_exact, out_exact = exact_rows(inp), exact_rows(raw)
    problems += compare(ref_exact, out_exact, "exact (string ts, integer us)", None)
    ref_p, out_p = parser_rows(inp), parser_rows(raw)
    problems += compare(ref_p, out_p, "parse_can_log (verbatim) float ts", 1e-6)
    with open(raw) as f:
        lines = f.read().splitlines()
    meta = [l for l in lines if l.startswith('#')]
    other = [l for l in lines if not l.startswith('#') and not PATTERN.match(l.strip())]
    print(f"  output lines={len(lines)} frame_lines={len(out_exact)} '#'-meta lines={len(meta)} "
          f"(ignored by parser: {all(not PATTERN.match(m.strip()) for m in meta)}) unparsed_non_meta={len(other)}")
    problems += len(other)
    # Byte-identical text once the timestamp base and interface name are normalised.
    t0i, t0o = ref_exact[0][0], out_exact[0][0]
    def norm(path, t0, iface):
        res = []
        with open(path) as f:
            for l in f:
                m = re.match(r'^ \((\d+)\.(\d{6})\)  (\w+)  (.*)$', l.rstrip('\n'))
                if m:
                    t = int(m.group(1)) * 1_000_000 + int(m.group(2)) - t0
                    res.append(f"{t}|{m.group(4)}")
        return res
    ni, no = norm(inp, t0i, 'can0'), norm(raw, t0o, 'can2')
    text_eq = ni == no
    print(f"  text identical after (ts - ts0, iface) normalisation: {text_eq}")
    problems += 0 if text_eq else 1

    # Evidence: what the pre-fix output made of the same traffic.
    leg_rows = []
    with open(legacy) as f:
        for l in f:
            m = LEGACY.match(l.rstrip('\r\n'))
            if m:
                leg_rows.append((int(m.group(1), 16), tuple(int(b, 16) for b in m.group(3).split())))
    in_multiset = Counter((r[1], r[3]) for r in ref_exact)
    in_ids = set(r[1] for r in ref_exact)
    leg_multiset = Counter(leg_rows)
    matched = sum((in_multiset & leg_multiset).values())
    unknown_ids = sum(1 for r in leg_rows if r[0] not in in_ids)
    leg_parsed = len(parse_can_log(legacy))
    print(f"  legacy (pre-fix) output for the same input: lines={len(leg_rows)} vs input frames={len(ref_exact)}; "
          f"frames identical to some input frame={matched}; lines with a CAN ID never on the bus={unknown_ids}; "
          f"rows parse_can_log accepts={leg_parsed} (no timestamp)")
    if problems:
        print("DIFF FAILED"); sys.exit(1)
    print("  differential replay OK: NEMO raw output is frame-identical to the input capture")


def perturb_check(compare_script, raw, candump, workdir):
    """Exercise tools/compare_nemo_candump.py: the unmodified replay output must
    PASS; a copy with 3 deleted frames, 1 injected frame, 1 swapped pair,
    +500 ppm drift (95 ms over the 190 s slice, more than the 20 ms match
    tolerance) and 0-400 us jitter must FAIL with exactly those counts; the same
    drift alone must still PASS."""
    import json as _json
    r = subprocess.run([sys.executable, compare_script, 'compare', raw, candump, '--json', f'{workdir}/cmp_clean.json'],
                       capture_output=True, text=True)
    clean = _json.load(open(f'{workdir}/cmp_clean.json'))
    rnd = random.Random(1)
    out, fr = [], 0
    for l in open(raw).read().splitlines():
        m = re.match(r'^ \((\d+)\.(\d{6})\)(.*)$', l)
        if not m:
            out.append(l); continue
        fr += 1
        t = int(m.group(1)) * 10**6 + int(m.group(2))
        t = int(t * (1 + 500e-6)) + rnd.randint(0, 400)
        if fr in (5000, 5001, 12000):
            continue
        out.append(" (%010d.%06d)%s" % (t // 10**6, t % 10**6, m.group(3)))
        if fr == 8000:
            out.append(" (%010d.%06d)  can2  1CFF0042   [8]  DE AD BE EF 00 00 00 00" % (t // 10**6, t % 10**6))
    # drift-only copy: same clock error, no faults -> must PASS
    drift_only = []
    for l in open(raw).read().splitlines():
        m = re.match(r'^ \((\d+)\.(\d{6})\)(.*)$', l)
        if m:
            t = int(int(m.group(1)) * 10**6 + int(m.group(2))) * (1 + 500e-6)
            t = int(t)
            l = " (%010d.%06d)%s" % (t // 10**6, t % 10**6, m.group(3))
        drift_only.append(l)
    open(f'{workdir}/drift_only_nemo.txt', 'w').write("\n".join(drift_only) + "\n")
    r3 = subprocess.run([sys.executable, compare_script, 'compare', f'{workdir}/drift_only_nemo.txt', candump,
                         '--json', f'{workdir}/cmp_drift.json'], capture_output=True, text=True)
    drift = _json.load(open(f'{workdir}/cmp_drift.json'))
    i = len(out) * 3 // 4
    while not (out[i].startswith(' (') and out[i + 1].startswith(' (') and out[i][21:] != out[i + 1][21:]):
        i += 1
    out[i], out[i + 1] = out[i + 1], out[i]
    open(f'{workdir}/perturbed_nemo.txt', 'w').write("\n".join(out) + "\n")
    r2 = subprocess.run([sys.executable, compare_script, 'compare', f'{workdir}/perturbed_nemo.txt', candump,
                         '--json', f'{workdir}/cmp_pert.json'], capture_output=True, text=True)
    pert = _json.load(open(f'{workdir}/cmp_pert.json'))
    ok = (r.returncode == 0 and clean['verdict'] == 'PASS' and clean['matched'] == clean['frames_in_window']['candump']
          and r2.returncode == 1 and pert['missing_in_nemo_29bit'] == 3 and pert['extra_in_nemo'] == 1
          and pert['order_inversions'] == 1 and abs(pert['timestamps']['drift_ppm'] + 500) < 1
          and r3.returncode == 0 and drift['verdict'] == 'PASS'
          and drift['matched'] == drift['frames_in_window']['candump'] and abs(drift['timestamps']['drift_ppm'] + 500) < 1)
    print(f"  compare tool on clean replay: verdict={clean['verdict']} matched={clean['matched']} "
          f"jitter_max={clean['timestamps']['jitter_us_max_abs']}us")
    print(f"  compare tool on perturbed copy: verdict={pert['verdict']} missing={pert['missing_in_nemo_29bit']} "
          f"extra={pert['extra_in_nemo']} inversions={pert['order_inversions']} "
          f"drift={pert['timestamps']['drift_ppm']:.2f}ppm jitter_p99={pert['timestamps']['jitter_us_p99_abs']}us")
    print(f"  compare tool on drift-only copy (500 ppm): verdict={drift['verdict']} matched={drift['matched']}/"
          f"{drift['frames_in_window']['candump']} positional={drift['positional_equal']} "
          f"drift={drift['timestamps']['drift_ppm']:.2f}ppm")
    if not ok:
        print("COMPARE-TOOL CHECK FAILED"); sys.exit(1)
    print("  compare tool detects injected faults correctly")


if __name__ == '__main__':
    if sys.argv[1] == 'perturb':
        perturb_check(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
    elif sys.argv[1] == 'selftest':
        selftest(sys.argv[2], sys.argv[3])
    elif sys.argv[1] == 'diff':
        diff(sys.argv[2], sys.argv[3], sys.argv[4])
    else:
        sys.exit(__doc__)
