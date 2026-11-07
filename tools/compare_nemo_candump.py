#!/usr/bin/env python3
"""Compare a NEMO raw-capture serial log against a simultaneous SocketCAN
`candump -t a` log of the same bus (hardware validation of the raw capture
fix; procedure in tools/validate_raw_capture.md).

    compare_nemo_candump.py compare NEMO_LOG CANDUMP_LOG [--tol-ms 20] [--json out.json]
    compare_nemo_candump.py record PORT OUT_LOG [--seconds N]     (needs pyserial)

Both logs are parsed with the same regex as the analysis pipeline's
parse_can_log (the downstream analysis pipeline log parser), with
timestamps kept as exact integer microseconds.

compare:
  1. Estimates the clock offset NEMO->candump from frames whose (id, data) is
     unique in both logs (median of differences).
  2. Restricts both logs to the common time window.
  3. Matches frames by (id, dlc, data) in time order within --tol-ms, then
     refits offset + drift (least squares on the matched pairs) and re-matches
     (3 passes), so crystal drift over long recordings is absorbed.
  4. Reports: frame counts, matched, missing in NEMO, extra in NEMO (11-bit
     frames counted separately - NEMO's RX mailboxes are 29-bit only),
     position-by-position ID/DLC/data equality of the two ordered streams,
     order inversions, timestamp offset / drift / jitter, and NEMO's own
     '# NEMO-SYNC' counters (drop, ovr, batch, late, txlost, hwm) and
     '# NEMO-GAP' lines.
  Exit code 0 = PASS (no missing, no extra 29-bit frames, no order
  inversions, NEMO drop/ovr/txlost unchanged during the log), 1 = FAIL,
  2 = usage/input error.

record:
  Logs NEMO's USB serial output verbatim to OUT_LOG and, after every
  '# NEMO-SYNC' line, appends '# HOST-RX t=<epoch seconds>' with the host
  receive time (used as a cross-check of the frame-based offset).
"""
import argparse
import json
import re
import statistics
import sys
import time
from collections import defaultdict, deque

FRAME = re.compile(r'\((\d+\.\d+)\)\s+\w+\s+([0-9A-Fa-f]+)\s+\[(\d)\]\s+(.*)')  # = parse_can_log regex
SYNC = re.compile(r'^# NEMO-SYNC (.*)$')
HOSTRX = re.compile(r'^# HOST-RX t=(\d+\.\d+)')


def parse_log(path):
    frames, syncs, gaps, host = [], [], [], []
    last_sync_t = None
    with open(path, errors='replace') as f:
        for raw in f:
            line = raw.rstrip('\r\n')
            m = FRAME.match(line.strip())
            if m:
                s, us = m.group(1).split('.')
                data = tuple(int(b, 16) for b in m.group(4).split()) if 'remote' not in m.group(4) else ('RTR',)
                hexid = m.group(2)
                frames.append((int(s) * 1_000_000 + int(us.ljust(6, '0')[:6]), int(hexid, 16),
                               int(m.group(3)), data, len(hexid) == 8))
                continue
            m = SYNC.match(line)
            if m:
                kv = dict(p.split('=', 1) for p in m.group(1).split() if '=' in p)
                syncs.append(kv)
                ts = kv.get('t')
                if ts:
                    s, us = ts.split('.')
                    last_sync_t = int(s) * 1_000_000 + int(us)
                continue
            if line.startswith('# NEMO-GAP'):
                gaps.append(line)
                continue
            m = HOSTRX.match(line)
            if m and last_sync_t is not None:
                host.append((last_sync_t, round(float(m.group(1)) * 1e6)))
    return frames, syncs, gaps, host


def linfit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return 0.0, my - mx
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return b, my - b * mx


def pct(v, p):
    if not v:
        return float('nan')
    v = sorted(v)
    return v[min(len(v) - 1, int(round(p / 100 * (len(v) - 1))))]


def compare(nemo_path, dump_path, tol_ms=20.0, json_out=None):
    nemo, syncs, gaps, host = parse_log(nemo_path)
    dump, _, _, _ = parse_log(dump_path)
    if not nemo or not dump:
        print(f"no frames parsed (nemo={len(nemo)}, candump={len(dump)})")
        return 2

    key = lambda fr: (fr[1], fr[2], fr[3])
    # 1. coarse offset from keys unique in both logs
    cn, cd = defaultdict(list), defaultdict(list)
    for fr in nemo:
        cn[key(fr)].append(fr[0])
    for fr in dump:
        cd[key(fr)].append(fr[0])
    diffs = [cd[k][0] - v[0] for k, v in cn.items() if len(v) == 1 and len(cd.get(k, ())) == 1]
    if len(diffs) < 3:
        # fall back: every key, nearest occurrence (periodic traffic repeats keys)
        diffs = [cd[k][0] - v[0] for k, v in cn.items() if k in cd]
    if not diffs:
        print("no common frames between the logs - wrong bus / no overlap?")
        return 1
    off0 = statistics.median(diffs)

    # 2.+3. Match frames by key in time order within the tolerance, mapping NEMO
    # time into candump time with a linear model (slope = 1 + relative drift).
    # Pass 1 uses slope 1 and the coarse offset; later passes refit slope and
    # intercept on the matched pairs, so crystal drift over long recordings
    # (e.g. 80 ppm x 30 min = 144 ms) cannot push frames out of the window.
    tol = tol_ms * 1000

    def run_match(slope, icpt):
        to_d = lambda t: t * slope + icpt
        lo = max(to_d(nemo[0][0]), dump[0][0]) + 2 * tol
        hi = min(to_d(nemo[-1][0]), dump[-1][0]) - 2 * tol
        N = [fr for fr in nemo if lo <= to_d(fr[0]) <= hi]
        D = [fr for fr in dump if lo <= fr[0] <= hi]
        pool = defaultdict(deque)
        for i, fr in enumerate(N):
            pool[key(fr)].append(i)
        matchN = [None] * len(N)
        matchD = [None] * len(D)
        for j, fr in enumerate(D):
            q = pool.get(key(fr))
            while q and to_d(N[q[0]][0]) < fr[0] - tol:
                q.popleft()  # too old -> stays unmatched
            if q and abs(to_d(N[q[0]][0]) - fr[0]) <= tol:
                i = q.popleft()
                matchN[i], matchD[j] = j, i
        return lo, hi, N, D, matchN, matchD

    slope, icpt = 1.0, float(off0)
    for _pass in range(3):
        lo, hi, N, D, matchN, matchD = run_match(slope, icpt)
        pairs = [(i, matchN[i]) for i in range(len(N)) if matchN[i] is not None]
        if len(pairs) < 2:
            break
        slope, icpt = linfit([N[i][0] for i, _ in pairs], [D[j][0] for _, j in pairs])

    pairs = [(i, matchN[i]) for i in range(len(N)) if matchN[i] is not None]
    missing = [D[j] for j in range(len(D)) if matchD[j] is None]
    extra = [N[i] for i in range(len(N)) if matchN[i] is None]
    missing29 = [f for f in missing if f[4]]
    missing11 = [f for f in missing if not f[4]]
    extra29 = [f for f in extra if f[4]]

    # order: NEMO order of matched frames must follow candump order
    js = [j for _, j in pairs]
    inversions = sum(1 for a, b in zip(js, js[1:]) if b < a)

    # position-by-position equality of the two ordered 29-bit streams, between
    # the first and last matched pair (so window edges cannot shift the streams)
    pos_eq = pos_total = 0
    if pairs:
        i0, j0 = pairs[0]
        i1, j1 = pairs[-1]
        jmin, jmax = min(js), max(js)
        Ns = N[i0:i1 + 1]
        Ds = [f for f in D[jmin:jmax + 1] if f[4]]
        pos_total = max(len(Ns), len(Ds))
        pos_eq = sum(1 for a, b in zip(Ns, Ds) if key(a) == key(b))

    # 4. timestamps: fit dump_ts = nemo_ts*(1+drift) + offset on matched pairs
    xs = [N[i][0] for i, _ in pairs]
    ys = [D[j][0] for _, j in pairs]
    res_stats = {}
    if len(pairs) >= 2:
        b, a = linfit(xs, ys)
        res = [y - (b * x + a) for x, y in zip(xs, ys)]
        raw_off = [y - x for x, y in zip(xs, ys)]
        res_stats = {
            'offset_us_median': statistics.median(raw_off),
            'drift_ppm': (b - 1.0) * 1e6,
            'jitter_us_std': statistics.pstdev(res),
            'jitter_us_p50_abs': pct([abs(r) for r in res], 50),
            'jitter_us_p99_abs': pct([abs(r) for r in res], 99),
            'jitter_us_max_abs': max(abs(r) for r in res),
        }
    host_off = None
    if host:
        host_off = statistics.median(h - n for n, h in host)

    last = syncs[-1] if syncs else {}
    nemo_counters = {k: int(last[k]) for k in ('rx', 'drop', 'ovr', 'batch', 'late', 'txlost') if k in last}
    if 'hwm' in last:
        nemo_counters['hwm'] = last['hwm']
    first = syncs[0] if syncs else {}
    for k in ('drop', 'ovr', 'batch', 'late', 'txlost'):
        if k in first and k in last:
            nemo_counters[k + '_during_log'] = int(last[k]) - int(first[k])

    ok = (not missing29 and not extra29 and inversions == 0
          and nemo_counters.get('drop_during_log', nemo_counters.get('drop', 0)) == 0
          and nemo_counters.get('ovr_during_log', nemo_counters.get('ovr', 0)) == 0
          and nemo_counters.get('txlost_during_log', nemo_counters.get('txlost', 0)) == 0)

    rep = {
        'nemo_log': nemo_path, 'candump_log': dump_path, 'tolerance_ms': tol_ms,
        'window_s': (hi - lo) / 1e6,
        'frames_in_window': {'nemo': len(N), 'candump': len(D), 'candump_29bit': sum(1 for f in D if f[4])},
        'matched': len(pairs),
        'missing_in_nemo_29bit': len(missing29), 'missing_in_nemo_11bit_expected': len(missing11),
        'extra_in_nemo': len(extra29),
        'order_inversions': inversions,
        'positional_equal': f"{pos_eq}/{pos_total}",
        'timestamps': res_stats,
        'host_rx_offset_us_median': host_off,
        'nemo_sync_counters': nemo_counters, 'nemo_gap_lines': len(gaps),
        'mode_seen': sorted(set(s.get('mode', '?') for s in syncs)),
        'verdict': 'PASS' if ok else 'FAIL',
        'first_missing': [f"{f[0]} {f[1]:08X} {' '.join('%02X' % b for b in f[3] if isinstance(b, int))}" for f in missing29[:10]],
        'first_extra': [f"{f[0]} {f[1]:08X} {' '.join('%02X' % b for b in f[3] if isinstance(b, int))}" for f in extra29[:10]],
    }
    for k, v in rep.items():
        print(f"{k:32s} {v}")
    if json_out:
        with open(json_out, 'w') as f:
            json.dump(rep, f, indent=2)
    return 0 if ok else 1


def record(port, out, seconds=None):
    import serial  # pyserial
    end = time.time() + seconds if seconds else None
    with serial.Serial(port, 115200, timeout=0.5) as s, open(out, 'w', newline='\n') as f:
        buf = b''
        while end is None or time.time() < end:
            chunk = s.read(4096)
            now = time.time()
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('ascii', 'replace').rstrip('\r')
                f.write(text + '\n')
                if text.startswith('# NEMO-SYNC'):
                    f.write(f"# HOST-RX t={now:.6f}\n")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('compare')
    c.add_argument('nemo_log')
    c.add_argument('candump_log')
    c.add_argument('--tol-ms', type=float, default=20.0)
    c.add_argument('--json')
    r = sub.add_parser('record')
    r.add_argument('port')
    r.add_argument('out')
    r.add_argument('--seconds', type=float)
    a = ap.parse_args()
    if a.cmd == 'compare':
        return compare(a.nemo_log, a.candump_log, a.tol_ms, a.json)
    return record(a.port, a.out, a.seconds)


if __name__ == '__main__':
    sys.exit(main())
