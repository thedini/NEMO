# Bench validation: `attack spam` address-claim flood (DEFECT D25 fix)

Verifies that the spam / DoS positive control actually transmits on the wire,
at a bounded, configurable rate, and stops cleanly. Covers branch
`fix/attack-spam-sends`.

> **Safety / scope:** This is a bench-only procedure on the PhD injection
> fixture. CAN1 on the NEMO board transmits; `can0` on OpenPlotter is the
> listen-only tap (CAN2 side of the fixture). Do not run against any
> operational bus.

## Background (what was broken)

Before this fix, starting the spam attack sent essentially nothing:
the proactive prime loop in `startSpamAttack()` was commented out, and
`Attack_Controller::update()` had no spam branch, so the only transmit path
was the reactive `attackHandler()` replying to inbound PGN 60928 claims. On a
quiet bench bus with no other claimers, that fired rarely or never. In
addition, `loop()` suspends the board's own sensor transmit while an attack is
active. Net effect: the "LOUD" DoS window was really ~30 s of silence from the
injector's source addresses (0x19-0x1B).

After the fix, the flood is driven proactively from `update()` at a
bounded rate (default 100 frames/s, hard cap 1000 fps), cycling source
addresses 0..251.

## Expected on the wire

* **PGN:** 60928 (0xEE00), ISO Address Claim.
* **Priority:** 0 (attacker uses highest priority), destination global (0xFF).
* **Extended CAN IDs:** `0x00EEFF00` through `0x00EEFFFB` — i.e. `00EEFFxx`
  where `xx` is the claimed source address, sweeping `00`..`FB` (0..251) and
  wrapping. (Priority 0 => top bits are zero; a normal priority-6 claim would
  be `18EEFFxx`, so `00EEFFxx` is a clear fingerprint of this attack.)
* **DLC:** 8 (the 64-bit NAME payload).
* **Rate:** ~100 frames/s at the default (one frame per ~10 ms). The address
  low byte in the ID increments by 1 each frame and wraps 251 -> 0.

## Procedure

1. On OpenPlotter, confirm the listen-only interface is up:

   ```
   ip -details link show can0
   ```

   (Bring up if needed, matching the fixture bitrate, e.g.:
   `sudo ip link set can0 up type can bitrate 250000`)

2. Start an absolute-timestamped capture **before** starting the attack:

   ```
   candump -t a can0,00EEFF00:1FFFFF00 > /tmp/spam_validate.log &
   ```

   The mask `00EEFF00:1FFFFF00` isolates PGN 60928 priority-0 global claims
   (any source address). Omit the filter to see the whole bus.

3. Trigger the attack and record ground truth. Either path emits a JSON line
   on the NEMO USB serial console with a millis timestamp:

   * Menu: navigate to Attacks -> Spam -> Start.
   * Serial (campaign runner): send
     `{"cmd":"attack","type":"spam","action":"start"}`
     or with an explicit rate
     `{"cmd":"attack","type":"spam","action":"start","rate":200}`.

   Expected serial start event:
   `{"evt":"attack","type":"spam","state":"start","t_ms":<ms>,"rate_fps":100,"sa_lo":0,"sa_hi":251}`

4. Let it run ~5-10 s, then stop (menu Stop, or
   `{"cmd":"attack","type":"spam","action":"stop"}`). Expected serial stop
   event:
   `{"evt":"attack","type":"spam","state":"stop","t_ms":<ms>,"frames":<N>,"dur_ms":<ms>}`

5. Stop the capture (`kill %1`) and check it:

   ```
   # Frame count and rate over the window
   wc -l /tmp/spam_validate.log
   # Rough rate: lines divided by (last_ts - first_ts)
   awk 'NR==1{t0=$1} {t=$1} END{gsub(/[()]/,"",t0); gsub(/[()]/,"",t); print NR" frames over "t-t0" s"}' /tmp/spam_validate.log
   # Confirm IDs sweep 00EEFF00..00EEFFFB
   cut -d" " -f4 /tmp/spam_validate.log | sort -u | head
   ```

## Pass criteria

* [ ] Frames appear on `can0` **only** between the start and stop serial
      events (nothing in the quiet window before start / after stop).
* [ ] Sustained rate ~= configured `rate_fps` (default ~100/s; +/- a few % is
      fine given loop jitter and the per-update burst cap).
* [ ] Observed IDs are all `00EEFFxx` with `xx` in `00..FB` and the low byte
      increments/wraps — no ID with `xx > FB` (never SA > 251).
* [ ] Serial `frames` count in the stop event is consistent with the captured
      frame count (they are both "frames the driver accepted"; small
      differences from bus-tap drops are acceptable).
* [ ] After stop, the board's three sensor devices (SA 0x19-0x1B) resume their
      normal periodic PGNs within ~1 s, and three ISO Address Claims for
      sensors 0/1/2 are seen immediately after stop.

## Rate / cap sanity check (optional)

Request an out-of-range rate and confirm it is clamped to the hard cap:

```
{"cmd":"attack","type":"spam","action":"start","rate":100000}
```

The start event should report `"rate_fps":1000` (SPAM_MAX_RATE_FPS), and the
captured rate should be ~1000/s, not higher.
