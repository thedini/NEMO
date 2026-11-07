# Hardware validation: NEMO raw CAN capture vs. SocketCAN `candump`

This is the procedure for checking, on real hardware, that NEMO's serial capture
output (branch `fix/raw-candump-capture`) is frame-identical to an independent
SocketCAN capture of the same bus. Everything up to step 3 has already been
checked off-target (`SRC/test/native_raw_capture/run.sh`: unit tests, the real
NMEA2000 library in the loop, and a replay of a real 24 h capture slice). This
procedure covers what host tests cannot: the FlexCAN interrupt, interrupt
latency under real traffic, USB serial throughput, and clock drift.

> **Approval required.** Do not run steps 3 onward without the bench owner's
> approval. The NEMO devices are on the live bench, where a passive benign
> capture is running. Flashing reboots NEMO. On boot, NEMO's **CAN1** node
> (simulated sensors, `N2km_NodeOnly`) runs ISO address claim and, for active
> sensors, sends product information. That is traffic on the bench bus. Run
> the procedure outside the passive-capture window, or write the reboot time
> into the bench log so the analysis can exclude it. This branch does not
> change CAN1 or anything it transmits. CAN2, the capture interface, stays
> `N2km_ListenOnly`.

## What you need

| Item | Notes |
|---|---|
| NEMO built from this branch | `cd SRC && pio run -e teensy40`. Record `git rev-parse HEAD` and `sha256sum .pio/build/teensy40/firmware.hex`. |
| Rollback image | A build of the previous firmware (e.g. `fddd41c`). Keep the `.hex` next to the new one before you flash. |
| canpi on the same bus | SocketCAN `can0` at 250 kbit/s, `candump -t a can0`. If the passive capture already writes `candump -t a`, reuse its file. Do not start a second candump on the same socket. |
| Capture host (Windows laptop) | Python 3 + `pyserial` (`pip install pyserial`). Close the NEMO GUI or any serial monitor; only one program can open the port. |
| `tools/compare_nemo_candump.py` | Has a `record` mode (serial logger) and a `compare` mode (the check itself). |

NEMO firmware settings: `DEBUG false` and `NEMO_CAPTURE_MODE` left at its default
(`NEMO_CAPTURE_RAW`), both in `SRC/include/constants.h`. GUI JSON streaming must
be off, which is the default after boot. While the GUI stream is on, NEMO
deliberately prints no capture lines.

## Output format (what to expect on the serial port)

```
# NEMO-CAPTURE v=1 mode=raw fmt=candump-ta iface=can2 clock=micros64 origin=boot
# NEMO-SYNC v=1 mode=raw iface=can2 t=0000000003.000012 rx=0 drop=0 ovr=0 batch=0 late=0 hwm=0/2047 txlost=0 wraps=0
 (0000000003.512877)  can2  09F11324   [8]  D8 75 F9 FF FF FF FF FF
 (0000000003.513491)  can2  18EAB801   [3]  00 EE 00
```

* Frame lines use the exact `candump -t a` layout. The timestamp is NEMO's
  boot-relative clock: `micros()` at the CAN interrupt, extended to 64 bits so
  it does not wrap after 71.6 min. `parse_can_log` in the analysis repository
  reads these lines unchanged.
* `#` lines are metadata, and the analysis parsers ignore them:
  * `NEMO-SYNC` is printed once per second. Its counters are cumulative:
    * `rx`: frames captured.
    * `drop`: frames lost because the capture FIFO was full.
    * `ovr`: FlexCAN mailbox overruns.
    * `batch`: interrupts that delivered more than one frame. Order inside such
      a batch is the driver's priority order.
    * `late`: frames that bypassed the interrupt hook, so their timestamp is
      the read time.
    * `hwm`: FIFO high-water mark.
    * `txlost`: capture lines the USB serial port did not accept. Teensy's
      `usb_serial_write()` waits at most 120 ms (`TX_TIMEOUT_MSEC`,
      `cores/teensy4/usb_serial.c`) for a host that has stopped reading. After
      that it drops output until the host reads again, so a stalled terminal
      costs one 120 ms pause of `loop()`, not a hang. The CAN interrupt keeps
      filling the FIFO meanwhile.
  * `NEMO-GAP` is printed in the stream exactly where frames were dropped.
* `iface=n2kreasm` or `mode=RECONSTRUCTED` means the firmware was built with the
  legacy output. That output is **not** raw frames. Do not use it for this
  validation.

## Procedure

1. **Before.** Write in the bench log: date and time, the approver, canpi status
   (which capture is running and to which file), and the NEMO serial port
   (e.g. `COM5`).
2. **Build** on the capture host or an analysis machine: `cd SRC && pio run -e teensy40`. Keep
   `firmware.hex` and its sha256.
3. **Flash** (needs approval): `pio run -e teensy40 -t upload`, or use the
   Teensy Loader with the `.hex`. Write the flash time in the bench log.
4. **Start canpi capture** if it is not already running:
   `candump -t a can0 > canpi_$(date +%Y%m%d_%H%M%S).log`.
5. **Record NEMO** for at least 30 min (longer is better; include periods with
   fast-packet traffic such as 129029 GNSS and 126720 / 126208 addressed
   traffic, and ISO requests 59904):
   `python tools/compare_nemo_candump.py record COM5 nemo_YYYYMMDD_HHMMSS.log --seconds 1800`
   The recorder writes the serial stream unchanged. After every `NEMO-SYNC`
   line it adds `# HOST-RX t=<epoch>`, the host receive time, which is used
   later as a cross-check of the alignment.
6. **Stop** both captures. If the passive capture is long, cut the canpi file
   down to the NEMO recording window plus about 1 min on each side. The
   compare tool also restricts itself to the overlapping window.
7. **Compare** (any machine with Python 3, no extra packages):
   `python3 tools/compare_nemo_candump.py compare nemo_….log canpi_….log --json result.json`
8. **Archive** both logs, `result.json`, the firmware sha256 and the git hash
   together, e.g. under `data/bench/<date>/raw_capture_validation/`.

## Pass criteria

| Field in report | Required | Meaning |
|---|---|---|
| `missing_in_nemo_29bit` | 0 | Every 29-bit frame canpi saw, NEMO saw too. |
| `extra_in_nemo` | 0 | NEMO reported no frame that was not on the bus. |
| `order_inversions` | 0 | Matched frames appear in the same order in both logs. |
| `positional_equal` | N/N | The two ordered streams have the same ID, DLC and data at every position. |
| `nemo_sync_counters.drop_during_log`, `ovr_during_log` | 0 | No FIFO drops and no FlexCAN overruns. |
| `nemo_sync_counters.txlost_during_log` | 0 | The serial port took every line. If it is not 0, the host logger fell behind; the missing frames were lost on USB, not on CAN. |
| `nemo_sync_counters.hwm` | well below `/2047` | Early warning for throughput. `loop()` passes at most 20 frames per `ParseMessages()` call to the library and the printer (`NMEA2000.cpp:2632`), and every loop iteration also updates the I2C OLED, so capture throughput is capped at 20 × loop rate. A `hwm` that keeps climbing towards 2047 shows the main loop is too slow for the bus load. Mitigation: call `NMEA2000_CAN2.ParseMessages()` more than once per `loop()` while the FIFO is deep. |
| `nemo_sync_counters.batch_during_log` | 0 expected | Must be 0 for strict ordering. If it is not 0, the inversions (if any) sit inside batches. Report the number. |
| `nemo_sync_counters.late_during_log` | 0 expected | At most a handful right after boot is acceptable. |
| `timestamps.drift_ppm` | within ±100 | Crystal tolerance of the two clocks. Informational. |
| `timestamps.jitter_us_p99_abs` | report | Residual after offset and drift are removed. Expected to be tens of µs: the two sides timestamp at different points (canpi kernel RX vs. NEMO end-of-frame IRQ). Informational. |

Expected, not failures:

* `missing_in_nemo_11bit_expected` > 0 if anything on the bus sends 11-bit
  frames. NEMO's RX mailboxes are configured for 29-bit identifiers only
  (`NMEA2000_Teensyx.cpp` `setMB(i,RX,EXT)`), so NEMO never receives them.
* Frames with DLC 0 are written as `[0] ` by both candump and NEMO. Because
  `parse_can_log` strips the line, it ignores them on both sides. Both the
  compare tool and `parse_can_log` use that same regex, so DLC-0 frames are
  left out of the comparison on both sides alike.

Not covered by host tests, so this run is their first runtime check: the Teensy
glue in `SRC/lib/Raw_Capture/NMEA2000_TeensyxRawTap.cpp`, i.e. the vector swap,
the wrapper around the driver ISR, `micros()` in the ISR and the IRQ locking.
That file is only compiled, by the firmware build. The native harness
exercises `RawCapture.*` and the real NMEA2000 library through a fake driver
that mirrors the glue, not the glue itself. Note also that the
Teensyx mailbox RX path never sets the remote-frame flag, so RTR frames (not
used by NMEA2000) would appear as data frames.

If anything fails, keep the logs. The report lists the first 10 missing and
extra frames with their timestamps.

## Rollback

Flash the previous `.hex` with the Teensy Loader and note it in the bench log.
No configuration is stored on the device, so nothing else needs undoing.

## How the compare tool itself was checked

`SRC/test/native_raw_capture/run.sh <slice>` replays a real canpi capture slice
through the firmware's capture path (in the native harness) and runs
`compare_nemo_candump.py` twice:

* On the replay output, which must PASS.
* On a copy with 3 frames deleted, 1 frame injected, 1 adjacent pair swapped,
  +500 ppm drift (95 ms over 190 s, more than the 20 ms match tolerance) and
  0–400 µs jitter. This copy must FAIL, and the report must show exactly
  3 missing, 1 extra, 1 inversion and −500 ppm drift.
* On a copy with only the 500 ppm drift, which must still PASS with every
  frame matched. The matcher refits offset and drift between passes, so long
  recordings with real crystal error are not reported as missing frames.
