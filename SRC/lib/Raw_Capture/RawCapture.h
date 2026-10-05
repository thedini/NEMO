/**
 * \file RawCapture.h
 * \brief Hardware-independent core of NEMO's raw CAN capture output.
 *
 * NEMO's capture output must be usable as raw CAN ground truth: every frame
 * received on the listen bus, with its exact 29-bit identifier, DLC and data
 * bytes, in arrival order, with a monotonic timestamp. This file holds the
 * parts of that path that do not touch Teensy hardware so they can be unit
 * tested on a host (see SRC/test/native_raw_capture):
 *
 *  - MicrosExtender  : extends the 32-bit micros() counter to 64 bits
 *                      (micros() wraps every ~71.6 minutes).
 *  - CapturePipeline : single-producer (CAN ISR) / single-consumer (main loop)
 *                      FIFO of timestamped frames, with drop/overrun accounting.
 *  - drainPriorityRing : moves frames out of the driver's priority ring buffer
 *                      immediately inside the ISR, so arrival order is kept.
 *  - formatCandumpLine : `candump -t a` compatible text line.
 *  - formatSyncLine / formatBannerLine : '#'-prefixed metadata lines that
 *                      candump parsers ignore.
 *  - n2kToCanId / reconstructFrames : the legacy "reassembled message" output,
 *                      kept only for the explicitly selected RECONSTRUCTED mode.
 *
 * Nothing here allocates after construction or calls Arduino APIs.
 */
#ifndef NEMO_RAW_CAPTURE_H
#define NEMO_RAW_CAPTURE_H

#include <stdint.h>
#include <stddef.h>

namespace nemo_cap {

/** Version of the capture text format (reported in banner and sync lines). */
static const int kCaptureFormatVersion = 1;

/** Per-frame flags. */
enum {
  RF_EXTENDED = 0x01,  ///< 29-bit identifier (all NMEA2000 traffic).
  RF_REMOTE   = 0x02,  ///< Remote transmission request.
  RF_OVERRUN  = 0x04,  ///< Controller reported mailbox overrun before this frame (>=1 frame lost in hardware).
  RF_BATCHED  = 0x08,  ///< Drained together with other frames in one ISR call; order inside the batch
                       ///< is the driver's priority order and the timestamp is the ISR entry time.
  RF_LATE     = 0x10   ///< Not seen by the ISR hook (fallback path); timestamp is read time, not arrival.
};

/** One received CAN frame with its arrival timestamp. */
struct RawFrame {
  uint64_t ts_us;    ///< Monotonic microseconds since boot (64-bit, wrap-free).
  uint32_t id;       ///< CAN identifier exactly as received (29 or 11 bits).
  uint8_t  len;      ///< DLC (0..8).
  uint8_t  flags;    ///< RF_* flags.
  uint8_t  data[8];  ///< Data bytes; only the first len are meaningful.
  uint16_t dropped_before; ///< Frames lost (capture FIFO full) immediately before this one (saturates at 65535).
};

/**
 * \brief Extends a free-running 32-bit microsecond counter to 64 bits.
 *
 * Correct as long as it is called at least once per 2^32 us (~71.6 min).
 * NEMO calls it from the CAN ISR for every frame and from loop() on every
 * sync tick, so an idle bus cannot cause a missed wrap. Not reentrant: the
 * caller serialises access (the Teensy glue masks the CAN IRQ).
 */
class MicrosExtender {
public:
  MicrosExtender() : last_(0), high_(0), started_(false) {}
  uint64_t extend(uint32_t now) {
    if (started_ && now < last_) high_++;
    started_ = true;
    last_ = now;
    return ((uint64_t)high_ << 32) | now;
  }
  uint32_t wraps() const { return high_; }
private:
  uint32_t last_;
  uint32_t high_;
  bool started_;
};

/** Counters reported in sync lines. All are cumulative since boot. */
struct CaptureStats {
  uint32_t rx;          ///< Frames accepted into the capture FIFO.
  uint32_t sw_drop;     ///< Frames lost because the capture FIFO was full.
  uint32_t hw_overrun;  ///< Frames flagged by the controller as preceded by a mailbox overrun.
  uint32_t isr_batches; ///< ISR calls that yielded more than one frame.
  uint32_t late;        ///< Frames that bypassed the ISR hook (fallback path).
  uint16_t fifo_hwm;    ///< Highest FIFO occupancy seen.
  uint16_t fifo_size;   ///< FIFO capacity.
  uint32_t tx_lost;     ///< Output lines the serial port did not accept in full (filled in by the
                        ///< firmware, not the pipeline: USB host not reading).
};

/**
 * \brief SPSC FIFO between the CAN ISR (producer) and loop() (consumer).
 *
 * The storage is supplied by the caller (so firmware can place it where it
 * likes). The usable capacity is size-1 entries.
 */
class CapturePipeline {
public:
  CapturePipeline(RawFrame *storage, uint16_t size);

  /** Producer side. Returns false (and counts a drop) when the FIFO is full. */
  bool push(uint64_t ts_us, uint32_t id, uint8_t len, const uint8_t *data, uint8_t flags);

  /**
   * Consumer side. Returns false when empty. out.dropped_before tells the
   * consumer how many frames were lost (FIFO full) between the previous frame
   * and this one, so the gap can be marked at the right place in the stream.
   */
  bool pop(RawFrame &out);

  /** Number of frames currently queued. */
  uint16_t count() const;

  void noteIsrBatch() { stats_.isr_batches++; }
  void noteLate() { stats_.late++; }

  const CaptureStats &stats() const { return stats_; }
  MicrosExtender &clock() { return clock_; }

private:
  RawFrame *buf_;
  uint16_t size_;
  volatile uint16_t head_;
  volatile uint16_t tail_;
  uint32_t pendingDrops_;
  CaptureStats stats_;
  MicrosExtender clock_;
};

/**
 * \brief Drain every frame from the driver's receive ring into the pipeline.
 *
 * Called inside the CAN ISR immediately after the driver's own ISR has copied
 * the hardware mailboxes into its ring. Because the ring is drained on every
 * interrupt it never holds more than the frames of the current interrupt, so
 * the driver's priority-ordered read (lowest priority number first) cannot
 * reorder frames across interrupts. If one interrupt yields several frames
 * (CAN IRQ masked for longer than one frame time) they are flagged RF_BATCHED.
 *
 * Ring must provide `const Msg *getReadRef()`; Msg must have members
 * id, len, buf[8], flags.extended, flags.remote, flags.overrun
 * (tNMEA2000_Teensyx::CAN_message_t has exactly these).
 *
 * \return number of frames drained.
 */
template <class Ring>
int drainPriorityRing(Ring *ring, CapturePipeline &pipe, uint64_t ts_us) {
  static const int kBatchMax = 16;
  struct Tmp { uint32_t id; uint8_t len; uint8_t flags; uint8_t data[8]; };
  Tmp tmp[kBatchMax];
  int n = 0;
  int total = 0;
  bool batched = false;
  for (;;) {
    const auto *m = ring->getReadRef();
    if (m == 0 || n == kBatchMax) {
      // Flush what we have. A batch is only known to be a batch once a second
      // frame shows up, so frames are staged before being pushed.
      if (n > 1 || (m != 0 && n == kBatchMax)) batched = true;
      for (int i = 0; i < n; i++) {
        pipe.push(ts_us, tmp[i].id, tmp[i].len, tmp[i].data,
                  (uint8_t)(tmp[i].flags | (batched ? RF_BATCHED : 0)));
      }
      n = 0;
      if (m == 0) break;
    }
    Tmp &t = tmp[n++];
    t.id = m->id;
    t.len = m->len > 8 ? 8 : m->len;
    t.flags = (uint8_t)((m->flags.extended ? RF_EXTENDED : 0) |
                        (m->flags.remote ? RF_REMOTE : 0) |
                        (m->flags.overrun ? RF_OVERRUN : 0));
    for (int i = 0; i < 8; i++) t.data[i] = (i < t.len) ? m->buf[i] : 0;
    total++;
  }
  if (batched) pipe.noteIsrBatch();
  return total;
}

/** Interface names. Both match the `\w+` interface field of candump parsers. */
static const char kRawIfName[] = "can2";            ///< NEMO listens on Teensy CAN2.
static const char kReconstructedIfName[] = "n2kreasm"; ///< Legacy reassembled output; NOT raw frames.

/**
 * \brief Format one frame as a `candump -t a` line (including trailing '\n').
 *
 * Example:  " (0000000012.345678)  can2  09F11324   [8]  D8 75 F9 FF FF FF FF FF\n"
 * Extended IDs are printed as 8 hex digits, standard IDs as 3. A remote frame
 * prints "remote request" instead of data, as candump does. Layout follows
 * can-utils candump.c (" (%010llu.%06llu)  %s  ") + lib.c snprintf_long_canframe().
 *
 * \return number of characters written (excluding NUL), 0 if cap too small.
 */
size_t formatCandumpLine(char *out, size_t cap, uint64_t ts_us, const char *ifname,
                         uint32_t id, uint8_t len, const uint8_t *data, uint8_t flags);

/** Capture mode names used in banner/sync lines. */
enum CaptureMode { MODE_RAW = 0, MODE_RECONSTRUCTED = 1 };
const char *modeName(CaptureMode m);

/**
 * \brief '#'-prefixed periodic sync line for host time alignment.
 *
 * "# NEMO-SYNC v=1 mode=raw iface=can2 t=0000000012.345678 rx=.. drop=.. ovr=.. batch=.. late=.. hwm=../.. txlost=.. wraps=..\n"
 * The host records its own clock when the line arrives; t is NEMO's clock at
 * the moment the line was formatted (same clock as the frame timestamps).
 */
size_t formatSyncLine(char *out, size_t cap, uint64_t ts_us, CaptureMode mode,
                      const char *ifname, const CaptureStats &s, uint32_t wraps);

/** '#'-prefixed line marking frames lost at this point in the stream. */
size_t formatGapLine(char *out, size_t cap, uint64_t ts_us, uint32_t dropped, uint32_t totalDropped);

/** '#'-prefixed startup banner describing the format. */
size_t formatBannerLine(char *out, size_t cap, CaptureMode mode, const char *ifname);

/**
 * \brief Build a 29-bit CAN ID from NMEA2000 fields, preserving the PDU1 destination.
 *
 * Same rule as the NMEA2000 library's N2ktoCanID(): for PDU1 PGNs (PF < 240)
 * the PS byte carries the destination address. Returns 0 for an invalid PDU1
 * PGN (low byte non-zero), like the library.
 */
uint32_t n2kToCanId(uint8_t priority, uint32_t pgn, uint8_t source, uint8_t destination);

/** Sink for reconstructFrames(). */
typedef void (*FrameSink)(void *ctx, uint32_t id, uint8_t len, const uint8_t *data);

/**
 * \brief Legacy output: re-fragment an ASSEMBLED NMEA2000 message into CAN frames.
 *
 * This is NOT ground truth. It reproduces the pre-fix behaviour (messages with
 * DataLen > 8 are split as fast packets using NEMO's own sequence counter and
 * 0xFF padding; DataLen <= 8 is emitted as one frame) with one fix: the PDU1
 * destination is kept in the CAN ID. Original sequence IDs, frame interleaving,
 * duplicates and lost frames cannot be recovered from an assembled message.
 */
void reconstructFrames(uint8_t priority, uint32_t pgn, uint8_t source, uint8_t destination,
                       const uint8_t *data, int dataLen, uint8_t &seqCounter,
                       FrameSink sink, void *ctx);

}  // namespace nemo_cap

#endif
