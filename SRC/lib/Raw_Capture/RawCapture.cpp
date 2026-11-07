/**
 * \file RawCapture.cpp
 * \brief Hardware-independent core of NEMO's raw CAN capture output.
 */
#include "RawCapture.h"

#include <stdio.h>
#include <string.h>

namespace nemo_cap {

//*****************************************************************************
CapturePipeline::CapturePipeline(RawFrame *storage, uint16_t size)
    : buf_(storage), size_(size), head_(0), tail_(0), pendingDrops_(0) {
  memset(&stats_, 0, sizeof(stats_));
  stats_.fifo_size = (uint16_t)(size > 0 ? size - 1 : 0);
}

//*****************************************************************************
bool CapturePipeline::push(uint64_t ts_us, uint32_t id, uint8_t len, const uint8_t *data, uint8_t flags) {
  if (buf_ == 0 || size_ < 2) { stats_.sw_drop++; pendingDrops_++; return false; }
  uint16_t h = head_;
  uint16_t next = (uint16_t)((h + 1) % size_);
  if (next == tail_) {
    stats_.sw_drop++;
    pendingDrops_++;
    return false;
  }
  RawFrame &f = buf_[h];
  f.ts_us = ts_us;
  f.id = id;
  f.len = len > 8 ? 8 : len;
  f.flags = flags;
  for (int i = 0; i < 8; i++) f.data[i] = (i < f.len && data != 0) ? data[i] : 0;
  f.dropped_before = (uint16_t)(pendingDrops_ > 0xFFFF ? 0xFFFF : pendingDrops_);
  pendingDrops_ = 0;
  if (flags & RF_OVERRUN) stats_.hw_overrun++;
  head_ = next;
  stats_.rx++;
  uint16_t c = count();
  if (c > stats_.fifo_hwm) stats_.fifo_hwm = c;
  return true;
}

//*****************************************************************************
bool CapturePipeline::pop(RawFrame &out) {
  uint16_t t = tail_;
  if (t == head_) return false;
  out = buf_[t];
  tail_ = (uint16_t)((t + 1) % size_);
  return true;
}

//*****************************************************************************
uint16_t CapturePipeline::count() const {
  uint16_t h = head_, t = tail_;
  return (uint16_t)((h + size_ - t) % size_);
}

//*****************************************************************************
// Small, allocation-free formatting helpers (avoid printf %llu, which newlib-nano lacks).
namespace {

static const char kHex[] = "0123456789ABCDEF";

struct Out {
  char *p;
  size_t cap;
  size_t n;
  bool ok;
  Out(char *b, size_t c) : p(b), cap(c), n(0), ok(c > 0) { if (ok) p[0] = 0; }
  void ch(char c) {
    if (!ok) return;
    if (n + 1 >= cap) { ok = false; return; }
    p[n++] = c;
    p[n] = 0;
  }
  void str(const char *s) { while (*s) ch(*s++); }
  void hex(uint32_t v, int digits) {
    for (int i = digits - 1; i >= 0; i--) ch(kHex[(v >> (4 * i)) & 0xF]);
  }
  void dec(uint64_t v, int minDigits) {
    char tmp[20];
    int k = 0;
    do { tmp[k++] = (char)('0' + v % 10); v /= 10; } while (v && k < 20);
    for (int i = k; i < minDigits; i++) ch('0');
    while (k) ch(tmp[--k]);
  }
  void ts(uint64_t us) {
    dec(us / 1000000ULL, 10);
    ch('.');
    dec(us % 1000000ULL, 6);
  }
  size_t done() { return ok ? n : 0; }
};

}  // namespace

//*****************************************************************************
size_t formatCandumpLine(char *out, size_t cap, uint64_t ts_us, const char *ifname,
                         uint32_t id, uint8_t len, const uint8_t *data, uint8_t flags) {
  // Byte-for-byte the layout of `candump -t a`, e.g.
  //  " (1769452128.292872)  can0  09F11324   [8]  D8 75 F9 FF FF FF FF FF"
  Out o(out, cap);
  o.str(" (");
  o.ts(ts_us);
  o.str(")  ");
  o.str(ifname);
  o.str("  ");
  if (flags & RF_EXTENDED) o.hex(id & 0x1FFFFFFF, 8);
  else o.hex(id & 0x7FF, 3);
  o.str("   [");
  if (len > 8) len = 8;
  o.dec(len, 1);
  // can-utils lib.c snprintf_long_canframe(): "[n]" is followed by one space,
  // then every data byte is written as " XX" (so DLC 0 leaves a trailing space).
  o.str("] ");
  if (flags & RF_REMOTE) {
    o.str(" remote request");
  } else {
    for (int i = 0; i < len; i++) {
      o.ch(' ');
      o.hex(data[i], 2);
    }
  }
  o.ch('\n');
  return o.done();
}

//*****************************************************************************
const char *modeName(CaptureMode m) {
  return m == MODE_RECONSTRUCTED ? "RECONSTRUCTED" : "raw";
}

//*****************************************************************************
size_t formatSyncLine(char *out, size_t cap, uint64_t ts_us, CaptureMode mode,
                      const char *ifname, const CaptureStats &s, uint32_t wraps) {
  Out o(out, cap);
  o.str("# NEMO-SYNC v="); o.dec(kCaptureFormatVersion, 1);
  o.str(" mode="); o.str(modeName(mode));
  o.str(" iface="); o.str(ifname);
  o.str(" t="); o.ts(ts_us);
  o.str(" rx="); o.dec(s.rx, 1);
  o.str(" drop="); o.dec(s.sw_drop, 1);
  o.str(" ovr="); o.dec(s.hw_overrun, 1);
  o.str(" batch="); o.dec(s.isr_batches, 1);
  o.str(" late="); o.dec(s.late, 1);
  o.str(" hwm="); o.dec(s.fifo_hwm, 1); o.ch('/'); o.dec(s.fifo_size, 1);
  o.str(" txlost="); o.dec(s.tx_lost, 1);
  o.str(" wraps="); o.dec(wraps, 1);
  o.ch('\n');
  return o.done();
}

//*****************************************************************************
size_t formatGapLine(char *out, size_t cap, uint64_t ts_us, uint32_t dropped, uint32_t totalDropped) {
  Out o(out, cap);
  o.str("# NEMO-GAP t="); o.ts(ts_us);
  o.str(" dropped="); o.dec(dropped, 1);
  o.str(" total="); o.dec(totalDropped, 1);
  o.ch('\n');
  return o.done();
}

//*****************************************************************************
size_t formatBannerLine(char *out, size_t cap, CaptureMode mode, const char *ifname) {
  Out o(out, cap);
  o.str("# NEMO-CAPTURE v="); o.dec(kCaptureFormatVersion, 1);
  o.str(" mode="); o.str(modeName(mode));
  o.str(" fmt=candump-ta iface="); o.str(ifname);
  o.str(" clock=micros64 origin=boot");
  if (mode == MODE_RECONSTRUCTED) {
    o.str(" WARNING=frames-reassembled-by-NEMO-not-raw");
  }
  o.ch('\n');
  return o.done();
}

//*****************************************************************************
uint32_t n2kToCanId(uint8_t priority, uint32_t pgn, uint8_t source, uint8_t destination) {
  uint8_t pf = (uint8_t)(pgn >> 8);
  if (pf < 240) {  // PDU1: PS carries the destination address
    if ((pgn & 0xFF) != 0) return 0;
    return ((uint32_t)(priority & 0x7) << 26) | (pgn << 8) | ((uint32_t)destination << 8) | source;
  }
  return ((uint32_t)(priority & 0x7) << 26) | (pgn << 8) | source;  // PDU2
}

//*****************************************************************************
void reconstructFrames(uint8_t priority, uint32_t pgn, uint8_t source, uint8_t destination,
                       const uint8_t *data, int dataLen, uint8_t &seqCounter,
                       FrameSink sink, void *ctx) {
  uint32_t canId = n2kToCanId(priority, pgn, source, destination);
  if (canId == 0) {
    // Invalid PDU1 PGN; fall back to the legacy layout rather than drop it.
    canId = ((uint32_t)(priority & 0x7) << 26) | (pgn << 8) | source;
  }

  if (dataLen <= 8) {
    sink(ctx, canId, (uint8_t)(dataLen < 0 ? 0 : dataLen), data);
    return;
  }

  // Legacy fast-packet re-fragmentation (unchanged from the pre-fix firmware).
  uint8_t frame[8];
  int dataOffset = 0;
  int frameCount = 0;
  uint8_t seqId = (uint8_t)((seqCounter & 0x07) << 5);
  while (dataOffset < dataLen) {
    if (frameCount == 0) {
      frame[0] = (uint8_t)(seqId | (frameCount & 0x1F));
      frame[1] = (uint8_t)dataLen;
      int n = dataLen - dataOffset < 6 ? dataLen - dataOffset : 6;
      for (int i = 0; i < 6; i++) frame[2 + i] = i < n ? data[dataOffset + i] : 0xFF;
      dataOffset += n;
    } else {
      frame[0] = (uint8_t)(seqId | (frameCount & 0x1F));
      int n = dataLen - dataOffset < 7 ? dataLen - dataOffset : 7;
      for (int i = 0; i < 7; i++) frame[1 + i] = i < n ? data[dataOffset + i] : 0xFF;
      dataOffset += n;
    }
    sink(ctx, canId, 8, frame);
    frameCount++;
  }
  seqCounter++;
}

}  // namespace nemo_cap
