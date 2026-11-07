/**
 * Native (host) tests for NEMO's raw CAN capture path.
 *
 * Builds against the SAME NMEA2000 library sources the firmware uses
 * (SRC/.pio/libdeps/teensy40/NMEA2000-library) and the same Raw_Capture core
 * (SRC/lib/Raw_Capture/RawCapture.*). A fake driver, tFakeBus, stands in for
 * tNMEA2000_TeensyxRawTap: its "ISR" puts frames into the library's own
 * tPriorityRingBuffer (as tNMEA2000_Teensyx::flexcan_interrupt does) and
 * drains them with nemo_cap::drainPriorityRing (as the firmware ISR hook
 * does); its CANGetFrame pops the capture FIFO, calls the raw tap and returns
 * the frame to the real tNMEA2000::ParseMessages().
 *
 * Usage:
 *   test_raw_capture                         run all unit/integration tests
 *   test_raw_capture replay IN OUT_RAW OUT_LEGACY [start_us32 [naive]]
 *                                            differential replay of a candump -t a log
 *                                            ("naive": control run tapping the unmodified
 *                                            driver's CANGetFrame, to measure reordering)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>

#include "NMEA2000.h"
#include "N2kMsg.h"
#include "RingBuffer.h"
#include "RawCapture.h"

using namespace nemo_cap;

// ---------------------------------------------------------------------------
// Host millis() for the NMEA2000 library (N2kTimer.cpp is built with the
// generic, non-Linux branch so the open/wait scheduler follows this clock).
static uint32_t gMillis = 1000;
extern "C" uint32_t millis() { return gMillis; }

// NMEA2000.cpp globals (not declared in the public header).
extern void CanIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn, unsigned char &src, unsigned char &dst);
extern unsigned long N2ktoCanID(unsigned char priority, unsigned long PGN, unsigned long Source, unsigned char Destination);

// ---------------------------------------------------------------------------
static int gChecks = 0, gFails = 0;
#define CHECK(cond) do { gChecks++; if (!(cond)) { gFails++; \
  fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_EQ_STR(a, b) do { gChecks++; std::string _a = (a), _b = (b); if (_a != _b) { gFails++; \
  fprintf(stderr, "FAIL %s:%d:\n  got:      [%s]\n  expected: [%s]\n", __FILE__, __LINE__, _a.c_str(), _b.c_str()); } } while (0)

static std::vector<std::string> gScenarioLog;  // human-readable evidence lines, with PASS/FAIL
static int gBlockStart = 0;                    // gFails at the start of the current test block
static void blockStart() { gBlockStart = gFails; }
static void logResult(const std::string &text) {
  gScenarioLog.push_back(std::string(gFails == gBlockStart ? "PASS " : "FAIL ") + text);
}

// ---------------------------------------------------------------------------
// Independent oracle for the candump -t a layout (printf based, deliberately
// not sharing code with formatCandumpLine). Layout from can-utils candump.c +
// lib.c snprintf_long_canframe(): " (%010llu.%06llu)  IFACE  %08X   [n] " + " XX"*n
static std::string oracleLine(uint64_t ts_us, const char *ifn, uint32_t id, int len, const uint8_t *d) {
  char b[128];
  int n = snprintf(b, sizeof(b), " (%010llu.%06llu)  %s  %08X   [%d] ",
                   (unsigned long long)(ts_us / 1000000ULL), (unsigned long long)(ts_us % 1000000ULL),
                   ifn, (unsigned)id, len);
  for (int i = 0; i < len; i++) n += snprintf(b + n, sizeof(b) - n, " %02X", d[i]);
  return std::string(b) + "\n";
}

// ---------------------------------------------------------------------------
// Verbatim copy of the PRE-FIX firmware output (SRC/src/main.cpp @ fddd41c,
// lines 383-455), with Serial redirected into a string, so tests can count
// exactly what the old output got wrong for the same input.
namespace legacy {
struct FakeSerial {
  std::string out;
  void print(const char *s) { out += s; }
  void print(uint32_t v, int base) { char b[16]; snprintf(b, sizeof(b), base == 16 ? "%X" : "%u", (unsigned)v); out += b; }
  void print(int v) { char b[16]; snprintf(b, sizeof(b), "%d", v); out += b; }
  void print(uint8_t v) { print((int)v); }
  void println() { out += "\r\n"; }  // Arduino Print::println() emits CRLF
};
static FakeSerial Serial;
static const int HEX = 16;
static uint8_t fastPacketSequence = 0;
static int min(int a, int b) { return a < b ? a : b; }
void PrintCanFrame(uint32_t canId, uint8_t len, const uint8_t *data);

// ---- begin verbatim (fddd41c SRC/src/main.cpp:383-455) ----
void PrintCandumpFormat(const tN2kMsg &N2kMsg) {
  // Reconstruct CAN ID: Priority (3 bits) | PGN (18 bits) | Source (8 bits)
  uint32_t canId = ((uint32_t)N2kMsg.Priority << 26) | ((uint32_t)N2kMsg.PGN << 8) | N2kMsg.Source;

  if (N2kMsg.DataLen <= 8) {
    // Single frame message - print directly
    PrintCanFrame(canId, N2kMsg.DataLen, N2kMsg.Data);
  } else {
    // Fast-packet message - re-fragment into 8-byte CAN frames
    uint8_t frame[8];
    int dataOffset = 0;
    int frameCount = 0;
    uint8_t seqId = (fastPacketSequence & 0x07) << 5;  // Sequence ID in upper 3 bits of counter byte

    while (dataOffset < N2kMsg.DataLen) {
      if (frameCount == 0) {
        // First frame: [SeqID + FrameNum] [TotalLen] [6 data bytes]
        frame[0] = seqId | (frameCount & 0x1F);
        frame[1] = N2kMsg.DataLen;
        int bytesToCopy = min(6, (int)N2kMsg.DataLen - dataOffset);
        for (int i = 0; i < bytesToCopy; i++) {
          frame[2 + i] = N2kMsg.Data[dataOffset + i];
        }
        // Pad remaining bytes with 0xFF
        for (int i = bytesToCopy; i < 6; i++) {
          frame[2 + i] = 0xFF;
        }
        dataOffset += bytesToCopy;
        PrintCanFrame(canId, 8, frame);
      } else {
        // Subsequent frames: [SeqID + FrameNum] [7 data bytes]
        frame[0] = seqId | (frameCount & 0x1F);
        int bytesToCopy = min(7, (int)N2kMsg.DataLen - dataOffset);
        for (int i = 0; i < bytesToCopy; i++) {
          frame[1 + i] = N2kMsg.Data[dataOffset + i];
        }
        // Pad remaining bytes with 0xFF
        for (int i = bytesToCopy; i < 7; i++) {
          frame[1 + i] = 0xFF;
        }
        dataOffset += bytesToCopy;
        PrintCanFrame(canId, 8, frame);
      }
      frameCount++;
    }

    fastPacketSequence++;
  }
}


void PrintCanFrame(uint32_t canId, uint8_t len, const uint8_t *data) {
  Serial.print("can1  ");
  if (canId < 0x10000000) Serial.print("0");
  if (canId < 0x1000000) Serial.print("0");
  if (canId < 0x100000) Serial.print("0");
  if (canId < 0x10000) Serial.print("0");
  if (canId < 0x1000) Serial.print("0");
  if (canId < 0x100) Serial.print("0");
  if (canId < 0x10) Serial.print("0");
  Serial.print(canId, HEX);

  Serial.print("   [");
  Serial.print(len);
  Serial.print("]  ");

  for (int i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    if (i < len - 1) Serial.print(" ");
  }
  Serial.println();
}
// ---- end verbatim ----
}  // namespace legacy

// ---------------------------------------------------------------------------
// Fake driver. Mirrors tNMEA2000_TeensyxRawTap: ISR -> priority ring -> drain
// -> capture FIFO -> CANGetFrame -> raw tap -> library.
struct FakeMsg {  // same members as tNMEA2000_Teensyx::CAN_message_t
  uint32_t id = 0;
  struct { bool extended = 0; bool remote = 0; bool overrun = 0; bool reserved = 0; } flags;
  uint8_t len = 8;
  uint8_t buf[8] = {0};
};

struct Assembled { uint8_t prio; uint32_t pgn; uint8_t src, dst; int len; std::vector<uint8_t> data; };

class tFakeBus : public tNMEA2000 {
public:
  tFakeBus(uint16_t fifoSize = 2048)
      : storage(fifoSize), pipe(&storage[0], fifoSize), ring(2048, 8), sends(0), naive(false) {}

  std::vector<RawFrame> storage;
  CapturePipeline pipe;
  tPriorityRingBuffer<FakeMsg> ring;   // the driver's rxRing
  std::vector<RawFrame> tapped;        // what the raw tap saw (arrival order)
  std::string rawOut;                  // raw capture text
  int sends;
  bool naive;  // true: emulate tapping CANGetFrame of the UNMODIFIED driver (no ISR drain)
  uint64_t naiveClock = 0;

  static FakeMsg mk(uint32_t id, uint8_t len, const uint8_t *d, bool ext = true, bool rtr = false, bool ovr = false) {
    FakeMsg m; m.id = id; m.len = len; m.flags.extended = ext; m.flags.remote = rtr; m.flags.overrun = ovr;
    for (int i = 0; i < 8; i++) m.buf[i] = i < len ? d[i] : 0;
    return m;
  }
  // One CAN interrupt delivering the given frames (normally exactly one).
  void isr(uint32_t micros32, const std::vector<FakeMsg> &frames) {
    for (const FakeMsg &f : frames) {                   // flexcan_interrupt(): mailbox -> rxRing
      FakeMsg *slot = ring.getAddRef((uint8_t)((f.id >> 26) & 0x7));
      if (slot) *slot = f;
    }
    if (naive) { naiveClock = micros32; return; }
    uint64_t ts = pipe.clock().extend(micros32);        // isrHook(): timestamp, drain
    drainPriorityRing(&ring, pipe, ts);
  }
  void isr1(uint32_t micros32, uint32_t id, std::vector<uint8_t> d) { isr(micros32, {mk(id, (uint8_t)d.size(), d.data())}); }

protected:
  bool CANSendFrame(unsigned long, unsigned char, const unsigned char *, bool) override { sends++; return true; }
  bool CANOpen() override { return true; }
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) override {
    RawFrame f;
    if (naive) {
      const FakeMsg *m = ring.getReadRef();  // what tNMEA2000_Teensyx::CANGetFrame does
      if (!m) return false;
      f.ts_us = naiveClock; f.id = m->id; f.len = m->len; f.flags = RF_EXTENDED; f.dropped_before = 0;
      memcpy(f.data, m->buf, 8);
    } else if (!pipe.pop(f)) {
      return false;
    }
    tapped.push_back(f);
    char line[96];
    if (f.dropped_before) {
      size_t n = formatGapLine(line, sizeof(line), f.ts_us, f.dropped_before, pipe.stats().sw_drop);
      rawOut.append(line, n);
    }
    size_t n = formatCandumpLine(line, sizeof(line), f.ts_us, kRawIfName, f.id, f.len, f.data, f.flags);
    rawOut.append(line, n);
    id = f.id; len = f.len; memcpy(buf, f.data, f.len);
    return true;
  }
};

static std::vector<Assembled> gHandled;
static std::string gReconOut;
static uint8_t gReconSeq = 0;
static uint64_t gReconTs = 0;
static void reconSink(void *, uint32_t id, uint8_t len, const uint8_t *d) {
  char line[96];
  size_t n = formatCandumpLine(line, sizeof(line), gReconTs, kReconstructedIfName, id, len, d, RF_EXTENDED);
  gReconOut.append(line, n);
}
static void handler(const tN2kMsg &m) {
  Assembled a; a.prio = m.Priority; a.pgn = m.PGN; a.src = m.Source; a.dst = m.Destination; a.len = m.DataLen;
  a.data.assign(m.Data, m.Data + m.DataLen);
  gHandled.push_back(a);
  legacy::PrintCandumpFormat(m);
  reconstructFrames(m.Priority, m.PGN, m.Source, m.Destination, m.Data, m.DataLen, gReconSeq, reconSink, 0);
}

// Open the fake bus exactly as main.cpp does for CAN2 and run it to os_Open.
static void openListenOnly(tFakeBus &bus) {
  bus.SetMsgHandler(handler);
  bus.SetMode(tNMEA2000::N2km_ListenOnly);
  bus.SetN2kCANReceiveFrameBufSize(2048);
  bus.Open();
  for (int i = 0; i < 50; i++) { gMillis += 10; bus.ParseMessages(); }
}
static void resetGlobals() {
  blockStart();
  gHandled.clear(); gReconOut.clear(); gReconSeq = 0; legacy::Serial.out.clear(); legacy::fastPacketSequence = 0;
}
static void pump(tFakeBus &bus, int n = 10) { for (int i = 0; i < n; i++) { gMillis += 1; bus.ParseMessages(); } }

static std::string oracleFrames(const std::vector<std::pair<uint64_t, FakeMsg>> &v) {
  std::string s;
  for (auto &p : v) s += oracleLine(p.first, "can2", p.second.id, p.second.len, p.second.buf);
  return s;
}
static int countLines(const std::string &s) { int n = 0; for (char c : s) n += c == '\n'; return n; }

// ===========================================================================
static void test_formatter() {
  blockStart();
  const uint8_t d[8] = {0xD8, 0x75, 0xF9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  char b[128];
  size_t n = formatCandumpLine(b, sizeof(b), 1769452128292872ULL, "can0", 0x09F11324, 8, d, RF_EXTENDED);
  // Line 1 of a 24-hour SocketCAN bench capture, byte for byte.
  CHECK_EQ_STR(std::string(b, n), " (1769452128.292872)  can0  09F11324   [8]  D8 75 F9 FF FF FF FF FF\n");
  const uint8_t r[3] = {0x00, 0xEE, 0x00};
  n = formatCandumpLine(b, sizeof(b), 1769452135024175ULL, "can0", 0x18EAB801, 3, r, RF_EXTENDED);
  CHECK_EQ_STR(std::string(b, n), " (1769452135.024175)  can0  18EAB801   [3]  00 EE 00\n");
  n = formatCandumpLine(b, sizeof(b), 12345678ULL, "can2", 0x00000001, 0, d, RF_EXTENDED);
  CHECK_EQ_STR(std::string(b, n), " (0000000012.345678)  can2  00000001   [0] \n");  // candump keeps the space after [0]
  n = formatCandumpLine(b, sizeof(b), 0, "can2", 0x123, 2, d, 0);
  CHECK_EQ_STR(std::string(b, n), " (0000000000.000000)  can2  123   [2]  D8 75\n");
  n = formatCandumpLine(b, sizeof(b), 0, "can2", 0x1FFFFFFF, 0, d, RF_EXTENDED | RF_REMOTE);
  CHECK_EQ_STR(std::string(b, n), " (0000000000.000000)  can2  1FFFFFFF   [0]  remote request\n");
  // Overflow protection: too small buffer -> 0, no overrun.
  char tiny[10];
  CHECK(formatCandumpLine(tiny, sizeof(tiny), 0, "can2", 1, 8, d, RF_EXTENDED) == 0);
  // Oracle agreement on random frames.
  uint32_t seed = 12345;
  int agree = 0;
  for (int k = 0; k < 20000; k++) {
    seed = seed * 1103515245u + 12345u; uint32_t id = seed & 0x1FFFFFFF;
    seed = seed * 1103515245u + 12345u; int len = seed % 9;
    uint8_t dd[8]; for (int i = 0; i < 8; i++) { seed = seed * 1103515245u + 12345u; dd[i] = seed >> 24; }
    seed = seed * 1103515245u + 12345u; uint64_t ts = ((uint64_t)seed << 20) ^ k;
    n = formatCandumpLine(b, sizeof(b), ts, "can2", id, (uint8_t)len, dd, RF_EXTENDED);
    if (std::string(b, n) == oracleLine(ts, "can2", id, len, dd)) agree++;
  }
  CHECK(agree == 20000);
  logResult("formatter: 5 literal lines byte-exact (2 copied from the real 24h capture); 20000/20000 random frames match independent printf oracle");
}

static void test_sync_banner_gap() {
  blockStart();
  CaptureStats s; memset(&s, 0, sizeof(s)); s.rx = 10; s.sw_drop = 2; s.hw_overrun = 1; s.isr_batches = 3; s.late = 4; s.fifo_hwm = 7; s.fifo_size = 2047; s.tx_lost = 5;
  char b[200];
  size_t n = formatSyncLine(b, sizeof(b), 5000001, MODE_RAW, "can2", s, 1);
  CHECK_EQ_STR(std::string(b, n), "# NEMO-SYNC v=1 mode=raw iface=can2 t=0000000005.000001 rx=10 drop=2 ovr=1 batch=3 late=4 hwm=7/2047 txlost=5 wraps=1\n");
  n = formatBannerLine(b, sizeof(b), MODE_RAW, "can2");
  CHECK_EQ_STR(std::string(b, n), "# NEMO-CAPTURE v=1 mode=raw fmt=candump-ta iface=can2 clock=micros64 origin=boot\n");
  n = formatBannerLine(b, sizeof(b), MODE_RECONSTRUCTED, "n2kreasm");
  CHECK_EQ_STR(std::string(b, n), "# NEMO-CAPTURE v=1 mode=RECONSTRUCTED fmt=candump-ta iface=n2kreasm clock=micros64 origin=boot WARNING=frames-reassembled-by-NEMO-not-raw\n");
  n = formatGapLine(b, sizeof(b), 1, 3, 9);
  CHECK_EQ_STR(std::string(b, n), "# NEMO-GAP t=0000000000.000001 dropped=3 total=9\n");
}

static void test_micros_extender() {
  blockStart();
  MicrosExtender c;
  CHECK(c.extend(0xFFFFFF00u) == 0xFFFFFF00ULL);
  CHECK(c.extend(0xFFFFFFFFu) == 0xFFFFFFFFULL);
  CHECK(c.extend(0x00000010u) == 0x100000010ULL);   // wrap
  CHECK(c.extend(0x00000010u) == 0x100000010ULL);   // equal value is not a wrap
  CHECK(c.extend(0x80000000u) == 0x180000000ULL);
  CHECK(c.extend(0x00000001u) == 0x200000001ULL);   // second wrap
  CHECK(c.wraps() == 2);
  // 10 simulated days, 1 Hz sync ticks only (idle bus): monotonic and exact.
  MicrosExtender d; uint64_t truth = 4000000000ULL; bool mono = true; uint64_t prev = 0; bool exact = true;
  for (int i = 0; i < 864000; i++) {
    uint64_t v = d.extend((uint32_t)truth);
    if (i && v <= prev) mono = false;
    exact = exact && (v == truth);
    prev = v; truth += 1000000;
  }
  CHECK(mono); CHECK(exact);
  logResult("micros extender: wrap at 2^32 and 2^33, 10 days of 1 Hz ticks monotonic and exact");
}

static void test_pipeline_overflow() {
  blockStart();
  RawFrame st[4];
  CapturePipeline p(st, 4);  // capacity 3
  uint8_t d[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  for (int i = 0; i < 6; i++) p.push(100 + i, 0x100 + i, 8, d, RF_EXTENDED);
  CHECK(p.stats().rx == 3); CHECK(p.stats().sw_drop == 3); CHECK(p.stats().fifo_hwm == 3);
  RawFrame f;
  CHECK(p.pop(f) && f.id == 0x100 && f.dropped_before == 0);
  CHECK(p.pop(f) && f.id == 0x101 && f.dropped_before == 0);
  p.push(200, 0x200, 8, d, RF_EXTENDED);  // first frame after the loss
  CHECK(p.pop(f) && f.id == 0x102 && f.dropped_before == 0);
  CHECK(p.pop(f) && f.id == 0x200 && f.dropped_before == 3 && f.ts_us == 200);
  CHECK(!p.pop(f));
  // Overrun flag counting
  p.push(300, 0x300, 1, d, RF_EXTENDED | RF_OVERRUN);
  CHECK(p.stats().hw_overrun == 1);
  logResult("FIFO overflow: capacity 3, 6 pushed -> 3 kept, 3 dropped, gap reported on the first frame after the loss (not before older frames)");
}

static void test_canid_rules() {
  blockStart();
  // n2kToCanId == library N2ktoCanID, and CanIdToN2k round-trips (dest preserved for PDU1).
  int checked = 0, ok = 0;
  const uint32_t pgns[] = {59392, 59904, 60160, 60416, 60928, 61184, 65280, 126208, 126464, 126720, 126992, 126993,
                           126996, 127245, 127250, 127488, 128267, 129025, 129026, 129029, 130306, 130312, 130816, 0x1EF00};
  for (uint32_t pgn : pgns)
    for (int prio = 0; prio < 8; prio++)
      for (int dst = 0; dst < 256; dst += 17)
        for (int src = 0; src < 256; src += 51) {
          checked++;
          uint32_t a = n2kToCanId(prio, pgn, src, dst);
          uint32_t b = (uint32_t)N2ktoCanID(prio, pgn, src, dst);
          unsigned char p2, s2, d2; unsigned long g2;
          CanIdToN2k(a, p2, g2, s2, d2);
          bool pdu1 = ((pgn >> 8) & 0xFF) < 240;
          if (a == b && p2 == prio && g2 == pgn && s2 == src && d2 == (pdu1 ? dst : 0xFF)) ok++;
        }
  CHECK(ok == checked);
  char ev[160]; snprintf(ev, sizeof(ev), "CAN-ID rule: %d/%d (prio,pgn,src,dst) combos equal library N2ktoCanID and round-trip through library CanIdToN2k", ok, checked);
  logResult(ev);
}

// --- Library-integrated scenarios -----------------------------------------
struct Scenario {
  const char *name;
  std::vector<std::pair<uint64_t, FakeMsg>> fed;
};

static void runScenario(const char *name, tFakeBus &bus, const std::vector<std::pair<uint32_t, FakeMsg>> &frames,
                        std::vector<std::pair<uint64_t, FakeMsg>> &fedOut, int pumpEvery = 1) {
  int k = 0;
  uint64_t base = 0; (void)base;
  for (auto &p : frames) {
    bus.isr(p.first, {p.second});
    fedOut.push_back({bus.pipe.clock().extend(p.first), p.second});
    if (++k % pumpEvery == 0) pump(bus, 1);
  }
  pump(bus, 20);
  (void)name;
}

static FakeMsg Frm(uint32_t id, std::vector<uint8_t> d) { return tFakeBus::mk(id, (uint8_t)d.size(), d.data()); }

static void test_scenarios() {
  // ---- S1: single frames incl. PDU1 addressed 59904 request ----
  {
    resetGlobals(); tFakeBus bus; openListenOnly(bus);
    std::vector<std::pair<uint32_t, FakeMsg>> in = {
      {1000000, Frm(0x09F11324, {0xD8, 0x75, 0xF9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF})},   // 127251 rate of turn
      {1000600, Frm(0x18EAB801, {0x00, 0xEE, 0x00})},                                  // 59904 ISO request src 01 -> dst B8
      {1001200, Frm(0x09F11224, {0x78, 0x38, 0x5B, 0x00, 0x00, 0x1C, 0x08, 0xFD})},   // 127250 heading
    };
    std::vector<std::pair<uint64_t, FakeMsg>> fed;
    runScenario("S1", bus, in, fed);
    CHECK_EQ_STR(bus.rawOut, oracleFrames(fed));
    CHECK_EQ_STR(bus.rawOut,
      " (0000000001.000000)  can2  09F11324   [8]  D8 75 F9 FF FF FF FF FF\n"
      " (0000000001.000600)  can2  18EAB801   [3]  00 EE 00\n"
      " (0000000001.001200)  can2  09F11224   [8]  78 38 5B 00 00 1C 08 FD\n");
    CHECK(bus.tapped.size() == 3);
    // The library DID decode the destination (it's in N2kMsg.Destination, PGN has PS cleared) ...
    bool sawReq = false;
    for (auto &a : gHandled) if (a.pgn == 59904) { sawReq = true; CHECK(a.dst == 0xB8); CHECK(a.src == 0x01); }
    CHECK(sawReq);
    // ... but the old firmware output rebuilt the ID from PGN only: destination lost.
    CHECK(legacy::Serial.out.find("18EA0001   [3]  00 EE 00") != std::string::npos);
    CHECK(legacy::Serial.out.find("18EAB801") == std::string::npos);
    // The fixed RECONSTRUCTED mode keeps it.
    CHECK(gReconOut.find("n2kreasm  18EAB801   [3]  00 EE 00") != std::string::npos);
    CHECK(bus.sends == 0);
    logResult("S1 single + PDU1 59904 (src 01 -> dst B8): raw byte-exact (3/3); legacy prints 18EA0001 (dst lost); reconstructed-mode prints 18EAB801");
  }
  // ---- S2: 8-byte fast packet (PGN 129029, DataLen 8 -> 2 frames on the wire) ----
  {
    resetGlobals(); tFakeBus bus; openListenOnly(bus);
    std::vector<std::pair<uint32_t, FakeMsg>> in = {
      {2000000, Frm(0x0DF80524, {0x40, 0x08, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66})},
      {2000500, Frm(0x0DF80524, {0x41, 0x77, 0x88, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF})},
    };
    std::vector<std::pair<uint64_t, FakeMsg>> fed;
    runScenario("S2", bus, in, fed);
    CHECK_EQ_STR(bus.rawOut, oracleFrames(fed));
    CHECK(countLines(bus.rawOut) == 2);
    CHECK(gHandled.size() == 1 && gHandled[0].len == 8);
    // Legacy: one 8-byte frame of assembled payload, not the 2 wire frames.
    CHECK_EQ_STR(legacy::Serial.out, "can1  0DF80524   [8]  11 22 33 44 55 66 77 88\r\n");
    logResult("S2 8-byte fast packet 129029: raw = 2 wire frames byte-exact; legacy = 1 frame of assembled payload (seq/len header bytes gone)");
  }
  // ---- S3: multi-frame fast packet with original seq ID, PDU1 126720 addressed ----
  {
    resetGlobals(); tFakeBus bus; openListenOnly(bus);
    // seq id 5 (0xA0): 126720 proprietary fast packet src 01 -> dst 24, 12 bytes
    std::vector<std::pair<uint32_t, FakeMsg>> in = {
      {3000000, Frm(0x1DEF2401, {0xA0, 0x0C, 0x2A, 0xE5, 0x98, 0x61, 0x06, 0x02})},
      {3000620, Frm(0x1DEF2401, {0xA1, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x03})},
    };
    std::vector<std::pair<uint64_t, FakeMsg>> fed;
    runScenario("S3", bus, in, fed);
    CHECK_EQ_STR(bus.rawOut, oracleFrames(fed));
    CHECK(gHandled.size() == 1 && gHandled[0].pgn == 126720 && gHandled[0].dst == 0x24 && gHandled[0].len == 12);
    // Legacy: dest lost (1DEF0001), seq id replaced by NEMO counter (0x00), tail padded 0xFF instead of wire bytes.
    CHECK_EQ_STR(legacy::Serial.out,
      "can1  1DEF0001   [8]  00 0C 2A E5 98 61 06 02\r\n"
      "can1  1DEF0001   [8]  01 02 FF FF FF FF 03 FF\r\n");   // wire padding byte 03 replaced by FF
    CHECK(gReconOut.find("n2kreasm  1DEF2401   [8]  00 0C 2A") != std::string::npos);
    logResult("S3 126720 fast packet seq=5 src 01 -> dst 24: raw byte-exact; legacy loses dst (1DEF0001), rewrites seq 5 -> 0 and replaces the wire padding byte");
  }
  // ---- S4: two interleaved fast packets (129029) from different sources ----
  {
    resetGlobals(); tFakeBus bus; openListenOnly(bus);
    std::vector<std::pair<uint32_t, FakeMsg>> in;
    uint32_t t = 4000000;
    // src 0x24 seq 1, src 0x01 seq 6, 43-byte payload = 7 frames each, alternating
    for (int fr = 0; fr < 7; fr++) {
      for (int s = 0; s < 2; s++) {
        uint8_t src = s ? 0x01 : 0x24, seq = s ? 6 : 1;
        std::vector<uint8_t> d(8, 0xFF);
        d[0] = (uint8_t)((seq << 5) | fr);
        if (fr == 0) { d[1] = 43; for (int i = 2; i < 8; i++) d[i] = (uint8_t)(src + i); }
        else for (int i = 1; i < 8; i++) d[i] = (uint8_t)(src + fr * 7 + i);
        in.push_back({t, Frm(0x0DF80500 | src, d)}); t += 530;
      }
    }
    std::vector<std::pair<uint64_t, FakeMsg>> fed;
    runScenario("S4", bus, in, fed, 3);
    CHECK_EQ_STR(bus.rawOut, oracleFrames(fed));
    CHECK(bus.tapped.size() == 14);
    CHECK(gHandled.size() == 2);
    // Legacy output: two messages printed back-to-back (de-interleaved), seq ids 0 and 1.
    CHECK(countLines(legacy::Serial.out) == 14);
    CHECK(legacy::Serial.out.substr(0, 39) == "can1  0DF80524   [8]  00 2B 26 27 28 29");
    logResult("S4 two interleaved 129029 fast packets (src 24 seq1, src 01 seq6): raw 14/14 frames in wire order; legacy de-interleaves and renumbers seq to 0/1");
  }
  // ---- S5: duplicate middle frame ----
  {
    resetGlobals(); tFakeBus bus; openListenOnly(bus);
    std::vector<std::pair<uint32_t, FakeMsg>> in = {
      {5000000, Frm(0x0DF80524, {0x60, 0x0A, 1, 2, 3, 4, 5, 6})},
      {5000500, Frm(0x0DF80524, {0x61, 7, 8, 9, 10, 0xFF, 0xFF, 0xFF})},
      {5001000, Frm(0x0DF80524, {0x61, 7, 8, 9, 10, 0xFF, 0xFF, 0xFF})},   // duplicate
    };
    std::vector<std::pair<uint64_t, FakeMsg>> fed;
    runScenario("S5", bus, in, fed);
    CHECK_EQ_STR(bus.rawOut, oracleFrames(fed));
    CHECK(bus.tapped.size() == 3);
    CHECK(gHandled.size() == 1);                    // library completes the message on frame 2, ignores the dup (orphan)
    CHECK(countLines(legacy::Serial.out) == 2);     // legacy: duplicate invisible
    logResult("S5 duplicate fast-packet frame: raw shows all 3 frames incl. duplicate; library/legacy show 1 message, duplicate invisible");
  }
  // ---- S6: missing middle frame ----
  {
    resetGlobals(); tFakeBus bus; openListenOnly(bus);
    std::vector<std::pair<uint32_t, FakeMsg>> in = {
      {6000000, Frm(0x0DF80524, {0x20, 20, 1, 2, 3, 4, 5, 6})},
      // frame 0x21 lost on the wire
      {6001000, Frm(0x0DF80524, {0x22, 14, 15, 16, 17, 18, 19, 20})},
    };
    std::vector<std::pair<uint64_t, FakeMsg>> fed;
    runScenario("S6", bus, in, fed);
    CHECK_EQ_STR(bus.rawOut, oracleFrames(fed));
    CHECK(bus.tapped.size() == 2);
    CHECK(gHandled.size() == 0);                    // library drops the broken message
    CHECK(legacy::Serial.out.empty());              // legacy: nothing at all
    logResult("S6 missing middle frame: raw shows the 2 frames that did arrive; library drops message; legacy prints nothing");
  }
  // ---- S7: timestamp wrap of micros() during traffic ----
  {
    resetGlobals(); tFakeBus bus; openListenOnly(bus);
    std::vector<std::pair<uint32_t, FakeMsg>> in = {
      {0xFFFFFE00u, Frm(0x09F11324, {1, 2, 3, 4, 5, 6, 7, 8})},
      {0xFFFFFFFFu, Frm(0x09F11324, {1, 2, 3, 4, 5, 6, 7, 9})},
      {0x00000137u, Frm(0x09F11324, {1, 2, 3, 4, 5, 6, 7, 10})},
    };
    std::vector<std::pair<uint64_t, FakeMsg>> fed;
    runScenario("S7", bus, in, fed);
    CHECK_EQ_STR(bus.rawOut,
      " (0000004294.966784)  can2  09F11324   [8]  01 02 03 04 05 06 07 08\n"
      " (0000004294.967295)  can2  09F11324   [8]  01 02 03 04 05 06 07 09\n"
      " (0000004294.967607)  can2  09F11324   [8]  01 02 03 04 05 06 07 0A\n");
    logResult("S7 micros() wrap between frames: timestamps 4294.966784 -> 4294.967295 -> 4294.967607 (monotonic, +311 us across the wrap)");
  }
  // ---- S8: frames arriving while the library is still in os_WaitOpen ----
  {
    resetGlobals(); tFakeBus bus;
    bus.SetMsgHandler(handler); bus.SetMode(tNMEA2000::N2km_ListenOnly); bus.SetN2kCANReceiveFrameBufSize(2048);
    bus.Open();                  // os_OpenCAN -> CANOpen -> os_WaitOpen (200 ms)
    gMillis += 1;
    bus.isr1(7000000, 0x09F11324, {1, 2, 3, 4, 5, 6, 7, 8});
    bus.ParseMessages();         // Open() still waiting: library reads and DISCARDS the frame
    for (int i = 0; i < 50; i++) { gMillis += 10; bus.ParseMessages(); }
    CHECK(bus.tapped.size() == 1);
    CHECK(gHandled.empty());
    logResult("S8 frame during the library's 200 ms open wait: raw tap prints it; library discards it (handler never called)");
  }
  // ---- S9: ISR batch (IRQ masked > 1 frame time): flagged, priority order ----
  {
    resetGlobals(); tFakeBus bus; openListenOnly(bus);
    // arrival order: prio 6, prio 3, prio 2 delivered in ONE interrupt
    bus.isr(8000000, {Frm(0x19F80101, {1}), Frm(0x0DF80102, {2}), Frm(0x09F80103, {3})});
    pump(bus, 5);
    CHECK(bus.tapped.size() == 3);
    CHECK(bus.pipe.stats().isr_batches == 1);
    bool allFlagged = true; for (auto &f : bus.tapped) allFlagged = allFlagged && (f.flags & RF_BATCHED);
    CHECK(allFlagged);
    CHECK(bus.tapped[0].id == 0x09F80103 && bus.tapped[2].id == 0x19F80101);  // documented limitation
    logResult("S9 3 frames in one interrupt: all flagged RF_BATCHED, counted (batch=1); intra-batch order is driver priority order (documented limitation)");
  }
  // ---- S10: why the tap cannot be a plain CANGetFrame override of the unmodified driver ----
  {
    resetGlobals(); tFakeBus bus; openListenOnly(bus); bus.naive = true;
    // 4 frames arrive (one per interrupt) while loop() is busy; loop then reads them.
    bus.isr(9000000, {Frm(0x19F80101, {1})});
    bus.isr(9000500, {Frm(0x0DF80102, {2})});
    bus.isr(9001000, {Frm(0x19F80103, {3})});
    bus.isr(9001500, {Frm(0x09F80104, {4})});
    pump(bus, 2);
    std::vector<uint32_t> got; for (auto &f : bus.tapped) got.push_back(f.id & 0xFF);
    CHECK(got.size() == 4);
    CHECK(got == (std::vector<uint32_t>{4, 2, 1, 3}));   // priority order, not arrival 1,2,3,4
    logResult("S10 control: tapping the UNMODIFIED driver's CANGetFrame returns arrival 1,2,3,4 as 4,2,1,3 (priority ring) -> ISR-level drain is required");
  }
  // ---- S11: capture FIFO overflow inside the full pipeline -> GAP marker ----
  {
    resetGlobals(); tFakeBus bus(8); openListenOnly(bus);   // capacity 7
    for (int i = 0; i < 10; i++) bus.isr1(10000000 + i * 500, 0x09F11300 | i, {(uint8_t)i});
    bus.isr1(10010000, 0x09F113AA, {0xAA});  // will be dropped too (still full)
    pump(bus, 5);
    bus.isr1(10020000, 0x09F113BB, {0xBB});
    pump(bus, 5);
    CHECK(bus.tapped.size() == 8);
    CHECK(bus.pipe.stats().sw_drop == 4);
    CHECK(bus.rawOut.find("# NEMO-GAP t=0000000010.020000 dropped=4 total=4\n (0000000010.020000)  can2  09F113BB") != std::string::npos);
    logResult("S11 FIFO overflow end-to-end: 4 frames dropped, '# NEMO-GAP dropped=4' emitted exactly before the first frame after the loss");
  }
}

// ===========================================================================
// Differential replay: feed a real candump -t a log through the fake driver +
// real library; write raw capture output and legacy output.
static int replay(const char *inPath, const char *outRaw, const char *outLegacy, uint32_t startUs32, bool naive) {
  FILE *fi = fopen(inPath, "r");
  if (!fi) { perror(inPath); return 2; }
  resetGlobals();
  tFakeBus bus; openListenOnly(bus);
  bus.naive = naive;  // control experiment: tap the unmodified driver's CANGetFrame instead
  char line[256];
  unsigned long long sec, usec;
  bool first = true;
  unsigned long long t0 = 0;
  uint64_t lastSyncMs = 0;
  int fed = 0, bad = 0;
  uint32_t rng = 7;
  int untilPump = 1;
  while (fgets(line, sizeof(line), fi)) {
    char ifn[32], idhex[16];
    int dlc, pos = 0;
    if (sscanf(line, " (%llu.%llu) %31s %15s [%d]%n", &sec, &usec, ifn, idhex, &dlc, &pos) != 5) { bad++; continue; }
    uint8_t d[8] = {0};
    const char *p = line + pos;
    for (int i = 0; i < dlc && i < 8; i++) { unsigned v; int adv; if (sscanf(p, " %2x%n", &v, &adv) != 1) break; d[i] = (uint8_t)v; p += adv; }
    unsigned long long tus = sec * 1000000ULL + usec;
    if (first) { t0 = tus; first = false; }
    uint32_t m32 = (uint32_t)(startUs32 + (tus - t0));
    bus.isr(m32, {tFakeBus::mk((uint32_t)strtoul(idhex, 0, 16), (uint8_t)dlc, d)});
    fed++;
    // Emulate loop(): ParseMessages at irregular intervals (1..25 frames).
    if (--untilPump == 0) {
      uint64_t nowMs = (tus - t0) / 1000;
      gMillis = 2000 + (uint32_t)nowMs;
      bus.ParseMessages();
      if (nowMs - lastSyncMs >= 1000) {
        char s[200]; uint64_t now64 = bus.pipe.clock().extend(m32);
        size_t n = formatSyncLine(s, sizeof(s), now64, MODE_RAW, kRawIfName, bus.pipe.stats(), bus.pipe.clock().wraps());
        bus.rawOut.append(s, n); lastSyncMs = nowMs;
      }
      rng = rng * 1103515245u + 12345u; untilPump = 1 + (rng >> 16) % 25;
    }
  }
  fclose(fi);
  for (int i = 0; i < 200; i++) { gMillis += 1; bus.ParseMessages(); }
  FILE *fo = fopen(outRaw, "w"); fputs(bus.rawOut.c_str(), fo); fclose(fo);
  fo = fopen(outLegacy, "w"); fputs(legacy::Serial.out.c_str(), fo); fclose(fo);
  printf("replay: fed=%d unparsed_input_lines=%d tapped=%zu raw_lines=%d handler_msgs=%zu legacy_lines=%d "
         "sw_drop=%u batches=%u fifo_hwm=%u wraps=%u can_sends=%d\n",
         fed, bad, bus.tapped.size(), countLines(bus.rawOut), gHandled.size(), countLines(legacy::Serial.out),
         bus.pipe.stats().sw_drop, bus.pipe.stats().isr_batches, bus.pipe.stats().fifo_hwm,
         bus.pipe.clock().wraps(), bus.sends);
  return (bus.tapped.size() == (size_t)fed && bus.pipe.stats().sw_drop == 0) ? 0 : 1;
}

int main(int argc, char **argv) {
  if (argc >= 5 && strcmp(argv[1], "replay") == 0) {
    uint32_t start = argc >= 6 ? (uint32_t)strtoul(argv[5], 0, 0) : 1000000u;
    bool naive = argc >= 7 && strcmp(argv[6], "naive") == 0;
    return replay(argv[2], argv[3], argv[4], start, naive);
  }
  test_formatter();
  test_sync_banner_gap();
  test_micros_extender();
  test_pipeline_overflow();
  test_canid_rules();
  test_scenarios();
  for (auto &s : gScenarioLog) printf("  %s\n", s.c_str());
  printf("%d checks, %d failures\n", gChecks, gFails);
  return gFails ? 1 : 0;
}
