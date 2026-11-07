/**
 * \file NMEA2000_TeensyxRawTap.cpp
 * \brief Raw-frame tap for tNMEA2000_Teensyx (Teensy 4.x). See header.
 */
#if defined(__IMXRT1062__)

#include "NMEA2000_TeensyxRawTap.h"

tNMEA2000_TeensyxRawTap *tNMEA2000_TeensyxRawTap::hookInstance[3] = {0, 0, 0};

//*****************************************************************************
tNMEA2000_TeensyxRawTap::tNMEA2000_TeensyxRawTap(tCANDevice _bus, nemo_cap::RawFrame *fifoStorage,
                                                 uint16_t fifoSize, tPins _txPin, tPins _rxPin)
    : tNMEA2000_Teensyx(_bus, _txPin, _rxPin), pipe(fifoStorage, fifoSize), rawTap(0), hookInstalled(false) {}

//*****************************************************************************
bool tNMEA2000_TeensyxRawTap::CANOpen() {
  // Unmodified driver open: begin() installs flexcan_isr_canN in the vector
  // table, sets bit rate and mailboxes, then enables mailbox interrupts.
  bool ret = tNMEA2000_Teensyx::CANOpen();
  if (busNumber < 1 || busNumber > 3) return ret;

  // Swap the vector for our wrapper. Frames that arrived between
  // enableMBInterrupts() and this swap are still in rxRing; the wrapper drains
  // them on its first call, or CANGetFrame()'s fallback returns them (RF_LATE).
  NVIC_DISABLE_IRQ(IrqMessage);
  hookInstance[busNumber - 1] = this;
  void (*hook)(void) = busNumber == 1 ? isrHookCan1 : (busNumber == 2 ? isrHookCan2 : isrHookCan3);
  attachInterruptVector((IRQ_NUMBER_t)IrqMessage, hook);
  hookInstalled = true;
  NVIC_ENABLE_IRQ(IrqMessage);
  return ret;
}

void tNMEA2000_TeensyxRawTap::isrHookCan1() { if (hookInstance[0]) hookInstance[0]->isrHook(); }
void tNMEA2000_TeensyxRawTap::isrHookCan2() { if (hookInstance[1]) hookInstance[1]->isrHook(); }
void tNMEA2000_TeensyxRawTap::isrHookCan3() { if (hookInstance[2]) hookInstance[2]->isrHook(); }

//*****************************************************************************
void tNMEA2000_TeensyxRawTap::isrHook() {
  // Timestamp first: this is the closest software point to frame arrival
  // (end-of-frame interrupt).
  uint64_t ts = pipe.clock().extend(micros());
  flexcan_interrupt();                         // unmodified driver ISR -> rxRing
  nemo_cap::drainPriorityRing(rxRing, pipe, ts);  // rxRing -> FIFO, arrival order
}

//*****************************************************************************
bool tNMEA2000_TeensyxRawTap::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  nemo_cap::RawFrame f;

  irqLock();
  bool ok = pipe.pop(f);
  irqRelease();

  if (!ok) {
    // Fallback: anything still in the driver ring (hook not yet installed).
    unsigned long fid;
    unsigned char flen = 0;
    unsigned char fbuf[8];
    if (!tNMEA2000_Teensyx::CANGetFrame(fid, flen, fbuf)) return false;
    irqLock();
    f.ts_us = pipe.clock().extend(micros());
    pipe.noteLate();
    irqRelease();
    f.id = (uint32_t)fid;
    f.len = flen > 8 ? 8 : flen;
    f.flags = nemo_cap::RF_EXTENDED | nemo_cap::RF_LATE;  // RX mailboxes are EXT-only (setMB(i,RX,EXT))
    for (int i = 0; i < 8; i++) f.data[i] = i < f.len ? fbuf[i] : 0;
    f.dropped_before = 0;
  }

  if (rawTap != 0) rawTap(f);

  id = f.id;
  len = f.len;
  memcpy(buf, f.data, f.len);
  return true;
}

//*****************************************************************************
uint64_t tNMEA2000_TeensyxRawTap::CaptureNowUs() {
  // Before CANOpen() IrqMessage is not yet set and no ISR can touch the clock.
  if (hookInstalled) irqLock();
  uint64_t t = pipe.clock().extend(micros());
  if (hookInstalled) irqRelease();
  return t;
}

//*****************************************************************************
void tNMEA2000_TeensyxRawTap::GetCaptureStats(nemo_cap::CaptureStats &out, uint32_t &wraps) {
  if (hookInstalled) irqLock();
  out = pipe.stats();
  wraps = pipe.clock().wraps();
  if (hookInstalled) irqRelease();
}

#endif  // __IMXRT1062__
