/**
 * \file NMEA2000_TeensyxRawTap.h
 * \brief tNMEA2000_Teensyx with a raw-frame tap taken at the CAN interrupt.
 *
 * Why the tap lives at the interrupt and not in a message handler:
 *  - tNMEA2000::ParseMessages() (NMEA2000.cpp) calls CANGetFrame() and hands
 *    each frame to SetN2kCANBufMsg(), which assembles fast packets, discards
 *    out-of-sequence / orphan / duplicate frames and only then calls the
 *    message handler with an assembled tN2kMsg. Anything seen by the handler
 *    has already lost the original frames.
 *  - tNMEA2000_Teensyx keeps received frames in a tPriorityRingBuffer
 *    (rxRing). CANGetFrame() reads it with getReadRef(), which returns the
 *    lowest priority number first (RingBuffer.tpp). Whenever more than one
 *    frame is queued, CANGetFrame() therefore returns frames in priority order,
 *    not arrival order, and has no arrival timestamp.
 *
 * This class replaces the CAN interrupt vector after the driver has opened
 * the bus with a wrapper that (1) takes micros(), (2) runs the unmodified
 * driver ISR (tNMEA2000_Teensyx::flexcan_interrupt), (3) immediately drains
 * rxRing into a FIFO in arrival order with that timestamp. CANGetFrame() is
 * overridden to pop from that FIFO, call the raw tap (in loop() context, never
 * in the ISR) and return the identical frame to the library, so NMEA2000
 * message handling is unchanged.
 *
 * Transmit paths, bus bit rate and mailbox setup are inherited unchanged.
 */
#ifndef NMEA2000_TEENSYX_RAW_TAP_H
#define NMEA2000_TEENSYX_RAW_TAP_H

#if defined(__IMXRT1062__)

#include <Arduino.h>
#include <NMEA2000_Teensyx.h>
#include "RawCapture.h"

class tNMEA2000_TeensyxRawTap : public tNMEA2000_Teensyx {
public:
  /**
   * Called from CANGetFrame() (loop() context) for every received frame, in
   * arrival order, before the NMEA2000 library sees it.
   * frame.dropped_before = frames lost (capture FIFO full) immediately before this one.
   */
  typedef void (*tRawFrameTap)(const nemo_cap::RawFrame &frame);

  tNMEA2000_TeensyxRawTap(tCANDevice _bus, nemo_cap::RawFrame *fifoStorage, uint16_t fifoSize,
                          tPins _txPin = NMEA2000_TEENSYX_TX_PIN, tPins _rxPin = NMEA2000_TEENSYX_RX_PIN);

  void SetRawFrameTap(tRawFrameTap tap) { rawTap = tap; }

  /** Current capture clock (same clock as frame timestamps). Also keeps the 64-bit extension fresh. */
  uint64_t CaptureNowUs();

  /** Consistent copy of the capture counters. */
  void GetCaptureStats(nemo_cap::CaptureStats &out, uint32_t &wraps);

  /** True once the ISR hook has been installed (after the bus opened). */
  bool IsRawHookInstalled() const { return hookInstalled; }

protected:
  bool CANOpen() override;
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) override;

private:
  void isrHook();
  static void isrHookCan1();
  static void isrHookCan2();
  static void isrHookCan3();
  static tNMEA2000_TeensyxRawTap *hookInstance[3];

  nemo_cap::CapturePipeline pipe;
  tRawFrameTap rawTap;
  volatile bool hookInstalled;
};

#endif  // __IMXRT1062__
#endif
