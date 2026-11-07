/**
 * \file main.cpp
 * \brief Main entry point for NEMO (NMEA Educational Maritime Observatory) firmware.
 *
 *
 * The device uses two CAN interfaces:
 * - CAN1: Transmits simulated sensor data as multiple NMEA2000 devices
 * - CAN2: Listens to NMEA2000 traffic for monitoring and analysis
 *
 * \author Soups71
 
 */

#include<Arduino.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA2000_Teensyx.h>
#include "constants.h"
#include <PGN_Helpers.h>
#include <U8x8lib.h>
#include <U8g2lib.h>
#include <Menu.h>
#include <N2K_Monitor.h>
#include <Attack_Controller.h>
#include <Menu_Controller.h>
#include <Splash_Screen.h>
#include <Sensor.h>
#include <Serial_Controller.h>
#include <RawCapture.h>
#include <NMEA2000_TeensyxRawTap.h>



//Primary CAN interface for transmitting simulated sensor data
tNMEA2000_Teensyx NMEA2000_CAN1(tNMEA2000_Teensyx::CAN1);

// Capture FIFO between the CAN2 interrupt and loop() (see NMEA2000_TeensyxRawTap.h).
// 48 KB in RAM2 (DMAMEM) so RAM1 stays available for stack/variables; only the
// CPU touches it, and it needs no zero-initialisation.
DMAMEM static nemo_cap::RawFrame captureFifo[NEMO_CAPTURE_FIFO_SIZE];

// Secondary CAN interface for listening to NMEA2000 traffic.
// Same driver as before plus a raw-frame tap at the CAN interrupt, so the
// capture output sees every frame before fast-packet assembly.
tNMEA2000_TeensyxRawTap NMEA2000_CAN2(tNMEA2000_Teensyx::CAN2, captureFifo, NEMO_CAPTURE_FIFO_SIZE);



//Interval in milliseconds between sensor updates.
#define UPDATE_INTERVAL 1000

//NMEA2000 manufacturer code for simulated devices.
#define DEVICE_MANUFACTURER_CODE 2046

//NMEA2000 device class for simulated devices.
#define DEVICE_CLASS 25

//Number of simulated devices on the NMEA2000 bus.
#define NUM_DEVICES 3

#if NEMO_CAPTURE_MODE == NEMO_CAPTURE_RECONSTRUCTED
//Sequence counter used only by the legacy RECONSTRUCTED capture output.
static uint8_t reconstructSequence = 0;
#endif

//Timestamp (millis) of the last "# NEMO-SYNC" line.
static unsigned long lastCaptureSyncTime = 0;

//Capture lines the USB serial port did not take in full. Teensy's
//usb_serial_write() gives up after 120 ms when the host stops reading
//(TX_TIMEOUT_MSEC) and then drops output; this makes that loss visible.
static uint32_t captureTxLost = 0;

//Write one capture line to Serial, counting incomplete writes.
static void CaptureWrite(const char *line, size_t n) {
  if (n == 0) return;
  if (Serial.write((const uint8_t *)line, n) != n) captureTxLost++;
}


//Timestamps of last button press for each button.
unsigned long lastButtonPress[4] = {0, 0, 0, 0};

//Debounce delay in milliseconds.
unsigned long debounceDelay = 250;

//Timestamp of last sensor update.
unsigned long lastUpdateTime = 0;

//Text-mode display driver for menu system.
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

// Graphics-mode display driver for splash screen.
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//Simulated Engine RPM sensor (Device 0).
Sensor sensor1(SENSOR_PIN_1, MSG_ENGINE_RPM, &NMEA2000_CAN1, 0);

//Simulated Water Depth sensor (Device 1).
Sensor sensor2(SENSOR_PIN_2, MSG_WATER_DEPTH, &NMEA2000_CAN1, 1);

//Simulated Heading sensor (Device 2).
Sensor sensor3(SENSOR_PIN_3, MSG_HEADING, &NMEA2000_CAN1, 2);


//NMEA2000 network monitor instance
N2K_Monitor* n2kMonitor;

//Attack demonstration controller instance.
Attack_Controller* attackController;

//Menu system controller instance.
Menu_Controller* menuController;

//Serial command controller for GUI communication.
Serial_Controller* serialController;


/**
 * \brief Handles incoming NMEA2000 messages from CAN2 interface.
 * \param N2kMsg Reference to the received NMEA2000 message.
 */
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);


/**
 * \brief Configures the NMEA2000 CAN1 interface and simulated devices.
 */
void setupNMEA2000();


/**
 * \brief True when capture lines may be written to Serial
 *        (not in DEBUG builds, not while the GUI JSON stream owns the port).
 */
bool CaptureOutputEnabled();

/**
 * \brief Raw-frame tap: prints one received CAN2 frame in `candump -t a` format.
 *
 * Called by NMEA2000_CAN2 for every frame, in arrival order, with the
 * timestamp taken in the CAN interrupt, before the NMEA2000 library assembles it.
 * \param frame The received frame (frame.dropped_before = frames lost just before it).
 */
void OnRawCanFrame(const nemo_cap::RawFrame &frame);

/**
 * \brief Writes the capture banner and periodic "# NEMO-SYNC" lines.
 * \param force Write a sync line now regardless of the interval.
 */
void CaptureSyncTick(bool force);

#if NEMO_CAPTURE_MODE == NEMO_CAPTURE_RECONSTRUCTED
/**
 * \brief LEGACY: prints an ASSEMBLED NMEA2000 message re-fragmented into frames.
 *
 * Output is labelled interface "n2kreasm" and is NOT raw ground truth; see
 * nemo_cap::reconstructFrames(). Only built with NEMO_CAPTURE_MODE=NEMO_CAPTURE_RECONSTRUCTED.
 * \param N2kMsg Reference to the NMEA2000 message to print.
 */
void PrintReconstructedFormat(const tN2kMsg &N2kMsg);
#endif

/**
 * \brief Maps a button pin to its debounce array index.
 * \param button The button pin number (BUTTON_UP, BUTTON_DOWN, BUTTON_LEFT, or BUTTON_RIGHT).
 * \return The array index (0-3) corresponding to the button.
 */
int getButtonIndex(int button);

/**
 * \brief Checks if a button has been pressed with debounce filtering.
 * \param button The button pin number to check.
 * \return 1 if button is pressed and debounce time has elapsed, 0 otherwise.
 */
short buttonPressed(int button);



/**
 * \brief Arduino setup function - initializes all hardware and software components.
 *
 * This function is called once at startup and performs complete system
 * initialization:
 *
 * Hardware initialization:
 * - Serial communication at 115200 baud
 * - Sensor input pins (analog)
 * - Button input pins with internal pull-up resistors
 *
 * NMEA2000 initialization:
 * - CAN1 interface for sensor transmission (via setupNMEA2000())
 * - CAN2 interface for listening/monitoring
 *
 * Display initialization:
 * - Splash screen animation on startup
 * - OLED display for menu system
 *
 * Controller initialization:
 * - N2K_Monitor for NMEA2000 traffic analysis
 * - Attack_Controller for educational demonstrations
 * - Menu_Controller for user interface
 */
void setup(void)
{
  Serial.begin(115200);

  pinMode(SENSOR_PIN_1, INPUT);
  pinMode(SENSOR_PIN_2, INPUT);
  pinMode(SENSOR_PIN_3, INPUT);

  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);

  setupNMEA2000();
  NMEA2000_CAN2.SetMsgHandler(HandleNMEA2000Msg);
#if NEMO_CAPTURE_MODE == NEMO_CAPTURE_RAW
  NMEA2000_CAN2.SetRawFrameTap(OnRawCanFrame);
#endif
  NMEA2000_CAN2.SetMode(tNMEA2000::N2km_ListenOnly);
  NMEA2000_CAN2.SetN2kCANReceiveFrameBufSize(2048);

  // Open both CAN interfaces
  NMEA2000_CAN2.Open();

  // Show splash screen animation
  Splash_Screen::show(&u8g2);

  // Initialize display for menu system
  u8x8.begin();
  u8x8.setPowerSave(0);

  // Initialize NMEA2000 Monitor
  n2kMonitor = new N2K_Monitor();

  // Initialize Attack Controller
  attackController = new Attack_Controller(&NMEA2000_CAN1, n2kMonitor, &sensor1);

  // Initialize Menu Controller
  menuController = new Menu_Controller(&u8x8,
                                      BUTTON_UP, BUTTON_DOWN,
                                      BUTTON_LEFT, BUTTON_RIGHT,
                                      &sensor1, &sensor2, &sensor3,
                                      n2kMonitor, attackController);
  menuController->begin();

  // Initialize Serial Controller for GUI communication
  serialController = new Serial_Controller(&sensor1, &sensor2, &sensor3,
                                           attackController, n2kMonitor,
                                           &NMEA2000_CAN1);
  serialController->begin();

#if DEBUG
  Serial.println("System initialized");
#endif

  // Capture banner + first sync line (repeated every NEMO_CAPTURE_SYNC_INTERVAL_MS).
  CaptureSyncTick(true);
}

/**
 * \brief Arduino main loop - executes continuously after setup().
 *
 * This function implements the main program loop with the following
 * responsibilities:
 *
 * Sensor updates (every UPDATE_INTERVAL milliseconds):
 * - Normal operation: Update and transmit all sensor values
 * - Own-sensor impersonation: Continue normal transmissions alongside attack
 * - External attack: Only update sensor1 for potentiometer control
 *
 * CAN bus processing:
 * - Parse CAN1 messages (skipped during active attacks)
 * - Parse CAN2 messages for monitoring
 *
 * User interface:
 * - Update menu controller for real-time displays
 * - Process button inputs for navigation
 *
 * Button actions:
 * - UP: Navigate up in menu
 * - DOWN: Navigate down in menu
 * - LEFT: Navigate back/cancel
 * - RIGHT: Select/enter
 */

void loop(void)
{
  // Process serial commands from GUI
  serialController->update();

  unsigned long currentTime = millis();

  // Update sensors at regular intervals
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = currentTime;

    bool attackActive = attackController->isAttackActive();
    bool impersonatingOwn = attackController->isImpersonatingOwnSensor();

    if (!attackActive) {
      // Normal operation - update and send all sensors
      sensor1.update();
      sensor2.update();
      sensor3.update();

      // Send NMEA messages
      sensor1.sendMessage();
      sensor2.sendMessage();
      sensor3.sendMessage();

    } else if (impersonatingOwn) {
      // Impersonating own sensor - continue real transmissions for all sensors
      // This creates both real data (from sensors) and spoofed data (from attack)
      // User can see effect in Live Data with interleaved real/spoofed values
      sensor1.update();
      sensor2.update();
      sensor3.update();

      // Send messages from all sensors so Live Data shows real values
      sensor1.sendMessage();
      sensor2.sendMessage();
      sensor3.sendMessage();

    } else {
      // External attack (not impersonating own sensor) - only update sensor1 for potentiometer control
      sensor1.update();
    }
  }

  // Keep parsing both buses
  // Skip CAN1 parsing during attacks to prevent library from maintaining attack state
  if (!menuController->isAttackActive()) {
    NMEA2000_CAN1.ParseMessages();
  }
  NMEA2000_CAN2.ParseMessages();

  // Periodic capture sync line; also keeps the 64-bit capture clock extended.
  CaptureSyncTick(false);

  // Update menu controller (for real-time displays)
  menuController->update();

  // Handle button inputs
  if(buttonPressed(BUTTON_UP)){
    menuController->navigateUp();
  } else if(buttonPressed(BUTTON_DOWN)){
    menuController->navigateDown();
  } else if(buttonPressed(BUTTON_LEFT)){
    menuController->navigateBack();
  } else if(buttonPressed(BUTTON_RIGHT)){
    menuController->navigateSelect();
  }
}

void setupNMEA2000() {
  // Configure multi-device mode - each sensor appears as its own device on the bus
  NMEA2000_CAN1.SetDeviceCount(NUM_DEVICES);

  NMEA2000_CAN1.SetMode(tNMEA2000::N2km_NodeOnly, 22);

  // Set product info for all sensors BEFORE Open()
  // Use updateDeviceInfo() directly to avoid SendIsoAddressClaim before bus is open
  sensor1.updateDeviceInfo();
  sensor2.updateDeviceInfo();
  sensor3.updateDeviceInfo();

  // Now open - this will initialize the CAN bus
  NMEA2000_CAN1.Open();

  // Apply initial active/inactive state for each sensor
  // Sensors default to inactive, so set them to null address and disable heartbeat
  Sensor* sensors[] = {&sensor1, &sensor2, &sensor3};
  for (int i = 0; i < NUM_DEVICES; i++) {
    if (sensors[i]->isActive()) {
      // Active sensor - set custom name and broadcast info
      char name[20];
      snprintf(name, sizeof(name), "Sensor %d", i + 1);
      sensors[i]->setCustomName(name);
      delay(10);
      NMEA2000_CAN1.SendProductInformation(i);
      delay(10);
    } else {
      // Inactive sensor - set to null address (254) and disable heartbeat
      NMEA2000_CAN1.SetHeartbeatIntervalAndOffset(0, 0, i);
      NMEA2000_CAN1.SetN2kSource(254, i);  // Null address - removes from bus
    }
  }
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  // Raw capture output is produced per frame by OnRawCanFrame(), before the
  // library assembled this message. Only the legacy mode prints from here.
#if NEMO_CAPTURE_MODE == NEMO_CAPTURE_RECONSTRUCTED
  if (CaptureOutputEnabled()) {
    PrintReconstructedFormat(N2kMsg);
  }
#endif

  // Forward to Serial_Controller for GUI streaming (JSON format)
  if (serialController != nullptr && serialController->isStreamEnabled()) {
    uint32_t canId = ((uint32_t)N2kMsg.Priority << 26) |
                     ((uint32_t)N2kMsg.PGN << 8) | N2kMsg.Source;
    serialController->streamCanFrame(canId, N2kMsg.DataLen, N2kMsg.Data);
  }

  if(attackController != nullptr && attackController -> isSpamActive()) {
    attackController->attackHandler(N2kMsg);
  }
  // Handle incoming NMEA2000 messages directly
  if(n2kMonitor != nullptr && !attackController -> isSpamActive()) {
    n2kMonitor->handleN2kMessage(N2kMsg);
  }
}

short buttonPressed(int button){
  int idx = getButtonIndex(button);
  if(!digitalRead(button) && (millis() - lastButtonPress[idx]) > debounceDelay){
    lastButtonPress[idx] = millis();
    return 1;
  }else{
    return 0;
  }
}

int getButtonIndex(int button) {
  if (button == BUTTON_UP) return 0;
  if (button == BUTTON_DOWN) return 1;
  if (button == BUTTON_LEFT) return 2;
  if (button == BUTTON_RIGHT) return 3;
  return 0;
}

bool CaptureOutputEnabled() {
#if DEBUG
  return false;
#else
  return serialController == nullptr || !serialController->isStreamEnabled();
#endif
}

void OnRawCanFrame(const nemo_cap::RawFrame &frame) {
  if (!CaptureOutputEnabled()) return;
  char line[96];
  size_t n;
  if (frame.dropped_before != 0) {
    nemo_cap::CaptureStats st;
    uint32_t wraps;
    NMEA2000_CAN2.GetCaptureStats(st, wraps);
    n = nemo_cap::formatGapLine(line, sizeof(line), frame.ts_us, frame.dropped_before, st.sw_drop);
    CaptureWrite(line, n);
  }
  n = nemo_cap::formatCandumpLine(line, sizeof(line), frame.ts_us, nemo_cap::kRawIfName,
                                  frame.id, frame.len, frame.data, frame.flags);
  CaptureWrite(line, n);
}

void CaptureSyncTick(bool force) {
  unsigned long nowMs = millis();
  if (!force && nowMs - lastCaptureSyncTime < NEMO_CAPTURE_SYNC_INTERVAL_MS) return;
  lastCaptureSyncTime = nowMs;

  // Always advance the 64-bit clock, even when output is off, so a quiet bus
  // can never let micros() wrap unseen.
  uint64_t nowUs = NMEA2000_CAN2.CaptureNowUs();
  if (!CaptureOutputEnabled()) return;

#if NEMO_CAPTURE_MODE == NEMO_CAPTURE_RECONSTRUCTED
  const nemo_cap::CaptureMode mode = nemo_cap::MODE_RECONSTRUCTED;
  const char *ifname = nemo_cap::kReconstructedIfName;
#else
  const nemo_cap::CaptureMode mode = nemo_cap::MODE_RAW;
  const char *ifname = nemo_cap::kRawIfName;
#endif
  char line[200];
  size_t n;
  if (force) {
    n = nemo_cap::formatBannerLine(line, sizeof(line), mode, ifname);
    CaptureWrite(line, n);
  }
  nemo_cap::CaptureStats st;
  uint32_t wraps;
  NMEA2000_CAN2.GetCaptureStats(st, wraps);
  st.tx_lost = captureTxLost;
  n = nemo_cap::formatSyncLine(line, sizeof(line), nowUs, mode, ifname, st, wraps);
  CaptureWrite(line, n);
}

#if NEMO_CAPTURE_MODE == NEMO_CAPTURE_RECONSTRUCTED
static uint64_t reconstructTimestamp = 0;

static void PrintReconstructedFrame(void *, uint32_t canId, uint8_t len, const uint8_t *data) {
  char line[96];
  size_t n = nemo_cap::formatCandumpLine(line, sizeof(line), reconstructTimestamp,
                                         nemo_cap::kReconstructedIfName, canId, len, data,
                                         nemo_cap::RF_EXTENDED);
  CaptureWrite(line, n);
}

void PrintReconstructedFormat(const tN2kMsg &N2kMsg) {
  // Timestamp = time the assembled message reached the handler, not frame arrival.
  reconstructTimestamp = NMEA2000_CAN2.CaptureNowUs();
  nemo_cap::reconstructFrames(N2kMsg.Priority, N2kMsg.PGN, N2kMsg.Source, N2kMsg.Destination,
                              N2kMsg.Data, N2kMsg.DataLen, reconstructSequence,
                              PrintReconstructedFrame, nullptr);
}
#endif
