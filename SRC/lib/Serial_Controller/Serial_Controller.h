/**
 * \file Serial_Controller.h
 * \brief JSON-line serial command protocol for GUI control of NEMO.
 *
 * Accepts JSON-line commands over USB serial and sends JSON-line events back.
 * Coexists with the OLED/button interface — both work simultaneously.
 *
 * Protocol: JSON lines terminated by '\n' at 115200 baud.
 *
 * Commands (GUI → NEMO):
 *   {"cmd":"sensor","id":0,"type":"wind_speed","value":0.5,"active":true,"mfr":229}
 *   {"cmd":"attack","type":"spam","action":"start"}
 *   {"cmd":"attack","type":"impersonate","addr":5,"pgn":127250,"fields":{"Heading":180.0}}
 *   {"cmd":"attack","action":"stop"}
 *   {"cmd":"raw","canid":"09F11224","data":"2E9999000064F6FD"}
 *   {"cmd":"status"}
 *   {"cmd":"monitor"}
 *   {"cmd":"stream","enable":true}
 *
 * Events (NEMO → GUI):
 *   {"evt":"ack","cmd":"...","ok":true}
 *   {"evt":"status","sensors":[...],"attack":"none"}
 *   {"evt":"can","ts":1234567,"id":"09F11224","dlc":8,"data":"2E9999000064F6FD"}
 *   {"evt":"monitor","devices":[...]}
 *   {"evt":"error","msg":"..."}
 */

#ifndef SERIAL_CONTROLLER_H
#define SERIAL_CONTROLLER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <NMEA2000_Teensyx.h>

// Forward declarations
class Sensor;
class Attack_Controller;
class N2K_Monitor;

class Serial_Controller {
private:
    Sensor* sensors[3];                    ///< Pointers to the 3 sensor instances
    Attack_Controller* attackController;    ///< Pointer to attack controller
    N2K_Monitor* n2kMonitor;               ///< Pointer to network monitor
    tNMEA2000_Teensyx* nmea2000_can1;      ///< Pointer to CAN1 interface for raw injection

    static const int LINE_BUF_SIZE = 512;
    char lineBuf[LINE_BUF_SIZE];           ///< Line buffer for incoming serial data
    int linePos;                           ///< Current position in line buffer

    bool streamEnabled;                    ///< Whether CAN2 traffic streaming is active

    /**
     * \brief Process a complete JSON line command
     * \param line Null-terminated JSON string
     */
    void processCommand(const char* line);

    // Command handlers
    void handleSensorCommand(JsonDocument& doc);
    void handleAttackCommand(JsonDocument& doc);
    void handleRawCommand(JsonDocument& doc);
    void handleStatusCommand();
    void handleMonitorCommand();
    void handleStreamCommand(JsonDocument& doc);

    /**
     * \brief Look up MessageType enum from string name
     * \param name Sensor type string (e.g. "heading", "wind_speed")
     * \return MessageType value, or -1 if not found
     */
    int sensorTypeFromString(const char* name);

    /**
     * \brief Get string name for a MessageType enum
     * \param type MessageType value
     * \return String name
     */
    const char* sensorTypeToString(int type);

public:
    /**
     * \brief Construct Serial_Controller
     * \param s1 Pointer to sensor 1
     * \param s2 Pointer to sensor 2
     * \param s3 Pointer to sensor 3
     * \param atk Pointer to attack controller
     * \param mon Pointer to N2K monitor
     * \param can1 Pointer to CAN1 interface
     */
    Serial_Controller(Sensor* s1, Sensor* s2, Sensor* s3,
                      Attack_Controller* atk, N2K_Monitor* mon,
                      tNMEA2000_Teensyx* can1);

    /**
     * \brief Initialize (called in setup)
     */
    void begin();

    /**
     * \brief Process serial input — call every loop iteration
     */
    void update();

    /**
     * \brief Send a JSON event line to the GUI
     * \param json Complete JSON string (without newline)
     */
    void sendEvent(const char* json);

    /**
     * \brief Forward a CAN2 message as a stream event (if streaming enabled)
     * \param canId 29-bit CAN identifier
     * \param dlc Data length code
     * \param data Pointer to data bytes
     */
    void streamCanFrame(uint32_t canId, uint8_t dlc, const uint8_t* data);

    /**
     * \brief Check if CAN streaming is enabled
     * \return true if streaming
     */
    bool isStreamEnabled() { return streamEnabled; }
};

#endif // SERIAL_CONTROLLER_H
