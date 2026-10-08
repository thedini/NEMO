/**
 * \file Serial_Controller.cpp
 * \brief JSON-line serial command protocol implementation for NEMO.
 *
 * Provides bidirectional JSON-line communication over USB serial,
 * allowing a Python GUI to control sensors, attacks, monitoring,
 * and raw CAN frame injection.
 */

#include "Serial_Controller.h"
#include <Sensor.h>
#include <Attack_Controller.h>
#include <N2K_Monitor.h>
#include <PGN_Helpers.h>

// Sensor type name lookup table — matches MessageType enum order
static const char* SENSOR_TYPE_NAMES[] = {
    "engine_rpm",    // 0 MSG_ENGINE_RPM
    "water_depth",   // 1 MSG_WATER_DEPTH
    "heading",       // 2 MSG_HEADING
    "speed",         // 3 MSG_SPEED
    "rudder",        // 4 MSG_RUDDER
    "wind_speed",    // 5 MSG_WIND_SPEED
    "wind_angle",    // 6 MSG_WIND_ANGLE
    "water_temp",    // 7 MSG_WATER_TEMP
    "outside_temp",  // 8 MSG_OUTSIDE_TEMP
    "pressure",      // 9 MSG_PRESSURE
    "humidity",      // 10 MSG_HUMIDITY
    "battery_volt",  // 11 MSG_BATTERY_VOLT
    "tank_level"     // 12 MSG_TANK_LEVEL
};
static const int SENSOR_TYPE_COUNT = 13;

//*****************************************************************************
Serial_Controller::Serial_Controller(Sensor* s1, Sensor* s2, Sensor* s3,
                                     Attack_Controller* atk, N2K_Monitor* mon,
                                     tNMEA2000_Teensyx* can1) {
    sensors[0] = s1;
    sensors[1] = s2;
    sensors[2] = s3;
    attackController = atk;
    n2kMonitor = mon;
    nmea2000_can1 = can1;
    linePos = 0;
    streamEnabled = false;
}

//*****************************************************************************
void Serial_Controller::begin() {
    // Serial is already initialized in main setup()
    linePos = 0;
}

//*****************************************************************************
void Serial_Controller::update() {
    // Read available serial bytes and parse complete lines
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (linePos > 0) {
                lineBuf[linePos] = '\0';
                processCommand(lineBuf);
                linePos = 0;
            }
        } else {
            if (linePos < LINE_BUF_SIZE - 1) {
                lineBuf[linePos++] = c;
            } else {
                // Buffer overflow — discard and reset
                linePos = 0;
                sendEvent("{\"evt\":\"error\",\"msg\":\"command too long\"}");
            }
        }
    }
}

//*****************************************************************************
void Serial_Controller::sendEvent(const char* json) {
    Serial.println(json);
}

//*****************************************************************************
void Serial_Controller::processCommand(const char* line) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, line);

    if (err) {
        char errBuf[128];
        snprintf(errBuf, sizeof(errBuf),
                 "{\"evt\":\"error\",\"msg\":\"JSON parse error: %s\"}", err.c_str());
        sendEvent(errBuf);
        return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) {
        sendEvent("{\"evt\":\"error\",\"msg\":\"missing cmd field\"}");
        return;
    }

    if (strcmp(cmd, "sensor") == 0) {
        handleSensorCommand(doc);
    } else if (strcmp(cmd, "attack") == 0) {
        handleAttackCommand(doc);
    } else if (strcmp(cmd, "raw") == 0) {
        handleRawCommand(doc);
    } else if (strcmp(cmd, "status") == 0) {
        handleStatusCommand();
    } else if (strcmp(cmd, "monitor") == 0) {
        handleMonitorCommand();
    } else if (strcmp(cmd, "stream") == 0) {
        handleStreamCommand(doc);
    } else {
        char errBuf[128];
        snprintf(errBuf, sizeof(errBuf),
                 "{\"evt\":\"error\",\"msg\":\"unknown cmd: %s\"}", cmd);
        sendEvent(errBuf);
    }
}

//*****************************************************************************
void Serial_Controller::handleSensorCommand(JsonDocument& doc) {
    if (!doc["id"].is<int>()) {
        sendEvent("{\"evt\":\"error\",\"msg\":\"sensor cmd requires id\"}");
        return;
    }

    int id = doc["id"];
    if (id < 0 || id > 2) {
        sendEvent("{\"evt\":\"error\",\"msg\":\"invalid sensor id (0-2)\"}");
        return;
    }

    Sensor* sensor = sensors[id];

    // Set sensor type if provided
    if (doc["type"].is<const char*>()) {
        int typeIdx = sensorTypeFromString(doc["type"]);
        if (typeIdx >= 0) {
            sensor->setMessageType((MessageType)typeIdx);
            sensor->updateDeviceInfo();
        } else {
            sendEvent("{\"evt\":\"error\",\"msg\":\"unknown sensor type\"}");
            return;
        }
    }

    // Set value if provided (0.0-1.0 normalized)
    if (doc["value"].is<float>() || doc["value"].is<int>()) {
        float val = doc["value"];
        sensor->setValue(val);
        sensor->setSerialControlled(true);
    }

    // Set manufacturer code if provided
    if (doc["mfr"].is<int>()) {
        sensor->setManufacturerCode(doc["mfr"]);
    }

    // Set active state if provided
    if (doc["active"].is<bool>()) {
        sensor->setActive(doc["active"]);
    }

    sendEvent("{\"evt\":\"ack\",\"cmd\":\"sensor\",\"ok\":true}");
}

//*****************************************************************************
void Serial_Controller::handleAttackCommand(JsonDocument& doc) {
    const char* action = doc["action"];

    if (action && strcmp(action, "stop") == 0) {
        if (attackController->isSpamActive()) {
            attackController->stopSpamAttack();
        }
        if (attackController->isImpersonateActive()) {
            attackController->stopImpersonate();
        }
        sendEvent("{\"evt\":\"ack\",\"cmd\":\"attack\",\"ok\":true}");
        return;
    }

    const char* type = doc["type"];
    if (!type) {
        sendEvent("{\"evt\":\"error\",\"msg\":\"attack cmd requires type or action:stop\"}");
        return;
    }

    if (strcmp(type, "spam") == 0) {
        if (action && strcmp(action, "start") == 0) {
            // Optional "rate" (frames/second); omitted -> firmware default.
            // Out-of-range values are clamped inside startSpamAttack().
            if (doc["rate"].is<int>()) {
                attackController->startSpamAttack((uint32_t)doc["rate"]);
            } else {
                attackController->startSpamAttack();
            }
            sendEvent("{\"evt\":\"ack\",\"cmd\":\"attack\",\"ok\":true}");
        } else {
            sendEvent("{\"evt\":\"error\",\"msg\":\"spam requires action:start\"}");
        }
    } else if (strcmp(type, "impersonate") == 0) {
        if (!doc["addr"].is<int>() || !doc["pgn"].is<int>()) {
            sendEvent("{\"evt\":\"error\",\"msg\":\"impersonate requires addr and pgn\"}");
            return;
        }
        uint8_t addr = doc["addr"];
        uint32_t pgn = doc["pgn"];
        attackController->startImpersonate(addr, pgn);
        sendEvent("{\"evt\":\"ack\",\"cmd\":\"attack\",\"ok\":true}");
    } else {
        sendEvent("{\"evt\":\"error\",\"msg\":\"unknown attack type\"}");
    }
}

//*****************************************************************************
void Serial_Controller::handleRawCommand(JsonDocument& doc) {
    const char* canidStr = doc["canid"];
    const char* dataStr = doc["data"];

    if (!canidStr || !dataStr) {
        sendEvent("{\"evt\":\"error\",\"msg\":\"raw requires canid and data\"}");
        return;
    }

    // Parse hex CAN ID
    uint32_t canId = strtoul(canidStr, nullptr, 16);

    // Parse hex data string into bytes
    uint8_t data[8];
    int dataLen = strlen(dataStr) / 2;
    if (dataLen > 8) dataLen = 8;

    for (int i = 0; i < dataLen; i++) {
        char hex[3] = { dataStr[i*2], dataStr[i*2+1], '\0' };
        data[i] = (uint8_t)strtoul(hex, nullptr, 16);
    }

    // Build a raw NMEA2000 message
    // Extract source address (bottom 8 bits of CAN ID)
    // Extract PGN (bits 8-25 of 29-bit extended ID)
    // Extract priority (bits 26-28)
    tN2kMsg msg;
    msg.Source = canId & 0xFF;
    msg.PGN = (canId >> 8) & 0x3FFFF;
    msg.Priority = (canId >> 26) & 0x7;
    msg.DataLen = dataLen;
    for (int i = 0; i < dataLen; i++) {
        msg.Data[i] = data[i];
    }

    // Send via NMEA2000 library (device index -1 = raw send)
    nmea2000_can1->SendMsg(msg, -1);

    sendEvent("{\"evt\":\"ack\",\"cmd\":\"raw\",\"ok\":true}");
}

//*****************************************************************************
void Serial_Controller::handleStatusCommand() {
    JsonDocument doc;
    doc["evt"] = "status";

    // Sensor array
    JsonArray sensorArr = doc["sensors"].to<JsonArray>();
    for (int i = 0; i < 3; i++) {
        JsonObject s = sensorArr.add<JsonObject>();
        s["id"] = i;
        s["type"] = sensorTypeToString(sensors[i]->getMessageType());
        s["value"] = sensors[i]->getNormalizedValue();
        s["active"] = sensors[i]->isActive();
        s["mfr"] = sensors[i]->getManufacturerCode();
        s["addr"] = sensors[i]->getSourceAddress();
        s["name"] = sensors[i]->getCustomName();
        s["serial_ctrl"] = sensors[i]->isSerialControlled();
    }

    // Attack state
    if (attackController->isSpamActive()) {
        doc["attack"] = "spam";
        doc["spam_count"] = attackController->getSpamMessageCount();
    } else if (attackController->isImpersonateActive()) {
        doc["attack"] = "impersonate";
        doc["imp_addr"] = attackController->getImpTargetAddress();
        doc["imp_pgn"] = attackController->getImpTargetPGN();
    } else {
        doc["attack"] = "none";
    }

    doc["stream"] = streamEnabled;

    // Serialize and send
    char buf[1024];
    serializeJson(doc, buf, sizeof(buf));
    sendEvent(buf);
}

//*****************************************************************************
void Serial_Controller::handleMonitorCommand() {
    JsonDocument doc;
    doc["evt"] = "monitor";

    JsonArray devArr = doc["devices"].to<JsonArray>();

    auto& devices = n2kMonitor->getDevices();
    for (auto& pair : devices) {
        const DeviceInfo& dev = pair.second;
        JsonObject d = devArr.add<JsonObject>();
        d["addr"] = dev.sourceAddress;
        d["name"] = dev.name;
        d["last_seen"] = dev.lastSeen;

        JsonArray pgns = d["pgns"].to<JsonArray>();
        for (auto& pgnPair : dev.pgns) {
            pgns.add(pgnPair.first);
        }
    }

    char buf[2048];
    serializeJson(doc, buf, sizeof(buf));
    sendEvent(buf);
}

//*****************************************************************************
void Serial_Controller::handleStreamCommand(JsonDocument& doc) {
    if (doc["enable"].is<bool>()) {
        streamEnabled = doc["enable"];
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "{\"evt\":\"ack\",\"cmd\":\"stream\",\"ok\":true,\"enabled\":%s}",
                 streamEnabled ? "true" : "false");
        sendEvent(buf);
    } else {
        sendEvent("{\"evt\":\"error\",\"msg\":\"stream requires enable:bool\"}");
    }
}

//*****************************************************************************
void Serial_Controller::streamCanFrame(uint32_t canId, uint8_t dlc, const uint8_t* data) {
    if (!streamEnabled) return;

    // Build JSON event manually for speed (avoid ArduinoJson overhead on hot path)
    char buf[128];
    int pos = snprintf(buf, sizeof(buf),
                       "{\"evt\":\"can\",\"ts\":%lu,\"id\":\"%08lX\",\"dlc\":%d,\"data\":\"",
                       millis(), (unsigned long)canId, dlc);

    for (int i = 0; i < dlc && pos < (int)sizeof(buf) - 4; i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X", data[i]);
    }

    if (pos < (int)sizeof(buf) - 3) {
        buf[pos++] = '"';
        buf[pos++] = '}';
        buf[pos] = '\0';
        Serial.println(buf);
    }
}

//*****************************************************************************
int Serial_Controller::sensorTypeFromString(const char* name) {
    for (int i = 0; i < SENSOR_TYPE_COUNT; i++) {
        if (strcmp(name, SENSOR_TYPE_NAMES[i]) == 0) {
            return i;
        }
    }
    return -1;
}

//*****************************************************************************
const char* Serial_Controller::sensorTypeToString(int type) {
    if (type >= 0 && type < SENSOR_TYPE_COUNT) {
        return SENSOR_TYPE_NAMES[type];
    }
    return "unknown";
}
