#include <Arduino.h>
#include <avr/wdt.h>
static const uint8_t R1_PIR=2,  R1_TRIG=3,  R1_ECHO=4,  R1_IR=5;
static const uint8_t R2_PIR=6,  R2_TRIG=7,  R2_ECHO=8,  R2_IR=9;
static const uint8_t R3_PIR=10, R3_TRIG=A0, R3_ECHO=12, R3_IR=A1;
static const uint8_t SR_DATA=11, SR_CLK=13, SR_LATCH=A2;
static const uint16_t B_R1_RED=(1<<0), B_R1_YEL=(1<<1), B_R1_GRN=(1<<2);
static const uint16_t B_R2_RED=(1<<3), B_R2_YEL=(1<<4), B_R2_GRN=(1<<5);
static const uint16_t B_R3_RED=(1<<8), B_R3_YEL=(1<<9), B_R3_GRN=(1<<10);
static const uint16_t ALL_RED_BITS = B_R1_RED | B_R2_RED | B_R3_RED;
static const uint32_t MIN_GREEN_MS       = 5000UL;
static const uint32_t GREEN_CAP_MS       = 10000UL;
static const uint32_t YELLOW_MS          = 3000UL;
static const uint32_t ALL_RED_MS         = 2000UL;
static const uint32_t DEFAULT_GREEN_MS   = 5000UL;
static const uint32_t ANTI_STARVE_MS     = 25000UL;
static const uint32_t SENSOR_STUCK_MS    = 90000UL;
static const uint32_t PIR_LATCH_MS       = 8000UL;
static const uint32_t US_LATCH_MS        = 6000UL;
static const uint32_t IR_LATCH_MS        = 5000UL;
static const uint32_t PIR_MIN_ACTIVE_MS  = 300UL;
static const uint32_t US_MIN_ACTIVE_MS   = 500UL;
static const uint32_t IR_MIN_ACTIVE_MS   = 400UL;
static const uint32_t TRAFFIC_HOLDOFF_MS = 4000UL;
static const uint32_t STATIC_TIMEOUT_MS  = 25000UL;
static const uint32_t LOG_INTERVAL_MS    = 500UL;
static const uint16_t US_MAX_DIST_CM     = 350;
static const uint8_t  US_TRIGGER_CHANGE  = 10;
static const uint8_t  US_CHANGE_CM       = 8;
static const uint8_t PIR_PINS[3]  = {R1_PIR,  R2_PIR,  R3_PIR };
static const uint8_t TRIG_PINS[3] = {R1_TRIG, R2_TRIG, R3_TRIG};
static const uint8_t ECHO_PINS[3] = {R1_ECHO, R2_ECHO, R3_ECHO};
static const uint8_t IR_PINS[3]   = {R1_IR,   R2_IR,   R3_IR  };
struct Road {
    bool    pirRaw;
    bool    irRaw;
    int32_t usDist;
    int32_t prevUsDist;
    uint32_t pirActiveStart;
    bool     pirConfirmed;
    uint32_t pirLatchExpiry;
    bool     pirLatch;
    uint32_t usActiveStart;
    bool     usConfirmed;
    uint32_t usLatchExpiry;
    bool     usLatch;
    int32_t  lastUsDist;
    uint32_t staticSince;
    bool     usFault;
    uint32_t irActiveStart;
    bool     irConfirmed;
    uint32_t irLatchExpiry;
    bool     irLatch;
    int      votes;
    bool     trafficPresent;
    uint32_t lastVoteDropTime;
    bool     inHoldoff;
    uint32_t trafficOnSince;
    bool     sensorFault;
    bool     prevTraffic;
    int      prevVotes;
    uint32_t lastGreenAt;
    uint32_t redSince;
};
static Road    road[3];
static int8_t  activeRoad  = -1;
static uint8_t defaultRoad = 0;
static void writeLEDs(uint16_t pattern) {
    digitalWrite(SR_LATCH, LOW);
    shiftOut(SR_DATA, SR_CLK, MSBFIRST,
             (uint8_t)((pattern >> 8) & 0xFF));
    shiftOut(SR_DATA, SR_CLK, MSBFIRST,
             (uint8_t)(pattern & 0xFF));
    digitalWrite(SR_LATCH, HIGH);
}
static void setAllRed() {
    writeLEDs(ALL_RED_BITS);
}
static uint16_t buildPattern(uint8_t r, bool yellow) {
    uint16_t p = ALL_RED_BITS;
    if (r == 0) {
        p &= ~B_R1_RED;
        p |= yellow ? B_R1_YEL : B_R1_GRN;
    }
    if (r == 1) {
        p &= ~B_R2_RED;
        p |= yellow ? B_R2_YEL : B_R2_GRN;
    }
    if (r == 2) {
        p &= ~B_R3_RED;
        p |= yellow ? B_R3_YEL : B_R3_GRN;
    }
    return p;
}
static int32_t readUS(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long d = pulseIn(echo, HIGH, 25000UL);
    return d ? (int32_t)(d * 0.034f / 2.0f) : 999;
}
static void snapshotAllSensors() {
    for (uint8_t r = 0; r < 3; r++) {
        road[r].pirRaw = (digitalRead(PIR_PINS[r]) == HIGH);
        road[r].usDist = readUS(TRIG_PINS[r], ECHO_PINS[r]);
        road[r].irRaw  = (digitalRead(IR_PINS[r]) == LOW);
        wdt_reset();
    }
}
static void printLine() {
    Serial.println(
        F("+----+----------+-----------+----------+-------+--------+--------+"));
}
static void printTableHeader() {
    printLine();
    Serial.println(
        F("| Rd | PIR r/L  | US  cm/St | IR  r/L  | Votes | Traf   | Red(s) |"));
    printLine();
}
static void printRoadRow(uint8_t r) {
    Road& rd = road[r];
    Serial.print(F("| R"));
    Serial.print(r + 1);
    Serial.print(F(" | "));
    Serial.print(rd.pirRaw ? F("Y") : F("n"));
    Serial.print(F("/"));
    Serial.print(rd.pirLatch ? F("LATCH   ") : F("        "));
    Serial.print(F(" | "));
    char distBuf[5];
    if (rd.usDist == 999) {
        snprintf(distBuf, sizeof(distBuf), "---");
    } else {
        snprintf(distBuf, sizeof(distBuf), "%3ld", (long)rd.usDist);
    }
    Serial.print(distBuf);
    Serial.print(F("cm/"));
    if      (rd.sensorFault) Serial.print(F("FLT "));
    else if (rd.usFault)     Serial.print(F("PRK "));
    else if (rd.usLatch)     Serial.print(F("L   "));
    else                     Serial.print(F("    "));
    Serial.print(F(" | "));
    Serial.print(rd.irRaw ? F("Y") : F("n"));
    Serial.print(F("/"));
    Serial.print(rd.irLatch ? F("LATCH   ") : F("        "));
    Serial.print(F(" | "));
    Serial.print(F("  "));
    Serial.print(rd.votes);
    Serial.print(F("  | "));
    if      (rd.sensorFault)    Serial.print(F("FAULT  "));
    else if (rd.trafficPresent) Serial.print(F("YES    "));
    else if (rd.inHoldoff)      Serial.print(F("HOLD   "));
    else                        Serial.print(F("no     "));
    Serial.print(F(" | "));
    uint32_t s = (millis() - rd.redSince) / 1000UL;
    char secBuf[5];
    snprintf(secBuf, sizeof(secBuf), "%4lu", (unsigned long)s);
    Serial.print(secBuf);
    Serial.println(F("s |"));
}
static void printStatusTable() {
    printTableHeader();
    for (uint8_t r = 0; r < 3; r++) {
        printRoadRow(r);
    }
    printLine();
}
static void processFusion(uint8_t r) {
    uint32_t now     = millis();
    bool     newPir  = road[r].pirRaw;
    int32_t  newDist = road[r].usDist;
    bool     newIr   = road[r].irRaw;
    if (newPir) {
        if (!road[r].pirActiveStart)
            road[r].pirActiveStart = now;
        if (!road[r].pirConfirmed &&
            (now - road[r].pirActiveStart) >= PIR_MIN_ACTIVE_MS) {
            road[r].pirConfirmed = true;
        }
        if (road[r].pirConfirmed) {
            road[r].pirLatchExpiry = now + PIR_LATCH_MS;
            road[r].pirLatch = true;
        }
    } else {
        road[r].pirActiveStart = 0;
        road[r].pirConfirmed   = false;
    }
    if (road[r].pirLatch && now >= road[r].pirLatchExpiry) {
        road[r].pirLatch = false;
    }
    bool distChanged =
        (abs(newDist - road[r].lastUsDist) >= US_CHANGE_CM);
    road[r].lastUsDist = newDist;
    if (newDist > 0 && newDist < US_MAX_DIST_CM &&
        !newPir && !distChanged) {
        if (!road[r].staticSince)
            road[r].staticSince = now;
        if (!road[r].usFault &&
            (now - road[r].staticSince) >= STATIC_TIMEOUT_MS) {
            road[r].usFault = true;
        }
    } else {
        road[r].usFault = false;
        road[r].staticSince = 0;
    }
    bool usTriggered = false;
    if (!road[r].usFault &&
        newDist > 0 &&
        newDist < US_MAX_DIST_CM) {
        int32_t prev = road[r].prevUsDist;
        if (prev == 999 ||
            abs(prev - newDist) >= US_TRIGGER_CHANGE) {
            usTriggered = true;
        }
        road[r].prevUsDist = newDist;
    } else {
        road[r].prevUsDist = newDist;
    }
    if (usTriggered && road[r].pirLatch) {
        if (!road[r].usActiveStart)
            road[r].usActiveStart = now;
        if (!road[r].usConfirmed &&
            (now - road[r].usActiveStart) >= US_MIN_ACTIVE_MS) {
            road[r].usConfirmed = true;
        }
        if (road[r].usConfirmed) {
            road[r].usLatchExpiry = now + US_LATCH_MS;
            road[r].usLatch = true;
        }
    } else if (!usTriggered) {
        road[r].usActiveStart = 0;
        road[r].usConfirmed   = false;
    }
    if (road[r].usLatch && now >= road[r].usLatchExpiry) {
        road[r].usLatch = false;
    }
    if (newIr) {
        if (!road[r].irActiveStart)
            road[r].irActiveStart = now;
        if (!road[r].irConfirmed &&
            (now - road[r].irActiveStart) >= IR_MIN_ACTIVE_MS) {
            road[r].irConfirmed = true;
        }
        if (road[r].irConfirmed) {
            road[r].irLatchExpiry = now + IR_LATCH_MS;
            road[r].irLatch = true;
        }
    } else {
        road[r].irActiveStart = 0;
        road[r].irConfirmed   = false;
    }
    if (road[r].irLatch && now >= road[r].irLatchExpiry) {
        road[r].irLatch = false;
    }
    int v = 0;
    if (road[r].pirLatch) v++;
    if (road[r].usLatch && !road[r].usFault) v++;
    if (road[r].irLatch) v++;
    road[r].votes = v;
    if (v >= 2 && !road[r].sensorFault) {
        road[r].trafficPresent = true;
        road[r].inHoldoff = false;
        road[r].lastVoteDropTime = 0;
        if (!road[r].trafficOnSince)
            road[r].trafficOnSince = now;
    } else {
        if (road[r].trafficPresent) {
            if (!road[r].inHoldoff) {
                road[r].inHoldoff = true;
                road[r].lastVoteDropTime = now;
            }
            if ((now - road[r].lastVoteDropTime)
                < TRAFFIC_HOLDOFF_MS) {
                road[r].trafficPresent = true;
            } else {
                road[r].trafficPresent = false;
                road[r].inHoldoff = false;
                road[r].lastVoteDropTime = 0;
                road[r].trafficOnSince = 0;
            }
        }
    }
    if (road[r].trafficPresent &&
        road[r].trafficOnSince) {
        if (!road[r].sensorFault &&
            (now - road[r].trafficOnSince)
            >= SENSOR_STUCK_MS) {
            road[r].sensorFault = true;
            road[r].trafficPresent = false;
            road[r].pirLatch = false;
            road[r].usLatch  = false;
            road[r].irLatch  = false;
        }
    } else if (!road[r].trafficPresent &&
               road[r].sensorFault) {
        road[r].sensorFault = false;
        road[r].trafficOnSince = 0;
    }
}
static bool allRoadsEqualTraffic() {
    int cnt = 0;
    int fv = -1;
    for (uint8_t r = 0; r < 3; r++) {
        if (!road[r].trafficPresent)
            continue;
        cnt++;
        if (fv < 0)
            fv = road[r].votes;
        else if (road[r].votes != fv)
            return false;
    }
    return (cnt > 1);
}
static int8_t chooseSmartRoad() {
    uint32_t now = millis();
    for (uint8_t r = 0; r < 3; r++) {
        if (road[r].sensorFault)
            continue;
        bool anyLatch =
            road[r].pirLatch ||
            road[r].usLatch  ||
            road[r].irLatch;
        uint32_t redDur =
            now - road[r].redSince;
        if (anyLatch &&
            redDur >= ANTI_STARVE_MS) {
            return (int8_t)r;
        }
    }
    bool any = false;
    for (uint8_t r = 0; r < 3; r++) {
        if (road[r].trafficPresent) {
            any = true;
            break;
        }
    }
    if (!any) return -1;
    if (allRoadsEqualTraffic())
        return -1;
    int8_t chosen = -1;
    uint32_t maxW = 0;
    for (uint8_t r = 0; r < 3; r++) {
        if (!road[r].trafficPresent ||
            road[r].sensorFault)
            continue;
        uint32_t w =
            now - road[r].redSince;
        bool better = (w > maxW);
        bool tieWin =
            (w == maxW) &&
            (chosen >= 0) &&
            (r < (uint8_t)chosen);
        if (better || tieWin) {
            maxW = w;
            chosen = (int8_t)r;
        }
    }
    return chosen;
}
static void runGreenPhase(uint8_t r, bool isDefault);
static void runGreenPhase(uint8_t r, bool isDefault) {
    activeRoad = (int8_t)r;
    road[r].lastGreenAt = millis();
    uint32_t phaseStart = millis();
    uint32_t hardDeadline =
        phaseStart + GREEN_CAP_MS;
    uint32_t phaseEnd =
        phaseStart +
        (isDefault ? DEFAULT_GREEN_MS :
                     MIN_GREEN_MS);
    if (phaseEnd > hardDeadline)
        phaseEnd = hardDeadline;
    bool extended = false;
    bool switchPending = false;
    int8_t pendingRoad = -1;
    writeLEDs(buildPattern(r, false));
    uint32_t lastStatusLog = 0;
    while (millis() < phaseEnd) {
        wdt_reset();
        snapshotAllSensors();
        for (uint8_t i = 0; i < 3; i++) {
            processFusion(i);
        }
        if (!isDefault &&
            !extended &&
            road[r].trafficPresent) {
            if (hardDeadline > phaseEnd) {
                phaseEnd = hardDeadline;
                extended = true;
            }
        }
        if (isDefault && !switchPending) {
            int8_t smart =
                chooseSmartRoad();
            if (smart >= 0 &&
                smart != (int8_t)r) {
                switchPending = true;
                pendingRoad = smart;
                uint32_t cut =
                    millis() + 1000UL;
                if (cut < phaseEnd)
                    phaseEnd = cut;
            }
        }
        if (millis() - lastStatusLog
            >= LOG_INTERVAL_MS) {
            lastStatusLog = millis();
            printStatusTable();
        }
        delay(200);
    }
    writeLEDs(buildPattern(r, true));
    uint32_t yellEnd =
        millis() + YELLOW_MS;
    while (millis() < yellEnd) {
        wdt_reset();
        delay(100);
    }
    setAllRed();
    delay(ALL_RED_MS);
    road[r].redSince = millis();
    activeRoad = -1;
    if (switchPending &&
        pendingRoad >= 0) {
        runGreenPhase(
            (uint8_t)pendingRoad,
            false
        );
    }
}
void setup() {
    Serial.begin(9600);
    wdt_enable(WDTO_4S);
    pinMode(SR_DATA,  OUTPUT);
    pinMode(SR_CLK,   OUTPUT);
    pinMode(SR_LATCH, OUTPUT);
    for (uint8_t r = 0; r < 3; r++) {
        pinMode(PIR_PINS[r],  INPUT);
        pinMode(TRIG_PINS[r], OUTPUT);
        pinMode(ECHO_PINS[r], INPUT);
        pinMode(IR_PINS[r],   INPUT);
    }
    uint32_t now = millis();
    for (uint8_t i = 0; i < 3; i++) {
        Road& rd = road[i];
        rd.pirRaw = false;
        rd.irRaw  = false;
        rd.usDist     = 999;
        rd.prevUsDist = 999;
        rd.pirActiveStart = 0;
        rd.pirConfirmed   = false;
        rd.pirLatchExpiry = 0;
        rd.pirLatch       = false;
        rd.usActiveStart = 0;
        rd.usConfirmed   = false;
        rd.usLatchExpiry = 0;
        rd.usLatch       = false;
        rd.lastUsDist  = 999;
        rd.staticSince = 0;
        rd.usFault     = false;
        rd.irActiveStart = 0;
        rd.irConfirmed   = false;
        rd.irLatchExpiry = 0;
        rd.irLatch       = false;
        rd.votes = 0;
        rd.trafficPresent = false;
        rd.lastVoteDropTime = 0;
        rd.inHoldoff = false;
        rd.trafficOnSince = 0;
        rd.sensorFault = false;
        rd.prevTraffic = false;
        rd.prevVotes = 0;
        rd.lastGreenAt =
            now - (uint32_t)(i * 4000UL);
        rd.redSince =
            now - (uint32_t)(i * 4000UL);
    }
    setAllRed();
}
static uint32_t lastRawLog = 0;
void loop() {
    wdt_reset();
    snapshotAllSensors();
    for (uint8_t r = 0; r < 3; r++) {
        processFusion(r);
        wdt_reset();
    }
    if (millis() - lastRawLog
        >= LOG_INTERVAL_MS) {
        lastRawLog = millis();
        printStatusTable();
    }
    int8_t smart =
        chooseSmartRoad();
    if (smart >= 0) {
        runGreenPhase(
            (uint8_t)smart,
            false
        );
    } else {
        runGreenPhase(
            defaultRoad,
            true
        );
        defaultRoad =
            (defaultRoad + 1) % 3;
    }
}
