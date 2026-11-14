#include <Arduino.h>
#include <Servo.h>

enum LegId : uint8_t {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    BACK_LEFT = 2,
    BACK_RIGHT = 3
};

enum GaitMode : uint8_t {
    MODE_STAND = 0,
    MODE_FORWARD,
    MODE_BACKWARD,
    MODE_TURN_LEFT,
    MODE_TURN_RIGHT,
    MODE_WAVE
};

constexpr uint8_t SERVO_PINS[4] = {3, 5, 6, 9};

// Trim these per leg so all 4 feet sit square on the ground at neutral
constexpr int NEUTRAL_ANGLES[4] = {90, 90, 90, 90};

// Flip to -1 if a leg swings opposite to expected
constexpr int INVERT_LEG[4] = {1, -1, 1, -1};

constexpr int MIN_ANGLE = 25;
constexpr int MAX_ANGLE = 155;
constexpr unsigned long STEP_INTERVAL_MS = 18;

Servo servos[4];
int currentAngles[4];
int targetAngles[4];

GaitMode currentMode = MODE_STAND;
unsigned long lastUpdateMs = 0;
unsigned long gaitStartMs = 0;

int clampAngle(int angle) {
    if (angle < MIN_ANGLE) return MIN_ANGLE;
    if (angle > MAX_ANGLE) return MAX_ANGLE;
    return angle;
}

int legTarget(LegId leg, int offset) {
    int mapped = NEUTRAL_ANGLES[leg] + (INVERT_LEG[leg] * offset);
    return clampAngle(mapped);
}

void setMode(GaitMode mode) {
    currentMode = mode;
    gaitStartMs = millis();

    if (mode == MODE_STAND) {
        for (uint8_t i = 0; i < 4; i++) {
            targetAngles[i] = clampAngle(NEUTRAL_ANGLES[i]);
        }
    }
}

void computeGaitTargets(unsigned long elapsedMs) {
    const float cyclePeriodMs = 1400.0f;
    float phase = fmodf(static_cast<float>(elapsedMs), cyclePeriodMs) / cyclePeriodMs * TWO_PI;

    switch (currentMode) {
        case MODE_STAND:
            for (uint8_t i = 0; i < 4; i++) {
                targetAngles[i] = clampAngle(NEUTRAL_ANGLES[i]);
            }
            break;

        case MODE_FORWARD: {
            int diagA = static_cast<int>(roundf(sinf(phase) * 28.0f));
            int diagB = static_cast<int>(roundf(sinf(phase + PI) * 28.0f));
            targetAngles[FRONT_LEFT]  = legTarget(FRONT_LEFT, diagA);
            targetAngles[BACK_RIGHT]  = legTarget(BACK_RIGHT, diagA);
            targetAngles[FRONT_RIGHT] = legTarget(FRONT_RIGHT, diagB);
            targetAngles[BACK_LEFT]   = legTarget(BACK_LEFT, diagB);
            break;
        }

        case MODE_BACKWARD: {
            int diagA = static_cast<int>(roundf(-sinf(phase) * 28.0f));
            int diagB = static_cast<int>(roundf(-sinf(phase + PI) * 28.0f));
            targetAngles[FRONT_LEFT]  = legTarget(FRONT_LEFT, diagA);
            targetAngles[BACK_RIGHT]  = legTarget(BACK_RIGHT, diagA);
            targetAngles[FRONT_RIGHT] = legTarget(FRONT_RIGHT, diagB);
            targetAngles[BACK_LEFT]   = legTarget(BACK_LEFT, diagB);
            break;
        }

        case MODE_TURN_LEFT: {
            int leftSwing  = static_cast<int>(roundf(sinf(phase) * 26.0f));
            int rightSwing = static_cast<int>(roundf(-sinf(phase) * 26.0f));
            targetAngles[FRONT_LEFT]  = legTarget(FRONT_LEFT, leftSwing);
            targetAngles[BACK_LEFT]   = legTarget(BACK_LEFT, leftSwing);
            targetAngles[FRONT_RIGHT] = legTarget(FRONT_RIGHT, rightSwing);
            targetAngles[BACK_RIGHT]  = legTarget(BACK_RIGHT, rightSwing);
            break;
        }

        case MODE_TURN_RIGHT: {
            int leftSwing  = static_cast<int>(roundf(-sinf(phase) * 26.0f));
            int rightSwing = static_cast<int>(roundf(sinf(phase) * 26.0f));
            targetAngles[FRONT_LEFT]  = legTarget(FRONT_LEFT, leftSwing);
            targetAngles[BACK_LEFT]   = legTarget(BACK_LEFT, leftSwing);
            targetAngles[FRONT_RIGHT] = legTarget(FRONT_RIGHT, rightSwing);
            targetAngles[BACK_RIGHT]  = legTarget(BACK_RIGHT, rightSwing);
            break;
        }

        case MODE_WAVE: {
            for (uint8_t i = 0; i < 4; i++) {
                float legPhase = phase - (i * (HALF_PI));
                int offset = static_cast<int>(roundf(sinf(legPhase) * 32.0f));
                targetAngles[i] = legTarget(static_cast<LegId>(i), offset);
            }
            break;
        }
    }
}

void updateServosSmoothly(int maxStepDeg = 3) {
    for (uint8_t i = 0; i < 4; i++) {
        int diff = targetAngles[i] - currentAngles[i];
        if (diff > maxStepDeg) diff = maxStepDeg;
        if (diff < -maxStepDeg) diff = -maxStepDeg;
        currentAngles[i] += diff;
        servos[i].write(currentAngles[i]);
    }
}

void handleSerialCommand(char cmd) {
    switch (tolower(static_cast<unsigned char>(cmd))) {
        case 's':
            setMode(MODE_STAND);
            Serial.println(F("Mode: STAND"));
            break;
        case 'f':
            setMode(MODE_FORWARD);
            Serial.println(F("Mode: FORWARD"));
            break;
        case 'b':
            setMode(MODE_BACKWARD);
            Serial.println(F("Mode: BACKWARD"));
            break;
        case 'l':
            setMode(MODE_TURN_LEFT);
            Serial.println(F("Mode: TURN_LEFT"));
            break;
        case 'r':
            setMode(MODE_TURN_RIGHT);
            Serial.println(F("Mode: TURN_RIGHT"));
            break;
        case 'w':
            setMode(MODE_WAVE);
            Serial.println(F("Mode: WAVE"));
            break;
        default:
            break;
    }
}

void setup() {
    Serial.begin(115200);
    for (uint8_t i = 0; i < 4; i++) {
        currentAngles[i] = clampAngle(NEUTRAL_ANGLES[i]);
        targetAngles[i] = currentAngles[i];
        servos[i].attach(SERVO_PINS[i], 500, 2500);
        servos[i].write(currentAngles[i]);
        delay(120);
    }
    setMode(MODE_FORWARD);
    Serial.println(F("Quadruped ready. Commands: s=stand, f=fwd, b=back, l=left, r=right, w=wave"));
}

void loop() {
    while (Serial.available() > 0) {
        handleSerialCommand(static_cast<char>(Serial.read()));
    }

    unsigned long now = millis();
    if (now - lastUpdateMs >= STEP_INTERVAL_MS) {
        lastUpdateMs = now;
        computeGaitTargets(now - gaitStartMs);
        updateServosSmoothly(3);
    }
}