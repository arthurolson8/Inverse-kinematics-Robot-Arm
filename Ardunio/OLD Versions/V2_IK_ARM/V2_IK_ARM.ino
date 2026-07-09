//====================================================================
// Robot_Masterfile.ino
// Single-file build: all modules merged into one sketch file.
// (Config / Joint / Controller / Gripper / Calibration)
//====================================================================

#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Arduino_Modulino.h>


//====================================================================
// CONFIG
//====================================================================
const byte NUM_JOINTS = 6;
//====================================================
//                  LIBRARIES
//====================================================

#include <Arduino.h>

//====================================================
//                SERVO PINS
//====================================================

const byte BASE_PIN         = 2;
const byte SHOULDER_PIN     = 4;
const byte ELBOW_PIN        = 7;
const byte WRIST_PITCH_PIN  = 9;
const byte WRIST_ROLL_PIN   = 11;
const byte GRIPPER_PIN      = 13;

//====================================================
//             JOYSTICK ANALOG PINS
//====================================================

// Joystick 1
const byte J1_X = A0;
const byte J1_Y = A1;

// Joystick 2
const byte J2_X = A2;
const byte J2_Y = A3;

// Joystick 3
const byte J3_X = A4;

// (Reserved in case you use the second axis later)
// const byte J3_Y = A5;

//====================================================
//          JOYSTICK BUTTON PINS (OPTIONAL)
//====================================================

const byte J1_SW = 3;
const byte J2_SW = 5;
const byte J3_SW = 6;

//====================================================
//              SERVO HOME POSITIONS
//====================================================

// These are the positions the robot moves to on startup.
// Adjust after assembling the arm.

const float BASE_HOME         = 135.0;
const float SHOULDER_HOME     = 156.0;
const float ELBOW_HOME        = 118.0;
const float WRIST_PITCH_HOME  = 206.0;
const float WRIST_ROLL_HOME   = 90.0;
const float GRIPPER_HOME      = 30.0;

//====================================================
//                SERVO LIMITS
//====================================================

// Degrees

const float BASE_MIN = 20;
const float BASE_MAX = 250;

const float SHOULDER_MIN = 50;
const float SHOULDER_MAX = 220;

const float ELBOW_MIN = 40;
const float ELBOW_MAX = 243;

const float WRIST_PITCH_MIN = 42.15;
const float WRIST_PITCH_MAX = 230;

const float WRIST_ROLL_MIN = 0;
const float WRIST_ROLL_MAX = 180;

const float GRIPPER_MIN = 20;
const float GRIPPER_MAX = 105;

//====================================================
//            SERVO DIRECTION
//====================================================

// false = normal
// true = reversed

const bool BASE_REVERSE         = false;
const bool SHOULDER_REVERSE     = false;
const bool ELBOW_REVERSE        = false;
const bool WRIST_PITCH_REVERSE  = false;
const bool WRIST_ROLL_REVERSE   = false;
const bool GRIPPER_REVERSE      = false;

//====================================================
//              MAX SERVO SPEED
//====================================================

// Degrees moved per update.
// Larger value = faster motion.

const float BASE_SPEED         = 1.2;
const float SHOULDER_SPEED     = 0.9;
const float ELBOW_SPEED        = 1.0;
const float WRIST_PITCH_SPEED  = 1.1;
const float WRIST_ROLL_SPEED   = 1.5;
const float GRIPPER_SPEED      = 2.0;


//====================================================
// Servo Types
//====================================================

// DS3218 Servos
const float DS3218_RANGE = 270.0;

// SG90 Servos
const float SG90_RANGE = 180.0;


//====================================================
// Pulse Widths
//====================================================

// DS3218
const int DS3218_MIN_US = 500;
const int DS3218_MAX_US = 2500;

// SG90
const int SG90_MIN_US = 500;
const int SG90_MAX_US = 2400;

//====================================================
//             JOYSTICK SETTINGS
//====================================================

const int DEADZONE = 60;

// These will be calibrated automatically
// during startup.

extern int J1_X_CENTER;
extern int J1_Y_CENTER;

extern int J2_X_CENTER;
extern int J2_Y_CENTER;

extern int J3_X_CENTER;

//====================================================
//             SERVO PWM CALIBRATION
//====================================================

// Most DS3218 servos work well with these.
// We'll fine tune later if necessary.

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

//====================================================
//               DEBUG OPTIONS
//====================================================

const bool ENABLE_SERIAL_DEBUG = true;


//====================================================
// Number of Joints
//====================================================


// Joint Numbers

enum JointID
{
    BASE = 0,
    SHOULDER,
    ELBOW,
    WRIST_PITCH,
    WRIST_ROLL,
    GRIPPER
};
//====================================================
// JOINT CONFIGURATION (ARRAY-BASED)
//====================================================

const byte servoPins[NUM_JOINTS] =
{
    BASE_PIN,
    SHOULDER_PIN,
    ELBOW_PIN,
    WRIST_PITCH_PIN,
    WRIST_ROLL_PIN,
    GRIPPER_PIN
};

const float homePositions[NUM_JOINTS] =
{
    BASE_HOME,
    SHOULDER_HOME,
    ELBOW_HOME,
    WRIST_PITCH_HOME,
    WRIST_ROLL_HOME,
    GRIPPER_HOME
};

const float minAngles[NUM_JOINTS] =
{
    BASE_MIN,
    SHOULDER_MIN,
    ELBOW_MIN,
    WRIST_PITCH_MIN,
    WRIST_ROLL_MIN,
    GRIPPER_MIN
};

const float maxAngles[NUM_JOINTS] =
{
    BASE_MAX,
    SHOULDER_MAX,
    ELBOW_MAX,
    WRIST_PITCH_MAX,
    WRIST_ROLL_MAX,
    GRIPPER_MAX
};

const float servoSpeeds[NUM_JOINTS] =
{
    BASE_SPEED,
    SHOULDER_SPEED,
    ELBOW_SPEED,
    WRIST_PITCH_SPEED,
    WRIST_ROLL_SPEED,
    GRIPPER_SPEED
};

const bool servoReverse[NUM_JOINTS] =
{
    BASE_REVERSE,
    SHOULDER_REVERSE,
    ELBOW_REVERSE,
    WRIST_PITCH_REVERSE,
    WRIST_ROLL_REVERSE,
    GRIPPER_REVERSE
};

const float servoRange[NUM_JOINTS] =
{
    DS3218_RANGE,
    DS3218_RANGE,
    DS3218_RANGE,
    DS3218_RANGE,
    SG90_RANGE,
    SG90_RANGE
};

const int servoMinPulse[NUM_JOINTS] =
{
    DS3218_MIN_US,
    DS3218_MIN_US,
    DS3218_MIN_US,
    DS3218_MIN_US,
    SG90_MIN_US,
    SG90_MIN_US
};

const int servoMaxPulse[NUM_JOINTS] =
{
    DS3218_MAX_US,
    DS3218_MAX_US,
    DS3218_MAX_US,
    DS3218_MAX_US,
    SG90_MAX_US,
    SG90_MAX_US
};
enum RobotMode
{
    MODE_NORMAL,
    MODE_CALIBRATION,
    MODE_IK_ARM
};

extern RobotMode currentMode;

//====================================================================
// JOINT (declaration)
//====================================================================
#include <Arduino.h>
#include <Servo.h>
class Joint
{
private:

    Servo servo;

    byte pin;

    float position;
    float homePosition;

    float minimumAngle;
    float maximumAngle;

    float speed;

    bool reverse;

    float servoRange;

    int minPulse;
    int maxPulse;

public:

    Joint();

    void begin(
        byte servoPin,
        float home,
        float minimum,
        float maximum,
        float maxSpeed,
        bool reversed,
        float rangeDegrees,
        int minMicroseconds,
        int maxMicroseconds
    );

    void update(int joystickValue, int joystickCenter);

    void write();

    void hold();

    void home();

    void setPosition(float newPosition);

    float getPosition();

    void setSpeed(float newSpeed);

    void setReverse(bool state);

    void setLimits(float minimum,float maximum);

    float getMinLimit();

    float getMaxLimit();
};

// Global robot joints
extern Joint joints[NUM_JOINTS];

//====================================================================
// JOINT (implementation)
//====================================================================
//====================================================
// Global Joint Array
//====================================================

Joint joints[NUM_JOINTS];

//====================================================
// Constructor
//====================================================

Joint::Joint()
{
    pin = 0;

    position = 90.0;
    homePosition = 90.0;

    minimumAngle = 0.0;
    maximumAngle = 180.0;

    speed = 1.0;

    reverse = false;

    servoRange = 180.0;

    minPulse = 500;
    maxPulse = 2500;
}

//====================================================
// Begin
//====================================================

void Joint::begin(
    byte servoPin,
    float home,
    float minimum,
    float maximum,
    float maxSpeed,
    bool reversed,
    float rangeDegrees,
    int minMicroseconds,
    int maxMicroseconds
)
{
    pin = servoPin;

    homePosition = home;
    position = home;

    minimumAngle = minimum;
    maximumAngle = maximum;

    speed = maxSpeed;

    reverse = reversed;

    servoRange = rangeDegrees;

    minPulse = minMicroseconds;
    maxPulse = maxMicroseconds;

    servo.attach(pin, minPulse, maxPulse);

    write();
}

//====================================================
// Update Joint
//====================================================

void Joint::update(int joystickValue, int joystickCenter)
{
    int offset = joystickValue - joystickCenter;

    if (abs(offset) <= DEADZONE)
        return;

    float multiplier =
        (float)(abs(offset) - DEADZONE) /
        (512.0 - DEADZONE);

    multiplier = constrain(multiplier, 0.0f, 1.0f);

    float movement = multiplier * speed;

    if (reverse)
        movement = -movement;

    if (offset > 0)
        position += movement;
    else
        position -= movement;

    position = constrain(position, minimumAngle, maximumAngle);
}

//====================================================
// Hold Position
//====================================================

void Joint::hold()
{
    write();
}

//====================================================
// Home Position
//====================================================

void Joint::home()
{
    position = homePosition;
    write();
}

//====================================================
// Write Servo
//====================================================

void Joint::write()
{
    int pulse = map(
        (long)(position * 10),
        0,
        (long)(servoRange * 10),
        minPulse,
        maxPulse
    );

    servo.writeMicroseconds(pulse);
}

//====================================================
// Set Position
//====================================================

void Joint::setPosition(float newPosition)
{
    position = constrain(newPosition, minimumAngle, maximumAngle);
}

//====================================================
// Get Position
//====================================================

float Joint::getPosition()
{
    return position;
}

//====================================================
// Set Speed
//====================================================

void Joint::setSpeed(float newSpeed)
{
    speed = newSpeed;
}

//====================================================
// Reverse Direction
//====================================================

void Joint::setReverse(bool state)
{
    reverse = state;
}

//====================================================
// Set Limits
//====================================================

void Joint::setLimits(float minimum, float maximum)
{
    minimumAngle = minimum;
    maximumAngle = maximum;

    position = constrain(position, minimumAngle, maximumAngle);
}

//====================================================
// Get Limits
//====================================================

float Joint::getMinLimit()
{
    return minimumAngle;
}

float Joint::getMaxLimit()
{
    return maximumAngle;
}

//====================================================================
// CONTROLLER (declaration)
//====================================================================
#include <Arduino.h>

//====================================================
//      Global joystick center values
//====================================================

extern int J1_X_CENTER;
extern int J1_Y_CENTER;

extern int J2_X_CENTER;
extern int J2_Y_CENTER;

extern int J3_X_CENTER;

//====================================================
//      Functions
//====================================================

// Reads joystick centers during startup
void calibrateJoysticks();

// Optional debug print
void printJoystickValues();

//====================================================================
// CONTROLLER (implementation)
//====================================================================
//====================================================
//      Center Values
//====================================================

int J1_X_CENTER = 512;
int J1_Y_CENTER = 512;

int J2_X_CENTER = 512;
int J2_Y_CENTER = 512;

int J3_X_CENTER = 512;

//====================================================
//      Calibrate Joysticks
//====================================================

void calibrateJoysticks()
{
    long j1x = 0;
    long j1y = 0;

    long j2x = 0;
    long j2y = 0;

    long j3x = 0;

    const int samples = 100;

    Serial.println();
    Serial.println("Calibrating Joysticks...");
    Serial.println("Don't touch the joysticks!");

    delay(1000);

    for(int i = 0; i < samples; i++)
    {
        j1x += analogRead(J1_X);
        j1y += analogRead(J1_Y);

        j2x += analogRead(J2_X);
        j2y += analogRead(J2_Y);

        j3x += analogRead(J3_X);

        delay(5);
    }

    J1_X_CENTER = j1x / samples;
    J1_Y_CENTER = j1y / samples;

    J2_X_CENTER = j2x / samples;
    J2_Y_CENTER = j2y / samples;

    J3_X_CENTER = j3x / samples;

    Serial.println("Done!");
    Serial.println();

    printJoystickValues();
}

//====================================================
//      Debug
//====================================================

void printJoystickValues()
{
    Serial.println("Joystick Centers");

    Serial.print("J1 X: ");
    Serial.println(J1_X_CENTER);

    Serial.print("J1 Y: ");
    Serial.println(J1_Y_CENTER);

    Serial.print("J2 X: ");
    Serial.println(J2_X_CENTER);

    Serial.print("J2 Y: ");
    Serial.println(J2_Y_CENTER);

    Serial.print("J3 X: ");
    Serial.println(J3_X_CENTER);

    Serial.println();
}

//====================================================================
// GRIPPER (declaration)
//====================================================================
#include <Arduino.h>
#include <Arduino_Modulino.h>

class Gripper
{
private:

    ModulinoKnob knob;

    int lastKnobValue;

public:

    Gripper();

    void begin();

    void update();

    // Returns true once each time the knob is clicked
    bool buttonPressed();

    // Returns the current encoder position
    int knobPosition();
};

extern Gripper gripperController;

//====================================================================
// GRIPPER (implementation)
//====================================================================
Gripper gripperController;

//====================================================
// Constructor
//====================================================

Gripper::Gripper()
{
    lastKnobValue = 0;
}

//====================================================
// Begin
//====================================================

void Gripper::begin()
{
    Modulino.begin();

    knob.begin();

    lastKnobValue = knob.get();
}

//====================================================
// Update
//====================================================

void Gripper::update()
{
    int value = knob.get();

    int change = value - lastKnobValue;

    if(change != 0)
    {
        float pos = joints[GRIPPER].getPosition();

        pos += change * GRIPPER_SPEED;

        joints[GRIPPER].setPosition(pos);

        lastKnobValue = value;
    }

    joints[GRIPPER].write();
}

//====================================================
// Returns true once when the knob button is clicked
//====================================================

bool Gripper::buttonPressed()
{
    return knob.isPressed();
}

//====================================================
// Returns current knob encoder position
//====================================================

int Gripper::knobPosition()
{
    return knob.get();
}

//====================================================================
// CALIBRATION (declaration)
//====================================================================
#include <Arduino.h>

// Call once in setup(), AFTER joints[] have been begin()'d and
// AFTER gripperController.begin(). Loads any previously saved
// limits from EEPROM and applies them to the joints. Also checks
// whether the knob button is being held down, and if so, enters
// calibration mode instead of continuing to home position.
void calibrationInit();

// Call every loop() iteration while currentMode == MODE_CALIBRATION.
void calibrationUpdate();

// Call every loop() iteration while currentMode == MODE_NORMAL.
// Lets you enter Calibration Mode by typing 'c' + Enter, or
// IK Test Mode by typing 'i' + Enter, over Serial. Typing
// 'p' + Enter prints current raw joint angles (read-only,
// doesn't affect calibration or EEPROM) -- handy for finding
// physical reference poses while jogging with the joystick.
void checkSerialModeEntry();

//====================================================================
// CALIBRATION (implementation)
//====================================================================
#include <EEPROM.h>
#include <Arduino_Modulino.h>


ModulinoKnob calibrationKnob;

//--------------------------------------------------
// Internal state
//--------------------------------------------------

static int currentServo = 0;

// Limits found during THIS calibration session.
static float minimumReached[NUM_JOINTS];
static float maximumReached[NUM_JOINTS];

static unsigned long lastPrint = 0;

// Edge-detection for the knob button
static bool wasPressed = false;
static unsigned long pressStart = 0;

const unsigned long LONG_PRESS_MS = 2000;
const unsigned long ENTRY_HOLD_MS = 2000;
const unsigned long ENTRY_WINDOW_MS = 3000;

//--------------------------------------------------

const char* servoNames[NUM_JOINTS] =
{
    "BASE",
    "SHOULDER",
    "ELBOW",
    "WRIST PITCH",
    "WRIST ROLL",
    "GRIPPER"
};

//--------------------------------------------------
// EEPROM layout
//--------------------------------------------------
// [0]              magic byte (0xA5 = valid data present)
// [1 .. ]           NUM_JOINTS floats: calibrated min angles
// [1 + 4*N .. ]     NUM_JOINTS floats: calibrated max angles

const int EEPROM_MAGIC_ADDR = 0;
const byte EEPROM_MAGIC_VALUE = 0xA5;
const int EEPROM_MIN_ADDR = 1;
const int EEPROM_MAX_ADDR = EEPROM_MIN_ADDR + (NUM_JOINTS * sizeof(float));

//--------------------------------------------------

void printCurrentServo()
{
    Serial.println();
    Serial.println("==============================");
    Serial.print("Calibration Servo: ");
    Serial.println(servoNames[currentServo]);

    Serial.print("Angle: ");
    Serial.println(joints[currentServo].getPosition());

    Serial.println("Rotate Joystick 1 X to move this joint");
    Serial.println("Click Modulino Button   = Save + Next Servo");
    Serial.println("Hold Modulino Button 2s = Save + Exit Calibration");
    Serial.println("==============================");
}

//--------------------------------------------------
// Apply this session's found min/max into the joints
//--------------------------------------------------

void applyLimitsToJoints()
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        joints[i].setLimits(minimumReached[i], maximumReached[i]);
    }
}

//--------------------------------------------------
// Persist all limits to EEPROM
//--------------------------------------------------

void writeLimitsToEEPROM()
{
    EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);

    int addr = EEPROM_MIN_ADDR;
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        EEPROM.put(addr, minimumReached[i]);
        addr += sizeof(float);
    }

    addr = EEPROM_MAX_ADDR;
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        EEPROM.put(addr, maximumReached[i]);
        addr += sizeof(float);
    }

    Serial.println("Calibration saved to EEPROM.");
}

//--------------------------------------------------
// Load limits from EEPROM (if any were ever saved) and
// apply them to the joints. Returns true if valid data
// was found and applied.
//--------------------------------------------------

bool loadAndApplySavedLimits()
{
    byte magic = EEPROM.read(EEPROM_MAGIC_ADDR);

    if (magic != EEPROM_MAGIC_VALUE)
        return false;

    int addr = EEPROM_MIN_ADDR;
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        EEPROM.get(addr, minimumReached[i]);
        addr += sizeof(float);
    }

    addr = EEPROM_MAX_ADDR;
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        EEPROM.get(addr, maximumReached[i]);
        addr += sizeof(float);
    }

    // Sanity check: reject obviously corrupt data instead of
    // applying garbage limits (e.g. NaN, or min >= max).
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        if (isnan(minimumReached[i]) || isnan(maximumReached[i]))
            return false;

        if (minimumReached[i] >= maximumReached[i])
            return false;
    }

    applyLimitsToJoints();

    Serial.println("Loaded saved calibration from EEPROM.");
    return true;
}

//--------------------------------------------------
// Save current joint's session limits (called on each click)
//--------------------------------------------------

void saveCurrentJointLimits()
{
    float p = joints[currentServo].getPosition();

    if (p < minimumReached[currentServo])
        minimumReached[currentServo] = p;

    if (p > maximumReached[currentServo])
        maximumReached[currentServo] = p;

    Serial.print(servoNames[currentServo]);
    Serial.print(" limits set -> Min: ");
    Serial.print(minimumReached[currentServo]);
    Serial.print("  Max: ");
    Serial.println(maximumReached[currentServo]);
}

//--------------------------------------------------
// calibrationInit()
//--------------------------------------------------
// Called once from setup(). Loads any saved calibration,
// then gives the user a short window to hold the knob button
// to enter calibration mode instead of homing immediately.
//--------------------------------------------------

void calibrationInit()
{
    Modulino.begin();
    calibrationKnob.begin();

    // Start each joint's session limits at its current (home) position
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        minimumReached[i] = joints[i].getPosition();
        maximumReached[i] = joints[i].getPosition();
    }

    // Load previously saved calibration, if any, and apply it
    // immediately so normal mode uses the correct limits even
    // if the user does NOT enter calibration mode this boot.
    loadAndApplySavedLimits();

    // Give the user a couple seconds to hold the button and
    // request calibration mode.
    Serial.println();
    Serial.println("Hold knob button now to enter Calibration Mode...");

    unsigned long windowStart = millis();
    unsigned long heldSince = 0;
    bool holding = false;

    while (millis() - windowStart < ENTRY_WINDOW_MS)
    {
        bool pressed = calibrationKnob.isPressed();

        if (pressed && !holding)
        {
            holding = true;
            heldSince = millis();
        }
        else if (!pressed)
        {
            holding = false;
        }

        if (holding && (millis() - heldSince >= ENTRY_HOLD_MS))
        {
            currentMode = MODE_CALIBRATION;
            currentServo = 0;

            Serial.println();
            Serial.println("Entering Calibration Mode.");
            printCurrentServo();
            return;
        }

        delay(10);
    }
}

//--------------------------------------------------
// checkSerialModeEntry()
//--------------------------------------------------
// Lets the user type 'c' + Enter to enter Calibration Mode,
// 'i' + Enter to enter (3-joint) IK Test Mode, or 'k' + Enter
// to enter 2D IK Test Mode (Shoulder/Elbow only, no Base --
// for testing before the base is wired up), at any time during
// normal operation. Combined into one function so a single
// pass through the Serial buffer can't accidentally consume
// and discard a character meant for another mode.
//--------------------------------------------------

void checkSerialModeEntry()
{
    while (Serial.available() > 0)
    {
        char c = Serial.read();

        if (c == 'c' || c == 'C')
        {
            currentMode = MODE_CALIBRATION;
            currentServo = 0;

            for (int i = 0; i < NUM_JOINTS; i++)
            {
                minimumReached[i] = joints[i].getPosition();
                maximumReached[i] = joints[i].getPosition();
            }

            wasPressed = false;

            Serial.println();
            Serial.println("Entering Calibration Mode (via Serial).");
            printCurrentServo();
            return;
        }

        if (c == 'k' || c == 'K')
        {
            currentMode = MODE_IK_ARM;
            ikArmEnter();
            return;
        }

        if (c == 'p' || c == 'P')
        {
            // Read-only: does NOT touch calibration or EEPROM.
            // Safe to use any time, purely for finding reference
            // poses while jogging the arm with the joystick.
            Serial.println();
            Serial.println("---- Current Joint Angles (raw, calibrated scale) ----");
            Serial.print("Base:         "); Serial.println(joints[BASE].getPosition());
            Serial.print("Shoulder:     "); Serial.println(joints[SHOULDER].getPosition());
            Serial.print("Elbow:        "); Serial.println(joints[ELBOW].getPosition());
            Serial.print("Wrist Pitch:  "); Serial.println(joints[WRIST_PITCH].getPosition());
            Serial.print("Wrist Roll:   "); Serial.println(joints[WRIST_ROLL].getPosition());
            Serial.print("Gripper:      "); Serial.println(joints[GRIPPER].getPosition());
            Serial.println("-------------------------------------------------------");
            return;
        }
    }
}

//--------------------------------------------------
// calibrationUpdate()
//--------------------------------------------------

void calibrationUpdate()
{
    //--------------------------------------------------
    // Move selected servo with joystick
    //--------------------------------------------------

    int joystick = analogRead(J1_X);

    joints[currentServo].update(joystick, J1_X_CENTER);
    joints[currentServo].write();

    //--------------------------------------------------
    // Track the widest range reached so far this session
    //--------------------------------------------------

    float p = joints[currentServo].getPosition();

    if (p < minimumReached[currentServo])
        minimumReached[currentServo] = p;

    if (p > maximumReached[currentServo])
        maximumReached[currentServo] = p;

    //--------------------------------------------------
    // Edge-triggered button handling
    //--------------------------------------------------

    bool pressed = calibrationKnob.isPressed();

    if (pressed && !wasPressed)
    {
        // Rising edge: button just pressed down
        pressStart = millis();
    }

    if (!pressed && wasPressed)
    {
        // Falling edge: button just released -> decide click type
        unsigned long heldFor = millis() - pressStart;

        saveCurrentJointLimits();

        if (heldFor >= LONG_PRESS_MS)
        {
            // Long press -> save everything and exit
            applyLimitsToJoints();
            writeLimitsToEEPROM();

            currentMode = MODE_NORMAL;

            Serial.println();
            Serial.println("Calibration complete. Limits saved. Exiting.");
        }
        else
        {
            // Short click -> move to next servo
            currentServo++;

            if (currentServo >= NUM_JOINTS)
                currentServo = 0;

            printCurrentServo();
        }
    }

    wasPressed = pressed;

    //--------------------------------------------------
    // Status print every 300 ms
    //--------------------------------------------------

    if (millis() - lastPrint > 300)
    {
        lastPrint = millis();

        Serial.print("Servo: ");
        Serial.print(servoNames[currentServo]);

        Serial.print("  Angle: ");
        Serial.print(p);

        Serial.print("  Min: ");
        Serial.print(minimumReached[currentServo]);

        Serial.print("  Max: ");
        Serial.println(maximumReached[currentServo]);
    }
}

//====================================================================
// INVERSE KINEMATICS TEST -- 2D INCREMENTAL JOG (Base not used)
//====================================================================
// This works purely in the Shoulder/Elbow vertical plane (x = out
// from the shoulder, z = up from the base axis). The Base joint is
// never touched here, so this works fine even with the base servo
// unplugged.
//
// Instead of jumping to distant preset targets, you nudge the
// target a small step at a time. Every proposed step is checked
// against your calibrated limits BEFORE moving -- if it would
// exceed a limit, the move is refused and the target stays where
// it was.
//
// By default the gripper is kept level (parallel to the table)
// automatically as Shoulder/Elbow move. Toggle this off with 'l'
// to take manual control of Wrist Pitch via the joystick instead.
//====================================================================

#include <math.h>

// Link lengths in mm -- update these if you re-measure
const float IK_L0 = 22.0;
const float IK_L1 = 120.0;
const float IK_L2 = 120.0;

const int IK_MOVE_STEPS = 20;      // steps per jog move (small moves = few steps needed)
const int IK_STEP_DELAY_MS = 15;

//--------------------------------------------------
// MATH-TO-RAW CALIBRATION MAPPING
//--------------------------------------------------
// computeIK2D() produces angles in a math frame where 0 degrees
// means: shoulder horizontal, elbow fully extended (straight).
// Both read 0 simultaneously at ONE pose: arm fully extended and
// horizontal. Wrist Pitch math 0 means "gripper level," which
// happens at that SAME pose (since auto-level keeps it there).
//
// HOW TO FIND THESE VALUES (arm stays fully assembled):
//   1. Jog Shoulder/Elbow until the arm is fully extended and
//      horizontal, and Wrist Pitch so the gripper is level.
//   2. Type 'p' + Enter in Normal Mode (read-only, doesn't touch
//      calibration/EEPROM) and copy the 3 raw numbers below.
//   3. For each joint's sign: nudge it a known direction, check
//      with 'p' again:
//        Shoulder: lift UP from horizontal. Raw up -> sign=1, down -> -1.
//        Elbow:    bend in from full extension. Raw up -> sign=1, down -> -1.
//        Wrist:    tilt gripper tip UP. Raw up -> sign=1, down -> -1.
//--------------------------------------------------

float shoulderZeroRaw = 50.0;    // PLACEHOLDER
int   shoulderSign    = -1;

float elbowZeroRaw    = 149.0;   // PLACEHOLDER
int   elbowSign       = -1;

float wristZeroRaw    = 135.0;   // PLACEHOLDER
int   wristSign       = 1;

float mapShoulderToRaw(float mathDeg) { return shoulderZeroRaw + shoulderSign * mathDeg; }
float mapElbowToRaw(float mathDeg)    { return elbowZeroRaw    + elbowSign    * mathDeg; }
float mapWristToRaw(float mathDeg)    { return wristZeroRaw    + wristSign    * mathDeg; }

float rawToMathShoulder(float rawDeg) { return (rawDeg - shoulderZeroRaw) / shoulderSign; }
float rawToMathElbow(float rawDeg)    { return (rawDeg - elbowZeroRaw)    / elbowSign; }

//--------------------------------------------------
// 2-link inverse kinematics (Shoulder + Elbow only).
// Target (x, z) is the WRIST joint's position, in the
// vertical plane straight out from the shoulder.
//--------------------------------------------------

bool computeIK2D(float x, float z, float &outShoulderDeg, float &outElbowDeg)
{
    float dr = x;
    float dz = z - IK_L0;

    float D = sqrt(dr * dr + dz * dz);

    if (D > (IK_L1 + IK_L2) || D < fabs(IK_L1 - IK_L2))
    {
        return false;
    }

    float cosTheta2 = (D * D - IK_L1 * IK_L1 - IK_L2 * IK_L2) / (2 * IK_L1 * IK_L2);
    cosTheta2 = constrain(cosTheta2, -1.0, 1.0);
    float theta2 = acos(cosTheta2);

    float theta1 = atan2(dz, dr) - atan2(IK_L2 * sin(theta2), IK_L1 + IK_L2 * cos(theta2));

    outShoulderDeg = degrees(theta1);
    outElbowDeg = degrees(theta2);

    return true;
}

// Forward kinematics: math-frame Shoulder/Elbow angles -> (x, z).
// Used to initialize the jog target at wherever the arm already is.
void computeFK2D(float shoulderMathDeg, float elbowMathDeg, float &outX, float &outZ)
{
    float theta1 = radians(shoulderMathDeg);
    float theta2 = radians(elbowMathDeg);

    float dr = IK_L1 * cos(theta1) + IK_L2 * cos(theta1 + theta2);
    float dz = IK_L1 * sin(theta1) + IK_L2 * sin(theta1 + theta2);

    outX = dr;
    outZ = dz + IK_L0;
}

//--------------------------------------------------
// Jog state
//--------------------------------------------------

float ikTargetX = 0;
float ikTargetZ = 0;
bool  ikAutoLevel = true;
float ikStepSize = 10.0;        // mm per keyboard jog command
const float IK_JOG_SPEED = 1.5; // mm per loop tick at full joystick deflection
unsigned long lastIkArmPrint = 0;

//--------------------------------------------------
// Limit check helper -- prints a pass/fail line, returns pass/fail
//--------------------------------------------------

bool checkJointLimit(const char* name, float angle, int jointIndex)
{
    float lo = joints[jointIndex].getMinLimit();
    float hi = joints[jointIndex].getMaxLimit();
    bool ok = (angle >= lo && angle <= hi);

    Serial.print(name);
    Serial.print(": ");
    Serial.print(angle);
    Serial.print(ok ? "  OK  " : "  OUT OF RANGE  ");
    Serial.print("(calibrated limits ");
    Serial.print(lo);
    Serial.print(" to ");
    Serial.print(hi);
    Serial.println(")");

    return ok;
}

//--------------------------------------------------
// Move Shoulder + Elbow (and optionally Wrist Pitch) smoothly
// from wherever they currently are to the given raw targets.
// Checks Serial for 'x' abort on every step.
//--------------------------------------------------

void move2Link(float shoulderRaw, float elbowRaw, bool moveWrist, float wristRaw)
{
    float startShoulder = joints[SHOULDER].getPosition();
    float startElbow    = joints[ELBOW].getPosition();
    float startWrist     = joints[WRIST_PITCH].getPosition();

    for (int step = 1; step <= IK_MOVE_STEPS; step++)
    {
        if (Serial.available() > 0)
        {
            char c = Serial.read();
            if (c == 'x' || c == 'X')
            {
                Serial.println("ABORTED. Holding current position.");
                return;
            }
        }

        float t = (float)step / (float)IK_MOVE_STEPS;

        joints[SHOULDER].setPosition(startShoulder + t * (shoulderRaw - startShoulder));
        joints[ELBOW].setPosition(startElbow + t * (elbowRaw - startElbow));
        joints[SHOULDER].write();
        joints[ELBOW].write();

        if (moveWrist)
        {
            joints[WRIST_PITCH].setPosition(startWrist + t * (wristRaw - startWrist));
            joints[WRIST_PITCH].write();
        }

        delay(IK_STEP_DELAY_MS);
    }
}

//--------------------------------------------------
// Attempt to jog the target to (newX, newZ). Validates against
// calibrated limits first -- if it fails, nothing moves and the
// stored target is left unchanged.
//--------------------------------------------------

void ikArmPrintStatus()
{
    Serial.println("---------------------------------------------");
    Serial.print("Target -> reach=");
    Serial.print(ikTargetX);
    Serial.print("  height=");
    Serial.println(ikTargetZ);
    Serial.print("Step size: ");
    Serial.print(ikStepSize);
    Serial.println(" mm");
    Serial.print("Auto-level gripper: ");
    Serial.println(ikAutoLevel ? "ON" : "OFF (jog Wrist Pitch yourself with the joystick)");
    Serial.println("Joysticks: J1_X = base, J1_Y = height, J2_X = reach,");
    Serial.println("           J2_Y = wrist (manual, only when auto-level OFF), J3_X = wrist roll");
    Serial.println("Keys: w/s = height up/down   a/d = reach -/+   l = toggle auto-level");
    Serial.println("      +/- = step size   p = print status   x = exit");
    Serial.println("---------------------------------------------");
}

// Discrete keyboard move: validated, prints diagnostics, smooth
// interpolated motion. Good for careful, deliberate nudges.
void ikArmTryMove(float newX, float newZ)
{
    float shoulderDeg, elbowDeg;
    bool reachable = computeIK2D(newX, newZ, shoulderDeg, elbowDeg);

    if (!reachable)
    {
        Serial.println("REJECTED: unreachable in the reach/height plane. Target unchanged.");
        return;
    }

    float shoulderRaw = mapShoulderToRaw(shoulderDeg);
    float elbowRaw = mapElbowToRaw(elbowDeg);

    float wristRaw = joints[WRIST_PITCH].getPosition();
    bool moveWrist = false;

    if (ikAutoLevel)
    {
        float wristMathDeg = -(shoulderDeg + elbowDeg);
        wristRaw = mapWristToRaw(wristMathDeg);
        moveWrist = true;
    }

    bool ok = true;
    ok &= checkJointLimit("Shoulder", shoulderRaw, SHOULDER);
    ok &= checkJointLimit("Elbow", elbowRaw, ELBOW);
    if (moveWrist)
        ok &= checkJointLimit("Wrist Pitch", wristRaw, WRIST_PITCH);

    if (!ok)
    {
        Serial.println("REJECTED: would exceed calibrated limits. Target unchanged.");
        return;
    }

    ikTargetX = newX;
    ikTargetZ = newZ;

    move2Link(shoulderRaw, elbowRaw, moveWrist, wristRaw);
    ikArmPrintStatus();
}

// Continuous joystick move: validated SILENTLY (no per-tick Serial
// spam) and applied INSTANTLY (no multi-step interpolation -- the
// joystick itself already produces small, smooth increments every
// loop tick, same as normal joint control). If the proposed point
// would exceed a limit or is unreachable, it's simply ignored and
// the target holds at its last good value -- like a soft stop.
void ikArmJoystickMove(float newX, float newZ)
{
    float shoulderDeg, elbowDeg;
    bool reachable = computeIK2D(newX, newZ, shoulderDeg, elbowDeg);

    if (!reachable)
        return;

    float shoulderRaw = mapShoulderToRaw(shoulderDeg);
    float elbowRaw = mapElbowToRaw(elbowDeg);

    float wristRaw = joints[WRIST_PITCH].getPosition();
    bool moveWrist = false;

    if (ikAutoLevel)
    {
        float wristMathDeg = -(shoulderDeg + elbowDeg);
        wristRaw = mapWristToRaw(wristMathDeg);
        moveWrist = true;
    }

    if (shoulderRaw < joints[SHOULDER].getMinLimit() || shoulderRaw > joints[SHOULDER].getMaxLimit())
        return;

    if (elbowRaw < joints[ELBOW].getMinLimit() || elbowRaw > joints[ELBOW].getMaxLimit())
        return;

    if (moveWrist && (wristRaw < joints[WRIST_PITCH].getMinLimit() || wristRaw > joints[WRIST_PITCH].getMaxLimit()))
        return;

    ikTargetX = newX;
    ikTargetZ = newZ;

    joints[SHOULDER].setPosition(shoulderRaw);
    joints[ELBOW].setPosition(elbowRaw);
    joints[SHOULDER].write();
    joints[ELBOW].write();

    if (moveWrist)
    {
        joints[WRIST_PITCH].setPosition(wristRaw);
        joints[WRIST_PITCH].write();
    }
}

//--------------------------------------------------
// Call once when entering IK Arm Mode. Initializes the jog
// target at wherever the arm ALREADY is, so the first input
// nudges it instead of jumping somewhere else.
//--------------------------------------------------

void ikArmEnter()
{
    float shoulderMath = rawToMathShoulder(joints[SHOULDER].getPosition());
    float elbowMath = rawToMathElbow(joints[ELBOW].getPosition());

    computeFK2D(shoulderMath, elbowMath, ikTargetX, ikTargetZ);

    ikAutoLevel = true;
    ikStepSize = 10.0;

    Serial.println();
    Serial.println("Entering IK Arm Mode (V2). Base included, joystick-driven.");
    ikArmPrintStatus();
}

// Call in loop() while currentMode == MODE_IK_ARM
void ikArmUpdate()
{
    //--------------------------------------------------
    // Base: driven directly, exactly like Normal Mode
    //--------------------------------------------------
    joints[BASE].update(analogRead(J1_X), J1_X_CENTER);
    joints[BASE].write();

    //--------------------------------------------------
    // Height (z) and Reach (r) -- continuous IK jogging
    //--------------------------------------------------
    int heightOffset = analogRead(J1_Y) - J1_Y_CENTER;
    int reachOffset  = analogRead(J2_X) - J2_X_CENTER;

    float proposedX = ikTargetX;
    float proposedZ = ikTargetZ;
    bool wantMove = false;

    if (abs(heightOffset) > DEADZONE)
    {
        float m = (float)(abs(heightOffset) - DEADZONE) / (512.0 - DEADZONE);
        m = constrain(m, 0.0f, 1.0f);
        float delta = m * IK_JOG_SPEED;

        // Flipped per testing feedback: stick "up" should raise height.
        if (heightOffset > 0)
            proposedZ -= delta;
        else
            proposedZ += delta;

        wantMove = true;
    }

    if (abs(reachOffset) > DEADZONE)
    {
        float m = (float)(abs(reachOffset) - DEADZONE) / (512.0 - DEADZONE);
        m = constrain(m, 0.0f, 1.0f);
        float delta = m * IK_JOG_SPEED;

        // Flipped per testing feedback: stick "forward" should extend reach.
        if (reachOffset > 0)
            proposedX -= delta;
        else
            proposedX += delta;

        wantMove = true;
    }

    if (wantMove)
    {
        ikArmJoystickMove(proposedX, proposedZ);
    }

    //--------------------------------------------------
    // Wrist Pitch: manual via joystick ONLY when auto-level is off.
    // When auto-level is on, ikArmJoystickMove()/ikArmTryMove()
    // already keep it level automatically.
    //--------------------------------------------------
    if (!ikAutoLevel)
    {
        joints[WRIST_PITCH].update(analogRead(J2_Y), J2_Y_CENTER);
        joints[WRIST_PITCH].write();
    }

    //--------------------------------------------------
    // Wrist Roll and Gripper: always directly controllable,
    // same as Normal Mode -- IK doesn't involve these.
    //--------------------------------------------------
    joints[WRIST_ROLL].update(analogRead(J3_X), J3_X_CENTER);
    joints[WRIST_ROLL].write();

    gripperController.update();

    //--------------------------------------------------
    // Throttled status printout (only while actively jogging)
    //--------------------------------------------------
    if (wantMove && millis() - lastIkArmPrint > 300)
    {
        lastIkArmPrint = millis();
        Serial.print("reach=");
        Serial.print(ikTargetX);
        Serial.print("  height=");
        Serial.print(ikTargetZ);
        Serial.print("  base=");
        Serial.println(joints[BASE].getPosition());
    }

    //--------------------------------------------------
    // Serial commands -- still available for precise nudges
    //--------------------------------------------------
    if (Serial.available() <= 0)
        return;

    char c = Serial.read();

    switch (c)
    {
        case 'w': case 'W':
            // Flipped per testing feedback
            ikArmTryMove(ikTargetX, ikTargetZ - ikStepSize);
            break;

        case 's': case 'S':
            ikArmTryMove(ikTargetX, ikTargetZ + ikStepSize);
            break;

        case 'd': case 'D':
            ikArmTryMove(ikTargetX - ikStepSize, ikTargetZ);
            break;

        case 'a': case 'A':
            ikArmTryMove(ikTargetX + ikStepSize, ikTargetZ);
            break;

        case 'l': case 'L':
            ikAutoLevel = !ikAutoLevel;
            Serial.print("Auto-level gripper: ");
            Serial.println(ikAutoLevel ? "ON" : "OFF (jog Wrist Pitch yourself with the joystick)");
            break;

        case '+':
            ikStepSize += 5.0;
            Serial.print("Step size: ");
            Serial.print(ikStepSize);
            Serial.println(" mm");
            break;

        case '-':
            ikStepSize -= 5.0;
            if (ikStepSize < 1.0)
                ikStepSize = 1.0;
            Serial.print("Step size: ");
            Serial.print(ikStepSize);
            Serial.println(" mm");
            break;

        case 'p': case 'P':
            ikArmPrintStatus();
            break;

        case 'x': case 'X':
            Serial.println("Exiting IK Arm Mode.");
            currentMode = MODE_NORMAL;
            break;

        default:
            break; // ignore newline / unrecognized characters
    }
}


//====================================================================
// MAIN SKETCH (setup / loop)
//====================================================================
RobotMode currentMode = MODE_NORMAL;
//====================================================
// GLOBAL ROBOT OBJECTS
//====================================================


//====================================================
// SETUP
//====================================================

void setup()
{
    Serial.begin(115200);

    if (ENABLE_SERIAL_DEBUG)
    {
        Serial.println("Robot Arm Starting...");
    }

    //================================================
    // Initialize Joystick Calibration
    //================================================

    calibrateJoysticks();

    //================================================
    // Initialize All Joints
    //================================================

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        joints[i].begin(
            servoPins[i],
            homePositions[i],
            minAngles[i],
            maxAngles[i],
            servoSpeeds[i],
            servoReverse[i],
            servoRange[i],
            servoMinPulse[i],
            servoMaxPulse[i]
        );
    }

    //================================================
    // Initialize Gripper (Modulino Knob)
    //================================================

    gripperController.begin();

    //================================================
    // Load saved calibration (if any) and check whether
    // the user is holding the knob to enter Calibration Mode
    //================================================

    calibrationInit();

    // Move everything to home (skip if we're entering calibration,
    // so the arm stays wherever it currently is instead of jumping)
    if (currentMode == MODE_NORMAL)
    {
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            joints[i].home();
        }
    }

    if (ENABLE_SERIAL_DEBUG)
    {
        Serial.println("Setup Complete.");
    }
}

//====================================================
// MAIN LOOP
//====================================================

void loop()
{
    if(currentMode == MODE_NORMAL)
    {
        checkSerialModeEntry();

        joints[BASE].update(analogRead(J1_X), J1_X_CENTER);
        joints[SHOULDER].update(analogRead(J1_Y), J1_Y_CENTER);

        joints[ELBOW].update(analogRead(J2_X), J2_X_CENTER);
        joints[WRIST_PITCH].update(analogRead(J2_Y), J2_Y_CENTER);

        joints[WRIST_ROLL].update(analogRead(J3_X), J3_X_CENTER);

        gripperController.update();

        for(int i = 0; i < NUM_JOINTS; i++)
            joints[i].write();

        //====================================================
        // PRINT SERVO ANGLES
        //====================================================

        if (Serial.available())
        {
            char command = Serial.read();

            if (command == 'p' || command == 'P')
            {
                Serial.println();
                Serial.println("========== Servo Angles ==========");

                Serial.print("Base: ");
                Serial.println(joints[BASE].getPosition(), 1);

                Serial.print("Shoulder: ");
                Serial.println(joints[SHOULDER].getPosition(), 1);

                Serial.print("Elbow: ");
                Serial.println(joints[ELBOW].getPosition(), 1);

                Serial.print("Wrist Pitch: ");
                Serial.println(joints[WRIST_PITCH].getPosition(), 1);

                Serial.print("Wrist Roll: ");
                Serial.println(joints[WRIST_ROLL].getPosition(), 1);

                Serial.print("Gripper: ");
                Serial.println(joints[GRIPPER].getPosition(), 1);

                Serial.println("==================================");
                Serial.println();
            }
        }
    }
    else if (currentMode == MODE_CALIBRATION)
    {
        calibrationUpdate();
    }
    else if (currentMode == MODE_IK_ARM)
    {
        ikArmUpdate();
    }

    delay(10);
}



// Serial Monitor COMMANDS =========================
//C-enter calibration mode =  short click->Next, long clock->save
//i-enter IK test mode  = 1-5-> preset target, y-> actually move arm, h-> home, x-> exit IK immediately
//p Print current raw anggle of all 6 joints

//k->enter IK version

