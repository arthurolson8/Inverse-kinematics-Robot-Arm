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
const float SHOULDER_HOME     = 135.0;
const float ELBOW_HOME        = 135.0;
const float WRIST_PITCH_HOME  = 135.0;
const float WRIST_ROLL_HOME   = 90.0;
const float GRIPPER_HOME      = 30.0;

//====================================================
//                SERVO LIMITS
//====================================================

// Degrees

const float BASE_MIN = 20;
const float BASE_MAX = 250;

const float SHOULDER_MIN = 35;
const float SHOULDER_MAX = 230;

const float ELBOW_MIN = 40;
const float ELBOW_MAX = 235;

const float WRIST_PITCH_MIN = 40;
const float WRIST_PITCH_MAX = 230;

const float WRIST_ROLL_MIN = 0;
const float WRIST_ROLL_MAX = 180;

const float GRIPPER_MIN = 20;
const float GRIPPER_MAX = 90;

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
    MODE_IK_TEST
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
// Lets you enter calibration mode by typing 'c' + Enter over Serial,
// which is handy on the bench when the knob is hard to hold at boot.
void checkSerialCalibrationEntry();

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
// checkSerialCalibrationEntry()
//--------------------------------------------------
// Lets the user type 'c' + Enter over Serial to enter
// calibration mode at any time during normal operation.
//--------------------------------------------------

void checkSerialCalibrationEntry()
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
// INVERSE KINEMATICS TEST (safe, on-robot, no disassembly required)
//====================================================================
// This mode NEVER commands a servo outside its calibrated limits.
// Every target is validated against the real Joint limits before
// anything moves. If any joint would go out of range, the move is
// refused entirely (all-or-nothing) and nothing happens.
//
// NOTE: this only guarantees the servos stay within your calibrated
// mechanical range. It does NOT guarantee the math's zero-reference
// matches your physical assembly yet -- that's exactly what this
// test is for. Watch the arm closely on every move and be ready to
// power off if it does anything unexpected.
//====================================================================

#include <math.h>

// Link lengths in mm -- update these if you re-measure
const float IK_L0 = 22.0;
const float IK_L1 = 120.0;
const float IK_L2 = 120.0;
const float IK_L3 = 166.0;

// NOTE: we deliberately do NOT wrap the 4 output angles in a
// custom struct returned by value. Arduino's IDE auto-generates
// function prototypes and inserts them near the very top of the
// file -- before any struct you define further down -- which
// breaks compilation ("does not name a type"). Plain float
// parameters sidestep that entirely.

// A handful of sample targets to try. Add/edit freely.
// Format: { x, y, z, phi(gripper pitch, deg, 0=level, -90=straight down) }
struct IKPreset
{
    const char* label;
    float x, y, z, phi;
};

const IKPreset ikPresets[] =
{
    { "Forward, gripper level",     250, 0,    100, -30 },
    { "Forward-right, angled down", 150, 100,  80,  -60 },
    { "Forward-left, level",        100, -100, 150, 0   },
    { "Far reach, straight down",   300, 0,    50,  -90 },
    { "High and back, angled up",   50,  0,    250, 45  },
};

const int NUM_IK_PRESETS = sizeof(ikPresets) / sizeof(ikPresets[0]);

const int IK_MOVE_STEPS = 60;      // how many small steps each move is split into
const int IK_STEP_DELAY_MS = 25;   // delay between steps -- bigger = slower/safer

//--------------------------------------------------
// Inverse kinematics math.
// Returns true if reachable; angle outputs are written
// into the reference parameters.
//--------------------------------------------------

bool computeIK(float x, float y, float z, float phiDeg,
               float &outBaseDeg, float &outShoulderDeg,
               float &outElbowDeg, float &outWristPitchDeg)
{
    float phi = radians(phiDeg);

    float theta0 = atan2(y, x);
    float r = sqrt(x * x + y * y);

    float rw = r - IK_L3 * cos(phi);
    float zw = z - IK_L3 * sin(phi);

    float dr = rw;
    float dz = zw - IK_L0;

    float D = sqrt(dr * dr + dz * dz);

    if (D > (IK_L1 + IK_L2) || D < fabs(IK_L1 - IK_L2))
    {
        return false;
    }

    float cosTheta2 = (D * D - IK_L1 * IK_L1 - IK_L2 * IK_L2) / (2 * IK_L1 * IK_L2);
    cosTheta2 = constrain(cosTheta2, -1.0, 1.0);
    float theta2 = acos(cosTheta2);

    float theta1 = atan2(dz, dr) - atan2(IK_L2 * sin(theta2), IK_L1 + IK_L2 * cos(theta2));

    float theta3 = phi - (theta1 + theta2);

    outBaseDeg = degrees(theta0);
    outShoulderDeg = degrees(theta1);
    outElbowDeg = degrees(theta2);
    outWristPitchDeg = degrees(theta3);

    return true;
}

//--------------------------------------------------
// Validate against the REAL, calibrated joint limits.
// Prints a diagnostic for every joint checked, whether
// it passes or fails. Returns false if ANY joint fails,
// or if the target was geometrically unreachable.
//--------------------------------------------------

bool validateIKAgainstLimits(bool reachable, float baseDeg, float shoulderDeg,
                              float elbowDeg, float wristPitchDeg)
{
    if (!reachable)
    {
        Serial.println("REJECTED: target is geometrically unreachable.");
        return false;
    }

    struct { const char* name; float angle; int jointIndex; } checks[] = {
        { "Base",         baseDeg,       BASE },
        { "Shoulder",     shoulderDeg,   SHOULDER },
        { "Elbow",        elbowDeg,      ELBOW },
        { "Wrist Pitch",  wristPitchDeg, WRIST_PITCH },
    };

    bool allOk = true;

    for (int i = 0; i < 4; i++)
    {
        float lo = joints[checks[i].jointIndex].getMinLimit();
        float hi = joints[checks[i].jointIndex].getMaxLimit();
        float angle = checks[i].angle;

        bool ok = (angle >= lo && angle <= hi);

        Serial.print(checks[i].name);
        Serial.print(": ");
        Serial.print(angle);
        Serial.print(ok ? "  OK  " : "  OUT OF RANGE  ");
        Serial.print("(calibrated limits ");
        Serial.print(lo);
        Serial.print(" to ");
        Serial.print(hi);
        Serial.println(")");

        if (!ok)
            allOk = false;
    }

    return allOk;
}

//--------------------------------------------------
// Move slowly from current position to the target,
// interpolating in small steps. Checks Serial for an
// 'x' abort command on every single step -- typing 'x'
// during a move stops it immediately, holding position.
//--------------------------------------------------

void moveToIKTargetSafely(float baseDeg, float shoulderDeg, float elbowDeg, float wristPitchDeg)
{
    float startBase     = joints[BASE].getPosition();
    float startShoulder = joints[SHOULDER].getPosition();
    float startElbow    = joints[ELBOW].getPosition();
    float startWrist    = joints[WRIST_PITCH].getPosition();

    Serial.println("Moving... type 'x' + Enter at any time to abort.");

    for (int step = 1; step <= IK_MOVE_STEPS; step++)
    {
        // Abort check
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

        joints[BASE].setPosition(startBase + t * (baseDeg - startBase));
        joints[SHOULDER].setPosition(startShoulder + t * (shoulderDeg - startShoulder));
        joints[ELBOW].setPosition(startElbow + t * (elbowDeg - startElbow));
        joints[WRIST_PITCH].setPosition(startWrist + t * (wristPitchDeg - startWrist));

        joints[BASE].write();
        joints[SHOULDER].write();
        joints[ELBOW].write();
        joints[WRIST_PITCH].write();

        delay(IK_STEP_DELAY_MS);
    }

    Serial.println("Move complete.");
}

//--------------------------------------------------
// Menu / serial interface
//--------------------------------------------------

void printIKTestMenu()
{
    Serial.println();
    Serial.println("========== IK TEST MODE ==========");
    for (int i = 0; i < NUM_IK_PRESETS; i++)
    {
        Serial.print(i + 1);
        Serial.print(") ");
        Serial.print(ikPresets[i].label);
        Serial.print("  (x=");
        Serial.print(ikPresets[i].x);
        Serial.print(" y=");
        Serial.print(ikPresets[i].y);
        Serial.print(" z=");
        Serial.print(ikPresets[i].z);
        Serial.print(" phi=");
        Serial.print(ikPresets[i].phi);
        Serial.println(")");
    }
    Serial.println("h) Move to home position");
    Serial.println("x) Exit IK Test Mode (no move)");
    Serial.println("===================================");
    Serial.println("Type a number or letter + Enter:");
}

// Call in loop() while currentMode == MODE_NORMAL, so
// typing 'i' enters IK Test Mode at any time.
void ikTestCheckSerialEntry()
{
    while (Serial.available() > 0)
    {
        char c = Serial.read();

        if (c == 'i' || c == 'I')
        {
            currentMode = MODE_IK_TEST;
            printIKTestMenu();
        }
    }
}

// Call in loop() while currentMode == MODE_IK_TEST
void ikTestUpdate()
{
    if (Serial.available() <= 0)
        return;

    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() == 0)
        return;

    if (line == "x" || line == "X")
    {
        Serial.println("Exiting IK Test Mode.");
        currentMode = MODE_NORMAL;
        return;
    }

    if (line == "h" || line == "H")
    {
        Serial.println("Moving to home position...");

        float baseDeg = homePositions[BASE];
        float shoulderDeg = homePositions[SHOULDER];
        float elbowDeg = homePositions[ELBOW];
        float wristPitchDeg = homePositions[WRIST_PITCH];

        // Home positions are, by definition, already within
        // calibrated limits, but we validate anyway to be safe.
        if (validateIKAgainstLimits(true, baseDeg, shoulderDeg, elbowDeg, wristPitchDeg))
        {
            moveToIKTargetSafely(baseDeg, shoulderDeg, elbowDeg, wristPitchDeg);
        }

        printIKTestMenu();
        return;
    }

    int choice = line.toInt();

    if (choice < 1 || choice > NUM_IK_PRESETS)
    {
        Serial.println("Not a valid option.");
        printIKTestMenu();
        return;
    }

    IKPreset preset = ikPresets[choice - 1];

    Serial.println();
    Serial.print("Target: ");
    Serial.println(preset.label);

    float baseDeg, shoulderDeg, elbowDeg, wristPitchDeg;
    bool reachable = computeIK(preset.x, preset.y, preset.z, preset.phi,
                                baseDeg, shoulderDeg, elbowDeg, wristPitchDeg);

    bool ok = validateIKAgainstLimits(reachable, baseDeg, shoulderDeg, elbowDeg, wristPitchDeg);

    if (!ok)
    {
        Serial.println("Move REFUSED -- nothing was sent to the servos.");
        printIKTestMenu();
        return;
    }

    Serial.println("All joints within calibrated limits.");
    Serial.println("Type 'y' + Enter to move, or anything else to cancel:");

    // Block briefly waiting for explicit confirmation
    while (Serial.available() <= 0)
    {
        delay(20);
    }

    String confirm = Serial.readStringUntil('\n');
    confirm.trim();

    if (confirm == "y" || confirm == "Y")
    {
        moveToIKTargetSafely(baseDeg, shoulderDeg, elbowDeg, wristPitchDeg);
    }
    else
    {
        Serial.println("Cancelled -- nothing moved.");
    }

    printIKTestMenu();
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
        checkSerialCalibrationEntry();
        ikTestCheckSerialEntry();

        joints[BASE].update(analogRead(J1_X), J1_X_CENTER);
        joints[SHOULDER].update(analogRead(J1_Y), J1_Y_CENTER);

        joints[ELBOW].update(analogRead(J2_X), J2_X_CENTER);
        joints[WRIST_PITCH].update(analogRead(J2_Y), J2_Y_CENTER);

        joints[WRIST_ROLL].update(analogRead(J3_X), J3_X_CENTER);

        gripperController.update();

        for(int i = 0; i < NUM_JOINTS; i++)
            joints[i].write();
    }
    else if (currentMode == MODE_CALIBRATION)
    {
        calibrationUpdate();
    }
    else if (currentMode == MODE_IK_TEST)
    {
        ikTestUpdate();
    }

    delay(10);
}
