#include <math.h>

// Link lengths in mm (same as IK)
const float L0 = 22.0;
const float L1 = 120.0;
const float L2 = 120.0;
const float L3 = 166.0;

struct IKResult
{
    bool reachable;
    float baseDeg;
    float shoulderDeg;
    float elbowDeg;
    float wristPitchDeg;
};

struct FKResult
{
    float x;
    float y;
    float z;
    float phiDeg;   // resulting gripper pitch
};

//--------------------------------------------------
// Inverse Kinematics: (x, y, z, phi) -> joint angles
//--------------------------------------------------

IKResult computeIK(float x, float y, float z, float phiDeg)
{
    IKResult result;
    float phi = radians(phiDeg);

    float theta0 = atan2(y, x);
    float r = sqrt(x * x + y * y);

    float rw = r - L3 * cos(phi);
    float zw = z - L3 * sin(phi);

    float dr = rw;
    float dz = zw - L0;

    float D = sqrt(dr * dr + dz * dz);

    if (D > (L1 + L2) || D < fabs(L1 - L2))
    {
        result.reachable = false;
        return result;
    }

    float cosTheta2 = (D * D - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    cosTheta2 = constrain(cosTheta2, -1.0, 1.0);
    float theta2 = acos(cosTheta2);

    float theta1 = atan2(dz, dr) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));

    float theta3 = phi - (theta1 + theta2);

    result.reachable = true;
    result.baseDeg = degrees(theta0);
    result.shoulderDeg = degrees(theta1);
    result.elbowDeg = degrees(theta2);
    result.wristPitchDeg = degrees(theta3);

    return result;
}

//--------------------------------------------------
// Forward Kinematics: joint angles -> (x, y, z, phi)
//--------------------------------------------------
// This is the reverse walk: start at the base, add each
// link's contribution in order. If IK is correct, running
// FK on IK's own output should return the original target.
//--------------------------------------------------

FKResult computeFK(float baseDeg, float shoulderDeg, float elbowDeg, float wristPitchDeg)
{
    float theta0 = radians(baseDeg);
    float theta1 = radians(shoulderDeg);
    float theta2 = radians(elbowDeg);
    float theta3 = radians(wristPitchDeg);

    // Absolute direction of each link, measured from horizontal
    float link1Angle = theta1;
    float link2Angle = theta1 + theta2;
    float link3Angle = theta1 + theta2 + theta3;   // this is phi

    // Walk the chain in the vertical (radial/height) plane
    float dr = L1 * cos(link1Angle) + L2 * cos(link2Angle);
    float dz = L1 * sin(link1Angle) + L2 * sin(link2Angle);

    float rw = dr;
    float zw = dz + L0;

    float r = rw + L3 * cos(link3Angle);
    float z = zw + L3 * sin(link3Angle);

    FKResult result;
    result.x = r * cos(theta0);
    result.y = r * sin(theta0);
    result.z = z;
    result.phiDeg = degrees(link3Angle);

    return result;
}

//--------------------------------------------------
// Round-trip test: IK -> FK -> compare to original target
//--------------------------------------------------

void testIK(float x, float y, float z, float phiDeg)
{
    Serial.println("----------------------------------------");
    Serial.print("Target: x="); Serial.print(x);
    Serial.print(" y="); Serial.print(y);
    Serial.print(" z="); Serial.print(z);
    Serial.print(" phi="); Serial.println(phiDeg);

    IKResult ik = computeIK(x, y, z, phiDeg);

    if (!ik.reachable)
    {
        Serial.println("-> IK says UNREACHABLE");
        return;
    }

    Serial.print("IK angles -> base="); Serial.print(ik.baseDeg);
    Serial.print(" shoulder="); Serial.print(ik.shoulderDeg);
    Serial.print(" elbow="); Serial.print(ik.elbowDeg);
    Serial.print(" wrist="); Serial.println(ik.wristPitchDeg);

    FKResult fk = computeFK(ik.baseDeg, ik.shoulderDeg, ik.elbowDeg, ik.wristPitchDeg);

    Serial.print("FK check  -> x="); Serial.print(fk.x);
    Serial.print(" y="); Serial.print(fk.y);
    Serial.print(" z="); Serial.print(fk.z);
    Serial.print(" phi="); Serial.println(fk.phiDeg);

    float errX = fabs(fk.x - x);
    float errY = fabs(fk.y - y);
    float errZ = fabs(fk.z - z);
    float errPhi = fabs(fk.phiDeg - phiDeg);

    Serial.print("Error     -> dx="); Serial.print(errX);
    Serial.print(" dy="); Serial.print(errY);
    Serial.print(" dz="); Serial.print(errZ);
    Serial.print(" dphi="); Serial.println(errPhi);

    if (errX < 0.01 && errY < 0.01 && errZ < 0.01 && errPhi < 0.01)
    {
        Serial.println("PASS");
    }
    else
    {
        Serial.println("FAIL - check IK math");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("Running IK/FK round-trip tests...");

    // Straight out horizontally, gripper level
    testIK(300, 0, 22, 0);

    // Reaching down and forward, gripper pointing straight down
    testIK(150, 0, 50, -90);

    // Off to the side
    testIK(100, 150, 100, -45);

    // Close to the base (near-singularity)
    testIK(30, 0, 100, -90);

    // Near max reach
    testIK(400, 0, 22, 0);

    // Deliberately out of reach
    testIK(1000, 0, 22, 0);
}

void loop()
{
}
