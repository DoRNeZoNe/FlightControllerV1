/*
 * MadgwickFilter.cpp
 *
 *  Created on: Mar 12, 2023
 *      Author: Vipul Gupta
 */

#include <MadgwickFilter.h>

MadgwickFilter::MadgwickFilter(dzLogger *logger)
{
    this->logger = logger;
}

// Madgwick function needs to be fed North, East, and Down direction like
// (AN, AE, AD, GN, GE, GD, MN, ME, MD)
// Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
// Magneto direction is Right-Hand, Y-Forward, Z-Down
// So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
// we need to feed (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
// but we pass (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
// because gravity is by convention positive down, we need to ivnert the accel data

// get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
// acc[mg], gyro[deg/s], mag [mG]
// gyro will be convert from [deg/s] to [rad/s] inside of this function
void MadgwickFilter::updateRollPitchYaw(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float magX, float magY, float magZ)
{
    // Updating values as per Magdwick Filter
    this->ax = accX * -1;
    this->ay = accY;
    this->az = accZ;
    this->gx = gyroX * DEG_TO_RAD;
    this->gy = gyroY * DEG_TO_RAD * -1;
    this->gz= gyroZ * DEG_TO_RAD * -1;
    this->mx = magY;
    this->my = magX * -1;
    this->mz = magZ;
    MadgwickQuaternionUpdate();
    quaternionToRollPitchYaw();
    char stringData[50];
    sprintf(stringData, "X => %f | Y => %f | Z => %f\r\n", roll, pitch, yaw);
    this->logger->logViaUart(stringData);
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickFilter::MadgwickQuaternionUpdate()
{
    this->deltaT = (float)fabs((HAL_GetTick() - this->previousMillis) * 0.001);
    this->previousMillis = HAL_GetTick();

    double q0 = quaternion[0], q1 = quaternion[1], q2 = quaternion[2], q3 = quaternion[3]; // short name local variable for readability
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double hx, hy;
    double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Normalise accelerometer measurement
    double a_norm = ax * ax + ay * ay + az * az;
    if (a_norm == 0.)
        return; // handle NaN
    recipNorm = 1.0 / sqrt(a_norm);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    double m_norm = mx * mx + my * my + mz * mz;
    if (m_norm == 0.)
        return; // handle NaN
    recipNorm = 1.0 / sqrt(m_norm);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * deltaT;
    q1 += qDot2 * deltaT;
    q2 += qDot3 * deltaT;
    q3 += qDot4 * deltaT;

    // Normalise quaternion
    recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    quaternion[0] = q0;
    quaternion[1] = q1;
    quaternion[2] = q2;
    quaternion[3] = q3;
}

void MadgwickFilter::quaternionToRollPitchYaw()
{
    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    // rotation matrix coefficients for Euler angles and gravity components
    float a12, a22, a31, a32, a33;

    a12 = 2.0f * (this->quaternion[1] * this->quaternion[2] + this->quaternion[0] * this->quaternion[3]);
    a22 = this->quaternion[0] * this->quaternion[0] + this->quaternion[1] * this->quaternion[1] - this->quaternion[2] * this->quaternion[2] - this->quaternion[3] * this->quaternion[3];
    a31 = 2.0f * (this->quaternion[0] * this->quaternion[1] + this->quaternion[2] * this->quaternion[3]);
    a32 = 2.0f * (this->quaternion[1] * this->quaternion[3] - this->quaternion[0] * this->quaternion[2]);
    a33 = this->quaternion[0] * this->quaternion[0] - this->quaternion[1] * this->quaternion[1] - this->quaternion[2] * this->quaternion[2] + this->quaternion[3] * this->quaternion[3];
    roll = atan2f(a31, a33);
    pitch = -asinf(a32);
    yaw = atan2f(a12, a22);
    roll *= 180.0f / DZ_PI;
    pitch *= 180.0f / DZ_PI;
    yaw *= 180.0f / DZ_PI;
    //    yaw[2] += magnetic_declination;

    if (yaw >= 180.f)
    {
        yaw -= 360.f;
    }
    else if (yaw < -180.f)
    {
        yaw += 360.f;
    }

}
