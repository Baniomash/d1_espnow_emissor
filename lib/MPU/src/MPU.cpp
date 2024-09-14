#include "MPU.h"
#include <Wire.h>

void MPU::changeRegister(unsigned char address, unsigned char value)
{
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(address);
    Wire.write(value);
    Wire.endTransmission();
}

unsigned char MPU::readRegister(unsigned char address)
{
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();

    Wire.requestFrom(MPU_ADDRESS, 1);
    return Wire.read();
}

void MPU::initialize()
{
    Wire.begin();

    changeRegister(PWR_MGMT_1_REGISTER, 0b00000000);
    changeRegister(PWR_MGMT_2_REGISTER, 0b00000000);
}

void MPU::initialize(char  sdaPin, char  sclPin)
{
    Wire.begin(sdaPin, sclPin);

    changeRegister(PWR_MGMT_1_REGISTER, 0b00000000);
    changeRegister(PWR_MGMT_2_REGISTER, 0b00000000);
}

short int MPU::readAccelerometerXAxis()
{
    return readRegister(ACCEL_XOUT_H_REGISTER) << 8 | readRegister(ACCEL_XOUT_L_REGISTER);
}

short int MPU::readAccelerometerYAxis()
{
    return readRegister(ACCEL_YOUT_H_REGISTER) << 8 | readRegister(ACCEL_YOUT_L_REGISTER);
}

short int MPU::readAccelerometerZAxis()
{
    return readRegister(ACCEL_ZOUT_H_REGISTER) << 8 | readRegister(ACCEL_ZOUT_L_REGISTER);
}

void MPU::readAccelerometer(short int *xAxis, short int *yAxis, short int *zAxis)
{
    *xAxis = readAccelerometerXAxis();
    *yAxis = readAccelerometerYAxis();
    *zAxis = readAccelerometerZAxis();
}

short int MPU::readTemperature()
{
    return readRegister(TEMP_OUT_H_REGISTER) << 8 | readRegister(TEMP_OUT_L_REGISTER);
}

short int MPU::readGyroscopeXAxis()
{
    return readRegister(GYRO_XOUT_H_REGISTER) << 8 | readRegister(GYRO_XOUT_L_REGISTER);
}

short int MPU::readGyroscopeYAxis()
{
    return readRegister(GYRO_YOUT_H_REGISTER) << 8 | readRegister(GYRO_YOUT_L_REGISTER);
}

short int MPU::readGyroscopeZAxis()
{
    return readRegister(GYRO_ZOUT_H_REGISTER) << 8 | readRegister(GYRO_ZOUT_L_REGISTER);
}

void MPU::readGyroscope(short int *xAxis, short int *yAxis, short int *zAxis)
{
    *xAxis = readGyroscopeXAxis();
    *yAxis = readGyroscopeYAxis();
    *zAxis = readGyroscopeZAxis();
}

void MPU::disableAccelerometer()
{
    changeRegister(PWR_MGMT_2_REGISTER, 0b00100000);
}

void MPU::disableTemperature()
{
    changeRegister(PWR_MGMT_1_REGISTER, 0b00001000);
}

void MPU::disableGyroscope()
{
    changeRegister(PWR_MGMT_2_REGISTER, 0b00000111);
}

void MPU::enableInterruption()
{
    changeRegister(MOT_THR_REGISTER, 0b00000001);
    changeRegister(MOT_DUR_REGISTER, 0b00000001);
    changeRegister(INT_ENABLE_REGISTER, 0b01000000);
}
