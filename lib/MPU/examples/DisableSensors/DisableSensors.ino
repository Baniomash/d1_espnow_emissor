#include <MPU.h>

MPU motionSensor;

void setup()
{
  motionSensor.initialize();

  motionSensor.disableAccelerometer();
  motionSensor.disableTemperature();
  motionSensor.disableGyroscope();

  Serial.begin(9600);
}

void loop()
{
  
}