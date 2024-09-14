#include <MPU.h>

short int accelerometerXAxis;
short int accelerometerYAxis;
short int accelerometerZAxis;
short int temperature;
short int gyroscopeXAxis;
short int gyroscopeYAxis;
short int gyroscopeZAxis;

MPU motionSensor;

void setup()
{
  motionSensor.initialize();

  Serial.begin(9600);
}

void loop()
{
  motionSensor.readAccelerometer(&accelerometerXAxis, &accelerometerYAxis, &accelerometerZAxis);

  Serial.println("Accelerometer:");
  Serial.print("X-axis = ");
  Serial.print(accelerometerXAxis);
  Serial.print(" | Y-axis = ");
  Serial.print(accelerometerYAxis);
  Serial.print(" | Z-axis = ");
  Serial.println(accelerometerZAxis);
  Serial.println();

  temperature = motionSensor.readTemperature() / 340.00 + 36.53;

  Serial.println("Temperature:");
  Serial.print("T = ");
  Serial.println(temperature);
  Serial.println();

  motionSensor.readAccelerometer(&gyroscopeXAxis, &gyroscopeYAxis, &gyroscopeZAxis);

  Serial.println("Gyroscope:");
  Serial.print("X-axis = ");
  Serial.print(gyroscopeXAxis);
  Serial.print(" | Y-axis = ");
  Serial.print(gyroscopeYAxis);
  Serial.print(" | Z-axis = ");
  Serial.println(gyroscopeZAxis);
  Serial.println();

  delay(100);
}