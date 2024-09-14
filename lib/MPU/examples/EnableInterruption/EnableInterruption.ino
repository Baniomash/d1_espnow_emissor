#include <MPU.h>
#include <avr/sleep.h>

#define INTERRUPTION_PIN 2

bool conditionToSleep = false;

MPU motionSensor;

void wake()
{
  sleep_disable();
}

void sleep()
{
  attachInterrupt(digitalPinToInterrupt(INTERRUPTION_PIN), wake, HIGH);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sleep_enable();
  sleep_cpu();
}

void setup()
{
  pinMode(INTERRUPTION_PIN, INPUT);

  motionSensor.initialize();

  Serial.begin(9600);
}

void loop()
{
  if (conditionToSleep == true)
  {
    sleep();

    conditionToSleep = false;
  }
  else
  {
    conditionToSleep = true;
  }

  delay(1000);
}