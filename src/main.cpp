#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <MPU.h>
#include "gpio.h"
#include "user_interface.h"

#define INTERRUPTION_PIN 5 // D1 = 5; D6 = 12; TX = 1
#define SDA_PIN 0 // D4
#define SCL_PIN 2 // D5

#define NEUTRAL 0
#define UP 1
#define RIGHT 2
#define DOWN 3
#define LEFT 4
#define SLEEP 5

MPU motionSensor;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct struct_message {
  uint8_t move;
} struct_message;

struct_message myData;

unsigned long lastTime = 0;  
unsigned long timerDelay = 2000;

// void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
//   Serial.print("Last Packet Send Status: ");
//   if (sendStatus == 0){
//     Serial.println("Delivery success");
//   }
//   else{
//     Serial.println("Delivery fail");
//   }
// }


void wait(unsigned long milliseconds)
{
  unsigned long currentTime = millis();
  unsigned long previousTime = millis();

  while (currentTime - previousTime <= milliseconds)
  {
    currentTime = millis();
  }
}

// void IRAM_ATTR wake() {
//   Serial.println("Waking up!");
//   wait(200);
// }

// void light_sleep(){
//   wifi_set_opmode(NULL_MODE);
//   wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
//   wifi_fpm_open();
//   gpio_pin_wakeup_enable(GPIO_ID_PIN(INTERRUPTION_PIN), GPIO_PIN_INTR_LOLEVEL);
//   wifi_fpm_do_sleep(0xFFFFFFF);
//  }

void deep_sleep(){
  // attachInterrupt(digitalPinToInterrupt(INTERRUPTION_PIN), wake, HIGH);
  // esp_deep_sleep_start();
  ESP.deepSleep(0);
}


unsigned char verifyMovement()
{
  short int xAxis = 0;
  short int yAxis = 0;
  short int zAxis = 0;

  motionSensor.readAccelerometer(&xAxis, &yAxis, &zAxis);
  
  if (yAxis < -5000)
  {
    Serial.println("Cima");
    return UP;
  }
  else if (zAxis > 5000)
  {
    Serial.println("Direita");
    return RIGHT;
  }
  else if (yAxis > 5000)
  {
    if (yAxis < 15000)
    {
      Serial.println("Baixo");
      return DOWN;
    }
    else
    {
      Serial.println("Sleep!");
      return SLEEP;
    }
  }
  else if (zAxis < -5000)
  {
    Serial.println("Esquerda");
    return LEFT;
  }
  else
  {
    return NEUTRAL;
  }
}

void setup() {
  // pinMode(INTERRUPTION_PIN, INPUT);
  
  gpio_init();

  motionSensor.initialize(SDA_PIN, SCL_PIN);

  motionSensor.disableTemperature();

  motionSensor.enableInterruption();

  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

}
 
void loop() {
  unsigned long currentTime = 0;
  unsigned long previousTime = 0;

  unsigned char movementPerformed = NEUTRAL;
  unsigned char movementsAmount = 0;
  unsigned short int movementsSequence = 0;

  while (currentTime - previousTime <= 1000)
  {
    if (movementsAmount == 3)
    {
      myData.move = movementsSequence;
      esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      Serial.println("Enviado!");

      movementsSequence -= movementPerformed;
      movementsSequence /= 10;

      movementsAmount = 2;
      wait(100);
    }
    else
    {
      movementPerformed = verifyMovement();
      
      if (movementPerformed != NEUTRAL)
      {
        if (movementPerformed == SLEEP)
        {
          deep_sleep();
        }
        else
        {
          if (movementsAmount >= 1)
          {
            movementsSequence = movementsSequence * 10;
          }

          movementsSequence = movementsSequence + movementPerformed;

          movementsAmount++;

          previousTime = millis();

          wait(250);
        }
      }
    }

    currentTime = millis();
  }
}