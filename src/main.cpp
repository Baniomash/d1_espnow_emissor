#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

int data = 123;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  if(esp_now_init() != 0){
    Serial.println("ESP-NOW initialization failed.");
    while (true);
  }
}

void loop() {
  esp_now_send(broadcastAddress, (uint8_t *)&data, sizeof(data));
  delay(2000);
}