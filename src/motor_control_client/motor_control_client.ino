/*
 * File: motor_control_client.ino
 * Programmer: Jeffrey Phillips
 * 
 * This file receives data from another controller to turn on and off
 * the motors used to turn cosmo
 * 
 */

// MY MAC Address:        94:B9:7E:E9:A2:E8

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

// Relay pin numbers for powering motors
const int motor1 = 12;
const int motor2 = 14;

// Received data
uint8_t rcvd_data;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Receive data and save to rcvd_data
  memcpy(&rcvd_data, incomingData, sizeof(rcvd_data));

  // debug
  Serial.print("Data received: ");
  Serial.println(rcvd_data);

  // if motor1 = 1 and motor2 = 0
  if((rcvd_data & 0b00000001) && !(rcvd_data & 0b00000010))
  {
    // turn on motor1
    Serial.println("Motor 1 ON");
    digitalWrite(motor1, HIGH);
  }
  else
  {
    // turn off motor1
    Serial.println("Motor 1 OFF");
    digitalWrite(motor1, LOW);
  }
  // if motor2 = 1 and motor1 = 0
  if((rcvd_data & 0b00000010) && !(rcvd_data & 0b00000001))
  {
    // turn on motor2
    Serial.println("Motor 2 ON");
    digitalWrite(motor2, HIGH);
  }
  else
  {
    // turn off motor2
    Serial.println("Motor 2 OFF");
    digitalWrite(motor2, LOW);
  }
  // if received data is not a value between 0 and 3
  if(rcvd_data >= 3 || rcvd_data < 0)
  {
    // turn off both motors
    Serial.println("An Error has Occured!");
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, LOW);
    
    // User must power cycle to reset motors
    while(true) 
    {
      delay(50);
    }
  }
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Motor output initialization
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

  Serial.println("Waiting for Data");
  delay(2000);

  

  
}
