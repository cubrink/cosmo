/*
 * File: motor_control_client.ino
 * Programmer: Jeffrey Phillips
 * 
 * This file receives data from another controller to turn on and off
 * the motors used to turn cosmo
 * 
 */

// MY MAC Address:  94:B9:7E:E9:A2:E8

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

// MAC Address of game controller:  94:B9:7E:E9:91:BC 
uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0xE9, 0x91, 0xBC};

// Relay pin numbers for powering motors
const int motor1 = 12;
const int motor2 = 14;

// Onboard LED indicator
const int onboard_led = 2;

// Received data
uint8_t rcvd_data;

// Connection Status (false = not connected, true = connected)
bool connection_status = false;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Receive data and save to rcvd_data
  memcpy(&rcvd_data, incomingData, sizeof(rcvd_data));

  // debug
  Serial.print("Data received: ");
  Serial.println(rcvd_data);

  // if motor1 = 1 and motor2 = 0
  if(connection_status && ((rcvd_data & 0b00000001) && !(rcvd_data & 0b00000010)))
  {
    // turn on motor1
    digitalWrite(motor1, HIGH);
    //Serial.println("Motor 1 ON");
  }
  else
  {
    // turn off motor1
    digitalWrite(motor1, LOW);
    //Serial.println("Motor 1 OFF");
  }
  // if motor2 = 1 and motor1 = 0
  if(connection_status && ((rcvd_data & 0b00000010) && !(rcvd_data & 0b00000001)))
  {
    // turn on motor2
    digitalWrite(motor2, HIGH);
    //Serial.println("Motor 2 ON");
  }
  else
  {
    // turn off motor2
    digitalWrite(motor2, LOW);
    //Serial.println("Motor 2 OFF");
  }
  // if received data is not a value between 0 and 3
  if(rcvd_data >= 3 || rcvd_data < 0)
  {
    // turn off both motors
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, LOW);
    Serial.println("An Error has Occured!");
    
    // User must power cycle to reset motors
    while(true) 
    {
      delay(50);
    }
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == 0){
    connection_status = true;
    Serial.println("Connected");
  }
  else{
    connection_status = false;
    Serial.println("Connection ERROR");
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

  // LED initialization
  pinMode(onboard_led, OUTPUT);
  digitalWrite(onboard_led, LOW);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for Send CB to get connection status to main controller
  esp_now_register_send_cb(OnDataSent);

  // Register the main game controller (for connection status)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add the controller
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  // Send message via ESP-NOW to verify connection with main controller
  uint8_t to_send = 1;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &to_send, sizeof(to_send));

  // Set LED status
  if(connection_status)
  {
    digitalWrite(onboard_led, HIGH);
  }
  else
  {
    digitalWrite(onboard_led, LOW);
  }
  

  // Request connection status every 500 ms
  delay(500);
}
