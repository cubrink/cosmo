/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>

const int button1 = 36;


// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0xE9, 0xA2, 0xE8};
//uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0xE9, 0xA2, 0xE8};

uint8_t rcvd_data;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    uint8_t buttons;
} struct_message;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;
struct_message outputData;


// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  pinMode(button1, INPUT);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  uint8_t motor_signals;

  motor_signals = 0b00000000;
  esp_now_send(broadcastAddress, &motor_signals, sizeof(motor_signals));
  delay(10000);

  motor_signals = 0b00000001;
  esp_now_send(broadcastAddress, &motor_signals, sizeof(motor_signals));
  delay(10000);

  motor_signals = 0b00000000;
  esp_now_send(broadcastAddress, &motor_signals, sizeof(motor_signals));
  delay(10000);

  motor_signals = 0b00000010;
  esp_now_send(broadcastAddress, &motor_signals, sizeof(motor_signals));
  delay(10000);
}
