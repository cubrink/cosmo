/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <BleGamepad.h>    // https://github.com/lemmingDev/ESP32-BLE-Gamepad
#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>


BleGamepad bleGamepad;

int previousButton1State = HIGH;

const int button1 = 36;

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0xE9, 0xA2, 0xE8};

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

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  rcvd_data = incomingReadings.buttons;
  Serial.print("Data received: ");
  Serial.println(rcvd_data);
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial.println("Beginning setup");
  pinMode(button1, INPUT);
  bleGamepad.begin();

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
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Setup complete!");
}
 
void loop() {
  outputData.buttons = digitalRead(button1);

  Serial.println(outputData.buttons);
  if(bleGamepad.isConnected()) 
  {
    outputData.buttons = digitalRead(button1);
    if (outputData.buttons != previousButton1State)
    {
      if(outputData.buttons == LOW)
      {
        bleGamepad.press(BUTTON_1);
        Serial.println("Pressed");
      }
      else
      {
        bleGamepad.release(BUTTON_1);
        Serial.println("Depressed :(");
      }
    }
    else {
      Serial.println("Not changed");
    }
    previousButton1State = outputData.buttons;
  }
  else {
    Serial.println("Gamepad not connected");
  }
  
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outputData, sizeof(outputData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(100);
}
