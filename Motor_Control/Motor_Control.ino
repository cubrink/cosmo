#include <esp_now.h>
#include <Wire.h>

// MY MAC Address: 
// receiver MAC address
uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0xE9, 0xA2, 0xE8};

uint8_t rcvd_data;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    uint8_t buttons;
} struct_message;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;
struct_message outputData;

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

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
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
}
 
void loop() {

  Serial.println("Waiting for Data");
  delay(1000);
}
