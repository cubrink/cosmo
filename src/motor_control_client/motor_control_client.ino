#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

const int motor1 = 12;
const int motor2 = 14;

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
  
  if((rcvd_data & 0b00000001) && !(rcvd_data & 0b00000010))
  {
    Serial.println("Motor 1 ON");
    digitalWrite(motor1, HIGH);
  }
  else
  {
    Serial.println("Motor 1 OFF");
    digitalWrite(motor1, LOW);
  }
  if((rcvd_data & 0b00000010) && !(rcvd_data & 0b00000001))
  {
    Serial.println("Motor 2 ON");
    digitalWrite(motor2, HIGH);
  }
  else
  {
    Serial.println("Motor 2 OFF");
    digitalWrite(motor2, LOW);
  }
  if(rcvd_data >= 3)
  {
    Serial.printlnt("An Error has Occured!");
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, LOW);
    // User must power cycle to reset motors
    while(true) 
    {
      delay(50)
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
  delay(2000);

  

  
}
