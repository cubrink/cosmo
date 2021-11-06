/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/


//#include <BleGamepad.h>    // https://github.com/lemmingDev/ESP32-BLE-Gamepad
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <String.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


//BleGamepad bleGamepad;
uint16_t prev_value = HIGH;

const int button = 36;

BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Server!");
//  bleGamepad.begin();
  pinMode(button, INPUT);

  BLEDevice::init("ESP32-BLE-Server");
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setValue("Hello World says Neil");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  uint16_t my_value = digitalRead(button);
  Serial.println(my_value);
  pCharacteristic->setValue(my_value);
//  
//  if(bleGamepad.isConnected()) 
//  {
//    if (my_value != prev_value)
//    {
//      if(my_value == LOW)
//      {
//        bleGamepad.press(BUTTON_1);
//        Serial.println("Pressed");
//      }
//      else
//      {
//        bleGamepad.release(BUTTON_1);
//        Serial.println("Depressed :(");
//      }
//    }
//    prev_value = my_value;
//  }
  delay(100);
}
