#include <BleGamepad.h>    // https://github.com/lemmingDev/ESP32-BLE-Gamepad

const int joy_left_inline = 35;

BleGamepad gamepad;

void setup() {
  
  Serial.begin(115200);

  ////////////////////////////////
  //     Initialize gamepad     //
  ////////////////////////////////
  gamepad.begin();
  while (!gamepad.isConnected()) {
    Serial.println("Connecting to bluetooth...");
    delay(100);
  };
  Serial.println("Bluetooth connected.");
  pinMode(joy_left_inline, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t analog_value = analogRead(joy_left_inline);
  Serial.println(analog_value);
  analog_value *= 7;
  gamepad.setLeftThumb(0, analog_value);
  gamepad.sendReport();
}
