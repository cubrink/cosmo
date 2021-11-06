#include <BleGamepad.h>    // https://github.com/lemmingDev/ESP32-BLE-Gamepad

#define BUTTONPIN 35    // Pin button is attached to

BleGamepad bleGamepad;

int previousButton1State = HIGH;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Beginning setup");
  pinMode(BUTTONPIN, INPUT);
  bleGamepad.begin();
  Serial.println("Setup complete!");
}

void loop() 
{
  int button_value = digitalRead(BUTTONPIN);
  Serial.println(button_value);
  if(bleGamepad.isConnected()) 
  {
    int currentButton1State = digitalRead(BUTTONPIN);
    if (currentButton1State != previousButton1State)
    {
      if(currentButton1State == LOW)
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
    previousButton1State = currentButton1State;
  }
  delay(100);
}
