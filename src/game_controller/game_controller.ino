#include <BleGamepad.h>    // https://github.com/lemmingDev/ESP32-BLE-Gamepad
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include <MPU6050_light.h>

typedef struct  {
  uint16_t _inline;
  uint16_t _roll;
  uint16_t _pitch;
} analog_state_t;

typedef struct {
  double _roll;
  double _omega; // Angular velocity of roll
} orientation_t;

#define MOTOR_STOP             0b00000000
#define MOTOR_COUNTERCLOCKWISE 0b00000001
#define MOTOR_CLOCKWISE        0b00000010
#define MOTOR_IGNORE           0b10101010
#define MOTOR_KILL             0b11111111

const uint8_t motor_controller_address[] = {0x94, 0xB9, 0x7E, 0xE9, 0xA2, 0xE8};

const int e_stop = 14;         // Red

const int NUM_CONTROLLER_BUTTONS = 7;
// Define button pins
const int recalibrate_button = 12; // Black

const int fast_travel = 17;    // Right white
const int select_button = 4;   // Middle white
const int toggle_mode = 16;    // Left white

const int speed_up = 15;       // Green right
const int speed_down = 2;      // Green left 
const int time_speed_up = 26;  // Blue right
const int time_speed_down = 27;// Blue left

const int joy_left_unused = 34;
const int joy_left_inline = 35;
const int joy_right_roll = 36;
const int joy_right_pitch = 39; 

// SDA is GPIO21
// SCL is GPIO22


// Analog sticks normally idles at about 1950
const uint16_t joy_deadzone_min = 1900;
const uint16_t joy_deadzone_max = 2000;

const int led_connection_status = 18;
const int led_move_left = 5;
const int led_move_right = 19;


// Define button states
uint8_t curr_state;
uint8_t prev_state;
uint8_t changed;
analog_state_t analog_state;
orientation_t orientation;

const uint8_t motor_stop_count_threshold = 6;
uint8_t motor_stop_count;
MPU6050 mpu(Wire);
unsigned long timer = 0;
bool emergency;
BleGamepad gamepad;

uint8_t read_buttons();
void update_state();
void read_orientation();
void read_analogs();
int16_t analog_to_gamepad(uint16_t analog_value);


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

  ////////////////////////////////
  // Configure ESP-Now protocol //
  ////////////////////////////////
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, motor_controller_address, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  ////////////////////////////////
  //  Configure accelerometer   //
  ////////////////////////////////
  Wire.begin();
  while(mpu.begin()!=0){ } // stop everything if could not connect to MPU6050
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero


  ////////////////////////////////////
  //  Configure buttons/joysticks   //
  ////////////////////////////////////
  pinMode(recalibrate_button, INPUT);
  pinMode(e_stop, INPUT);
  pinMode(fast_travel, INPUT);
  pinMode(select_button, INPUT);
  pinMode(toggle_mode, INPUT);
  pinMode(speed_up, INPUT);
  pinMode(speed_down, INPUT);
  pinMode(time_speed_up, INPUT);
  pinMode(time_speed_down, INPUT);
  pinMode(joy_left_unused, INPUT);
  pinMode(joy_left_inline, INPUT);
  pinMode(joy_right_roll, INPUT);
  pinMode(joy_right_pitch, INPUT);
  pinMode(led_connection_status, OUTPUT);
  pinMode(led_move_left, OUTPUT);
  pinMode(led_move_right, OUTPUT);
  digitalWrite(led_connection_status, LOW);
  digitalWrite(led_move_left, LOW);
  digitalWrite(led_move_right, LOW);

  // Initialize states to track the sensor values
  analog_state = {0, 0, 0};
  orientation = {0, 0};
  curr_state = 0;
  prev_state = 0;
  changed = 0;

  motor_stop_count = 0;

  // flag set by pressing the e_stop button
  emergency = false;
}

void loop() {
  emergency |= !digitalRead(e_stop); // Set emergency flag if the user presses the e_stop button
                                     // E-stop is active low for safety.
                                     // If the circuit becomes damaged, a low value is read
                                     // which will flag an emergency
  update_state();
  uint8_t motor_signal = MOTOR_STOP;

  if (digitalRead(recalibrate_button)) {
    mpu.calcOffsets();
  }
  
  // Skip while gamepad is not connected or user e-stopped
  if (!emergency && gamepad.isConnected()) {
      // Handle button input
      for (int i = 0; i < NUM_CONTROLLER_BUTTONS; i++) {
          uint8_t button_code = (i+1);
          uint8_t mask = (1 << i);
        
          if (!(changed & mask)) {
            // This button hasn't changed, skip
            continue;
          }
          if (mask & curr_state) {
            // Button has been pressed
            gamepad.press(button_code);
          }
          else {
            // Button has been released
            gamepad.release(button_code);
          }
      }

    // Handle analog inputs
    int32_t surge = 0;
    int32_t roll = 0;
    int32_t pitch = 0;
    int32_t yaw = 0;
    
    // Inline movement - nothing special
    if ((analog_state._inline > joy_deadzone_min) && (analog_state._inline < joy_deadzone_max)) {
      // If inside deadzone, set no movement on analog stick
      surge = 0;
    }
    else {
      // Need to convert from range [0, 4095] to [-32767, 32767]
      surge = analog_to_gamepad(analog_state._inline);
    }
    
    // Handle roll
    // Constraints on roll:
    //   When the cockpit is currently moving we do NOT want to reverse the direction of motors
    //  If the user indicates a roll in the opposite direction of rotation, then
    //    - Disable the motor so the cockpit stops spinning
    //    - Disable the gamepad rotation signal so the game stops rotating while the cockpit slows down
    //    - Once the cockpit has come to a stop, then accept gamepad signal and turn on motor
    
    if ((analog_state._roll > joy_deadzone_min) && (analog_state._roll < joy_deadzone_max)) {
      // Inside deadzone, send nothing input
      roll = 0;
    }
    else {
      // TODO: Our rolling speed should be constant? Will have to adjust this later
      roll = analog_to_gamepad(analog_state._roll);
      float rotation_threshold = 0.2; // Maximum angular velocity before switching direction causes conflict
      float roll_threshold = 10000; // How far the joystick must be moved before we consider rolling
      bool under_rotation_threshold = abs(orientation._omega) <= rotation_threshold;
      bool under_roll_threshold = abs(roll) <= roll_threshold;
      
      if (under_roll_threshold) {
        // User hasn't moved the analog stick enough
        motor_signal = MOTOR_STOP;
        roll = 0;
      }
      else {
        if (roll > 0) {
          motor_signal = MOTOR_CLOCKWISE;
          roll = 32000;
        }
        else if (roll < 0) {
          motor_signal = MOTOR_COUNTERCLOCKWISE;
          roll = -32000;
        }
        else if (roll == 0) {
          motor_signal = MOTOR_STOP;
        }
        if (!under_rotation_threshold) {
          if ((roll < 0) && (orientation._omega > 0)) {
            // User is trying to set direction opposite of motor
            // Disable motor and wait for it to stop
            motor_signal = MOTOR_STOP;
            roll = 0;
          }
          if ((roll > 0) && (orientation._omega < 0)) {
            // User is trying to set direction opposite of motor
            // Disable motor and wait for it to stop
            motor_signal = MOTOR_STOP;
            roll = 0;
          }
        }  
      }
      
      
      
      
    }
    
    // Handle pitch (and yaw, if player is rotated)
    if ((analog_state._pitch > joy_deadzone_min) && (analog_state._pitch < joy_deadzone_max)) {
      // Inside deadzone, send nothing input
      pitch = 0;
      yaw = 0;
    }
    else {
      // Player is rotated inside the cockpit the but the true angle of the screen is not rotated
      // Therefore they have different concepts of 'up'
      // To resolve this, we can use trig to convert the angles
      int16_t analog_magnitude = analog_to_gamepad(analog_state._pitch);
      double theta = orientation._roll * PI / 180;
      yaw = sin(theta) * analog_magnitude;
      pitch = cos(theta) * analog_magnitude;
    }
    

    Serial.print(roll);
    Serial.print("\t");
    Serial.println(orientation._omega);
    gamepad.setLeftThumb((int16_t)roll, (int16_t)surge);
    gamepad.setRightThumb((int16_t)yaw, (int16_t)pitch);
    gamepad.sendReport();
    
    if ((motor_signal == MOTOR_STOP) && (motor_stop_count < motor_stop_count_threshold)) {
      motor_stop_count++;
    }
    else {
      esp_now_send(motor_controller_address, &motor_signal, sizeof(motor_signal));
      if (motor_signal == MOTOR_STOP) {
        motor_stop_count = 0;  
      }
    }
    
  }
  else {
    // Note: We also consider a gamepad disconnect as an emergency stop
    Serial.println("User has pressed the e_stop, quitting");
    uint8_t motor_signal = MOTOR_KILL; // Send error signal
    esp_now_send(motor_controller_address, &motor_signal, sizeof(motor_signal));
  }
  delay(10);
}

uint8_t read_buttons() {
  // Returns an 8 bit value where each bit value represents if a button is pressed
  return (
      (digitalRead(fast_travel)     << 0)
    | (digitalRead(select_button)   << 1)
    | (digitalRead(toggle_mode)     << 2)
    | (digitalRead(speed_up)        << 3)
    | (digitalRead(speed_down)      << 4)
    | (digitalRead(time_speed_up)   << 5)
    | (digitalRead(time_speed_down) << 6)
  );
}

void update_state() {
  // Update values for continuous sensors
  read_analogs();
  read_orientation();
  mpu.update();

  // Sets an 8 bit value where each bit represents if the state of the button has changed.
  prev_state = curr_state;
  curr_state = read_buttons();
  changed = prev_state ^ curr_state; // XOR to check if values have changed.
};

void read_analogs() {
  // Read roll, pitch and inline movement from analog sticks.
  analog_state._inline = analogRead(joy_left_inline);
  analog_state._roll = analogRead(joy_right_roll);
  analog_state._pitch = analogRead(joy_right_pitch);
}

void read_orientation() {
  orientation._roll = mpu.getAngleZ();
  orientation._omega = mpu.getGyroZ(); // TODO: Adjust if different when mounting to controller;
 
}

int16_t analog_to_gamepad(uint16_t analog_value) {
  // Joysticks on the gamepad have different value ranges than 
  // what the BLE gamepad library wants. The values need to be 
  // converted
  //  - analog_value is from [0, 4095]
  //  - result value needs to be [-32767, 32767]
  
  int32_t analog_range = 4095;
  int32_t gamepad_range = ((int32_t)32000 - (int32_t)-1*32000);
  int32_t gamepad_value = (((int32_t)analog_value * gamepad_range) / analog_range) + ((int32_t)-1*32000);
  return (int16_t)gamepad_value;
}
