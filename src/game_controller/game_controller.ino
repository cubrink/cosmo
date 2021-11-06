#include <BleGamepad.h>    // https://github.com/lemmingDev/ESP32-BLE-Gamepad
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_now.h>
#include <WiFi.h>

typedef struct  {
  uint16_t _inline;
  uint16_t _roll;
  uint16_t _pitch;
} analog_state_t;

typedef struct {
  double _pitch;
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

const int NUM_CONTROLLER_BUTTONS = 8;
// Define button pins
const int unknown_button = 12; // Black

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

bool emergency;
BleGamepad gamepad;
Adafruit_MPU6050 mpu;

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
  if (!mpu.begin()) 
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1) 
    {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  ////////////////////////////////////
  //  Configure buttons/joysticks   //
  ////////////////////////////////////
  pinMode(unknown_button, INPUT);
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
  orientation = {0, 0, 0};
  curr_state = 0;
  prev_state = 0;
  changed = 0;

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
  Serial.print(orientation._pitch);
  Serial.print("\t");
  Serial.println(orientation._roll);
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
    int32_t ljoy_y;
    int32_t rjoy_y;
    int32_t rjoy_x;
    
    // Inline movement - nothing special
    if ((analog_state._inline > joy_deadzone_min) && (analog_state._inline < joy_deadzone_max)) {
      // If inside deadzone, set no movement on analog stick
      ljoy_y = 0;
    }
    else {
      // Need to convert from range [0, 4095] to [-32767, 32767]
      ljoy_y = analog_to_gamepad(analog_state._inline);
    }
    
    // Handle roll
    // Constraints on roll:
    //   When the cockpit is currently moving we do NOT want to reverse the direction of motors
    //  If the user indicates a roll in the opposite direction of rotation, then
    //    - Disable the motor so the cockpit stops spinning
    //    - Disable the gamepad rotation signal so the game stops rotating while the cockpit slows down
    //    - Once the cockpit has come to a stop, then accept gamepad signal and turn on motor

    // Assumptions: TODO fix this before mounting
    //  - A clockwise rotation is positive
    if ((analog_state._roll > joy_deadzone_min) && (analog_state._roll < joy_deadzone_max)) {
      // Inside deadzone, send nothing input
      rjoy_x = 0;
    }
    else {
      rjoy_x = analog_to_gamepad(analog_state._roll);
      if ((rjoy_x > 0) && (orientation._omega > 0)) {
        // Motor is turning in same direction as user input
        // continue as normal
        motor_signal = MOTOR_CLOCKWISE;
      }
      if ((rjoy_x < 0) && (orientation._omega < 0)) {
        // Motor is turning in same direction as user input
        // continue as normal
        motor_signal = MOTOR_COUNTERCLOCKWISE;
      }
      if ((rjoy_x < 0) && (orientation._omega > 0)) {
        // User is trying to set direction opposite of motor
        // Disable motor and wait for it to stop
        motor_signal = MOTOR_STOP;
        rjoy_x = 0;
      }
      if ((rjoy_x > 0) && (orientation._omega < 0)) {
        // User is trying to set direction opposite of motor
        // Disable motor and wait for it to stop
        motor_signal = MOTOR_STOP;
        rjoy_x = 0;
      }
      
    }
    
    // Handle pitch
//    if ((analog_state._pitch > joy_deadzone_min) && (analog_state._pitch < joy_deadzone_max)) {
//      // Inside deadzone, send nothing input
//      rjoy_y = 0;
//    }
    



      
    gamepad.setLeftThumb(0, (int16_t)ljoy_y);
    gamepad.setRightThumb((int16_t)rjoy_x, (int16_t)rjoy_y);
    gamepad.sendReport();
    if (motor_signal != MOTOR_IGNORE) {
      esp_now_send(motor_controller_address, &motor_signal, sizeof(motor_signal));
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
      (digitalRead(unknown_button)  << 0)
    | (digitalRead(fast_travel)     << 1)
    | (digitalRead(select_button)   << 2)
    | (digitalRead(toggle_mode)     << 3)
    | (digitalRead(speed_up)        << 4)
    | (digitalRead(speed_down)      << 5)
    | (digitalRead(time_speed_up)   << 6)
    | (digitalRead(time_speed_down) << 7)
  );
}

void update_state() {
  // Update values for continuous sensors
  read_analogs();
  read_orientation();

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
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  float x_curr = a.acceleration.x / 256;
  float y_curr = a.acceleration.y / 256;
  float z_curr = a.acceleration.z / 256;

  // See: https://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
  orientation._pitch = atan(y_curr /  sqrt(pow(x_curr, 2) + pow(z_curr, 2))); // * 180 / PI; // Uncomment for degrees
  // orientation._pitch = atan(y_curr /  sqrt(pow(x_curr, 2) + pow(z_curr, 2))); // * 180 / PI; // Uncomment for degrees
//  orientation._roll = atan(-1 * x_curr / sqrt(pow(y_curr, 2) + pow(z_curr, 2))); // * 180 / PI;
  orientation._roll = atan(-1 * x_curr / z_curr); // * 180 / PI;
  orientation._omega = g.gyro.z; // TODO: Adjust if different when mounting to controller;
 
}

int16_t analog_to_gamepad(uint16_t analog_value) {
  // Joysticks on the gamepad have different value ranges than 
  // what the BLE gamepad library wants. The values need to be 
  // converted
  //  - analog_value is from [0, 4095]
  //  - result value needs to be [-32767, 32767]
  analog_value = analog_state._inline;
  analog_value -= 2048; // Offset so analog_value is 0-centered
  analog_value = (analog_value << 4) - 1;
  return analog_value;
}
