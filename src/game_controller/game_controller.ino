#include <BleGamepad.h>    // https://github.com/lemmingDev/ESP32-BLE-Gamepad

typedef struct analog_state_t {
  uint16_t _inline;
  uint16_t _roll;
  uint16_t _pitch;
};

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

bool emergency;
BleGamepad gamepad;

uint8_t read_buttons();
analog_state_t read_analogs();
void update_state();


void setup() {
  Serial.begin(115200);
  
  gamepad.begin();

  // Initialize buttons as inputs/outputs
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

  // Initialize button and analog states
  analog_state = {0, 0, 0};
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
  Serial.println(analog_state._roll);
  if (!emergency && gamepad.isConnected()) {
      // Skip while gamepad is not connected
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
  }
  else {
    // Note: We also consider a gamepad disconnect as an emergency stop
    Serial.println("User has pressed the e_stop, quitting");
    // TODO: Continiously send stop motors message
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
  // Returns an 8 bit value where each bit represents if the state of the button has changed.
  read_analogs();
  prev_state = curr_state;
  curr_state = read_buttons();
  changed = prev_state ^ curr_state; // XOR to check if values have changed.
};

analog_state_t read_analogs() {
  // Read roll, pitch and inline movement from analog sticks.
  analog_state._inline = analogRead(joy_left_inline);
  analog_state._roll = analogRead(joy_right_roll);
  analog_state._pitch = analogRead(joy_right_pitch);
}
