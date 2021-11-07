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
