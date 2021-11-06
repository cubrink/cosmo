const int unknown_button = 12;
const int e_stop = 14;

const int fast_travel = 17;
const int select_button = 4;
const int toggle_mode = 16;

const int speed_up = 15;
const int speed_down = 2;
const int time_speed_up = 26;
const int time_speed_down = 27;

const int joy_left_unknown = 35;
const int joy_left_forward_backward = 34;
const int joy_right_roll = 39;
const int joy_right_pitch = 36; 

const int led_connection_status = 18;
const int led_move_left = 5;
const int led_move_right = 19;



void setup() {
  Serial.begin(115200);

  pinMode(unknown_button, INPUT);
  pinMode(e_stop, INPUT);
  pinMode(fast_travel, INPUT);
  pinMode(select_button, INPUT);
  pinMode(toggle_mode, INPUT);
  pinMode(speed_up, INPUT);
  pinMode(speed_down, INPUT);
  pinMode(time_speed_up, INPUT);
  pinMode(time_speed_down, INPUT);
  pinMode(joy_left_unknown, INPUT);
  pinMode(joy_left_forward_backward, INPUT);
  pinMode(joy_right_roll, INPUT);
  pinMode(joy_right_pitch, INPUT);

  pinMode(led_connection_status, OUTPUT);
  pinMode(led_move_left, OUTPUT);
  pinMode(led_move_right, OUTPUT);
  digitalWrite(led_connection_status, LOW);
  digitalWrite(led_move_left, LOW);
  digitalWrite(led_move_right, LOW);

}

void loop() {
  String output = "";
  output += digitalRead(unknown_button);
  output += digitalRead(e_stop);
  output += digitalRead(fast_travel);
  output += digitalRead(select_button);
  output += digitalRead(toggle_mode);
  output += digitalRead(speed_up);
  output += digitalRead(speed_down);
  output += digitalRead(time_speed_up);
  output += digitalRead(time_speed_down);

  Serial.println(output);


  if(digitalRead(toggle_mode))
  {
    digitalWrite(led_move_left, HIGH);
  }
  else
  {
    digitalWrite(led_move_left, LOW);
  }
  if(digitalRead(select_button))
  {
    digitalWrite(led_connection_status, HIGH);
  }
  else
  {
    digitalWrite(led_connection_status, LOW);
  }
  if(digitalRead(fast_travel))
  {
    digitalWrite(led_move_right, HIGH);
  }
  else
  {
    digitalWrite(led_move_right, LOW);
  }

  
  delay(200);

}
