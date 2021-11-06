const int motor1 = 12;
const int motor2 = 14;

void setup() {
  Serial.begin(115200);

  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);

}

void loop() {
  digitalWrite(motor1, HIGH);
  Serial.println("Motor 1 ON");
  delay(10000);
  digitalWrite(motor1, LOW);
  Serial.println("Motor 1 OFF");
  delay(10000);
  
  digitalWrite(motor2, HIGH);
  Serial.println("Motor 2 ON");
  delay(10000);
  digitalWrite(motor2, LOW);
  Serial.println("Motor 2 OFF");
  delay(10000);

}
