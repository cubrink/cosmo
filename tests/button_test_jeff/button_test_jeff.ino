const int button = 36;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(button, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(digitalRead(button));
  delay(50);
}
