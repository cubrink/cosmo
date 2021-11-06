const int y_value = 36;
const int x_value = 39;

void setup() {
  Serial.begin(115200);
  
  
}

void loop() {
  Serial.print(analogRead(x_value));
  Serial.print("  ");
  Serial.println(analogRead(y_value));
  delay(100);

}
