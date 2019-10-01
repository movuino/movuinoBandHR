
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
}

void loop() {
  // read the analog in value:
  Serial.println(analogRead(38));
  delay(1);
}
