void setup() {
  // put your setup code here, to run once:
  pinMode(21, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(21,HIGH);
  delay(7000);
  digitalWrite(21, LOW);
  delay(10000);
}