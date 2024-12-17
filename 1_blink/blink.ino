#define LED 2

void setup(){
  // Set Pin Mode
  pinMode(LED, OUTPUT);
}

void loop(){
  delay(500);
  digitalWrite(LED, 1);
  delay(500);
  digitalWrite(LED, 0);
}