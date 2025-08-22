#define RELAY_PIN_1 25
#define RELAY_PIN_2 26
#define RELAY_PIN_3 27 //GPIO 27

void setup() {
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RELAY_PIN_3, OUTPUT);
}

void loop() {
  digitalWrite(RELAY_PIN_1, HIGH);
  delay(1000);
  digitalWrite(RELAY_PIN_1, LOW);
  delay(1000);
  digitalWrite(RELAY_PIN_2, HIGH);
  delay(2000);
  digitalWrite(RELAY_PIN_2, LOW);
  delay(2000);
  digitalWrite(RELAY_PIN_3, HIGH);
  delay(3000);
  digitalWrite(RELAY_PIN_3, LOW);
  delay(3000);
}
