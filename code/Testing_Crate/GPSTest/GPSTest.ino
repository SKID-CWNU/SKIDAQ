const int fan = 3;

void setup() {
  pinMode(fan, OUTPUT);
}

void loop() {
  int fanSpeed = 255;
  analogWrite(fan, fanSpeed);
}