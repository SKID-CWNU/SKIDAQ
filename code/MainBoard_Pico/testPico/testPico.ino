const int UPS = 14; // UP-Shift Switch
const int DWS = 15; // Down-Shift Switch

void setup() {
    delay(1000);
    Serial.begin(9600);
    pinMode(UPS,INPUT_PULLUP);
    pinMode(DWS,INPUT_PULLUP);
    Serial.print("Shifting Test");
    delay(5000);
}

void loop() {
        Serial.println("Upshift");
        Serial.print(digitalRead(14));
        Serial.print("Downshift");
        Serial.println(digitalRead(15));
        delay(500);
   
}