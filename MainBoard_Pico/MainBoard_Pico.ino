const int UPS = 14; // UP-Shift Switch
const int DWS = 15; // Down-Shift Switch
int upShift = 1; // Up Shifting State
int downShift = 1; // Down Shifting State

void setup() {
    delay(2000);
    Serial.begin(115200);
    pinMode(UPS,INPUT_PULLUP);
    pinMode(DWS,INPUT_PULLUP);
    Serial.print("Shifting Test");
    delay(7000);
}

void loop() {
    upShift = digitalRead(UPS);
    downShift = digitalRead(DWS);
    if (upShift == 0) {
        Serial.println("upShift");
        
        upShift = 1;
        delay(200);
    }
    if (downShift == 0) {
        Serial.println("downShift");
        downShift = 1;
          delay(200);
    }
    delay(100);
   
}


using namespace std;
class Tachometer {

private:
    enum {
        MIN_Shift_RPM = 500,
        Neutral_1st = 1
    };
    int RPMread;
    int END_gear;
    int OG_Gear;
public:
    void UpShift(RPMread, OG_Gear, END_gear);
    void DownShift(RPMread, OG_Gear, END_gear);
    void RPMChk();
    void GearChk();
    void EngineChk();

};
void Tachometer::UpShift(RPMread, OG_Gear, END_gear) {

}

void Tachometer::DownShift(RPMread, OG_Gear, END_gear) {
    
}

