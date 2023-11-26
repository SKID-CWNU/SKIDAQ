// Raspberry Pi Pico based Steering Wheel Main Circuit code //
// Solenoid Valve Controlled QuickShift Interface //
// Author: Lim Chae Won //

const int UPS = 18; // UP-Shift Switch IN
const int DWS = 19; // Down-Shift Switch IN
const int US = 14; // Solenoid Valve Out for Upshift
const int DS = 15; // Solenoid Valve Out for Downshift
const int Dyno = 21; // Signal Output to DynoJet Module
int upShift = 1; // Up Shifting State
int downShift = 1; // Down Shifting State

void setup() {
    delay(2000);
    Serial.begin(115200);
    // Set GPIO    
    pinMode(UPS,INPUT_PULLUP);
    pinMode(DWS,INPUT_PULLUP);
    pinMode(US, OUTPUT);
    pinMode(DS, OUTPUT);
    pinMode(Dyno, OUTPUT);
    
    digitalWrite(25, HIGH); // Turn on the Onboard LED for Indcation.
    Serial.print("Shifting Test");
    delay(7000);
    digitalWrite(25, LOW);
}

void loop() {
    upShift = digitalRead(UPS);
    downShift = digitalRead(DWS);
    if (upShift == 0) {
        UpShiftSeq();


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
    int SeqState;
public:
    void UpShiftSeq(RPMread, OG_Gear, END_gear);
    void DownShiftSeq(RPMread, OG_Gear, END_gear);
    void RPMChk();
    void GearChk();
    void EngineChk();

};
void Tachometer::UpShiftSeq(RPMread, OG_Gear, END_gear) {
    Serial.println("UpShift_Seq. Started");
    if (RPMread > MIN_Shift_RPM) {
        digitalWrite(US, HIGH);

    } else if (RPMread =< MIN_Shift_RPM) {
        Serial.println("Error: Upshift failed.");
        Serial.print("  Target: %d to %d. Reason: Low(<200) RPM - %d RPM.", OG_Gear, END_gear, RPMread);
    }
}

void Tachometer::DownShiftSeq(RPMread, OG_Gear, END_gear) {
    
}

