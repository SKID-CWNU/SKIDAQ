// Raspberry Pi Pico based Steering Wheel Main Circuit code //
// Solenoid Valve Controlled QuickShift Interface //
// Author: Lim Chae Won //

const int UPS = 18;  // UP-Shift Switch IN
const int DWS = 19;  // Down-Shift Switch IN
const int US = 14;   // Solenoid Valve Out for Upshift
const int DS = 15;   // Solenoid Valve Out for Downshift
const int Dyno = 21; // Signal Output to DynoJet Module
int TachoPin = 26;   // Tachometer Input with ADC
int upShift = 1;     // Up Shifting State
int downShift = 1;   // Down Shifting State

using namespace std;
class Tachometer
{

private:
    enum
    {
        MIN_Shift_RPM = 500
    };
    int Neutral_State;
    int RPMread;
    int SeqState;

public:
    void UpShiftSeq();
    void DownShiftSeq();
    int RPMChk();
    void Netrual();
};

int Tachometer::RPMChk()
{
    RPMread = analogRead(TachoPin);
    return RPMread;
}

void Tachometer::Netrual() {
    digitalWrite(downShift, HIGH);
}

void Tachometer::UpShiftSeq()
{
    Serial.println("UpShift_Seq. Started");
    // Tachometer::RPMChk();
    RPMread = 600; // Only for Testing
    if (RPMread > MIN_Shift_RPM)
    {
        digitalWrite(US, HIGH);
        delay(30);
        digitalWrite(US, LOW);
    }
    else if (RPMread < MIN_Shift_RPM)
    {
        Serial.println("Error: Upshift failed.");
        Serial.println(" Reason: Low(<200) RPM -");
        Serial.print(RPMread);
        Serial.print("RPM.");
    }
}

void Tachometer::DownShiftSeq()
{
    Serial.println("DownShift_Seq. Started");
    // Tachometer::RPMChk();
    RPMread = 600; // Only for Testing
    if (RPMread > MIN_Shift_RPM)
    {
        digitalWrite(DS, HIGH);
        delay(100);
        digitalWrite(DS, LOW);
    }
    else if (RPMread < MIN_Shift_RPM)
    {
        Serial.println("Error: Downshift failed.");
        Serial.println(" Reason: Low(<200) RPM -");
        Serial.print(RPMread);
        Serial.print("RPM.");
    }
}

Tachometer cbr600rr;

void setup()
{
    // Set GPIO
    pinMode(TachoPin, INPUT);
    pinMode(UPS, INPUT_PULLUP); // Connection: GND & GPIO (as Internal Pullup Resistor is enabled.)
    pinMode(DWS, INPUT_PULLUP);
    pinMode(US, OUTPUT); // Goes to the Relay
    pinMode(DS, OUTPUT);
    pinMode(Dyno, OUTPUT);

    // Startup Sequence
    delay(2000);
    Serial.begin(115200);
    digitalWrite(25, HIGH); // Turn on the Onboard LED for Indcation.
    Serial.print("Shifting Test");
    delay(7000);
    digitalWrite(25, LOW);
}
void loop()
{
    upShift = digitalRead(UPS);
    downShift = digitalRead(DWS);
    if (upShift == 0)
    {
        cbr600rr.UpShiftSeq();
        delay(200);
    }
    if (downShift == 0)
    {
        cbr600rr.DownShiftSeq();
        delay(200);
    }
    delay(100);
}