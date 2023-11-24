#include <iostream>
#include <cstring>

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
