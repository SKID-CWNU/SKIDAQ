#include <ADCInput.h>

ADCInput RPulse(A0);
// For stereo/dual mikes, could use thi
//   ADCInput(A0, A1);

void setup() {
  Serial.begin(115200);

  RPulse.begin(8000);

  while (1) {
    Serial.printf("%d\n", RPulse.read());
    // For stereo/dual mikes, use this 
    //   Serial.printf("%d %d\n", mike.
  }
}

void loop() {
  /* Nothing here */
}
