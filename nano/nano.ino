#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>
#include <EasyTransfer.h>

float angles[3]; // yaw pitch roll

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
EasyTransfer trans;

struct SEND_DATA_STRUCTURE {
  float yaw;
  float pitch;
  float roll;
};

SEND_DATA_STRUCTURE data;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  trans.begin(details(data), &Serial);

  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
}

void loop() {
  sixDOF.getEuler(angles);
  

  data.yaw = angles[0];
  data.roll = angles[1];
  data.pitch = angles[2];

  trans.sendData();
}

