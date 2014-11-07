// due.ino
// Quarduino project
// by Stas Zwierzchowski, Daniel Bäck and Martin Nilstomt

bool DEBUG = false; //Should be a preprocessor conditional

#include <SatelliteReceiver.h>
#include <EasyTransfer.h>
#include <Servo.h>

EasyTransfer trans;
SatelliteReceiver rx;

//Each servo
Servo motor[4];
//Current servo speed
byte curSpeed[4];
//Previous servo speed
byte prevSpeed[4];

byte prevCorr[4];

//Gyro start angles (yaw, pitch, roll)
float startAngles[3];

//Current angles but corrected (current angles + start angles)
float correctedAngles[3];
float targetAngles[3];
  
bool gyroDataReceived = false;
bool gyroIsReset = false;

struct RECEIVE_DATA_STRUCTURE {
  float yaw;                //32 bits
  float pitch;              //32 bits
  float roll;               //32 bits
};

RECEIVE_DATA_STRUCTURE data;

void setup(){
  //Debug serial
  Serial.begin (115200);
  //UNO serial
  Serial1.begin(115200);
  //Receiver serial
  Serial2.begin(115200);
  
  //Attach servo:
  motor[0].attach(2);
  motor[1].attach(3);
  motor[2].attach(4);
  motor[3].attach(5);
  
  //Reset all servos
  servoWriteAll(10);
  delay(2000);
  
  //Reset target angles
  targetAngles[0] = 0;
  targetAngles[1] = 0;
  targetAngles[2] = 0;
  
  //Begin receiving gyro data
  trans.begin(details(data), &Serial1);
}

int i = 0;

void loop()
{
  //Get receiver data
  rx.getFrame();
  gyroDataReceived = trans.receiveData();
  
  if (gyroDataReceived == true && gyroIsReset == false)
  {
    startAngles[0] = data.yaw;
    startAngles[1] = data.pitch;
    startAngles[2] = data.roll;  
  }
  
  //Correct angles
  correctedAngles[0] = data.yaw + startAnlges[0];
  correctedAngles[1] = data.pitch + startAnlges[1];
  correctedAngles[2] = data.roll + startAnlges[2];
  //max 1364
  
  //Set all speeds to pitch channel (calibrated to a 10 - 170 value)
  servoSetCurrentSpeed(map(rx.GetPitch(), 0, 1364, 10, 170));
  
  //If the current yaw is not correct
  if (targetAngles[0] != correctedAngles[0])
  {
    float delta = (tagetAngles[0] - correctedAngles[0]) * 2;
    curSpeed[0] -= (delta + prevCorr[0]);
    curSpeed[1] += (delta + prevCorr[0]);
    curSpeed[2] -= (delta + prevCorr[0]);
    curSpeed[3] += (delta + prevCorr[0]);
    
    prevCorr[0] += delta;
  }
  
  if (targetAngles[1] != correctedAngles[1])
  {
     float delta = (tagetAngles[1] - correctedAngles[1]) * 2;
     curSpeed[0] -= (delta + prevCorr[1]);
     curSpeed[1] -= (delta + prevCorr[1]);
     curSpeed[2] += (delta + prevCorr[1]);
     curSpeed[3] += (delta + prevCorr[1]);
     
     prevCorr[1] += delta;
  }
  
  if (targetAngles[2] != correctedAngles[2])
  {
     float delta = (tagetAngles[2] - correctedAngles[2]) * 2;
     curSpeed[0] += (delta + prevCorr[2]);
     curSpeed[1] += (delta + prevCorr[2]);
     curSpeed[2] -= (delta + prevCorr[2]);
     curSpeed[3] -= (delta + prevCorr[2]);
     
     prevCorr[2] += delta;
  }
  
  
  prevSpeed[0] = curSpeed[0];
  prevSpeed[1] = curSpeed[1];
  prevSpeed[2] = curSpeed[2];
  prevSpeed[3] = curSpeed[3];
  servoUpdate();
  
  if (DEBUG == true)
  {
    debug();  
  }
}

void debug()
{
  Serial.print("Gyro:\t");
  if (gyroDataReceived == true)
  {
    Serial.println("---------------");
    Serial.print("y: ");
    Serial.print(data.yaw);
    Serial.print('\t');
    
    Serial.print("p: ");
    Serial.print(data.pitch);
    Serial.print('\t');
    
    Serial.print("r: ");
    Serial.print(data.roll);
    Serial.print('\t');
  }
  
  
  /*
  Serial.print("Reveicer:\t");
  Serial.print(rx.getErrors());
  Serial.print("\t");
  Serial.print(rx.getBindType());
  Serial.print("\t");
  Serial.print(rx.getAile());
  Serial.print("\t");
  Serial.print(rx.getGear());
  Serial.print("\t");
  Serial.print(rx.getPitch());
  Serial.print("\t");
  Serial.print(rx.getElev());
  Serial.print("\t");
  Serial.print(rx.getRudd());
  Serial.println();
  */  
}

void servoUpdate() //Set all servos to their current speed
{
  for (int i = 0; i < 4; i++)
  {
    motor[i].write(curSpeed[i]);
  }  
}

void servoSetCurrentSpeed(byte val)
{
  for (int i = 0; i < 4; i++)
  {
    currentSpeed[i] = val;
  }  
}

void servoWriteAll(int val)
{
  for (int i = 0; i < 4; i++)
  {
    motor[i].write(val);  
  }  
}

