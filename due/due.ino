// due.ino
// Quarduino project
// by Stas Zwierzchowski, Daniel Bäck and Martin Nilstomt

bool DEBUG = false; //Should be a preprocessor conditional

#include <SpektrumSattelite.h>
#include <EasyTransfer.h>
#include <Servo.h>

EasyTransfer trans;
SpektrumSattelite rx;

//Each servo
Servo motor[4];
//Current servo speed
short curSpeed[4];
//Previous servo speed
short prevSpeed[4];

float prevCorr[4];

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
  Serial3.begin(115200);
  
  Serial.println("START");
  
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
    gyroIsReset = true;
  }
  
  //Correct angles
  correctedAngles[0] = data.yaw - startAngles[0];
  correctedAngles[1] = data.pitch - startAngles[1];
  correctedAngles[2] = data.roll - startAngles[2];
  //max 1364
  
  //Set all speeds to pitch channel (calibrated to a 10 - 170 value)
  servoSetCurrentSpeed(map(rx.getThro(), 0, 1364, 10, 170));
  
  //If the current yaw is not correct
  /*if (targetAngles[0] - correctedAngles[0] > 1 || targetAngles[0] - correctedAngles[0] < -1)
  {
    float delta = (targetAngles[0] - correctedAngles[0]) * 0.001;
    curSpeed[0] -= (delta + prevCorr[0]);
    curSpeed[1] += (delta + prevCorr[0]);
    curSpeed[2] -= (delta + prevCorr[0]);
    curSpeed[3] += (delta + prevCorr[0]);
    
    prevCorr[0] += delta;
  }*/
  
  if (targetAngles[1] - correctedAngles[1] > 1 || targetAngles[1] - correctedAngles[1] < -1)
  {
     float delta = (targetAngles[1] - correctedAngles[1]) * 0.005;
     curSpeed[0] += (delta + prevCorr[1]);
     curSpeed[1] += (delta + prevCorr[1]);
     curSpeed[2] -= (delta + prevCorr[1]);
     curSpeed[3] -= (delta + prevCorr[1]);
     
     prevCorr[1] += delta;
  }
  
  
  if (targetAngles[2] - correctedAngles[2] > 1 || targetAngles[2] - correctedAngles[2] < -1)
  {
     float delta = (targetAngles[2] - correctedAngles[2]) * 0.0005;
     curSpeed[0] += (delta + prevCorr[2]);
     curSpeed[1] -= (delta + prevCorr[2]);
     curSpeed[2] -= (delta + prevCorr[2]);
     curSpeed[3] += (delta + prevCorr[2]);
     
     prevCorr[2] += delta;
     
    Serial.print("target=");
    Serial.print(targetAngles[2]);
    Serial.print(", corrected=");
    Serial.print(correctedAngles[2]);
    Serial.print(", prev=");
    Serial.print(prevCorr[2]);
    Serial.print(", delta=");
    Serial.print(delta);
    Serial.print(", start2=");
    Serial.print(startAngles[2]);
    Serial.print('\n');
  }

  
  
  prevSpeed[0] = curSpeed[0];
  prevSpeed[1] = curSpeed[1];
  prevSpeed[2] = curSpeed[2];
  prevSpeed[3] = curSpeed[3];
  
  if (rx.getThro() < 20)
  {
    servoSetCurrentSpeed(10);
    prevCorr[0] = 0;
    prevCorr[1] = 0;
    prevCorr[2] = 0;
  }
  
  for (short i = 0; i < 4; i++)
  {
    if (curSpeed[i] < 10)
    {
      curSpeed[i] = 10; 
    }
   
    if (curSpeed[i] > 170)
    {
      curSpeed[i] = 170; 
    } 
  }
  
  servoUpdate();
  
  if (DEBUG == true)
  {
    debug();  
  }
}

void debug()
{
  /*Serial.print("Gyro:\t");
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
  */
  
  /*
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
  Serial.print('\n');*/
  
}

void servoUpdate() //Set all servos to their current speed
{
  for (int i = 0; i < 4; i++)
  {
    motor[i].write(curSpeed[i]);
    
    if (DEBUG == true)
    {
      Serial.print(i);
      Serial.print(":");
      Serial.print(curSpeed[i]); 
      Serial.print('\t');
    }
    
  }  
  
  if (DEBUG == true) {Serial.print(rx.getThro()); Serial.print("\n");}
}

void servoSetCurrentSpeed(byte val)
{
  for (int i = 0; i < 4; i++)
  {
    curSpeed[i] = val;
  }  
}

void servoWriteAll(int val)
{
  for (int i = 0; i < 4; i++)
  {
    motor[i].write(val);  
  }  
}

