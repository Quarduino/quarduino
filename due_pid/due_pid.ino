// due.ino
// Quarduino project
// by Stas Zwierzchowski, Daniel Bäck and Martin Nilstomt

#include <SpektrumSattelite.h>
#include <EasyTransfer.h>
#include <Servo.h>
#include "SevSeg.h"
#include <PID_v1.h>

SevSeg ledDisp;
  
EasyTransfer trans;
SpektrumSattelite rx;

//Each servo
Servo motor[4];

//Current servo speed
short curSpeed[4];

//Gyro start angles (yaw, pitch, roll)
double startAngles[3];

double outputValues[3];

//Current angles but corrected (current angles + start angles)
double correctedAngles[4];
double targetAngles[3];

bool gyroDataReceived = false;
bool gyroIsReset = false;

struct RECEIVE_DATA_STRUCTURE {
  float yaw;                //32 bits
  float pitch;              //32 bits
  float roll;               //32 bits
};

RECEIVE_DATA_STRUCTURE data;

//NEW
bool secure = false;
bool firstCheck = false;
bool secondCheck = false;
bool thirdCheck = false;

short prevAux2 = 0;

//TONE
unsigned long prevTime;
unsigned long toneStartTime;
bool tonePin = false;
bool genTone;
bool prevGenTone;
//----

PID pidPitch(&correctedAngles[1], &outputValues[1], &targetAngles[1], 1, 0, 0, DIRECT);
PID pidRoll(&correctedAngles[2], &outputValues[2], &targetAngles[2], 1, 0, 0, DIRECT);



void setup() {
  //Debug serial
  Serial.begin (115200);
  //UNO serial
  Serial1.begin(115200);
  //Receiver serial
  Serial3.begin(115200);

  ledDisp.Begin(0,41,47,49,39,43,51,35,31,29,45,37,33);
  ledDisp.Brightness(100);
  pinMode(11, OUTPUT);

  //Attach servo:
  motor[0].attach(2);
  motor[1].attach(3);
  motor[2].attach(4);
  motor[3].attach(5);

  //Reset all servos
  servoWriteAll(1000);
  delay(2000);

  //Reset target angles
  targetAngles[0] = 0;
  targetAngles[1] = 0;
  targetAngles[2] = 0;
  
  pidRoll.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  
  pidRoll.SetOutputLimits(-500, 500);
  pidPitch.SetOutputLimits(-500, 500);

  //Begin receiving gyro data
  trans.begin(details(data), &Serial1);
  
  genTone=true;
}

void loop()
{
  rx.getFrame();
  duetone();
  ledDisp.PrintOutput();
  if (secure == true)
  {
    if(rx.getTrans() == false)
    {
      halt(); 
    }
    else
    {
      fly();
    } 
  }
  else
  {
    security();
  }
}

void security()
{
  if (firstCheck && secondCheck && thirdCheck)
  {
    Serial.println("Secure");
    genTone = true;
    secure = true;  
    ledDisp.NewNum(0000, 0);
  }
  
  //FIRST CHECK
  else if (firstCheck == false)
  {
    //If transmitter is off
    if (rx.getTrans() == false)
    {
      Serial.println("Transmitter off");
      Serial.println("Turn on trans!");
      ledDisp.NewNum(1111, 0);
      genTone = true;
      firstCheck = true;
    }  
    else 
    {
      halt();  
    }
  }
  
  //SECOND CHECK
  else if (secondCheck == false)
  {
    //If transmitter is on
    if (rx.getTrans() == true)
    {
      Serial.println("Transmitter on");
      secondCheck = true;
    }  
  }
  
  //THIRD CHECK
  else if (thirdCheck == false)
  {
    if (prevAux2 == 0) //Default value
    {
      Serial.println("Waiting for Aux2");
      ledDisp.NewNum(2222, 0);
      genTone = true;
      prevAux2 = rx.getAux2();
    }  
    //If the difference between current and previous value is > 100
    else if (abs(rx.getAux2() - prevAux2) > 100)
    {
      Serial.println("Aux2 changed");
      thirdCheck = true;  
    }
    else
    {
      prevAux2 = rx.getAux2();
    }
  }
}

void fly()
{
  gyroDataReceived = trans.receiveData();
  
  double p = doublemap(rx.getGear(), 0, 1364, 0, 5);
  double i = doublemap(rx.getAux3(), 0, 1364, 0, 2);
 
  if (i < 0) { i = 0; }
  if (p < 0) { p = 0; }
  
  pidRoll.SetTunings(p, 0, 0);     //doublemap(rx.getGear(), 0, 1364, 1, 3)
  pidPitch.SetTunings(p, 0, 0);
  
  //Styr
  targetAngles[1] = doublemap(rx.getElev(), 0, 1364, 20, -20);
  targetAngles[2] = doublemap(rx.getAile(), 0, 1364, 20, -20);
  
  if (targetAngles[1] < 0.5 && targetAngles[1] > -0.5)
  {
    targetAngles[1] = 0;  
  }
  
  if (targetAngles[2] < 0.5 && targetAngles[2] > -0.5)
  {
    targetAngles[2] = 0;  
  }
  
  /*Serial.print(targetAngles[1]);
  Serial.print('\t');
  Serial.print(targetAngles[2]);
  Serial.print('\n');*/
  
  if (gyroDataReceived == true && gyroIsReset == false)
  {
    startAngles[0] = (double)data.yaw;
    startAngles[1] = (double)data.pitch;
    startAngles[2] = (double)data.roll;
    gyroIsReset = true;
    
    Serial.print("GYRO RESET TO ");
    Serial.print(data.yaw);
    Serial.print(", ");
    Serial.print(data.pitch);
    Serial.print(", ");
    Serial.print(data.roll);
    Serial.print('\n');
  }

  //Correct angles
  correctedAngles[0] = (double)data.yaw - startAngles[0];
  correctedAngles[1] = (double)data.pitch - startAngles[1];
  correctedAngles[2] = (double)data.roll - startAngles[2];
  //max 1364

  //Set all speeds to pitch channel (calibrated to a 1000 - 2000 value)
  servoSetCurrentSpeed(map(rx.getThro(), 0, 1364, 1000, 2000));
  
  if (rx.getAux4() > 500)
  {
    stabilize();
    ledDisp.NewNum(abs((int)correctedAngles[1]) + abs((int)correctedAngles[2] * 100), (byte)2);
  }
  else
  {
    //Re-reset gyro
    ledDisp.NewNum(0000, (byte)4);
    gyroIsReset = false;
  }
 
  if (rx.getThro() < 20)
  {
    servoSetCurrentSpeed(1000);
  }

  servoUpdate();
}

void stabilize()
{
  pidRoll.Compute();
  pidPitch.Compute();
  
  curSpeed[0] += (short)outputValues[1];
  curSpeed[1] += (short)outputValues[1];
  curSpeed[2] -= (short)outputValues[1];
  curSpeed[3] -= (short)outputValues[1];
  
  curSpeed[0] += (short)outputValues[2];
  curSpeed[1] -= (short)outputValues[2];
  curSpeed[2] -= (short)outputValues[2];
  curSpeed[3] += (short)outputValues[2];
  
  /*Serial.print(correctedAngles[1]);
  Serial.print('\t');
  Serial.print(correctedAngles[2]);
  Serial.print('\t');
  Serial.print(curSpeed[2]);
  Serial.print('\t');
  Serial.print(curSpeed[3]);
  Serial.print("\t:\t");
  
  Serial.print(outputValues[1]);
  Serial.print('\t');
  Serial.print(outputValues[2]);
  Serial.print('\n');*/
  
}

void servoUpdate() //Set all servos to their current speed
{
  for (int i = 0; i < 4; i++)
  {
    //Serial.print(curSpeed[i]);
    //Serial.print('\t');
    if (curSpeed[i] > 2000)
    {
      curSpeed[i] = 2000;
    }
    else if (curSpeed[i] < 1000)
    {
      curSpeed[i] = 1000;
    }
    
    motor[i].writeMicroseconds(curSpeed[i]);

  }
  //Serial.print('\n');
  
}

void servoSetCurrentSpeed(int val)
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
    motor[i].writeMicroseconds(val);
  }
}

void halt()
{
  ledDisp.NewNum(9999, 0);
  while (true) 
  {
    //Turn of engines
    servoWriteAll(1000);
    Serial.println("HALTING");
    ledDisp.PrintOutput();
    genTone = true;
    duetone();
  } 
}

void duetone()
{
  if (genTone == true)
  {
    if (prevGenTone == false)
    {
      toneStartTime = millis();  
    }
    
    if (micros() - prevTime > 200)
    {
      if (tonePin == false)
      {
        digitalWrite(11, HIGH);
        tonePin = true;
      }  
      else
      {
        digitalWrite(11, LOW);
        tonePin = false;  
      }
      
      prevTime = micros();
    } 
   
    if (millis() - toneStartTime > 1000)
    {
      genTone = false;  
    }
    
  }  
  
  prevGenTone = genTone;
}

double doublemap(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
