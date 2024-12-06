#include "Arduino.h"
#include "stepperClass.h"
#include "MyAS5600.h"
#include "HandleCommands.h"

#define dirPin 53
#define stepPin 52
#define enPin 51

#define j1DirPin 33
#define j1StepPin 32
#define j1EnPin 31

#define potBasePin A0
#define potJ1Pin A1

#define J0MagEncAddress 0x36

bool logEnabled = 1;

int potBaseVal = 0;
int potJ1Val = 0;

int potPin = A2;
int potVal = 0;
int val = 0;

struct Command {
  String id;
  double* ptr;
};

int nema23steps = 92304;  // 15:1 * (100/26) * 8 = 11538 * 8 = 92304

int nema17steps = 52720;  // 5.18:1 * (278/44) * 200 * 8= 32.728:1 * 200 * 8= 6545.6 * 8 = 52365;
//                                         6666
StepperMotor base(32.7404375 * 800, stepPin, dirPin, enPin, 2500, 5, 1900, 2603, 3500, 0, 0);  // Example pin numbers and maximum speed/acceleration
//                52685                          20000          355
StepperMotor motor(92304 * 0.5, j1StepPin, j1DirPin, j1EnPin, 30000, 20, 1091, 1290, 2324, 0, 1);  // Example pin numbers and maximum speed/acceleration
//                                                                      180
AS5600Encoder magEnc;  // 329.28 rot 10revs   ==>  329.28/3600
                       // 1348729 encSteps 3600deg ==> 134872.9 = 360deg


// 10 otaček 526850

//  6666 = 1.758 Hz -> 2.01 Hz
// 10000 = 2.965 Hz
// 15000 = 4.35 Hz
// 20000 = 5.36 Hz x
// 25000 = 6.85 Hz
// 30000 = 8.08 Hz
// 35000 = 9.365 Hz
// 45000 = 11.0 Hz
// 55000 = 13.370 Hz
// 70000 = 16.122 Hz
//100000 = 20.35 Hz 
//115000 = 23.400 Hz

//261887 | 261960

void setup() { //==============================================================================SETUP======================================================================
  Serial.begin(460800);
  Serial.setTimeout(0);  // Set timeout to 50 milliseconds

  //115200 11ms
  //230400 11ms
  //460800 7ms + optimized

  base.begin();
  base.setName("Base motor");
  base.setRevsMode(0);
  base.setControllerMode(2);

  motor.begin();
  motor.setName(" J1  motor");
  motor.setRevsMode(0);
  motor.setControllerMode(2);
  //motor.kd = 0.01;

  pinMode(potPin, INPUT);

  //motor.setMaxSpeed(6000); // Set initial speed              map(potVal, 0, 1023, 0, 92304)
  //motor.setSpeed(6000); // Set initial speed
  //motor.setAcc(10); // Set initial speed
  //base.setAcc(20);
  base.setAcc(2000);
  motor.setAcc(2000);
  //motor.moveTo(20000); // Move 1000 steps

  //motor.setZeroAndLimits();

  delay(1000);

  //base.moveToAngleD(90);
  //motor.moveToAngleD(90);
  //motor.moveToAngleD(90);

}

int readPos = 0;

int currMil;


void readHandleCommands(int control = 0) {
  if (Serial.available() > 0) {
    if (control == 0) {

      readPos = Serial.parseInt();  // Read integer value from serial
      Serial.read();                // Discard any remaining character in the buffer

    } 
    else if (control == 1) {

      String serialData = Serial.readString();  // Read the entire line from serial

      String commPart = "";
      String valuePart;

      separateTextAndNumber(serialData, commPart, valuePart);

      Serial.println("");
      Serial.print(" | Command: >");
      Serial.print(commPart);
      Serial.print("<");
      Serial.print(" | Value: >");
      Serial.print(valuePart);
      Serial.print("<");
      Serial.println("");


      if (commPart == "cp") {
        base.kp = valuePart.toFloat();
      } 
      else if (commPart == "cd") {
        base.kd = valuePart.toFloat();
      } 
      else if (commPart == "pos") {
        readPos = valuePart.toInt();
      } 
      else if (commPart == "") {
        readPos = valuePart.toInt();
      }
      else if (commPart == "log") {
        logEnabled = (valuePart.toInt() != 0);
      }
      else if (commPart == "cskp") {
        base.cskp = valuePart.toFloat();
      }
      else if (commPart == "cskd") {
        base.cskd = valuePart.toFloat();
      }
      else if (commPart == "bkp") {
        base.kp = valuePart.toFloat();
      }
      else if (commPart == "bkd") {
        base.kd = valuePart.toFloat();
      }
      else if (commPart == "mkp") {
        motor.kp = valuePart.toFloat();
      }
      else if (commPart == "mkd") {
        motor.kd = valuePart.toFloat();
      }
      else if (commPart == "cal") {
        base.calibrate();
        motor.calibrate();
      }
      else if (commPart == "bzero") {
        base.setZeroAndLimits();
      }
      else if (commPart == "mzero") {
        motor.setZeroAndLimits();
      } //===================Pohyb=============
      else if (commPart == "bpos") {
        base.moveTo(valuePart.toFloat(), "deg");
      }
      else if (commPart == "mpos") {
        motor.moveTo(valuePart.toFloat(), "deg");
      }

      Serial.read();
    }
  }
}



int lastLogMillis = 0;

void loop() { //==============================================================================LOOP======================================================================

  readHandleCommands(1);

  potBaseVal = analogRead(potBasePin);
  potJ1Val = analogRead(potJ1Pin);

  int potBaseAngle = map(analogRead(potBasePin), 0, 1023, 90, -90);
  int potJ1Angle = map(analogRead(potJ1Pin), 500, 1023, 0, 90);

  // 1x 1.17 kHz
  // 6x 1.3


  //base.moveToAngleD(90);

  //base.moveTo(readPos, "deg");
  //motor.moveTo(readPos, "deg");

  //base.moveToAngleD(potBaseAngle);
  //motor.moveToAngleD(potJ1Angle);

  if (millis() % 10 == 0) {
    //base.setSpeed(potVal);
  }

  //base.manualStepNB(readPos);
  unsigned long syncMicros = micros();
  base.update(syncMicros);
  motor.update(syncMicros);  

  if (logEnabled) {
    if (millis() - lastLogMillis >= 100) {
      lastLogMillis = millis();

      /*Serial.println("");
      Serial.print(potBaseVal);
      Serial.print(" / ");
      Serial.print(potJ1Val);*/
      printRepeated(" -", 80);
      Serial.println("");

      Serial.print(logEnabled);

      Serial.println("");

      base.printInfo();
      motor.printInfo();


      /*Serial.print(" potVal: ");
      Serial.print(potVal);
      Serial.print(" limSw: ");
      Serial.print(limSwitch);
      Serial.print(" J1LimSw: ");
      Serial.print(J1LimSwitch);
      Serial.print(" BaseSpeed: ");
      Serial.print(base.getSpeed());*/
      /*Serial.print(" | MagEnc => ");
      Serial.print(" rawPos: ");
      Serial.print(magEnc.getRawPos("raw"));
      Serial.print(" pos: ");
      Serial.print(magEnc.getPos("raw"));
      Serial.print(" sumPos: ");
      Serial.print(magEnc.getSumPos("raw"));
      Serial.print(" sumAngle: ");
      Serial.print(magEnc.getSumPos("deg"));
      Serial.print(" revs: ");
      Serial.print(magEnc.getRevs());
      Serial.print(" ANGLE: ");
      Serial.print(magEnc.getConvRevs("deg"));
      Serial.print(" magEncSpeed: ");
      Serial.print(magEnc.getSpeed("raw"));
      Serial.print(" baseEncSpeed: ");
      Serial.print(base.getEncSpeed("deg"));
      Serial.print(" sum/last: ");
      Serial.print(magEnc.lastSumPosition);
      Serial.print(" / ");
      Serial.print(magEnc.lastSumPosition);
      Serial.print(" revsToGo: ");
      Serial.print(base.getEncRevsToGo("deg"));
      Serial.println("");*/
      Serial.println("$END");
    }
  }
}
