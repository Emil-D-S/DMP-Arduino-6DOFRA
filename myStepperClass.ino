#include "Arduino.h"
#include "stepperClass.h"
#include "MyAS5600.h"
#include "HandleCommands.h"

#define stepPin 22
#define dirPin 24
#define enPin 50

#define j1StepPin 26
#define j1DirPin 28
#define j1EnPin 50

#define j2StepPin 30
#define j2DirPin 32
#define j2EnPin 50

#define j3StepPin 38
#define j3DirPin 40
#define j3EnPin 50

#define j4StepPin 42
#define j4DirPin 44
#define j4EnPin 50

#define j5StepPin 34
#define j5DirPin 36
#define j5EnPin 50

#define potBasePin A0
#define potJ1Pin A1

#define J0MagEncAddress 0x36

bool logEnabled = 0;

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
//StepperMotor base(32.7404375 * 800, stepPin, dirPin, enPin, 2500, 5, 1900, 2603, 3500, 0, 0);  // Example pin numbers and maximum speed/acceleration
//                52685                          20000          355
//StepperMotor motor(92304 * 0.5, j1StepPin, j1DirPin, j1EnPin, 30000, 20, 1091, 1290, 2324, 0, 1);  // Example pin numbers and maximum speed/acceleration
//                                                                      180
AS5600Encoder magEnc;  // 329.28 rot 10revs   ==>  329.28/3600
                       // 1348729 encSteps 3600deg ==> 134872.9 = 360deg


#define NUM_STEPPERS 6  // Maximum number of stepper motors

StepperMotor motors[NUM_STEPPERS] = {
  StepperMotor(32.7404375 * 800, stepPin, dirPin, enPin, 2500, 5, 2636, 3557, 482, 0, 0, 0, 0),     // Channel 0
  StepperMotor(92304 * 0.5, j1StepPin, j1DirPin, j1EnPin, 30000, 20, 940, 1222, 2480, 0, 1, 1, 0),  // Channel 1
  StepperMotor(9400 * 2, j2StepPin, j2DirPin, j2EnPin, 30000, 20, 3697, 440, 1109, 0, 2, 0, 0),     // Channel 1
  StepperMotor(200 * 14*2, j3StepPin, j3DirPin, j3EnPin, 30000, 20, 2350, 3800, 1676, 0, 3, 0, 1),    // Channel 1
  StepperMotor(200 * 14*2*(40.0/35.0), j4StepPin, j4DirPin, j4EnPin, 30000, 20, 998, 1840, 3030, 0, 4, 0, 0),    // Channel 1
  StepperMotor(200 * 14*2*(40.0/35.0), j5StepPin, j5DirPin, j5EnPin, 30000, 20, 521, 2027, 3579, 0, 5, 0, 1),    // Channel 1
  //200 * 14
};
/**/  //XXXXXXXXXXXXXXXXXXXXX

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

void setup() {  //==============================================================================SETUP======================================================================
  Serial.begin(460800);
  Serial.setTimeout(0);  // Set timeout to 50 milliseconds

  //115200 11ms
  //230400 11ms
  //460800 7ms + optimized

  const bool encRed = 0;

  motors[0].begin();
  motors[0].setName("Base motor");
  motors[0].setRevsMode(0);
  motors[0].setControllerMode(2);
  motors[0].setEncReading(encRed);

  motors[1].begin();
  motors[1].setName(" J1  motor");
  motors[1].setRevsMode(0);
  motors[1].setControllerMode(2);
  motors[1].setEncReading(encRed);

  motors[2].begin();
  motors[2].setName(" J2  motor");
  motors[2].setRevsMode(0);
  motors[2].setControllerMode(2);
  motors[2].setEncReading(encRed);

  motors[3].begin();
  motors[3].setName(" J3  motor");
  motors[3].setRevsMode(0);
  motors[3].setControllerMode(2);
  motors[3].setEncReading(encRed);

  motors[4].begin();
  motors[4].setName(" J4  motor");
  motors[4].setRevsMode(0);
  motors[4].setControllerMode(2);
  motors[4].setEncReading(encRed);

  motors[5].begin();
  motors[5].setName(" J5  motor");
  motors[5].setRevsMode(0);
  motors[5].setControllerMode(2);
  motors[5].setEncReading(encRed);


  /**/  //XXXXXXXXXXXXXXXXXXXXX
  pinMode(potPin, INPUT);

  //motor.setMaxSpeed(6000); // Set initial speed              map(potVal, 0, 1023, 0, 92304)
  //motor.setSpeed(6000); // Set initial speed
  //motor.setAcc(10); // Set initial speed
  //base.setAcc(20);

  motors[0].setAcc(2000);
  motors[1].setAcc(2000);
  motors[2].setAcc(2000);
  motors[3].setAcc(500);
  motors[4].setAcc(2000);
  motors[5].setAcc(2000);

  motors[0].setMaxSpeed(6000);
  motors[1].setMaxSpeed(6000);
  motors[2].setMaxSpeed(6000);
  motors[3].setMaxSpeed(6000);
  motors[4].setMaxSpeed(6000);
  motors[5].setMaxSpeed(6000);
  /**/  //XXXXXXXXXXXXXXXXXXXXX
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
      String serialData = Serial.readStringUntil('\n');
      readPos = serialData.toInt();
    } else if (control == 1) {
      String serialData = Serial.readStringUntil('\n');

      String commandPart;
      String valuePart;

      separateTextAndNumber(serialData, commandPart, valuePart);

      // Prepare for splitting valuePart into multiple values if there are multiple
      float values[10];  // Assuming a max of 10 values
      int valueCount = 0;

      // Adjusted splitting logic to handle negative signs
      String tempValue = valuePart;
      while (tempValue.length() > 0) {
        int slashIndex = tempValue.indexOf('/');
        String valueStr = "";
        if (slashIndex != -1) {
          valueStr = tempValue.substring(0, slashIndex);
          tempValue = tempValue.substring(slashIndex + 1);
        } else {
          valueStr = tempValue;
          tempValue = "";
        }

        if (valueStr.length() > 0) {
          values[valueCount++] = valueStr.toFloat();
        }
      }
      // Debugging output
      Serial.println("");
      Serial.print(" | Command: >");
      Serial.print(commandPart);
      Serial.print("<");
      Serial.print(" | Value: >");
      for (int i = 0; i < valueCount; i++) {
        Serial.print(values[i]);
        if (i < valueCount - 1) {
          Serial.print("; ");
        }
      }
      Serial.print("<");
      Serial.println("");

      // Handle different commands with the parsed values
      if (commandPart == "cp") {
        motors[0].kp = values[0];
      } else if (commandPart == "cd") {
        motors[0].kd = values[0];
      } else if (commandPart == "pos") {
        readPos = (valueCount > 0) ? (int)values[0] : 0;  // Handle single value
      } else if (commandPart == "log") {
        logEnabled = (valueCount > 0) ? (values[0] != 0) : false;
      } else if (commandPart == "cskp") {
        motors[0].cskp = values[0];
      } else if (commandPart == "cskd") {
        motors[0].cskd = values[0];
      } else if (commandPart == "bkp") {
        motors[0].kp = values[0];
      } else if (commandPart == "bkd") {
        motors[0].kd = values[0];
      } else if (commandPart == "mkp") {
        motors[1].kp = values[0];
      } else if (commandPart == "mkd") {
        motors[1].kd = values[0];
      } else if (commandPart == "cal") {
        motors[0].calibrate();
        motors[1].calibrate();
        motors[2].calibrate();
        motors[3].calibrate();
        motors[4].calibrate();
        motors[5].calibrate();
        Serial.println("Just calibrated");
      } else if (commandPart == "enc") {
        for (int i = 0; i < NUM_STEPPERS; i++) {
          motors[i].setEncReading((valueCount > 0 && values[0] != 0));
        }
      } else if (commandPart == "bzero") {
        motors[0].setZeroAndLimits();
      } else if (commandPart == "mzero") {
        motors[1].setZeroAndLimits();
      }
      //===================Pohyb=============
      else if (commandPart == "a") {
        if (valueCount > 0) {
          if (valueCount == 2) {
            motors[0].moveTo(values[0], "deg", values[0]);
          } else {
            motors[0].moveTo(values[0], "deg");
          }
        }
      } 
      else if (commandPart == "b") {
        if (valueCount > 0) {
          if (valueCount == 2) {
            motors[1].moveTo(values[0], "deg", values[1]);
          } else {
            motors[1].moveTo(values[0], "deg");
          }
        }
      } 
      else if (commandPart == "c") {
        if (valueCount > 0) {
          if (valueCount == 2) {
            motors[2].moveTo(values[0], "deg", values[2]);
          } else {
            motors[2].moveTo(values[0], "deg");
          }
        }
      } 
      else if (commandPart == "d") {
        if (valueCount > 0) {
          if (valueCount == 2) {
            motors[3].moveTo(values[0], "deg", values[3]);
          } else {
            motors[3].moveTo(values[0], "deg");
          }
        }
      } 
      else if (commandPart == "e") {
        if (valueCount > 0) {
          if (valueCount == 2) {
            motors[4].moveTo(values[0], "deg", values[4]);
          } else {
            motors[4].moveTo(values[0], "deg");
          }
        }
      } 
      else if (commandPart == "f") {
        if (valueCount > 0) {
          if (valueCount == 2) {
            motors[5].moveTo(values[0], "deg", values[5]);
          } else {
            motors[5].moveTo(values[0], "deg");
          }
        }
      } 
      else if (commandPart == "moveto") {
        if (valueCount > 0) {
          for (int i = 0; i < valueCount; i++) {
            motors[i].moveTo(values[i], "deg");
          }
        }
      }

      Serial.read();
    }
  }
}



/*
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
        motors[0].kp = valuePart.toFloat();
      } 
      else if (commPart == "cd") {
        motors[0].kd = valuePart.toFloat();
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
        motors[0].cskp = valuePart.toFloat();
      }
      else if (commPart == "cskd") {
        motors[0].cskd = valuePart.toFloat();
      }
      else if (commPart == "bkp") {
        motors[0].kp = valuePart.toFloat();
      }
      else if (commPart == "bkd") {
        motors[0].kd = valuePart.toFloat();
      }
      else if (commPart == "mkp") {
        motors[1].kp = valuePart.toFloat();
      }
      else if (commPart == "mkd") {
        motors[1].kd = valuePart.toFloat();
      }
      else if (commPart == "cal") {
        motors[0].calibrate();
        motors[1].calibrate();
      }
      else if (commPart == "enc") {
        for(int i = 0; i < NUM_STEPPERS; i++)
        {
          motors[i].setEncReading((valuePart.toInt() != 0));
        }
      }
      else if (commPart == "bzero") {
        motors[0].setZeroAndLimits();
      }
      else if (commPart == "mzero") {
        motors[1].setZeroAndLimits();
      } //===================Pohyb=============
      else if (commPart == "bpos") {
        motors[0].moveTo(valuePart.toFloat(), "deg");
      }
      else if (commPart == "mapos") {
        motors[1].moveTo(valuePart.toFloat(), "deg");
      }
      else if (commPart == "mbpos") {
        motors[2].moveTo(valuePart.toFloat(), "deg");
      }

      Serial.read();
    }
  }
}
*/
/**/  //XXXXXXXXXXXXXXXXXXXXX


int lastLogMillis = 0;

void loop() {  //==============================================================================LOOP======================================================================

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
  motors[0].update(syncMicros);
  motors[1].update(syncMicros);
  motors[2].update(syncMicros);
  motors[3].update(syncMicros);
  motors[4].update(syncMicros);
  motors[5].update(syncMicros);
  /**/  //XXXXXXXXXXXXXXXXXXXXX

  //logEnabled = 0;
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


      motors[0].printInfo();
      motors[1].printInfo();
      motors[2].printInfo();
      motors[3].printInfo();
      motors[4].printInfo();
      motors[5].printInfo();
      /**/  //XXXXXXXXXXXXXXXX

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
