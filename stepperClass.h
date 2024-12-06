#include "MyAS5600.h"
#include "MySerial.h"

class StepperMotor {
private:
  int stepPin;
  int dirPin;
  int enablePin;

  String motorName = "Some Stepper Name";

  String revsMode = "single";
  int controllerMode = 0;

  AS5600Encoder magEnc;
  int magEnclastMillis = 0;

  short minLim12b;
  short center12b;
  short maxLim12b;

  bool isCalibrated = false;

  float stepResolution;
  float speed;  // Current speed
  float maxSpeed;
  float acc;
  float maxAcc;

  unsigned long lastStepTime = 0;
  bool stepLowed = 0;

  int position;
  int targetPosition;
  short globDistToGo;
  int stepsToGo;
  int stepsToAcc;
  int minLim;
  int maxLim;
  bool dir;

  //float targetAngleDeg;
  //float currentAngle;
  double revsToGoPrev;
  double revsToGo;
  double revsToDecel;

  bool encDir;

  int wir;

  double encRevs;
  //float encAngle;

public:

  StepperMotor(float resolution, int stepPin, int dirPin, int enablePin, float maxSpeed, float maxAcc, int minLim12b, int center12b, int maxLim12b, bool _encDir, int _wir) {
    this->stepPin = stepPin;
    this->dirPin = dirPin;
    this->enablePin = enablePin;
    this->maxSpeed = maxSpeed;
    this->maxAcc = maxAcc;
    this->dir = 0;
    this->stepResolution = resolution;
    this->minLim12b = minLim12b;
    this->center12b = center12b;
    this->maxLim12b = maxLim12b;

    this->encDir = _encDir;
    this->wir = _wir;

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);

    digitalWrite(enablePin, LOW);  // Enable the motor by default
  }

  void begin() {
    magEnc.begin(134873, encDir, wir);
  }

  void calibrate() {
    isCalibrated = false;
  }

  void setName(String _name) {
    motorName = _name;
  }

  void setAcc(float _acc) {
    this->maxAcc = _acc;
  }

  void setSpeed(float speed) {
    if (speed > maxSpeed) {
      this->speed = maxSpeed;
    } else {
      this->speed = speed;
    }
  }

  void setMaxSpeed(float maxSpeed) {
    this->maxSpeed = maxSpeed;
  }

  void move(long steps) {
    targetPosition = position + steps;
  }

  void moveTo(long target, String opt) {
    if (opt == "deg") {
      targetPosition = stepResolution * (target / 360.0);
    } else if (opt == "rad") {
      targetPosition = stepResolution * (target / (2*PI));
    } else if (opt == "raw") {
      targetPosition = target;
    } else {
      throw std::invalid_argument("Invalid option passed as argument");
    }
  }

  void manualStepNB(unsigned long per = 1000) {
    static unsigned long lastStepMicros = 0;
    static bool stepCompleted = 1;

    if (micros() - lastStepMicros >= per * 0.5) {
      if (stepCompleted) {
        digitalWrite(stepPin, HIGH);
        stepCompleted = false;
      } else {
        digitalWrite(stepPin, LOW);
        stepCompleted = true;
      }
      lastStepMicros = micros();
    }
  }

  void step() {
    if (stepsToGo == 0) return;

    if (stepsToGo > 0) {
      position++;
    } else if (stepsToGo < 0) {
      position--;
    }
    if (speed >= 0) {
      dir = 0;
    } else {
      dir = 1;
    }
    digitalWrite(dirPin, dir);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000000 / abs(speed * 2));  // Adjust delay as per your motor specifications
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000000 / abs(speed * 2));  // Adjust delay as per your motor specifications
  }

  void stepNB(unsigned long _syncMicros) {
    if (speed == 0) return;

    unsigned long period = 1000000 / abs((int)speed);

    if (speed >= 0) {
      dir = 0;
    } else {
      dir = 1;
    }

    if (stepLowed) {
      digitalWrite(dirPin, dir);
    }

    unsigned long currentMicros = _syncMicros;

    if (currentMicros - lastStepTime >= period && stepLowed) {
      digitalWrite(stepPin, HIGH);
      lastStepTime = currentMicros;
      stepLowed = 0;
      if (dir == 0) {
        position++;
      } else {
        position--;
      }
    }
    if (currentMicros - lastStepTime >= 25 && !stepLowed) {
      digitalWrite(stepPin, LOW);
      lastStepTime = currentMicros;
      stepLowed = 1;
    }
  }

  void setRevsMode(int mode) {
    switch (mode) {
      case 0:
        revsMode = "single";
        break;
      case 1:
        revsMode = "multiple";
        break;
      default:
        revsMode = "single";
    }
  }

  void setControllerMode(int mode) {
    controllerMode = mode;
  }

  float kp = 5;
  float kd = 10;

  float cskp = 0.2;
  float cskd = 1;

  long lastAccUpdateMicros = 0;

  void update(unsigned long syncMicros) {               // <== res = 134872.9
    //Serial.println("");
    //Serial.println(motorName);
    if (!isCalibrated) {
      //magEnc.updateEvery_ms(1);
      magEnc.updateNB(1000);

      long deltaAccUpdateMicros = syncMicros - lastAccUpdateMicros;
      double kFixed = deltaAccUpdateMicros / 1000000.0;

      short encPos12b = magEnc.getRawPos12b("raw");
      float encSpeed = magEnc.getRawSpeed12b("raw");

      short distToGo = center12b - encPos12b;
      globDistToGo = distToGo;

      acc = 0;
      acc += distToGo * kp;
      acc += encSpeed * (-kd);
      acc = constrain(acc, -maxAcc, maxAcc);
      speed += acc * kFixed;

      short calDirCoeff;

      if(distToGo >= 0) {
        calDirCoeff = 1;
      } else {
        calDirCoeff = -1;
      }

      float speedMod;

      if (abs(distToGo) > 256) {
        speedMod = 0.4;
      } else if (abs(distToGo) > 64) {
        speedMod = 0.2;
      } else {
        speedMod = 0.1;
      }

      speed = calDirCoeff * maxSpeed * speedMod;
      
      lastAccUpdateMicros = syncMicros;
      speed = constrain(speed, -maxSpeed, maxSpeed);
      stepNB(syncMicros);
      if(distToGo == 0) {
        setZeroAndLimits();
      }



    } else if (controllerMode == 0) {  // =============================================== PID stepper control mode
      magEnc.updateEvery_ms(50);

      //encRevs = magEnc.getConvRevs("raw");
      encRevs = magEnc.getConvRevs("raw");
      float  encSpeed = magEnc.getSpeed("raw");

      float targetAngleRevs = (float)targetPosition / stepResolution;
      revsToGo = targetAngleRevs - encRevs;

      //float deltaATG = revsToGo-revsToGoPrev;

      //revsToDecel = (sq(encSpeed) - sq(0)) / (2 * acc * 2000);
      acc = 0;
      acc += revsToGo * kp;
      acc += encSpeed * (-kd);
      speed += acc;

      /*speed += revsToGo * kp;
      speed += encSpeed * (-kd);*/

      speed = constrain(speed, -maxSpeed, maxSpeed);

      //speed = 1000;
      //speed = 100000;  //  <====TEMPORARY====
      //speed = 50000;
      //5000 0.14
      //10000 0.26
      //15000 0.36
      //20000 0.38
      // 0.982Hz  0.37
      //if (abs(revsToGo) > 0.001) {
      stepNB(syncMicros);
      //}
    } else if (controllerMode == 1) {  // =============================================== encoder assisted step counting mode
      magEnc.updateEvery_ms(50);

      encRevs = magEnc.getConvRevs("raw");
      if (revsMode == "multiple") {
      }

      targetPosition = constrain(targetPosition, minLim, maxLim);
      stepsToGo = targetPosition - position;

      //int stepsToAcc = speed * stepsToGo + 0.5 * acc * stepsToGo;
      stepsToAcc = (sq(speed) - sq(0)) / (2 * acc * 2000);

      if (stepsToGo == 0) {
        speed = 0;
      }

      if (stepsToGo >= 0) {
        acc = maxAcc;
      } else if (stepsToGo < 0) {
        acc = -maxAcc;
      }

      if (abs(stepsToGo) > abs(stepsToAcc)) {
        speed += acc;
      }
      if (abs(stepsToGo) <= abs(stepsToAcc)) {
        speed -= acc;
      }

      speed = constrain(speed, -maxSpeed, maxSpeed);
      if (stepsToGo != 0) {
        step();
      }


    } else if (controllerMode == 2) {  // =============================================== classical step counting mode

      magEnc.updateNB(5);

      targetPosition = constrain(targetPosition, minLim, maxLim);
      stepsToGo = targetPosition - position;

      if (stepsToGo != 0) {

        acc = (stepsToG-o > 0) ? maxAcc : -maxAcc;
        long deltaAccUpdateMicros = syncMicros - lastAccUpdateMicros;
        double kFixed = deltaAccUpdateMicros / 1000000.0;
        double stepsToDecel = sq(speed) / (2 * abs(acc));
        double stepsToReachMaxSpeed = sq(maxSpeed) / (2 * abs(maxAcc));

        if (abs(stepsToGo) <= stepsToDecel) {

          speed -= acc * kFixed;
        } else if (abs(stepsToGo) < 2 * stepsToReachMaxSpeed) {

          double peakSpeed = sqrt(2 * abs(acc) * abs(stepsToGo) / 2);
          peakSpeed = constrain(peakSpeed, 0, maxSpeed);

          if (abs(speed) < peakSpeed) {
            speed += acc * kFixed;
          } else {
            speed -= acc * kFixed;
          }
        } else {

          if (abs(stepsToGo) > stepsToDecel) {
            speed += acc * kFixed;
          } else {
            speed -= acc * kFixed;
          }
        }

        if (abs(stepsToGo) <= 1) {
          speed = 0;
          position = targetPosition;
        }
      } else {
        acc = 0;
        speed = 0;
      }
      lastAccUpdateMicros = syncMicros;
      // Ensure the speed is within the allowed range
      speed = constrain(speed, -maxSpeed, maxSpeed);

      // Move the motor by one step based on the calculated speed
      stepNB(syncMicros);  // This function should implement the step movement according to the current speed

      /*
      targetPosition = constrain(targetPosition, minLim, maxLim);
      stepsToGo = targetPosition - position;

      if (stepsToGo != 0) {
        // Determine acceleration direction based on the direction of the remaining steps
        acc = (stepsToGo > 0) ? maxAcc : -maxAcc;
        
        // Calculate the number of steps required to decelerate to a stop
        stepsToAcc = sq(maxSpeed) / (4 * abs(maxAcc));

        // If we are close enough to the target, start decelerating
        long deltaAccUpdateMicros = micros() - lastAccUpdateMicros;
        double kFixed = deltaAccUpdateMicros / 1000000.0;
        
        if(abs(stepsToGo) > stepsToAcc) {
          speed += acc*kFixed;
        }
        else if(abs(stepsToGo) < stepsToAcc){
          speed -= acc*kFixed;
        } 
        lastAccUpdateMicros = micros();
      } 
      else {
        acc = 0;
        speed = 0;
      }

      // Ensure the speed is within the allowed range
      speed = constrain(speed, -maxSpeed, maxSpeed);

      // Move the motor by one step based on the calculated speed
      stepNB();  // This function should implement the step movement according to the current speed
      */


      /*if (stepsToGo == 0) {
        speed = 0;
        acc = 0;
      } else {

        // Determine acceleration direction
        acc = (stepsToGo > 0) ? maxAcc : -maxAcc;

        // Calculate steps to decelerate based on the current speed
        stepsToAcc = sq(speed) / (2 * abs(acc) * 2000);

        // Determine whether to accelerate or decelerate
        int absStepsToGo = abs(stepsToGo);
        int absStepsToDecel = abs(stepsToAcc);

        if (absStepsToGo >= absStepsToDecel) {
          if(speed < maxSpeed) {
            // Accelerate if we have more steps to go than needed for deceleration
            speed += acc;
          }
        } 
        else if (speed != 0) {
          // Decelerate if we're nearing the target
          speed -= (speed > 0) ? abs(acc) : -abs(acc);
        } 
        else {
          // Ensure we start accelerating from zero
          speed = 0;
        }

        // Constrain the speed to within the maximum limits
        speed = constrain(speed, -maxSpeed, maxSpeed);
      }
      stepNB();*/
    }
  }

  double getEncRevsToGo(String opt) {
    if (opt == "deg") {
      return revsToGo * 360;
    } else if (opt == "rad") {
      return revsToGo * 2 * PI;
    } else if (opt == "raw") {
      return revsToGo;
    } else {
      throw std::invalid_argument("Invalid option passed as argument");
    }
  }

  void setZeroAndLimits() {
    position = 0;
    minLim = -stepResolution*((center12b - minLim12b)/4096.0);
    maxLim = stepResolution*((maxLim12b - center12b)/4096.0);
    isCalibrated = true;
  }

  bool calStatus() {
    return isCalibrated;
  }

  long getPosition() {
    return position;
  }

  long getSpeed() {
    return speed;
  }

  long getTargetPosition() {
    return targetPosition;
  }

  long getStepsToGo() {
    return targetPosition - position;
  }

  void printInfo() {
    //char buffer[128];

    //sprintf(buffer, "Dir: %b stepsToGo: %d stepsToAcc: %d speed/max: %s/%s acc/max: %s/%s |", dir, stepsToGo, stepsToAcc, speed, maxSpeed, acc, maxAcc);
    //Serial.print(buffer);
    //Serial.println("");

    printRepeated(" -", 80);

    Serial.println("");
    Serial.print(motorName);
    Serial.print("\t | ");

    Serial.print(" Pos: ");
    Serial.print(position);
    Serial.print(" ");
    Serial.print(" Dir: ");
    Serial.print(dir);
    Serial.print(" ");
    Serial.print(" stepsToGo: ");
    Serial.print(stepsToGo);
    Serial.print(" ");
    Serial.print(" stepsToAcc: ");
    Serial.print(stepsToAcc);
    Serial.print(" ");
    Serial.print(" speed/max: ");
    Serial.print(" ");
    Serial.print(speed);
    Serial.print(" ");
    Serial.print("/");
    Serial.print(maxSpeed);
    Serial.print(" ");

    Serial.print(" acc/max: ");
    Serial.print(acc);
    Serial.print(" ");
    Serial.print("/ ");
    Serial.print(maxAcc);
    Serial.print(" ");

    Serial.print(" minLim/maxLim: ");
    Serial.print(minLim);
    Serial.print("><");
    Serial.print(maxLim);
    Serial.print(" ");

    Serial.print(" | ");
    Serial.println("");

    printRepeated(" ", motorName.length());

    Serial.print("\t | ");
    Serial.print(" MagEnc => ");
    Serial.print(" rawPos: ");
    Serial.print(magEnc.getRawPos12b("raw"));
    Serial.print(" ");
    Serial.print(" rawPosToGoPID: ");
    Serial.print(globDistToGo);
    Serial.print(" ");
    Serial.print(" magEncSpeed: ");
    Serial.print(magEnc.getRawSpeed12b("raw"));
    Serial.print(" ");
    Serial.print(" avgMagEncSpeed: ");
    Serial.print(magEnc.getAvgRawSpeed12b("raw"));
    Serial.print(" ");

    Serial.print(" revsToGo: ");
    Serial.print(getEncRevsToGo("deg"));
    Serial.print(" ");
    Serial.println("");
  }
};