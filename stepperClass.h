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
  double speed;  // Current speed
  float maxSpeed;
  double acc;
  float maxAcc;

  unsigned long lastStepTime = 0;
  bool stepLowed = 0;

  int position;
  int targetPosition;
  short globDistToGo;
  int stepsToGo;
  int stepsToAcc;
  double stepsToDecel;
  int minLim;
  int maxLim;
  bool dir;

  unsigned long ttm;
  //short avgStartPos12b = [16];

  //float targetAngleDeg;
  //float currentAngle;
  double revsToGoPrev;
  double revsToGo;
  double revsToDecel;

  static const short calEncSamples = 16;
  short calEncPositions[calEncSamples];

  bool encDir;
  bool swapDir;
  bool encReading = false;
  bool ignoreLimits = false;

  int wir;

  double encRevs;

  uint8_t tcaChannel; // TCA9548A multiplexer channel

public:

  StepperMotor(float resolution, int stepPin, int dirPin, int enablePin, float maxSpeed, float maxAcc, int minLim12b, int center12b, int maxLim12b, bool _encDir, uint8_t channel, bool swapDir, bool ignoreLimits = 0) {
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
    this->tcaChannel = channel;
    this->encDir = _encDir;
    this->swapDir = swapDir;
    this->ignoreLimits = ignoreLimits;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);

    digitalWrite(enablePin, LOW);  // Enable the motor by default
  }

  void begin() {
    magEnc.begin(134873, tcaChannel); // Pass TCA channel to encoder initialization
  }

  void updateEncoder() {
    magEnc.readRawPos(); // Read encoder data, now automatically handles TCA switching
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

  void setEncReading(bool enr = 0)
  {
    encReading = enr;
  }

  void setMaxSpeed(float maxSpeed) {
    this->maxSpeed = maxSpeed;
  }

  void move(long steps) {
    targetPosition = position + steps;
  }

  void moveTo(long target, String opt, unsigned long dt_millis = 0) {
    if (opt == "deg") {
      targetPosition = stepResolution * (target / 360.0);
    } else if (opt == "rad") {
      targetPosition = stepResolution * (target / (2*PI));
    } else if (opt == "raw") {
      targetPosition = target;
    } else {
      throw std::invalid_argument("Invalid option passed as argument");
    }
    if(dt_millis > 0)
    {
      ttm = dt_millis;
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

    unsigned long period = (int)(1000000 / abs(speed));

    if (speed >= 0) {
      dir = 0;
    } else {
      dir = 1;
    }

    if (stepLowed) {
      if(swapDir) {
        digitalWrite(dirPin, !dir);
      } else {
        digitalWrite(dirPin, dir);
      }
      
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

int getSignedDistanceWithoutCrossingLimits(int x, int minLim, int centerPos, int maxLim) {
  // Normalize all positions to 0-4095 range
  x = x & 0xFFF;
  minLim = minLim & 0xFFF;
  centerPos = centerPos & 0xFFF;
  maxLim = maxLim & 0xFFF;
  
  // Calculate direct distance (positive and negative)
  int directDist = centerPos - x;
  int wrappedDist;
  
  // If direct distance is positive, wrapped distance should be negative
  if (directDist >= 0) {
    wrappedDist = -(4096 - directDist);
  } else {
    // If direct distance is negative, wrapped distance should be positive
    wrappedDist = 4096 + directDist;
  }
  
  // Check if x and centerPos are on different sides of a limit
  bool crossesMinLim = false;
  bool crossesMaxLim = false;
  
  // Helper function to check if a point is between two positions (inclusive)
  auto isBetween = [](int value, int start, int end) {
    if (start <= end) {
      return value >= start && value <= end;
    } else {
      return value >= start || value <= end;
    }
  };
  
  // Check if path crosses limits
  if (directDist >= 0) {
    // Moving clockwise from x to centerPos
    crossesMinLim = isBetween(minLim, x, centerPos);
    crossesMaxLim = isBetween(maxLim, x, centerPos);
  } else {
    // Moving counterclockwise from x to centerPos
    crossesMinLim = isBetween(minLim, centerPos, x);
    crossesMaxLim = isBetween(maxLim, centerPos, x);
  }
  
  // If both limits are crossed, use the path through the closer limit
  if (crossesMinLim && crossesMaxLim) {
    // Calculate distances to both limits from x
    int distToMin = (x >= minLim) ? x - minLim : x + (4096 - minLim);
    int distToMax = (x >= maxLim) ? x - maxLim : x + (4096 - maxLim);
    
    // Use the path through the closer limit
    return (distToMin < distToMax) ? wrappedDist : directDist;
  }
  // If only one limit is crossed, use the path that doesn't cross it
  else if (crossesMinLim || crossesMaxLim) {
    return crossesMinLim ? wrappedDist : directDist;
  }
  // If no limits are crossed, use the shorter path
  else {
    return abs(directDist) <= abs(wrappedDist) ? directDist : wrappedDist;
  }
}





/*
int getSignedDistanceWithoutCrossingLimits(int x, int minLim, int centerPos, int maxLim) {
  // Normalize positions to be within the valid range
  x = (x + 4095 + 1) % (4095 + 1);
  centerPos = (centerPos + 4095 + 1) % (4095 + 1);
  minLim = (minLim + 4095 + 1) % (4095 + 1);
  maxLim = (maxLim + 4095 + 1) % (4095 + 1);

  // Check if x is within the valid range
  if (minLim <= maxLim) {
    if (x < minLim || x > maxLim) {
      // Outside the range, calculate distance to the closest limit
      int distanceToMin = (x < minLim) ? minLim - x: minLim + 4096 - x;
      int distanceToMax = (x > maxLim) ? x - maxLim : x + 4096 - maxLim
      //int closestLimit = (distanceToMin < distanceToMax) ? minLim : maxLim;
      if (distanceToMax > distanceToMin)
      {
        return centerPos - x;
      }
      else
      {

      }
      return (centerPos >= closestLimit) ? centerPos - closestLimit : -(closestLimit - centerPos);
    } else if (centerPos < minLim || centerPos > maxLim) {
      return 0; // Invalid center position
    }
    return centerPos - x;
  } else {
    // Wraparound case
    bool xInValidRange = (x >= minLim || x <= maxLim);
    bool centerInValidRange = (centerPos >= minLim || centerPos <= maxLim);

    if (!xInValidRange) {
      // Outside the range, calculate distance to the closest limit
      int distanceToMin = (x >= minLim) ? x - minLim : 4095 + 1 - (minLim - x);
      int distanceToMax = (x <= maxLim) ? maxLim - x : 4095 + 1 - (x - maxLim);
      int closestLimit = (distanceToMin < distanceToMax) ? minLim : maxLim;
      return (centerPos >= closestLimit) ? centerPos - closestLimit : -(closestLimit - centerPos);
    } else if (!centerInValidRange) {
      return 0; // Invalid center position
    }

    if (centerPos >= minLim) {
      if (x >= minLim) {
        return centerPos - x;
      } else {
        return -((4095 + 1 - centerPos) + x);
      }
    } else {
      if (x <= maxLim) {
        return centerPos - x;
      } else {
        return (4095 + 1 - x) + centerPos;
      }
    }
  }
}*/

/*int getSignedDistanceWithoutCrossingLimits(int x, int minLim, int centerPos, int maxLim) {
  // Normalize positions to be within the valid range
  x = (x + 4095 + 1) % (4095 + 1);
  centerPos = (centerPos + 4095 + 1) % (4095 + 1);
  minLim = (minLim + 4095 + 1) % (4095 + 1);
  maxLim = (maxLim + 4095 + 1) % (4095 + 1);

  // Check if x is within the valid range
  if (minLim <= maxLim) {
    if (x < minLim || x > maxLim) {
      // Outside the range, calculate distance to the closest limit
      int distanceToMin = (x >= minLim) ? x - minLim : 4095 + 1 - (minLim - x);
      int distanceToMax = (x <= maxLim) ? maxLim - x : 4095 + 1 - (x - maxLim);
      int closestLimit = (distanceToMin < distanceToMax) ? minLim : maxLim;
      return (centerPos >= closestLimit) ? centerPos - closestLimit : -(closestLimit - centerPos);
    } else if (centerPos < minLim || centerPos > maxLim) {
      return 0; // Invalid center position
    }
    return centerPos - x;
  } else {
    // Wraparound case
    bool xInValidRange = (x >= minLim || x <= maxLim);
    bool centerInValidRange = (centerPos >= minLim || centerPos <= maxLim);

    if (!xInValidRange) {
      // Outside the range, calculate distance to the closest limit
      int distanceToMin = (x >= minLim) ? x - minLim : 4095 + 1 - (minLim - x);
      int distanceToMax = (x <= maxLim) ? maxLim - x : 4095 + 1 - (x - maxLim);
      int closestLimit = (distanceToMin < distanceToMax) ? minLim : maxLim;
      return (centerPos >= closestLimit) ? centerPos - closestLimit : -(closestLimit - centerPos);
    } else if (!centerInValidRange) {
      return 0; // Invalid center position
    }

    if (centerPos >= minLim) {
      if (x >= minLim) {
        return centerPos - x;
      } else {
        return -((4095 + 1 - centerPos) + x);
      }
    } else {
      if (x <= maxLim) {
        return centerPos - x;
      } else {
        return (4095 + 1 - x) + centerPos;
      }
    }
  }
}
*/

  void update(unsigned long syncMicros) {               // <== res = 134872.9
    //Serial.println("");
    //Serial.println(motorName);
    if (!isCalibrated) {
      //magEnc.updateEvery_ms(1);
      //magEnc.updateNB(1000);
      bool instaCal = true;
      magEnc.update();
      short encPos12b = magEnc.getRawPos12b("raw");

      //magEnc.updateNB();
      if(!instaCal)
      {
        long deltaAccUpdateMicros = syncMicros - lastAccUpdateMicros;
        double kFixed = deltaAccUpdateMicros / 1000000.0;

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
      } else {
        /*int delta12b = center12b - encPos12b;
        long deltaSteps = map(delta12b, 0, 4096, 0, stepResolution);
        setZeroAndLimits(-deltaSteps);
        moveTo(-deltaSteps, "raw");*/
        int delta12b;

        // Check if the limits wrap around
        bool wrapAround = minLim12b > maxLim12b;

        for(int i = 0; i < calEncSamples; i++)
        {
          calEncPositions[i] = magEnc.getRawPos12b("raw");
        }

        short calPos12b = getMostFrequentValue(calEncPositions, calEncSamples);

        // Determine the direction of the path based on the limits
        /*if (!wrapAround) {
            // Normal case: minLim12b < maxLim12b
            if (calPos12b <= center12b) {
                if (calPos12b >= minLim12b && center12b <= maxLim12b) {
                    delta12b = center12b - calPos12b;  // Direct path within limits
                } else {
                    delta12b = 4096 - calPos12b + center12b;  // Path via wraparound
                }
            } else {
                if (calPos12b <= maxLim12b || center12b >= minLim12b) {
                    delta12b = -(calPos12b - center12b);  // Direct path via wraparound
                } else {
                    delta12b = center12b + 4096 - calPos12b;  // Long path avoiding limits
                }
            }
        } else {
            // Wraparound case: minLim12b > maxLim12b
            if (calPos12b >= minLim12b || calPos12b <= maxLim12b) {
                if (center12b >= minLim12b || center12b <= maxLim12b) {
                    delta12b = center12b - calPos12b;  // Direct path through wraparound
                } else {
                    delta12b = 4096 - calPos12b + center12b;  // Long path avoiding limits
                }
            } else {
                if (center12b <= maxLim12b || center12b >= minLim12b) {
                    delta12b = -(calPos12b - center12b);  // Direct path avoiding wraparound
                } else {
                    delta12b = center12b + 4096 - calPos12b;  // Long path via wraparound
                }
            }
        }*/

        // Map the adjusted delta12b to steps

        delta12b = getSignedDistanceWithoutCrossingLimits(calPos12b, minLim12b, center12b, maxLim12b);
        long deltaSteps = map(delta12b, -4096, 4096, -stepResolution, stepResolution);

        // Apply the offset and move to the new position
        setZeroAndLimits(-deltaSteps);
        moveTo(-deltaSteps, "raw");
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

      // Constrain target position within limits
      if(!ignoreLimits) {
        targetPosition = constrain(targetPosition, minLim, maxLim);
      }
      

      stepsToAcc = speed*abs(speed) / (2 * abs(maxAcc));

      // Handle stationary condition
      if (stepsToGo == 0) {
        speed = 0; // Stop if already at the target
        return;
      }

      // Determine acceleration direction
      //acc = (stepsToGo > 0) ? maxAcc : -maxAcc;
      acc = maxAcc;

      // Adjust speed dynamically
      if (stepsToGo > stepsToAcc) {
        // Accelerate smoothly towards the target
        speed += maxAcc;
      } else {
        // Decelerate smoothly as we approach the target
        speed -= maxAcc;
      }

      // Constrain speed to the allowed range
      speed = constrain(speed, -maxSpeed, maxSpeed);

      // Step the motor while updating position
      if (stepsToGo != 0) {
        step();
      }
      /*
      // Calculate steps to the new target
      stepsToGo = targetPosition - position;

      // Calculate steps required to decelerate from current speed
      stepsToAcc = sq(speed) / (2 * abs(maxAcc));

      // Handle stationary condition
      if (stepsToGo == 0) {
        speed = 0; // Stop if already at the target
        return;
      }

      // Determine acceleration direction
      acc = (stepsToGo > 0) ? maxAcc : -maxAcc;

      // Adjust speed dynamically
      if (abs(stepsToGo) > stepsToAcc) {
        // Accelerate smoothly towards the target
        speed += acc;
      } else {
        // Decelerate smoothly as we approach the target
        speed -= acc;
      }

      // Constrain speed to the allowed range
      speed = constrain(speed, -maxSpeed, maxSpeed);

      // Step the motor while updating position
      if (stepsToGo != 0) {
        step();
      }
      */
      /*
      encRevs = magEnc.getConvRevs("raw");
      if (revsMode == "multiple") {
      }

      targetPosition = constrain(targetPosition, minLim, maxLim);
      stepsToGo = targetPosition - position;

      //int stepsToAcc = speed * stepsToGo + 0.5 * acc * stepsToGo;
      stepsToAcc = sq(speed) / abs(2 * acc * 2000); // původně takhle(nevim proč tam bylo * 2000): stepsToAcc = (sq(speed) - sq(0)) / (2 * acc * 2000);

      if (stepsToGo == 0) {
        speed = 0;
        return;
      }

      if (stepsToGo >= 0) {
        acc = maxAcc;
      } else if (stepsToGo < 0) {
        acc = -maxAcc;
      } 
      //acc = (stepsToGo > 0) ? maxAcc : -maxAcc;

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
      */

    } else if (controllerMode == 2) {  // =============================================== classical step counting mode

      if(encReading)
      {
        magEnc.updateNB(5000);
      }

      if(!ignoreLimits) {
        targetPosition = constrain(targetPosition, minLim, maxLim);
      }
      
      stepsToGo = targetPosition - position;

      if (stepsToGo != 0) {
        long deltaAccUpdateMicros = syncMicros - lastAccUpdateMicros;
        double kFixed = deltaAccUpdateMicros / 1000000.0;

        // Compute the total distance and adjust ttmMaxSpeed
        double totalDistance = abs(stepsToGo);
        double accelTime = maxSpeed / maxAcc; // Time to accelerate to maxSpeed at maxAcc
        double accelDistance = 0.5 * maxAcc * sq(accelTime); // Distance covered during acceleration

        double ttmMaxSpeed;
        
        if (ttm > 2 * accelTime) {
          // Enough time for full acceleration, constant speed, and deceleration
          double constantSpeedTime = ttm - 2 * accelTime;
          ttmMaxSpeed = (totalDistance-2*accelDistance) / (constantSpeedTime);
        } else {
          // Not enough time for full constant speed; adjust speed to fit within ttm
          ttmMaxSpeed = sqrt(totalDistance * maxAcc);
        }
        ttmMaxSpeed = constrain(ttmMaxSpeed, 0, maxSpeed);

        // Determine acceleration or deceleration
        acc = (stepsToGo > 0) ? maxAcc : -maxAcc;

        // Determine the phase of motion
        stepsToDecel = sq(speed) / (2 * abs(maxAcc));
        stepsToDecel *= (speed > 0) ? 1 : -1;

        if ((stepsToGo > 0 && stepsToGo <= stepsToDecel) || (stepsToGo < 0 && stepsToGo >= stepsToDecel)) {
          // Deceleration phase
          speed -= acc * kFixed;
        } else if (abs(speed) < abs(ttmMaxSpeed)) {
          // Acceleration phase
          speed += acc * kFixed;
        } else {
          // Constant speed phase
          speed = (stepsToGo > 0) ? ttmMaxSpeed : -ttmMaxSpeed;
        }

        // Constrain speed within allowed limits
        speed = constrain(speed, -maxSpeed, maxSpeed);

        // Check if close enough to stop
        if (abs(stepsToGo) <= 1) {
          speed = 0;
          position = targetPosition;
        }
      } 
      else 
      {
        // No movement needed
        acc = 0;
        speed = 0;
      }

      lastAccUpdateMicros = syncMicros;

      // Move the motor by one step based on the calculated speed
      stepNB(syncMicros);


      /*
      targetPosition = constrain(targetPosition, minLim, maxLim);
      stepsToGo = targetPosition - position;

      if (stepsToGo != 0) {
        acc = (stepsToGo > 0) ? maxAcc : -maxAcc;
        
        long deltaAccUpdateMicros = syncMicros - lastAccUpdateMicros;
        double kFixed = deltaAccUpdateMicros / 1000000.0; // puvodne 1000000.0
        //double kFixed = max(deltaAccUpdateMicros, 200) / 1000000.0;

        if (ttm > 0) {
          double totalDistance = abs(stepsToGo);
          double requiredAcc = (2 * totalDistance) / (ttm * ttm);
          acc = (stepsToGo > 0) ? requiredAcc : -requiredAcc;
        } else {
            acc = (stepsToGo > 0) ? maxAcc : -maxAcc;
        }

        acc *= kFixed;
        stepsToDecel = sq(speed) / (2 * abs(maxAcc));
        stepsToDecel *= (speed > 0) ? 1 : -1;


        double peakSpeed = (ttm > 0) ? abs(stepsToGo) / ttm : sqrt(2 * abs(acc) * abs(stepsToGo));
        //double peakSpeed = sqrt(2 * abs(maxAcc) * abs(stepsToGo));
        peakSpeed = constrain(peakSpeed, 0, maxSpeed);

        if ((stepsToGo > 0 && stepsToGo <= stepsToDecel) || (stepsToGo < 0 && stepsToGo >= stepsToDecel)) {
          speed -= acc; // Decelerate
        } else //if (abs(speed) < peakSpeed) 
        {
          speed += acc;
        }

        speed = constrain(speed, -maxSpeed, maxSpeed);

        if (abs(stepsToGo) <= 1) {
          speed = 0;
          position = targetPosition;
        }

      } else {
        // No movement needed
        acc = 0;
        speed = 0;
      }
      lastAccUpdateMicros = syncMicros;
      // Ensure the speed is within the allowed range
      speed = constrain(speed, -maxSpeed, maxSpeed);

      // Move the motor by one step based on the calculated speed
      stepNB(syncMicros);  // This function should implement the step movement according to the current speed
      //============
      */

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

      /*
      targetPosition = constrain(targetPosition, minLim, maxLim);
      stepsToGo = targetPosition - position;

      if (stepsToGo != 0) {

        acc = (stepsToGo > 0) ? maxAcc : -maxAcc;
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
      *///============


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

  bool isValidConfig() {
    int clockwiseDiff = (maxLim12b - minLim12b + 4096) % 4096;  // Distance clockwise
    int counterClockwiseDiff = (4096 - clockwiseDiff) % 4096;   // Distance counter-clockwise
    return clockwiseDiff < 4096 && counterClockwiseDiff < 4096;
}

  /*void setZeroAndLimits(long customZero = 0) {
    position = customZero;

    if(minLim12b < center12b && center12b < maxLim12b) 
    {
      minLim = -stepResolution*((center12b - minLim12b)/4096.0);
      maxLim = stepResolution*((maxLim12b - center12b)/4096.0);
    } else if (minLim12b > center12b) {
      minLim = -stepResolution*((center12b + (4096-minLim12b))/4096.0);
      maxLim = stepResolution*((maxLim12b - center12b)/4096.0);
    } else if (center12b > maxLim12b) {
      minLim = -stepResolution*((center12b + minLim12b)/4096.0);
      maxLim = stepResolution*((maxLim12b + (4096-center12b))/4096.0);
    }
    
    isCalibrated = true;
  }*/

  void setZeroAndLimits(long customZero = 0) {
    position = customZero;

    // Compute minLim based on the shortest route to minLim12b
    double minDiff = (center12b >= minLim12b)
        ? center12b - minLim12b 
        : 4096 + center12b - minLim12b;
    minLim = -stepResolution * (minDiff / 4096.0);

    // Compute maxLim based on the shortest route to maxLim12b
    double maxDiff = (maxLim12b >= center12b)
        ? maxLim12b - center12b 
        : 4096 + maxLim12b - center12b;
    maxLim = stepResolution * (maxDiff / 4096.0);

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

  float getMostFrequentValue(short arr[], int size) {
    float mostFrequent = arr[0];
    int maxFrequency = 0;

    // Outer loop to iterate over each element
    for (int i = 0; i < size; i++) {
        int currentFrequency = 0;

        // Inner loop to count occurrences of the current element
        for (int j = 0; j < size; j++) {
            if (arr[j] == arr[i]) {
                currentFrequency++;
            }
        }

        // Update the most frequent value if a higher frequency is found
        if (currentFrequency > maxFrequency) {
            maxFrequency = currentFrequency;
            mostFrequent = arr[i];
        }
    }

    return mostFrequent;
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
    Serial.print(" TarPos: ");
    Serial.print(targetPosition);
    Serial.print(" ");
    Serial.print(" stepsToGo: ");
    Serial.print(stepsToGo);
    Serial.print(" ");
    Serial.print(" Dir: ");
    Serial.print(dir);
    Serial.print(" ");
    Serial.print(" stepsToDECEL: ");
    Serial.print(stepsToDecel);
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

    Serial.print(" min/center/max [12b]: ");
    Serial.print("<");
    Serial.print(minLim12b);
    Serial.print("|");
    Serial.print(center12b);
    Serial.print("|");
    Serial.print(maxLim12b);
    Serial.print(">");

    Serial.print("    minLim/maxLim: ");
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