#ifndef AS5600ENCODER_H
#define AS5600ENCODER_H

#include <Wire.h>
#include "MyMath.h"

#define AS5600_ADDRESS 0x36

class AS5600Encoder {
private:
  bool ready;
  unsigned long magEncLastMillis = 0;
  unsigned int resolution;
  float convRevs;
  float lastConvRevs;
  float revs;
  float speed;

  unsigned short lastRawPos12b;
  unsigned short rawPos12b;
  float rawSpeed12b;
  float avgRawSpeedSamples12b[16];
  float avgRawSpeed12b;

  signed short pos;
  unsigned short zeroOffset;
  unsigned short lastPosition;
  //int32_t sumPos;
  //uint16_t lastSumPosition;
  unsigned long uLastUpdateTime;

  bool dir;
  int wir;

  // Non-blocking read variables
  unsigned long lastMicrosNBER;
  unsigned long delayTimeNBER;
  bool wholeRawPosRead;

  
  enum ReadState {
    IDLE,
    SEND_REQUEST,
    REQUEST_DATA,
    READ_DATA
  };

  ReadState stateNBER;

public:
  // Constructor
  AS5600Encoder()
    : ready(false),
      magEncLastMillis(0),
      resolution(0),
      convRevs(0),
      lastConvRevs(0),
      revs(0),
      speed(0),

      rawPos12b(0),
      lastRawPos12b(0),
      rawSpeed12b(0),
      avgRawSpeed12b(0),

      pos(0),
      zeroOffset(0),
      lastPosition(0),
      uLastUpdateTime(micros()),

      lastMicrosNBER(0),
      delayTimeNBER(5),  // Adjust this based on non-blocking requirements
      wholeRawPosRead(false),
      stateNBER(IDLE)
    {
    for (unsigned int i = 0; i < 16; i++) {
      avgRawSpeedSamples12b[i] = 0.0;
    }
  }

  void begin(int res, bool _encDir, int _wir) {
    wir = _wir;
    dir = _encDir;
    if(wir == 0) {
      Wire.begin();
      Wire.setClock(100000);
    } 
    else if (wir == 1) {
      Wire2.begin();
      Wire2.setClock(100000);
    }

    readRawPos();
    resolution = res;
    ready = true;
    lastRawPos12b = rawPos12b;
  }

  ReadState state = IDLE;



  void readRawPosNB() {
    static uint8_t highByte = 0;
    static uint8_t lowByte = 0;
    

    switch (stateNBER) {
      case IDLE:
        // Wait for the next cycle
        if (micros() - lastMicrosNBER >= delayTimeNBER) {
          stateNBER = SEND_REQUEST;
        }
        break;

      case SEND_REQUEST:
        if (wir == 0) {
          Wire.beginTransmission(AS5600_ADDRESS);
          Wire.write(0x0C);  // Register to read high byte of angle
          Wire.endTransmission();
        } else if (wir == 1) {
          Wire2.beginTransmission(AS5600_ADDRESS);
          Wire2.write(0x0C);  // Register to read high byte of angle
          Wire2.endTransmission();
        }
        lastMicrosNBER = micros();
        stateNBER = REQUEST_DATA;
        break;

      case REQUEST_DATA:
        if (micros() - lastMicrosNBER >= delayTimeNBER) {
          if (wir == 0) {
            Wire.requestFrom(AS5600_ADDRESS, 2);
          } else if (wir == 1) {
            Wire2.requestFrom(AS5600_ADDRESS, 2);
          }
          lastMicrosNBER = micros();
          stateNBER = READ_DATA;
        }
        break;

      case READ_DATA:
        if ((wir == 0 && Wire.available() >= 2) || (wir == 1 && Wire2.available() >= 2)) {
          if (wir == 0) {
            highByte = Wire.read();  // High byte
            lowByte = Wire.read();   // Low byte
          } else if (wir == 1) {
            highByte = Wire2.read();  // High byte
            lowByte = Wire2.read();   // Low byte
          }

          rawPos12b = (highByte << 8) | lowByte;
          wholeRawPosRead = true;

          if (!dir) {
            rawPos12b = 4095 - rawPos12b;
          }
          stateNBER = IDLE;
        }
        break;
    }
  }

  void readRawPos() {
    //Wire.begin();
    //Wire.setClock(100000);

    rawPos12b = 0;

    if(wir == 0) {
      Wire.beginTransmission(AS5600_ADDRESS);
      Wire.write(0x0C);  // Register to read high byte of angle
      Wire.endTransmission();
      Wire.requestFrom(AS5600_ADDRESS, 2);

      if (Wire.available() == 2) {
        rawPos12b = Wire.read() << 8;  // High byte
        rawPos12b |= Wire.read();      // Low byte
      }
    }
    else if(wir == 1){
      Wire2.beginTransmission(AS5600_ADDRESS);
      Wire2.write(0x0C);  // Register to read high byte of angle
      Wire2.endTransmission();
      Wire2.requestFrom(AS5600_ADDRESS, 2);

      if (Wire2.available() == 2) {
        rawPos12b = Wire2.read() << 8;  // High byte
        rawPos12b |= Wire2.read();      // Low byte
      }
    }
    if(!dir) {
      rawPos12b = 4095 - rawPos12b;
    }
    
    
  }

  void setZero(uint16_t rpos) {
    zeroOffset = rpos;
  }

  void updateEvery_ms(unsigned int del) {
    if (millis() - magEncLastMillis >= del) {
      update();
      magEncLastMillis = millis();
    }
  }

  void updateNB(unsigned int del) {
    delayTimeNBER = del;
    readRawPosNB();
    if(wholeRawPosRead) {
      calcRawSpeed12b();
      wholeRawPosRead = false;
    }
    
  }

  void update() {
    //#if ready == false
    //#error AS5600 not initialized
    //#endif
    //readRawPosNonBlocking();
    readRawPosNB();
    calcRawSpeed12b();

    //compPos();
    //compSumPos();

    //convRevs = sumPos / ((float)resolution);

    //revs = sumPos / 4096.0;  // Since 4096 units = 1 rotation (for 12-bit encoder)

    /*unsigned long uCurrentTime = micros();
    unsigned long uTimeElapsed = uCurrentTime - uLastUpdateTime;

    float speedDelta = convRevs - lastConvRevs;
    speed = (1000000 * speedDelta) / uTimeElapsed;  // Speed in units per second
    uLastUpdateTime = uCurrentTime;
    lastConvRevs = convRevs;*/
  }

  void calcRawSpeed12b() {
    unsigned short currentRawPos12b = rawPos12b;
    unsigned long currentTime = micros();

    // Calculate the time difference in seconds
    float timeDifference = (currentTime - uLastUpdateTime) / 1000000.0;

    // Calculate the difference in angle considering rollover
    int rawPosDiff12b = currentRawPos12b - lastRawPos12b;

    if (rawPosDiff12b > 2048) {
      rawPosDiff12b -= 4096;
    } else if (rawPosDiff12b < -2048) {
      rawPosDiff12b += 4096;
    }

    // Calculate speed in degrees per second
    rawSpeed12b = rawPosDiff12b / timeDifference;
    
    for(unsigned int i = 0; i < 15; i++) {
      avgRawSpeedSamples12b[i+1] = avgRawSpeedSamples12b[i];
    }
    avgRawSpeedSamples12b[0] = rawSpeed12b;

    float avgRawSpeedSum12b = 0;
    for(unsigned int i = 0; i < 16; i++) {
      avgRawSpeedSum12b += avgRawSpeedSamples12b[i];
    }

    //avgRawSpeed12b = avgRawSpeedSum12b/16.0;
    avgRawSpeed12b = calculateAvgWithoutOutliers(avgRawSpeedSamples12b, 16, 2.0);

    // Update previous values for the next calculation
    lastRawPos12b = currentRawPos12b;
    uLastUpdateTime = currentTime;
  }

  void compPos() {
    pos = rawPos12b - zeroOffset;
    if (pos < 0) {
      pos += 4096;
    } else if (pos >= 4096) {
      pos -= 4096;
    }
  }

  void compSumPos() {
    int16_t delta = pos - lastPosition;
    // Handle wrapping of the delta
    if (delta > 2048) {
      delta -= 4096;
    } else if (delta < -2048) {
      delta += 4096;
    }
    sumPos += delta;
    lastPosition = pos;
  }



  unsigned int getRawPos12b(String opt) {
    if (opt == "deg") {
      return (rawPos12b / 4096) * 360;
    } else if (opt == "raw") {
      return rawPos12b;
    } else {
      throw std::invalid_argument("Invalid option passed as argument");
    }
  }

  float getRawSpeed12b(String opt) {
    if (opt == "deg") {
      return (rawPos12b / 4096) * 360;
    } else if (opt == "raw") {
      return rawSpeed12b;
    } else {
      throw std::invalid_argument("Invalid option passed as argument");
    }
  }

  float getAvgRawSpeed12b(String opt) {
    if (opt == "deg") {
      return (rawPos12b / 4096) * 360;
    } else if (opt == "raw") {
      return avgRawSpeed12b;
    } else {
      throw std::invalid_argument("Invalid option passed as argument");
    }
  }



  uint16_t getPos(String opt) {
    if (opt == "deg") {
      return (pos / 4096) * 360;
    } else if (opt == "raw") {
      return pos;
    } else {
      throw std::invalid_argument("Invalid option passed as argument");
    }
  }

  int getSumPos(String opt) {
    if (opt == "deg") {
      return (sumPos / 4096) * 360;
    } else if (opt == "rad") {
      return (sumPos / 4096) * 2 * PI;
    } else if (opt == "raw") {
      return sumPos;
    } else {
      throw std::invalid_argument("Invalid option passed as argument");
    }
  }

  float getConvRevs(String opt) {
    if (opt == "deg") {
      return convRevs * 360;
    } else if (opt == "rad") {
      return convRevs * 2 * PI;
    } else if (opt == "raw") {
      return convRevs;
    } else {
      throw std::invalid_argument("Invalid option passed as argument");
    }
  }

  float getSpeed(String opt) {
    if (opt == "deg") {
      return (speed / 4096.0) * 360;
    } else if (opt == "rad") {
      return (speed / 4096.0) * 2 * PI;
    } else if (opt == "raw") {
      return speed;
    } else {
      throw std::invalid_argument("Invalid option passed as argument");
    }
  }

  float getRevs() {
    return revs;
  }

  int sumPos;
  int32_t lastSumPosition;
};

#endif  // AS5600ENCODER_H
