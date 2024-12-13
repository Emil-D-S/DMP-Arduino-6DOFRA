#ifndef AS5600ENCODER_H
#define AS5600ENCODER_H
//ulozeno
#include <Wire.h>
#include "MyMath.h"

#define AS5600_ADDRESS 0x36
#define TCA_ADDRESS 0x70 // Default address of TCA9548A

void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel); // Select the channel
  Wire.endTransmission();
}

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
  uint8_t tcaChannel; // Channel on the TCA9548A multiplexer

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

  void begin(int res, uint8_t channel) {
    Wire.begin();
    Wire.setClock(100000);
    tcaChannel = channel; // Save the TCA channel for future use
    tcaSelect(tcaChannel); // Switch to the correct channel

    //readRawPos(); // Initial read to sync state
    resolution = res;
    ready = true;
    lastRawPos12b = rawPos12b;
  }

  ReadState state = IDLE;



  void readRawPosNB() {
    tcaSelect(tcaChannel);  // Select the TCA channel before I2C operations
    
    // Begin the read process if enough time has passed since the last read
    if (micros() - lastMicrosNBER >= delayTimeNBER) {
        Wire.beginTransmission(AS5600_ADDRESS);  // Begin I2C communication with the AS5600
        Wire.write(0x0C);  // Command to read the high byte of the angle
        Wire.endTransmission();
        lastMicrosNBER = micros();  // Store the time when the request was made
        stateNBER = REQUEST_DATA;  // Move to the next state to request data
    }

    // Request data from the AS5600 if it's ready
    if (stateNBER == REQUEST_DATA) {
        Wire.requestFrom(AS5600_ADDRESS, 2);  // Request 2 bytes (high and low byte)
        stateNBER = READ_DATA;  // Move to read data state
    }

    // Read the data if available
    if (stateNBER == READ_DATA) {
        if (Wire.available() >= 2) {
            uint8_t highByte = Wire.read();  // Read the high byte of the position
            uint8_t lowByte = Wire.read();   // Read the low byte of the position
            rawPos12b = (highByte << 8) | lowByte;  // Combine the two bytes into a 12-bit position value

            if (!dir) {
                rawPos12b = 4095 - rawPos12b;  // If direction is inverted, adjust the position
            }

            wholeRawPosRead = true;  // Mark that we successfully read the position
            stateNBER = IDLE;  // Reset the state machine to idle, ready for the next read
        }
    }

    /*
    Serial.print("readRawPosNB: ");
    Serial.println(rawPos12b);
    */
  }



  void readRawPos() {
    tcaSelect(tcaChannel); // Select the correct TCA channel
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(0x0C);  // Register to read high byte of angle
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDRESS, 2);

    if (Wire.available() == 2) {
      rawPos12b = Wire.read() << 8 | Wire.read();
      if (!dir) {
        rawPos12b = 4095 - rawPos12b; // Adjust for direction
      }
    }
    /*Serial.print("readRawPos: ");
    Serial.println(rawPos12b);
    */
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
    if (wholeRawPosRead) {
      calcRawSpeed12b();
      wholeRawPosRead = false;
    }
  }

  void update() {
    //#if ready == false
    //#error AS5600 not initialized
    //#endif
    //readRawPosNonBlocking();
    readRawPos(); //<==========
    //readRawPos();
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
      return (rawSpeed12b / 4096) * 360;
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
