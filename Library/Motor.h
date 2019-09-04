#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

class Motor{
  private:
    int mPowerPin;
    int mControl1Pin;
    int mControl2Pin;
    int mMaxRange;
    int mMinRange;
    int mMaxPower;
    
  public:
  Motor(int powerPin, int control1Pin, int control2Pin, int minRange, int maxRange, int maxPower); 
  void stop();
  void brake();
  void setPower(int power);
};

#endif
