#include "Motor.h"

 Motor :: Motor(int powerPin, int control1Pin, int control2Pin, int minRange, int maxRange, int maxPower){
mPowerPin = powerPin;
mControl1Pin = control1Pin;
mControl2Pin = control2Pin;
mMaxRange = minRange;
mMinRange = maxRange;
mMaxPower = maxPower;
}
Motor:: stop(){
  digitalWrite(mPowerPin, LOW); 
}
Motor:: brake(){
  digitalWrite(mPowerPin, HIGH); 
  digitalWrite(mControl1Pin, HIGH); 
  digitalWrite(mControl2Pin, HIGH); 
}

Motor:: setPower(int power){
  if (power > mMaxPower) power = mMaxPower;
    else if (power < -mMaxPower) power = -mMaxPower;

  if (power == 0){
    brake();
  }
  else if (power > 0){
    //Forward
    analogWrite(mPowerPin, map(power, 1, mMaxPower, mMinRange, mMaxRange));
    digitalWrite(mControl1Pin, HIGH);
    digitalWrite(mControl2Pin, LOW);
  }
  else{
    //backward
    analogWrite(mPowerPin, map(power, -1, -mMaxPower, mMinRange, mMaxRange));
    digitalWrite(mControl1Pin, LOW);
    digitalWrite(mControl2Pin, HIGH);    
  }
}
 
