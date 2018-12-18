#pragma once

#include "Arduino.h"

class WatchDog{
private:
  unsigned long previousMillis;
  float* temperatures;
  int timeCheck;
  float* ShiftArray(float arr[]);
  int currentTimes;
  int maxTemp;
  int maxDifference;

public:
  WatchDog();
  WatchDog(int timeCheck, int maxTemp, int maxDifference);
  bool Check(double powerRatio, double temperature);
  
};
