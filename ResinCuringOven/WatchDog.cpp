#include "WatchDog.h"

WatchDog::WatchDog(){
  timeCheck = 10;
  previousMillis = millis();
  currentTimes = 0;
  maxTemp = 60;
  maxDifference = 10;
}

WatchDog::WatchDog(int timeCheck, int maxTemp, int maxDifference){
  this->timeCheck = timeCheck;
  this->maxTemp = maxTemp;
  this->maxDifference = maxDifference;
  previousMillis = millis();
  currentTimes = 0;

  this->temperatures = new float[10];
}

bool WatchDog::Check(double powerRatio, double temperature){
  unsigned long currentMillis = millis();
  bool watchDog = false;//everything is good

  //every five seconds shift values
  if(currentMillis - previousMillis > 5000){
    
    if(currentTimes < timeCheck){
      temperatures[currentTimes++] = temperature;
    }
    else{
      temperatures = ShiftArray(temperatures);//pop first
      temperatures[timeCheck - 1] = temperature;
    }

    previousMillis = currentMillis;

    Serial.print("CT: "); Serial.print(currentTimes); Serial.print(" TC: "); Serial.println(timeCheck);
  }

  //at least 10 seconds of data collected
  if(currentTimes == timeCheck){
    
    float avgDif = 0.0f;

    //get average difference in temperatures
    for(int i = 0; i < timeCheck - 1; i++){
      avgDif += temperatures[i + 1] - temperatures[i];
    }

    avgDif /= (float)(timeCheck - 1);

    //power is greater than 75%, expect at least an average of 0.1deg climb per second
    if(powerRatio > 0.75 && temperature < maxTemp){
      watchDog = avgDif < -1.0;
    }
    else if(powerRatio > 0.9 && temperature < maxTemp - 10.0f){//initial heating
      watchDog = avgDif < 0.1;
    }
    else{//if temperature is climbing more than 5.0 degrees per second, disable
      watchDog = avgDif > 5.0;
    }

    Serial.print("AVG DIF: "); Serial.println(avgDif);
    Serial.print("Temperature: "); Serial.println(temperature);
    Serial.print("Power Ratio: "); Serial.println(powerRatio);

    //temperature is past maximum overshoot
    if(temperature - maxDifference > maxTemp){
      watchDog = true;
    }
  }

  return watchDog;
}

float* WatchDog::ShiftArray(float arr[]){
  for(int i = 0; i < timeCheck; i++){
    arr[i] = arr[i + 1];
  }

  arr[timeCheck - 1] = 0.0;

  return arr;
}
