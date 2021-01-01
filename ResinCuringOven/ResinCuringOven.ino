#include "Menu.h"
#include <math.h>
#include "PID.h"
#include "KalmanFilter.h"
#include "WatchDog.h"
#include "Adafruit_MCP9808.h"
#include <Wire.h>

static unsigned int sensorPin = A1;
static unsigned int FanPin    = 8;
static unsigned int MotorPin  = 9;
static unsigned int Other     = 10;
static unsigned int UVLedPin  = 11;
static unsigned int HeaterPin = 12;
static unsigned int LedPanel  = 6;

unsigned int desiredTemp = 0;
unsigned int desiredTime  = 0;

bool DISABLE = false;
bool RUNNING = true;

unsigned long previousMillis;
unsigned long startMillis;
float previousTemp;
byte previousHeat;
const int loopTime = 200;//milliseconds

KalmanFilter tempKF       = KalmanFilter(0.25, 25);
KalmanFilter heatOutputKF = KalmanFilter(0.1, 50);
PID heaterPWMPID          = PID(255, 0, 0);
WatchDog heaterWatch      = WatchDog(10, 100, 10);

void(* ResetArduino) (void) = 0;

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

void setup()
{
  pinMode(FanPin,     OUTPUT);
  pinMode(HeaterPin,  OUTPUT);
  pinMode(MotorPin,   OUTPUT);
  pinMode(UVLedPin,   OUTPUT);
  pinMode(Other,      OUTPUT);
  pinMode(LedPanel,   OUTPUT);
  pinMode(4,          INPUT_PULLUP);//Encoder Button
  pinMode(A0, INPUT); //Pot Pin
  //pinMode(2,          INPUT_PULLUP);//Encoder pinA
  //pinMode(3,          INPUT_PULLUP);//Encoder pinB

  digitalWrite(FanPin,    HIGH);//Relay controls are inverted logic
  digitalWrite(MotorPin,  HIGH);
  digitalWrite(UVLedPin,  HIGH);
  digitalWrite(Other,     HIGH);
  digitalWrite(HeaterPin, LOW);//SSR
  digitalWrite(LedPanel,  LOW);
  
  //attachInterrupt(0, PinA, CHANGE);
  //attachInterrupt(1, PinB, CHANGE);

  Serial.begin(115200);

  if (!tempsensor.begin(0x18)) {
    Serial.println("Couldn't find MCP9808!");
    while (1);
  }
    
  Serial.println("Found MCP9808!");
  tempsensor.setResolution(3);

  SetTimeTemp();

  Preheat();

  startMillis = millis();
  previousMillis = millis();
  tempsensor.wake();
  previousTemp = tempKF.Filter((float)tempsensor.readTempC());
  tempsensor.shutdown_wake(1);
  //previousTemp = tempKF.Filter((float)Thermistor(analogRead(sensorPin)));
}

void loop()
{
  unsigned int minutesRan = (millis() - startMillis) / 60000;
  
  if(minutesRan < desiredTime){
    //float currentTemp = tempKF.Filter((float)Thermistor(analogRead(sensorPin)));
    tempsensor.wake();
    float currentTemp = tempKF.Filter((float)tempsensor.readTempC());
    tempsensor.shutdown_wake(1);
    
    unsigned long currentMillis = millis();
    float dT = (currentMillis - previousMillis) / 1000.0;

    float pwmOut = heaterPWMPID.Calculate(desiredTemp, currentTemp, dT);
      
    byte heatOutput = constrain((int)pwmOut, 0, 255);
    analogWrite(LedPanel, heatOutput);
    analogWrite(HeaterPin, heatOutput);//Set SSR PWM duty cycle
      
    DISABLE = heaterWatch.Check((float)heatOutput / 255.0f, currentTemp);
    //DISABLE = false;
    
    SetLCDDisplay("Curing...", String(currentTemp, 1) + "C " + String(desiredTime - minutesRan) + " Mins");
    
    previousMillis = currentMillis;
    
    WatchDogSink();//Stops loop if disabled
    
    delay(loopTime);
  }
  else{
    TurnOff();
    
    while (digitalRead(4))//button click
    { 
      SetLCDDisplay("Finished!", "Reboot System");
      delay(loopTime);
    }
    
    ResetArduino();
  }
}

void SetLCDDisplay(String line1, String line2){
  if(line1.length() <= 16 && line2.length() <= 16){
    for(int i = line1.length(); i < 15; i++){
      line1 += " ";
    }
    
    for(int i = line2.length(); i < 15; i++){
      line2 += " ";
    }
    
    lcd.setCursor(0,0);
    lcd.print(line1);
    lcd.setCursor(0,1);
    lcd.print(line2);
  }
  else{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("LCD Char Limit");
  }
}

void Preheat(){
  bool preheating = true;
  
  digitalWrite(HeaterPin, HIGH);
  digitalWrite(MotorPin, LOW);
  digitalWrite(FanPin, LOW);
  digitalWrite(LedPanel, HIGH);

  WatchDog preHeatWatch = WatchDog(10, 100, 10);
  
  while(preheating){
    tempsensor.wake();
    float currentTemp = tempKF.Filter((float)tempsensor.readTempC());
    //float currentTemp = tempKF.Filter((float)Thermistor(analogRead(sensorPin)));
    SetLCDDisplay("Preheating:", "Temp: " + String(currentTemp, 1) + "C");
    preheating = currentTemp < desiredTemp;

    Serial.print("Starting T:"); Serial.print(currentTemp); Serial.print(" D: "); Serial.println(desiredTemp);
    
    DISABLE = preHeatWatch.Check(1.0f, currentTemp);
    
    Serial.print("Disable: "); Serial.println(DISABLE);
    
    if(!preheating){
      SetLCDDisplay("Preheat done.", "Cure starting.");
      digitalWrite(UVLedPin, LOW);
      
      delay(1500);
    }
    else{
      delay(500);
    }

    WatchDogSink();
    tempsensor.shutdown_wake(1);
  }
}

void SetTimeTemp(){
  lcd.begin();
  lcd.backlight();

  desiredTemp = selectTemp();
  delay(500);//debounce delay NEED THIS
  desiredTime = selectTime();

  Serial.print("Temp: " + String(desiredTemp) + "C");
  Serial.print(" Time: " + String(desiredTime) + " Mins");
  SetLCDDisplay("Temp: " + String(desiredTemp) + "C", "Time: " + String(desiredTime) + " Mins");

  delay(2000);
}

void WatchDogSink(){
  //Lock loop if watchdog triggers
  while(DISABLE){
    SetLCDDisplay("Reset; Watchdog", "triggered.");
    TurnOff();
    delay(500);
  }
}

void TurnOff(){
    digitalWrite(HeaterPin, LOW);
    digitalWrite(LedPanel, LOW);
    digitalWrite(UVLedPin, HIGH);
    digitalWrite(MotorPin, HIGH);
    digitalWrite(FanPin, HIGH);
}
