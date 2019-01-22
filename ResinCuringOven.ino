#include "Hardware.h"
#include "Menu.h"
#include <math.h>
#include "PID.h"
#include "KalmanFilter.h"
#include "WatchDog.h"

static unsigned char pinA      = 2;
static unsigned char pinB      = 3;
static unsigned char sensorPin = A1;
static unsigned char FanPin    = 8;
static unsigned char MotorPin  = 9;
static unsigned char Other     = 10;
static unsigned char UVLedPin  = 11;
static unsigned char HeaterPin = 12;
static unsigned char LedPanel  = 6;

unsigned char desiredTemp = 60;
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

void setup()
{
  pinMode(pinA,      INPUT_PULLUP);
  pinMode(pinB,      INPUT_PULLUP);
  pinMode(FanPin,    OUTPUT);
  pinMode(HeaterPin, OUTPUT);
  pinMode(MotorPin,  OUTPUT);
  pinMode(UVLedPin,  OUTPUT);
  pinMode(Other,     OUTPUT);
  pinMode(LedPanel,  OUTPUT);
  pinMode(4,         INPUT_PULLUP);

  digitalWrite(FanPin,    HIGH);//Relay controls are inverted logic
  digitalWrite(MotorPin,  HIGH);
  digitalWrite(UVLedPin,  HIGH);
  digitalWrite(Other,     HIGH);
  digitalWrite(HeaterPin, LOW);//SSR
  digitalWrite(LedPanel,  LOW);
  
  attachInterrupt(0, PinA, RISING);
  attachInterrupt(1, PinB, RISING);

  Serial.begin(115200);

  SetTimeTemp();

  Preheat();

  startMillis = millis();
  previousMillis = millis();
  previousTemp = tempKF.Filter((float)Thermistor(analogRead(sensorPin)));
}

void loop()
{
  unsigned int minutesRan = (millis() - startMillis) / 60000;
  
  if(minutesRan < desiredTime){
    float currentTemp = tempKF.Filter((float)Thermistor(analogRead(sensorPin)));
    
    unsigned long currentMillis = millis();
    float dT = (currentMillis - previousMillis) / 1000.0;

    float pwmOut = heaterPWMPID.Calculate(desiredTemp, currentTemp, dT);
      
    byte heatOutput = constrain((int)pwmOut, 0, 255);
    analogWrite(LedPanel, heatOutput);

    Serial.print("Heat: ");
    Serial.println(heatOutput);

    Serial.print("DT: ");
    Serial.println(dT);

    Serial.print("TempT: ");
    Serial.println(desiredTemp);
    
    Serial.print("TempA: ");
    Serial.println(currentTemp);
    
    Serial.print("PWMF: ");
    Serial.println(pwmOut);
      
    analogWrite(HeaterPin, heatOutput);//Set SSR PWM duty cycle
      
    //DISABLE = heaterWatch.Check((float)heatOutput / 255.0f, currentTemp);
    DISABLE = false;
    
    SetLCDDisplay("Curing...", String(currentTemp, 1) + "C " + String(desiredTime - minutesRan) + " Mins");
    
    previousMillis = currentMillis;
    
    WatchDogSink();//Stops loop if disabled
    
    delay(loopTime);
  }
  else{
    TurnOff();
    
    while (!digitalRead(4))//button click
    { 
      SetLCDDisplay("Finished...", "Click to reset.");
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
    lcd.print("Fail");
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
    float currentTemp = tempKF.Filter((float)Thermistor(analogRead(sensorPin)));
    SetLCDDisplay("Preheating:", "Temp: " + String(currentTemp, 1) + "C");
    preheating = currentTemp < desiredTemp;
    
    DISABLE = false;//preHeatWatch.Check(1.0f, currentTemp);

    if(DISABLE) break;
    
    if(!preheating){
      SetLCDDisplay("Preheat done.", "Cure starting.");
      
      delay(1500);
    }
    else{
      delay(500);
    }
  }

  WatchDogSink();
}

void SetTimeTemp(){
  lcd.begin();
  lcd.backlight();

  desiredTemp = selectTemp();
  delay(500);//debounce delay NEED THIS
  desiredTime = selectTime();

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
}
