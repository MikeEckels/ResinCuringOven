
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
static unsigned char ledPin    = 11;
static unsigned char HeaterPin = 12;
static unsigned char Led       = 13;

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
PID heaterPWMPID          = PID(1, 0, 0.25);
WatchDog heaterWatch      = WatchDog(10, 100, 10);

void(* ResetArduino) (void) = 0;

void setup()
{
  pinMode(pinA,      INPUT_PULLUP);
  pinMode(pinB,      INPUT_PULLUP);
  pinMode(FanPin,    OUTPUT);
  pinMode(HeaterPin, OUTPUT);
  pinMode(MotorPin,  OUTPUT);
  pinMode(ledPin,    OUTPUT);
  pinMode(Other,     OUTPUT);
  pinMode(Led,       OUTPUT);
  pinMode(4,         INPUT_PULLUP);

  digitalWrite(FanPin,    HIGH);
  digitalWrite(HeaterPin, LOW);
  digitalWrite(MotorPin,  HIGH);
  digitalWrite(ledPin,    HIGH);
  digitalWrite(Other,     HIGH);
  digitalWrite(Led,       LOW);
  
  attachInterrupt(0, PinA, RISING);
  attachInterrupt(1, PinB, RISING);

  Serial.begin(115200);

  SetTimeTemp();

  while (!digitalRead(4))//button click
  { 
    SetLCDDisplay("Running...", "Begin preheat.");
    delay(2000);
  }

  Preheat();
  
  digitalWrite(MotorPin, LOW);

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
      
    byte heatOutput = constrain((int)heaterPWMPID.Calculate(desiredTemp, currentTemp, dT), 0, 255);
      
    analogWrite(HeaterPin, heatOutput);//Set SSR PWM duty cycle
      
    DISABLE = heaterWatch.Check((float)heatOutput / 255.0f, currentTemp);
    
    SetLCDDisplay("Curing...", "C:" + String(currentTemp, 1) + " M:" + String(desiredTime - minutesRan));
    
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
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
}

void Preheat(){
  bool preheating = true;
  
  digitalWrite(HeaterPin, HIGH);

  WatchDog preHeatWatch = WatchDog(10, 100, 10);
  
  while(preheating){
    float currentTemp = tempKF.Filter((float)Thermistor(analogRead(sensorPin)));
    SetLCDDisplay("Preheating:", "Temp: " + String(currentTemp, 1));
    preheating = currentTemp < desiredTemp;
    
    DISABLE = preHeatWatch.Check(1.0f, currentTemp);

    if(DISABLE) break;
    
    if(preheating){
      SetLCDDisplay("Preheat done.", "Cure starting.");
      
      digitalWrite(Led, HIGH);
      digitalWrite(MotorPin, LOW);
      
      delay(1500);
    }
    else{
      delay(200);
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

  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.print(desiredTemp);

  lcd.setCursor(0, 1);
  lcd.print("Time:");
  lcd.print(desiredTime);

  delay(500);
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
    digitalWrite(Led, LOW);
    digitalWrite(ledPin, LOW);
    digitalWrite(MotorPin, HIGH);
}
