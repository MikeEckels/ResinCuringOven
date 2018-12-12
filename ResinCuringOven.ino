
#include "Hardware.h"
#include "Menu.h"
#include <math.h>

static unsigned char pinA = 2;
static unsigned char pinB = 3;
static unsigned char sensorPin = A1;
static unsigned char FanPin = 8;
static unsigned char MotorPin = 9;
static unsigned char Other = 10;
static unsigned char ledPin = 11;
static unsigned char HeaterPin = 12;
static unsigned char Led = 13;

unsigned char desiredTemp = 60;
unsigned int desiredTime = 0;
bool Ready = false;
bool upToTemp = false;

void setup()
{
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(FanPin, OUTPUT);
  pinMode(HeaterPin, OUTPUT);
  pinMode(MotorPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(Other, OUTPUT);
  pinMode(Led, OUTPUT);
  pinMode(4, INPUT_PULLUP);

  digitalWrite(FanPin, HIGH);
  digitalWrite(HeaterPin, LOW);
  digitalWrite(MotorPin, HIGH);
  digitalWrite(ledPin, HIGH);
  digitalWrite(Other, HIGH);
  digitalWrite(Led, LOW);
  
  attachInterrupt(0, PinA, RISING);
  attachInterrupt(1, PinB, RISING);

  Serial.begin(115200);
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

  while (!Ready)
  { 
   if (digitalRead(4) == false)
    {
    Ready = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Running...");
    delay(2000);
   }
  }
}

void loop()
{
  if(Ready)
  {
    int tempVal = analogRead(sensorPin);
    double curentTemp = Thermistor(tempVal);
    Serial.println(curentTemp);

    digitalWrite(MotorPin, LOW);
    
    if(!upToTemp)
    {
      digitalWrite(FanPin, LOW);
      digitalWrite(HeaterPin, HIGH);
      digitalWrite(Led, HIGH);
      Serial.println("Heating");

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Heating...");
      lcd.setCursor(0,1);
      lcd.print(curentTemp);
    }else
    {
      //digitalWrite(FanPin, HIGH);
      digitalWrite(HeaterPin, LOW);
      digitalWrite(Led, LOW);
      digitalWrite(ledPin, LOW);
      Serial.println("Cooling");

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Cooling Down...");
      lcd.setCursor(0,1);
      lcd.print(curentTemp);
    }
    

    if(curentTemp >= desiredTemp)
    {
      upToTemp = true;
    }else if (curentTemp <= desiredTemp)
    {
      upToTemp = false;
    }
    delay(5000);
  }
}
