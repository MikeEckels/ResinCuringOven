#include <Wire.h> 
#include "LiquidCrystal_I2C.h"
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool selectedTemp = 0;
bool selectedTime = 0;

unsigned char selectTemp()
{
  unsigned char temp = 60; 

  lcd.setCursor(0,0);
  lcd.print("Select Temp:");
  
  while (selectedTemp == false)
  {
    if (Encoder(1))
    {
      temp = 80;
      lcd.setCursor(0,1);
      lcd.print(String(temp) + "C");
      
      if(digitalRead(4) == false) selectedTemp = true;
    }
    else if (!Encoder(1))
    {
      temp = 60;
      lcd.setCursor(0,1);
      lcd.print(String(temp) + "C");

      if(digitalRead(4) == false) selectedTemp = true;
    }
  }
  return (temp);
}

int selectTime()
{
  int Time = 15;
  
  lcd.setCursor(0,0);
  lcd.print("Select Time:");
  
  while (selectedTime == false)
  {
    Time = Encoder(0);

    if(Time == 0){
      Time = 15;
    }

    String temp = "";

    temp += Time;
    temp += " Mins  ";

    lcd.setCursor(0,1);
    lcd.print(temp);
    delay(100);
    Serial.println(Time);

    if(digitalRead(4) == false) selectedTime = true;
  }
  lcd.clear();
  return(Time);
}
