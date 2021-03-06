#include <Wire.h> 
#include "LiquidCrystal_I2C.h"
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool selectedTemp = 0;
bool selectedTime = 0;

unsigned int selectTemp()
{
  unsigned int temp = 0; 

  lcd.setCursor(0,0);
  lcd.print("Select Temp:");
  
  while (selectedTemp == false)
  {
    if (analogRead(A0) > 512 ) {
      temp = 80;
      lcd.setCursor(0,1);
      lcd.print(String(temp) + "C");

      if(digitalRead(4) == false) selectedTemp = true;
    }
    else if (analogRead(A0) <= 512) {
      temp = 60;
      lcd.setCursor(0,1);
      lcd.print(String(temp) + "C");

      if(digitalRead(4) == false) selectedTemp = true;
    }
//    if (Encoder(1))
//    {
//      temp = 80;
//      lcd.setCursor(0,1);
//      lcd.print(String(temp) + "C");
//      
//      if(digitalRead(4) == false) selectedTemp = true;
//    }
//    else if (!Encoder(1))
//    {
//      temp = 60;
//      lcd.setCursor(0,1);
//      lcd.print(String(temp) + "C");
//
//      if(digitalRead(4) == false) selectedTemp = true;
//    }
  }
  return (temp);
}

unsigned int selectTime()
{
  unsigned int Time = 0;
  
  lcd.setCursor(0,0);
  lcd.print("Select Time:");
  
  while (selectedTime == false)
  {
    
    //Time = Encoder(0);
    Time = map(analogRead(A0), 0, 1022, 15, 180);

    //Serial.print("Time:"); Serial.println(Time);

    //if(Time <= 0){
      //Time += 15;
    //}

    String val = "";

    val += Time;
    val += " Mins  ";

    lcd.setCursor(0,1);
    lcd.print(val);
    delay(100);

    if(digitalRead(4) == false) selectedTime = true;
  }
  lcd.clear();
  return(Time);
}
