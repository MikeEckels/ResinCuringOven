
unsigned int minVal = 15;
unsigned int maxVal = 120;
unsigned int stepVal = 15;
unsigned int lastPos = 1;
static boolean rotating = false;       
volatile unsigned int encoderPos = 0;

boolean A_set = false;
boolean B_set = false;

void ResetEncoder(){
  encoderPos = 0;
}

void PinA() {
  if ( rotating ) delay (1);

  if ( digitalRead(2) != A_set ) {
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set )
      encoderPos = (encoderPos + minVal > maxVal) ? maxVal : encoderPos + stepVal;

    rotating = false;
  }
}

void PinB() {
  if ( rotating ) delay (1);
  if ( digitalRead(3) != B_set ) {
    B_set = !B_set;
    
    //  adjust counter - 1 if B leads A
    if ( B_set && !A_set && ((int)encoderPos - (int)stepVal) > 0)
      encoderPos = (encoderPos - minVal < minVal) ? minVal : encoderPos - stepVal;

    rotating = false;
  }
}

unsigned int Encoder(bool dir)//Dir of 1 returns dirction (1 or 0). Dir of 0 returns encoder index
{
  static unsigned int state = 0;
  rotating = true;  // reset the debouncer

  if (dir == 0)
  {
    if (lastPos != encoderPos)
    {
      //Serial.print("Index:");
      //Serial.println(encoderPos);
      state = encoderPos;
    }
  }else
  {
    if(encoderPos > lastPos) 
    {
      //Serial.print("Direction:");
      //Serial.println("RIGHT");
      state = minVal;
    }else if (encoderPos < lastPos)
    {
      //Serial.print("Direction:");
      //Serial.println("LEFT");
      state = 0;
    }
  }
  lastPos = encoderPos;
  return(state);
}

double Thermistor(int RawADC) {
  double Temp;
  Temp = log(((10230000/RawADC) - 10000));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
  Temp = Temp - 273.15; // Convert Kelvin to Celcius
  return Temp;
}
 
