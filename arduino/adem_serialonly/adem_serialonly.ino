/*
/* 
 Lieven Blancke, 2015-08-13    

  connect Arduino analog PIN 0 to the analog output of one of these sensors:
  - PPD60V analog output
  - TP7 on the PCB of PPD42N,
  - TP8 on the PCB of DSM5011

*/

// PPD Variable Setup
unsigned long analogcount = 0;
double analogtotal;
unsigned long starttime = 0;
unsigned long sampletime_ms = 1000;
unsigned long analogvalue = 0;
double LconAV = 0;
double Lugm3_2 = 0;

void setup()
{
  // Start Serial for debugging on the Serial Monitor
  Serial.begin(9600);
  
  starttime = millis();
}

void loop()
{ 
  // Read PPD value from Analog Input Pin 0
  analogvalue = analogvalue + analogRead(0);
  analogcount++;
  if ((millis()-starttime) > sampletime_ms) // every second
  {
    starttime = millis(); // moved to create more regular periods.
    analogvalue = analogvalue/analogcount;
    analogtotal = (analogvalue*5.0)/1024;
    LconAV = (240.0*pow(analogtotal,6) - 2491.3*pow(analogtotal,5) + 9448.7*pow(analogtotal,4) - 14840.0*pow(analogtotal,3) + 10684.0*pow(analogtotal,2) + 2211.8*(analogtotal) + 7.9623);
    Lugm3_2 = (0.0000000495*pow(LconAV,2) + 0.0015247767*(LconAV));//Cardboard
    if(Lugm3_2 < 0){
      Lugm3_2 = 0;
    }
 
    //Serial.println();
    Serial.print(F(" Analog Total: "));
    Serial.print(analogtotal);
    Serial.print(F(" Lower Concentration: "));
    Serial.print(LconAV);
    Serial.print(F(" Lower ugm3_2: "));
    Serial.print(Lugm3_2);
    Serial.println();
    Serial.println ("analogcount=" + String(analogcount, DEC));    
    analogvalue = 0;
    analogcount = 0;
  }
}
