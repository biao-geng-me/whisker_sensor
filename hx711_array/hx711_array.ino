#include "HX711.h"

// HX711 circuit wiring
const int num_sensor = 9;
const int nch = num_sensor*2; // total number of channels
const bool DEBUG = 0;
const float Delim= 2024.0; // delimiter value
const byte *pdel = (byte *) &Delim;

HX711 scale[nch];

long reading[nch] = {0};
float volt[nch] = {0};

unsigned long prevTime = 0;
int counter = 0;
int nAvg = 160; // 
int sampleTime = 0, currentTime;
float samplePerSec = 0;

int nread = 0;
unsigned long prev_micros, accum_micros = 0;

byte *p = (byte *) volt;



void setup() {
  Serial.begin(2000000);
  for(int i=1;i<=nch;i++){
    // Data pin, clock pin
    int dpin = 2*i;
    int cpin = 2*i+1;  
    // if(i==6) { // avoid pin 13 as inputd (2025-10-29 bgeng: The official doc says pin13 is hard to use as input, but the issue was loose ground connection not pin13.)
    //   dpin = 52;
    //   cpin = 53;
    // }
    scale[i-1].begin(dpin,cpin);
  }
}

void loop() {
  counter++;
  for(int i=1;i<=nch;i++){
    while(1){
      if (scale[i-1].is_ready()) {
        nread++;
        prev_micros = micros();
        reading[i-1] = scale[i-1].read();
        accum_micros += micros() - prev_micros;
        volt[i-1] = reading[i-1]/pow(2,24)*4.3*1000/128;
        break;
      }
    }
  }
  if (DEBUG) {
    if (counter == nAvg) { // average over nAvg samples
      counter = 0;
      currentTime = millis();
      sampleTime = currentTime - prevTime;
      prevTime = currentTime;
      samplePerSec = nAvg*1000.0 / sampleTime; 

      Serial.print(sampleTime);
      Serial.print (", ");
      Serial.print(float(sampleTime)/nAvg,2);
      Serial.print (", ");
      Serial.print(samplePerSec,1);
      Serial.print (", ");
      Serial.print(float(accum_micros)/nread,2);
      // Serial.print(",");
      // Serial.print(nread);
      nread = 0;
      accum_micros = 0;
      Serial.print ("\t");
      for (int i=1;i<=nch;i++){
        Serial.print(volt[i-1],7);
        if (i<nch) {
          Serial.print(",");
        }
        else{
          Serial.println();
        }
      }
    
    }
  }
  else {
    /* ascii print
    for (int i=1;i<=nch;i++){
      Serial.print(volt[i-1],1);
      Serial.print(",");
    }
    Serial.println();
    */

    /* binary bytes */
    Serial.write(p,nch*4);
    Serial.println();
    Serial.write(pdel,4);

  }

}

