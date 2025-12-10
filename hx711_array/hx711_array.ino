#include "HX711.h"

// HX711 circuit wiring
// maximum supported sensors (compile-time array size)
const int MAX_NUM_SENSOR = 9;
const int MAX_NCH = MAX_NUM_SENSOR * 2;

// actual (runtime) sensor count — will be provided by host at startup
int num_sensor = 0; // will be set by host at startup
int nch = 0; // updated in setup after host message
const bool DEBUG = 0;
const float Delim= 2024.0; // delimiter value

const byte *pdel = (byte *) &Delim;

// control pin to signal external device when recording starts/stops
const int CONTROL_PIN = 51;
const int CTRL_LED_PIN = 53; // onboard LED to indicate control pin state

// waiting LED pin used while waiting for host to send sensor count
const int WAIT_LED_PIN = 52;

// HX711 arrays sized by maximum channels; use only first `nch` entries.
HX711 scale[MAX_NCH];

long reading[MAX_NCH] = {0};
float volt[MAX_NCH] = {0};

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
  // initialize control pin low
  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, LOW);
  pinMode(CTRL_LED_PIN, OUTPUT);
  digitalWrite(CTRL_LED_PIN, LOW);
  pinMode(WAIT_LED_PIN, OUTPUT);
  digitalWrite(WAIT_LED_PIN, LOW);

  // wait for host to send number of sensors as ASCII line like "N=3"
  // light LED while waiting; no timeout
  digitalWrite(WAIT_LED_PIN, HIGH);
  char buf[32];
  int idx = 0;
  // block until num_sensor is set by a valid "N=<num>" line
  while (num_sensor == 0) {
    while (Serial.available() > 0) {
      char ch = (char)Serial.read();
      if (ch == '\r' || ch == '\n') {
        if (idx > 0) {
          buf[idx] = '\0';
          String line = String(buf);
          line.trim();
          if (line.startsWith("N=")) {
            int v = line.substring(2).toInt();
            if (v >= 1 && v <= MAX_NUM_SENSOR) {
              num_sensor = v;
              // acknowledge to host
              Serial.println("OK");
              break;
            }
          }
          // reset buffer
          idx = 0;
        }
      } else {
        if (idx < (int)sizeof(buf) - 1) {
          buf[idx++] = ch;
        }
      }
    }
    delay(10);
  }
  nch = num_sensor * 2;
  digitalWrite(WAIT_LED_PIN, LOW);

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
  // if host sends a start/stop character, toggle control pin
  if (Serial.available() > 0) {
    int c = Serial.read();
    if (c == 'S') {
      digitalWrite(CONTROL_PIN, HIGH);
      digitalWrite(CTRL_LED_PIN, HIGH);
    } else if (c == 'E') {
      digitalWrite(CONTROL_PIN, LOW);
      digitalWrite(CTRL_LED_PIN, LOW);
    }
  }
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

