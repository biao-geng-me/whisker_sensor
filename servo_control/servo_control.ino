#include <Servo.h>

// create Servo object
Servo servo1;

int initAngle = 90;
int servoPin1 = 9;
int potPin1 = A1; // potential meter
int angle0 = initAngle;
int angleInc = 5;
int ninc = 8;
int thold = 1; // seconds
long SERIAL_WAIT_TIME = 1800*1000L; // ms, L to indicate long integer


int cmdInt = 0;

void setup() {
  servo1.attach(servoPin1);
  servo1.write(initAngle);

  Serial.begin(9600);
  Serial.println("Arduino is ready");
  Serial.setTimeout(SERIAL_WAIT_TIME); //ms

  while(1){
      int potReading = analogRead(potPin1);
      angle0 = initAngle + map(potReading, 0, 1023, -90, 90);

      servo1.write(angle0);
      Serial.println(servo1.read());
      delay(50);
      if (Serial.available()){
        cmdInt = Serial.parseInt();
        Serial.println(cmdInt);

        if (cmdInt>900){
          return;
        }
      }
  }

}

void loop() {

  Serial.println("main");
  delay(1000);

  if (cmdInt == 901) step_inc_dec();
  if (cmdInt == 902) step_inc_release();
  if (cmdInt == 903) cyclic_triangle();
  if (cmdInt == 904) repeat_load();
  if (cmdInt == 905) cyclic_sine();
 
  Serial.println("enter cmd number to continue tests");
  cmdInt = Serial.parseInt();

}

void step_inc_dec() {
  // do a increment and increment cycle
  int a;
  Serial.println("running step inc-dec cycle");

  Serial.println("enter: increment size in degrees, number of steps, hold time (s)");
  angleInc = Serial.parseInt();
  ninc = Serial.parseInt();
  thold = Serial.parseInt();

  Serial.println(angleInc);
  Serial.println(ninc);
  Serial.println(thold);

  servo1.write(angle0);
  delay(2*1000);
  for (int i=1; i<=ninc; i++){
    a = angle0+angleInc*i;
    servo1.write(a);
    Serial.println(servo1.read());
    delay(thold*1000);
  }

  for (int i=ninc; i>=0; i--){
    a = angle0+angleInc*i;
    servo1.write(a);
    Serial.println(servo1.read());
    delay(thold*1000);
  }

}

void step_inc_release() {
  //
  int a,nrep,t_rep;

  Serial.println("running step inc-release cycle");

  Serial.println("enter: increment size in degrees, number of steps, hold time (s)");
  angleInc = Serial.parseInt();
  ninc = Serial.parseInt();
  thold = Serial.parseInt();

  Serial.println(angleInc);
  Serial.println(ninc);
  Serial.println(thold);

  Serial.println("enter: number of repeats, hold time between repeats(s)");
  nrep = Serial.parseInt();
  t_rep = Serial.parseInt();

  servo1.write(angle0);
  delay(3*1000);

  for (int j=1; j<=nrep; j++){
    Serial.print("repeat ");
    Serial.println(j);
    for (int i=1; i<=ninc; i++){
      // slow load
      int a0 = angle0+angleInc*(i-1);
      int a1 = angle0+angleInc*i;
      if (angleInc>0){
        for (int a = a0;a<=a1;a++){;
        servo1.write(a);
        Serial.println(a);
        delay(100);
        }
      }
      else {
        for (int a = a0;a>=a1;a--){;
        servo1.write(a);
        Serial.println(a);
        delay(100);
        }
      }

      delay(thold*1000);
    }
    
    // slow release
    if (angleInc>0) {
      for (int i=ninc*angleInc; i>=0; i--){
        a = angle0+i;
        servo1.write(a);
        Serial.println(servo1.read());
        delay(100);
      }
    }
    else {
      for (int i=ninc*angleInc; i<=0; i++){
        a = angle0+i;
        servo1.write(a);
        Serial.println(servo1.read());
        delay(100);
      }
    }
    delay(t_rep*1000);
  }
}

void cyclic_triangle() {
  float td = 0, freq;
  int aamp = 5, a;

  cmdInt = 0;
  Serial.println("running cyclic triangular load");
  Serial.println("enter amplitude in degrees");
  aamp = Serial.parseInt();
  Serial.println(aamp);

  Serial.println("enter frequency in Hz");
  freq = Serial.parseFloat();
  Serial.println(freq);

  td = 1000/freq/(aamp*4);
  servo1.write(angle0);
  delay(2*1000);

  while(1) {
    for (int i=0; i<=aamp; i++) {
      a = angle0+i;
      servo1.write(a);
      Serial.println(a);
      delay(td);
    }

    for (int i=aamp-1; i>=-aamp; i--) {
      a = angle0+i;
      servo1.write(a);
      Serial.println(a);
      delay(td);
    }

    for (int i=-aamp+1; i<0; i++) {
      a = angle0+i;
      servo1.write(a);
      Serial.println(a);
      delay(td);

    }

  }

}

void repeat_load() {
  // do a increment and increment cycle
  int a;
  Serial.println("running single load repeat");

  Serial.println("enter: load in degrees, number of repeats, hold time (s)");
  int angleLoad = Serial.parseInt();
  int nrep = Serial.parseInt();
  int th = Serial.parseInt();

  Serial.println(angleLoad);
  Serial.println(nrep);
  Serial.println(th);

  a = angle0+angleLoad;
  
  servo1.write(angle0);
  delay(3*1000);
  for (int i=1; i<=nrep; i++){
    
    for (int j=angle0; j<=a; j++){
      servo1.write(j);
      Serial.println(servo1.read());
      delay(100);
    }

    delay(th*1000);

    for (int j=a; j>=angle0; j--){
      servo1.write(j);
      Serial.println(servo1.read());
      delay(100);
    }

    servo1.write(angle0);
    Serial.println(servo1.read());
    delay(th*1000);
  }


}

void cyclic_sine() {
  const float K1 = 1000.0;
  float td = 0, freq, tinc;
  int aamp = 5, a;

  unsigned long t_start;

  cmdInt = 0;
  Serial.println("running cyclic sine load");
  Serial.println("enter amplitude in degrees");
  aamp = Serial.parseInt();
  Serial.println(aamp);

  Serial.println("enter frequency in Hz");
  freq = Serial.parseFloat();
  Serial.println(freq);

  td = 1000/freq/(aamp*4);
  servo1.write(angle0);
  delay(2*1000);

  t_start = millis();
  while(1) {
    Serial.println(servo1.read());
    tinc = (millis() - t_start)/K1;
    a = angle0+round(aamp*sin(2*3.1415926*freq*tinc));
    servo1.write(a);
    delay(td);

  }
}
