#include<Arduino.h>

// uno code 
// bool check;
float error;
int ir1 = 3;
int ir2 = 4;
int ir3 = 5;
int ir4 = 6;
int ir5 = 7;
int ir6 = 8;
int ir7 = 9;
int ir8 = 10;
int ir[8];

int perr;
int p, i=0, d;
int kp=0, ki=0, kd=0;
int pid;

void geterror2();
void geterror();
void getpid(); 
void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
}

float getDeviation(int sensors[], int n = 8){
  int counter = 0;
  float deviation = 0;
  for(int i=0; i<n; i++){
    if(sensors[i]){
      counter += 1;
      deviation += i-3.5;
    }
  }
  if(not counter){
    return 0;
  }
  deviation = -deviation/counter;
  return deviation;
}

void loop() {
  ir[0] = digitalRead(ir1);
  ir[1] = digitalRead(ir2);
  ir[2] = digitalRead(ir3);
  ir[3] = digitalRead(ir4);
  ir[4] = digitalRead(ir5);
  ir[5] = digitalRead(ir6);
  ir[6] = digitalRead(ir7);
  ir[7] = digitalRead(ir8);
  for(int i=0;i<8;i++) {
  Serial.print(ir[i]);
  }
  Serial.println(" ");
  error=getDeviation(ir);
  Serial.println(error);
  // int op = digitalread(ir_pin);
  delay(250);
    // put your main code here, to run repeatedly:

}
void geterror() {
  if (ir[0]== 1 && ir[1] == 0 && ir[2] == 0 && ir[3] == 0 && ir[4] == 0 && ir[5] == 0 && ir[6] == 0 && ir[7] == 0) {
    error=7;
  }
  else if (ir[0]== 1 && ir[1] == 1 && ir[2] == 0 && ir[3] == 0 && ir[4] == 0 && ir[5] == 0 && ir[6] == 0 && ir[7] == 0) {
    error=6;
  }
  else if (ir[0]== 0 && ir[1] == 1 && ir[2] == 0 && ir[3] == 0 && ir[4] == 0 && ir[5] == 0 && ir[6] == 0 && ir[7] == 0) {
    error=5;
  }
  else if (ir[0]== 0 && ir[1] == 1 && ir[2] == 1 && ir[3] == 0 && ir[4] == 0 && ir[5] == 0 && ir[6] == 0 && ir[7] == 0) {
    error=4;
  }
  else if (ir[0]== 0 && ir[1] == 0 && ir[2] == 1 && ir[3] == 0 && ir[4] == 0 && ir[5] == 0 && ir[6] == 0 && ir[7] == 0) {
    error=3;
  }
  else if (ir[0]== 0 && ir[1] == 0 && ir[2] == 1 && ir[3] == 1 && ir[4] == 0 && ir[5] == 0 && ir[6] == 0 && ir[7] == 0) {
    error=2;
  }else if (ir[0]== 0 && ir[1] == 0 && ir[2] == 0 && ir[3] == 1 && ir[4] == 0 && ir[5] == 0 && ir[6] == 0 && ir[7] == 0) {
    error=1;
  }
  else if (ir[0]== 0 && ir[1] == 0 && ir[2] == 0 && ir[3] == 1 && ir[4] == 1 && ir[5] == 0 && ir[6] == 0 && ir[7] == 0) {
    error=0;
  }
  else if (ir[0]== 0 && ir[1] == 0 && ir[2] == 0 && ir[3] == 0 && ir[4] == 1 && ir[5] == 0 && ir[6] == 0 && ir[7] == 0) {
    error=-1;
  }
  else if (ir[0]== 0 && ir[1] == 0 && ir[2] == 0 && ir[3] == 0 && ir[4] == 1 && ir[5] == 1 && ir[6] == 0 && ir[7] == 0) {
    error=-2;
  }
  else if (ir[0]== 0 && ir[1] == 0 && ir[2] == 0 && ir[3] == 0 && ir[4] == 0 && ir[5] == 1 && ir[6] == 0 && ir[7] == 0) {
    error=-3;
  }
  else if (ir[0]== 0 && ir[1] == 0 && ir[2] == 0 && ir[3] == 0 && ir[4] == 0 && ir[5] == 1 && ir[6] == 1 && ir[7] == 0) {
    error=-4;
  }
  else if (ir[0]== 0 && ir[1] == 0 && ir[2] == 0 && ir[3] == 0 && ir[4] == 0 && ir[5] == 0 && ir[6] == 1 && ir[7] == 0) {
    error=-5;
  }
  else if (ir[0]== 0 && ir[1] == 0 && ir[2] == 0 && ir[3] == 0 && ir[4] == 0 && ir[5] == 0 && ir[6] == 1 && ir[7] == 1) {
    error=-6;
  }
  else if (ir[0]== 0 && ir[1] == 0 && ir[2] == 0 && ir[3] == 0 && ir[4] == 0 && ir[5] == 0 && ir[6] == 0 && ir[7] == 1) {
    error=-7;
  }
  else if (ir[0]== 0 && ir[1] == 0 && ir[2] == 0 && ir[3] == 0 && ir[4] == 0 && ir[5] == 0 && ir[6] == 0 && ir[7] == 0) {
    error=9999;
  }
}

void getpid() {
  perr = error;
  p = error;
  i = i+error;
  d = error-perr;
  pid = (kp*p)+(ki*i)+(kd*d);
}
