#include <Arduino.h>

float error;

// Pin list
int ir1 = 3;
int ir2 = 4;
int ir3 = 5;
int ir4 = 6;
int ir5 = 7;
int ir6 = 8;
int ir7 = 9;
int ir8 = 10;
int pins[8] = {ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};

int ir[8];


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
}


void sensorsRead(int *sensor_data = ir, int *pins = pins, int n = 8){
  for(int i=0; i<n; i++){
    sensor_data[i] = digitalRead(pins[i]);
  }
}


float getDeviation(int sensor_data[], int n = 8){ 
  float index_shift = n/2+0.5;
  int number_of_high_sensors = 0;
  float sum_of_high_sensors = 0;
  for(int i=0; i<n; i++){
    if(sensor_data[i]){
      number_of_high_sensors += 1;
      
      sum_of_high_sensors += i-(index_shift);
    }
  }
  if(!number_of_high_sensors){
    return 0;
  }
  float deviation = -sum_of_high_sensors/number_of_high_sensors;
  return deviation;
}


void loop() {
  sensorsRead();
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


int perr;
int p, i=0, d;
int kp=0, ki=0, kd=0;
int pid;

void getpid() {
  perr = error;
  p = error;
  i = i+error;
  d = error-perr;
  pid = (kp*p)+(ki*i)+(kd*d);
}

