#include <Arduino.h>


float error;


// Pin list
int lmotor = 1;
int lmotorn = 2;
int rmotor = 11;
int rmotorl = 12;
int ir1 = 3;
int ir2 = 4;
int ir3 = 5;
int ir4 = 6;
int ir5 = 7;
int ir6 = 8;
int ir7 = 9;
int ir8 = 10;
int bsl = 127; // analogWrite base speed for left motor
int bsr = 127; // analogWrite base speed for right motor 
int pins[8] = {ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};


int ir[8];




void setup() {
  // put your setup code here, to run once:
  pinMode(1, OUTPUT);     // Motor pins are 1,2 and 11, 12 I'm fairly sure. Will have to check with
  pinMode(2, OUTPUT);     // Haridutt.
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  Serial.begin(9600);
}




void sensorsRead(int *sensor_data = ir, int *pins = pins, int n = 8){
  for(int i=0; i<n; i++){
    sensor_data[i] = digitalRead(pins[i]);
  }
}






// Function not ready yet
void filterSensors(int *sensor_data = ir, int n = 8){
    int high_streak_index[2] = {-1, -1};
    int high_streak = high_streak_index[1]-high_streak_inex[0]+1;
    for(int i=0; i<n; ++i){
        int streak_index[2] = {-1, -1};
        if(sensor_data[i]){
            streak_index[0] = i;
            do{
                ++i;
            } while(i<n && sensor_data[i]);
            streak_index[1] = i-1;
        }
        if(streak_index[1]-streak_index[0]+1 > high_streak){
            high_streak_index[0] = streak_index[0];
            high_streak_index[1] = streak_index[1];
        }
    }
}




float getDeviation(int sensor_data[], int n = 8){
  float index_shift = n/2+0.5;  // 3.5
  int number_of_high_sensors = 0; 
  float sum_of_high_sensors = 0;
  for(int i=0; i<n; i++){
    if(sensor_data[i]){
      number_of_high_sensors += 1; // number of on sensors
     
      sum_of_high_sensors += i-(index_shift); // adding on sensors based on their position on the ir board -- used to calculate deviation
    }
  }
  if(!number_of_high_sensors){
    return 0;  // edge case handling -- if everything is zero, return 0 instead of NaN
  }
  float deviation = -sum_of_high_sensors/number_of_high_sensors; // error value for pid
  return deviation;
}




void loop() {
  sensorsRead(); // get ir values
  for(int i=0;i<8;i++) {
  Serial.print(ir[i]); // debugging
  }
  Serial.println(" ");
  error=getDeviation(ir); 
  Serial.println(error); // also debugging
  pid=getpid(); // pid error value
  analogWrite(lmotor,bsl+pid);
  analogWrite(lmotorn, 0);
  analogWrite(rmotor, bsl-pid);
  analogWrite(rmoton, 0);
  
  // int op = digitalread(ir_pin);
  delay(250);
    // put your main code here, to run repeatedly:


}




int perr;
int p, i=0, d;
int kp=0.6, ki=0.4, kd=0.6; // these values need tweaking 
int pid;


float getpid() {
  perr = error;
  p = error;
  if (error==0) {
    i = 0;  
  }
  else {
    i = i+error;
  }
  d = error-perr;
  pid = (kp*p)+(ki*i)+(kd*d);
  return pid;
}
