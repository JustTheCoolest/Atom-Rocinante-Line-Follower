#include "C:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\Arduino.h" // BEFORE EXECUTING ON ARDUINO IDE, REMOVE THIS LINE. IT WILL THROW ERRORS.
//#include <assert.h>
#include "../lib/backend.cpp"

// Pin list
int lmotor = 3;
int lmotorn = 9;
int rmotor = 10;
int rmotorn = 11;
byte pins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int bsl = 127; // analogWrite base speed for left motor
int bsr = 127; // analogWrite base speed for right motor 

int ir[8];

void sensorsRead(int sensor_data[], byte const pins[], int n = 8){
  for(int i=0; i<n; i++){
    sensor_data[i] = analogRead(pins[i]);
  }
}

void printArray(int array[], int n=8){
  for(int i=0; i<n; ++i){
    Serial.print(array[i]);
    Serial.print(" ");
  }
  Serial.print("\n");
}

// Task : Stop at full black

  float current_pos;  //PID error
  float set_pos=0;    //Sensor 4,5 go high
  float reset,prev_error;
  unsigned long int prev_time;

//float perr;
//float p, i=0, d;
float kp=70, ki=0, kd=0; // these values need tweaking 
float pid;

float getPID()
{
  //Error is the difference between the postion of the bot and the position we want it to be
  unsigned long current_time=millis();
  double del_time=current_time-prev_time;

  float error=set_pos-current_pos;                         //Steady state error
  reset += error*del_time;                                 //Reset-The small errors that get accumulated over time *reset gets added over time , hence global variable
  float rate_error= error-prev_error/del_time;             //Rate of change of error

  float pid=kp*(error) + ki*(reset) + kd*(rate_error);     //Calculate PID value

  prev_error=error;
  prev_time=current_time;

  return pid;
  
}
void writeMotors(const int pid, const int sensor_data[]){
    for(int i=0; i<8; ++i)
    {
      if(sensor_data[i])
      {
        analogWrite(lmotor,bsl-pid);          //Check pid values, direction of turning and adjust
        analogWrite(rmotor,bsl+pid);
        break;
      }
    }
}


void startSpinning(int const spin){
  analogWrite(lmotor,255-spin);
  analogWrite(rmotor,spin);
}

void stopMoving(){
  analogWrite(lmotor, 255);
  analogWrite(lmotorn, 255);
  analogWrite(rmotor, 255);
  analogWrite(rmotorn, 255);
}


void calibrate(int thresholds[], byte const pins[], int const n = 8){
  startSpinning(255);
  int sensorData[n];
  int minValues[n], maxValues[n];
  for(int i=0; i<200; ++i){
    sensorsRead(sensorData, pins, n);
    for(int j=0; j<n; ++j){
      if(sensorData[j]<minValues[j] || i==0){
        minValues[j] = sensorData[j];
      }
      if(sensorData[j]>maxValues[j] || i==0){
        maxValues[j] = sensorData[j];
      }
      delay(1);
    }
  }
  // is this for stopping when everything is black?
  for(int i=0; i<n; ++i){
    thresholds[i] = 0.75*maxValues[i] - 0.25*minValues[i] + 0.5;
  }
  stopMoving();
}

int findEndOfStreak(int const sensor_data[], int j, int const n = 8){
  bool junk;
  return findEndOfStreak(sensor_data, &junk, j, n);
}

void swap(int *const a, int *const b){
  int temp = *a;
  *a = *b;
  *b = temp;
}

void reverseArray(int array[], const int n){
  for(int i=0; i<n/2; ++i){
    swap(&array[i], &array[n-i]);
  }
}

void findTurn(int sensor_data[], int const line_thickness, int const n = 8){
  // Find deviation must be called before this function
  if(findEndOfStreak(sensor_data, -1)>line_thickness+1){ // +1 for confidence
    startSpinning(0);
    return;
  }
  reverseArray(sensor_data, n);
  if(findEndOfStreak(sensor_data, -1)>line_thickness+1){ // +1 for confidence
    startSpinning(255);
    return;
  }
  return;
}

int thresholds[8];

void setup() {
  // put your setup code here, to run once:
  pinMode(3, OUTPUT);     // Motor pins
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  Serial.begin(9600);
  for(int i=0; i<8; ++i){
    pinMode(ir[i], INPUT); // Try removing the decalration if it doesn't work
  }
  // testFilterSensors();
  calibrate(thresholds, pins);
}

void loop() 
{
  int sensor_data[8];
  sensorsRead(sensor_data, pins);
  digitaliseData(ir, thresholds);
  float error=getDeviation(ir);
  Serial.print("error:");
  Serial.println(error);  // also debugging
  Serial.print("pid value:");

  pid=getPID(); // pid error value
  writeMotors(pid,sensor_data);

  
  // int op = digitalread(ir_pin);
}


