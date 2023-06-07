#include <Arduino.h> // BEFORE EXECUTING ON ARDUINO IDE, REMOVE THIS LINE. IT WILL THROW ERRORS.
//#include <assert.h>


float error;


// Pin list
int lmotor = 3;
int lmotorn = 9;
int rmotor = 10;
int rmotorn = 11;
byte pins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int bsl = 127; // analogWrite base speed for left motor
int bsr = 127; // analogWrite base speed for right motor 

int ir[8];

// Task: Convert sensor reads to analog values


void sensorsDigitalRead(int sensor_data[], byte pins[], int n = 8){
  for(int i=0; i<n; i++){
    sensor_data[i] = digitalRead(pins[i]);
  }
}

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

void digitaliseData(int sensor_data[], int const thresholds[], int const  n = 8){
  for(int i=0; i<n; ++i){
    sensor_data[i] = sensor_data[i] > thresholds[i];
  }
}

// Alternative: Implement both hasStreak and findEndOfStreak as two instances of a sequence detector


int findEndOfStreak(int sensor_data[], bool *streak_present, int j, int const n = 8){
  for(++j; j<n-2; ++j){
    if(!sensor_data[j]){
      if(!sensor_data[++j]){
        *streak_present = true;
        return j+1;
      }
      continue;
    }
  }
  return -1;
}


void filterSensors(int sensor_data[] = ir, int n = 8){
  bool streak_present = false;
  /*
  if(!hasStreak(sensor_data)){
    return; // // If there is no streak, a single 1 is not treated as noise, it could be our true data
  }
  */
  for(int i=0; i<n; ++i){
    if(!sensor_data[i]){
      continue;
    }
    int j = findEndOfStreak(sensor_data, &streak_present, i);
    if(j<-1){
      return;
    }
    if(j==i+3){
      sensor_data[i] = 0;
    }
    i = j;
  }
}


bool isEqual(int const a[], int const b[], int const n = 8){
  for(int i=0; i<n; ++i){
    if(a[i]!=b[i]){
      return false;
    }
  }
  return true;
}


void testFilterSensors(){
  int test_data[][2][8] = {
    {{0, 0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0, 0}}, // blank
    {{0, 0, 0, 1, 0, 0, 1, 0},{0, 0, 0, 1, 0, 0, 1, 0}}, // single ones, undifferentiable between real data and noise
    {{1, 1, 1, 0, 0, 0, 0, 0},{1, 1, 1, 0, 0, 0, 0, 0}}, // Simple streak
    {{1, 1, 1, 0, 0, 1, 0, 0}, {1, 1, 1, 0, 0, 0, 0, 0}}, // Stray correction
    {{1, 1, 1, 0, 0, 1, 0, 1}, {1, 1, 1, 0, 0, 1, 0, 1}}, // Dual streak with gap in between one. Single gaps are also considered part of the streak cause what if the gap...
    // ...actuallly the error and not the 1s
    {{1, 1, 1, 0, 1, 0, 0, 1}, {1, 1, 1, 0, 1, 0, 0, 0}}, // Streak with gap and stray
    {{0, 1, 0, 0, 0, 1, 0, 1}, {0, 0, 0, 0, 0, 1, 0, 1}}  // Stray to the left of streak //flag
  };
  int const n=8;
  for(int unsigned i=0; i<sizeof(test_data); ++i){
    int *sensor_data = test_data[i][0];
    int *expected = test_data[i][1];
    filterSensors(sensor_data);
    //assert(isEqual(sensor_data, expected, n));
  }
}


float getDeviation(int sensor_data[], int n = 8){
  float index_shift = n/2-0.5;  // 3.5
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

// Task : Stop at full black


float perr;
float p, i=0, d;
float kp=0.6, ki=0.4, kd=0.6; // these values need tweaking 
float pid;


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


void startSpinning(){
  digitalWrite(lmotor,HIGH);
  digitalWrite(lmotorn,LOW);
  digitalWrite(rmotor,LOW);
  digitalWrite(rmotorn,HIGH);
}

void stopMoving(){
  digitalWrite(lmotor, HIGH);
  digitalWrite(lmotorn, HIGH);
  digitalWrite(rmotor, HIGH);
  digitalWrite(rmotorn, HIGH);
}


void calibrate(int thresholds[], byte const pins[], int const n = 8){
  startSpinning();
  int sensorData[n];
  int minValues[n], maxValues[n];
  for(int i=0; i<50; ++i){
    sensorsRead(sensorData, pins, n);
    for(int j=0; j<n; ++j){
      if(sensorData[j]<minValues[j] || i==0){
        minValues[j] = sensorData[j];
      }
      if(sensorData[j]>maxValues[j] || i==0){
        maxValues[j] = sensorData[j];
      }
    }
  }
  for(int i=0; i<n; ++i){
    thresholds[i] = 0.75*maxValues[i] - 0.25*minValues[i] + 0.5;
  }
  stopMoving();
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
    pinMode(ir[i], INPUT);
  }
  // testFilterSensors();
  calibrate(thresholds, pins);
}

void loop() {
  sensorsRead(ir, pins); // get ir values
  // printArray(ir);
  digitaliseData(ir, thresholds); // Function not yet written
  error=getDeviation(ir);
  Serial.print("error:");
  Serial.println(error);  // also debugging
  Serial.print("pid value:");
  pid=getpid(); // pid error value
  analogWrite(lmotor,bsl+pid);
  analogWrite(lmotorn, 0);
  analogWrite(rmotor, bsl-pid);
  analogWrite(rmotorn, 0);
  
  // int op = digitalread(ir_pin);
}
