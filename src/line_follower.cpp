#include <Arduino.h> // BEFORE EXECUTING ON ARDUINO IDE, REMOVE THIS LINE. IT WILL THROW ERRORS.
#include <assert.h>


float error;


// Pin list
int lmotor = 3;
int lmotorn = 9;
int rmotor = 10;
int rmotorn = 11;
/*
int ir1 = 3;
int ir2 = 4;
int ir3 = 5;
int ir4 = 6;
int ir5 = 7;
int ir6 = 8;
int ir7 = 9;
int ir8 = 10;
int pins[8] = {ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};
*/
byte pins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int bsl = 127; // analogWrite base speed for left motor
int bsr = 127; // analogWrite base speed for right motor 

int ir[8];


void setup() {
  // put your setup code here, to run once:
  pinMode(1, OUTPUT);     // Motor pins are 3,9 and 10,11.
  pinMode(2, OUTPUT);     // From Haridutt.
  /*
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  */
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  Serial.begin(9600);
  for(int i=0; i<8; ++i){
    pinMode(ir[i], INPUT);
  }
}

// Task: Convert sensor reads to analog values


void sensorsRead(int *sensor_data, int *pins, int n = 8){
  for(int i=0; i<n; i++){
    sensor_data[i] = digitalRead(pins[i]);
  }
}

void sensorsRead(int sensor_data[], byte pins[], int n = 8){
  for(int i=0; i<n; i++){
    sensor_data[i] = analogRead(pins[i]);
  }
}

void printArray(int array[], int n=8){
  for(int i=0; i<n; ++i){
    Serial.print(array[i]);
    Serial.print(" ");
  }
}

void digitaliseData(int sensor_data[], int n = 8){
  // Function has to be written
}

// Alternative: Implement both hasStreak and findEndOfStreak as two instances of a sequence detector

/*
bool hasStreak(int const sensor_data[], int const n = 8){
  for(int i=0; i<n-1; ++i){
    if(sensor_data[i] && sensor_data[i+1]){ // Bug: 101
      return true;
    }
  }
  return false;
}
*/


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
    assert (isEqual(sensor_data, expected, n));
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


void loop() {
  sensorsRead(ir, pins); // get ir values

  // printArray(ir);
  digitaliseData(ir); // Function not yet written

  for(int i=0;i<8;i++) {
  Serial.print(ir[i]); // debugging
  }
  Serial.println(" ");
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
  delay(250);
}