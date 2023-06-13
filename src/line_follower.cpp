#include <Arduino.h> // BEFORE EXECUTING ON ARDUINO IDE, REMOVE THIS LINE. IT WILL THROW ERRORS.
#include <assert.h>


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

void digitaliseData(int sensor_data[], int const thresholds[], int const  n = 8)
{
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

  float current_pos;  //PID error
  float set_pos=0;    //Sensor 4,5 go high
  float reset,prev_error;
  unsigned long int prev_time;

//float perr;
//float p, i=0, d;
float kp=0.6, ki=0.4, kd=0.6; // these values need tweaking 
float pid;

signed  int value_assign[8]={-4,-3,-2,-1,1,2,3,4};

  //Find position of line
float getPosition(bool const dig_ir[]){
    int high_sensors=0;
    int sum_high=0;
    for(int i=0;i<8;i++){
      if (dig_ir[i]==1) high_sensors +=1;
      sum_high +=dig_ir[i]*value_assign[i];
    }
    if(high_sensors==0) return 0;   // if everything is zero, return 0 instead of NaN check this line
    
   /* Serial.print(" No of sensor high");  //For debugging
    Serial.print(high_sensors);
    Serial.print(" Sum");
    Serial.print(sum_high);*/
    float Position=sum_high/high_sensors;
    return Position;
}

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



void startSpinning(){
  analogWrite(lmotor,255);
  analogWrite(lmotorn,0);
  analogWrite(rmotor,0);
  analogWrite(rmotorn,255);
}

void stopMoving(){
  analogWrite(lmotor, 255);
  analogWrite(lmotorn, 255);
  analogWrite(rmotor, 255);
  analogWrite(rmotorn, 255);
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
  // is this for stopping when everything is black?
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
    pinMode(ir[i], INPUT); // Try removing the decalration if it doesn't work
  }
  // testFilterSensors();
  calibrate(thresholds, pins);
}

void loop() 
{
  int sensor_data[8];
sensorsRead(sensor_data, pins);
digitaliseData(ir, thresholds); // Function not yet written
  error=getDeviation(ir);
  Serial.print("error:");
  Serial.println(error);  // also debugging
  Serial.print("pid value:");

  pid=getPID(); // pid error value
  writeMotors(pid,sensor_data);

  
  // int op = digitalread(ir_pin);
}


