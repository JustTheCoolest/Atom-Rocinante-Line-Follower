#include "C:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\Arduino.h" // BEFORE EXECUTING ON ARDUINO IDE, REMOVE THIS LINE. IT WILL THROW ERRORS.
//#include <assert.h>
#include "../lib/backend.cpp"
#include "constants.cpp"

// Pin list
int lmotor = 3;
int lmotorn = 9;
int rmotor = 10;
int rmotorn = 11;
byte pins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

constexpr int n=8;

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

//float perr;
//float p, i=0, d;
float kp=70, ki=0, kd=0; // these values need tweaking 
float pid;

float getPID(float error)
{
  static float reset,prev_error;
  static unsigned long int prev_time;
  //Error is the difference between the postion of the bot and the position we want it to be
  unsigned long current_time=millis();
  double del_time=current_time-prev_time;                   //Steady state error
  reset += error*del_time;                                 //Reset-The small errors that get accumulated over time *reset gets added over time , hence global variable
  float rate_error= (error-prev_error)/del_time;             //Rate of change of error
  float pid=kp*(error) + ki*(reset) + kd*(rate_error);     //Calculate PID value

  prev_error=error;
  prev_time=current_time;

  return pid;
  
}

void stopMoving(){
  digitalWrite(lmotor, HIGH);
  digitalWrite(lmotorn, HIGH);
  digitalWrite(rmotor, HIGH);
  digitalWrite(rmotorn, HIGH);
}

bool isAllLow(const int sensor_data[], const int n=8){
  for(int i=0; i<n; ++i){
    if(sensor_data[i]){
      return false;
    }
  }
  return true;
}

void writeMotorWithPins(int motor_pwm, byte const analog_pin_for_forward, byte const digital_pin_for_forward){
  byte analog_pin, digital_pin;
  if(motor_pwm<0){
    analog_pin = digital_pin_for_forward;
    digital_pin = analog_pin_for_forward;
    motor_pwm *= -1;
  }
  else{
    analog_pin = analog_pin_for_forward;
    digital_pin = digital_pin_for_forward;
  }
  analogWrite(analog_pin_for_forward, motor_pwm);
  digitalWrite(digital_pin_for_forward, HIGH);
}

void writeMotors(const int pid){
  static const int base_pwm = 40;
  int left_motor_pwm = base_pwm+pid;
  int right_motor_pwm = base_pwm-pid;
  writeMotorWithPins(left_motor_pwm, lmotor, lmotorn);
  writeMotorWithPins(right_motor_pwm, rmotor, rmotorn);
}

bool checkWhiteToStopMoving(int sensor_data[], unsigned int const response_delay = 1500, const int n=8){
  if(!isAllLow(sensor_data, n)){
    return true;
  }
  static unsigned long int target_time;
  static bool response_delay_flag;
  if(!response_delay_flag){
    target_time = millis() + response_delay;
    response_delay_flag = true;
  }
  if(millis()>target_time){
    stopMoving();
    response_delay_flag = false;
  }
  return false;
}


void startSpinning(bool const clockwise_flag){
  digitalWrite(lmotor, clockwise_flag);
  digitalWrite(lmotorn, !clockwise_flag);
  digitalWrite(rmotor, !clockwise_flag);
  digitalWrite(rmotorn, clockwise_flag);
}


void calibrate(int thresholds[], byte const pins[], int const n = 8){
  analogWrite(lmotor, 127);
  digitalWrite(lmotorn, HIGH);
  analogWrite(rmotorn, 127);
  digitalWrite(rmotor, HIGH);
  //startSpinning(HIGH);
  int sensorData[n];
  int minValues[n], maxValues[n];
  for(int i=0; i<2000; ++i){
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
    thresholds[i] = minValues[i] + calibration_ratio * (maxValues[i] - minValues[i]);
  }
 //Serial.print("no stop");
  
}

/*
int findEndOfStreak(int const sensor_data[], int j, int const n = 8){
  bool junk;
  return findEndOfStreak(sensor_data, &junk, j, n);
}
*/

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
  // testFilterSensors();
  calibrate(thresholds, pins);
  printArray(thresholds);
}

void testingIncrementConstant(unsigned const int increment_delay){
  //kd = int(millis() / increment_delay);
}

// Task : Find line if out of line

void loop() 
{
  static int sensor_data[8];
  sensorsRead(sensor_data, pins);
  printArray(sensor_data);
  digitaliseData(sensor_data, thresholds);
  printArray(sensor_data);
  if(!checkWhiteToStopMoving(sensor_data)){
    return;
  }
  Line line;
  line.findBranches(sensor_data, n);
  struct streak branch = line.selectBranch(sensor_data, n);
  float error=line.getDeviation(branch, n);
  Serial.print(error);
  pid=getPID(error); // pid error value
  int number_of_high_sensors = numberOfHighSensors(sensor_data, n);
  pid = correctPidValueForOrientation(pid, number_of_high_sensors);
  Serial.print(pid);
  writeMotors(pid);
  Serial.print("\n");
  // int op = digitalread(ir_pin);
}


