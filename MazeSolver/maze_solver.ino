/* TODO: Base speed, response delay, wheels, continuous calibration */

#include <math.h>

// #include "constants.h"
constexpr float kp = 80,ki = 0,kd =0;
constexpr int base_pwm = 0; // max speed
constexpr unsigned int response_delay = 0;
constexpr float calibration_ratio = 0.6;

constexpr int n = 8;
constexpr int sensor_distances[] = {-4, -3, -2, -1, +1, +2, +3, +4};
constexpr bool black_line = false;
constexpr int line_width = 2;

// constexpr int west = 0, north = 1, east = 2, south = 3;
// int heading = north;
constexpr int left = -1, front = 0, right = 1, back = 2;

const int en[2] = {0, 0}; // pin to enable motors " can be shorted to 3v3 or 5v if no pins are available " 
const int mot_pins[6] = {0, 0, 0, 0, 0, 0}; // tb6612fng motor driver pins
int ir1 = A0;
int ir2 = A1;
int ir3 = A2;
int ir4 = A3;
int ir5 = A4;
int ir6 = A5;
int ir7 = A6;
int ir8 = A7;
int pins[8] = {ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};
// end constants.h

//int thresholds[8] = {150, 150, 150, 150, 150, 150, 150, 150};
int thresholds[8];

int left_motor_pwm;
int right_motor_pwm;
float current_pos;  //PID error
float set_pos=0;    //Sensor 4,5 go high
float reset,prev_error;
unsigned long int prev_time;

int an_ir[8];
int dig_ir[8];

// #include "TB6612FNG.cpp"
void assignMotor(int speed, const bool forward_direction, const int in1, const int in2, const int pwm_pin){
  // pwm: 0-255
  // dir: High is forward
  speed = speed>255 ? 255 :speed<0 ? 0 :speed;
  digitalWrite(in1, forward_direction);
  digitalWrite(in2, !forward_direction);
  analogWrite(pwm_pin, speed);
}

void writeMotor(const int motor_pwm, const int mot_pins[] = mot_pins){
  assignMotor(abs(motor_pwm), motor_pwm > 0, mot_pins[0], mot_pins[1], mot_pins[2]);
}

void writeMotors(const int left_motor_pwm, const int right_motor_pwm, const int mot_pins[6] = mot_pins){
  writeMotor(left_motor_pwm, mot_pins);
  writeMotor(right_motor_pwm, mot_pins+3);
}

void startSpinning(const bool clockwise_flag, const int mot_pins[6] = mot_pins){
  assignMotor(255, clockwise_flag, mot_pins[0], mot_pins[1], mot_pins[2]);
  assignMotor(255, !clockwise_flag, mot_pins[3], mot_pins[4], mot_pins[5]);
}

void stopMoving(const int mot_pins[6] = mot_pins){
  assignMotor(0, HIGH, mot_pins[0], mot_pins[1], mot_pins[2]);
  assignMotor(0, HIGH, mot_pins[3], mot_pins[4], mot_pins[5]);
}

void writeMotors(const int pid){
  left_motor_pwm = pid>0 ? base_pwm : base_pwm-pid;
  right_motor_pwm = pid>0 ? base_pwm-pid : base_pwm;
  writeMotors(base_pwm + pid, base_pwm - pid);  
}
// end of TB6612FNG.cpp

void sensorsRead(int sensor_data[]=an_ir, int const pins[]=pins, int n = 8){
  for(int i=0; i<n; i++){
    sensor_data[i] = analogRead(pins[i]);
  }
}

void digitaliseData(bool mode, int sensor_data[]=an_ir, int const thresholds[]=thresholds, int const  n = 8)
{
  int sign = mode ? +1 : -1;
  for(int i=0; i<n; ++i){
    dig_ir[i] = sign * sensor_data[i] > sign * thresholds[i];
  }
}

  
  /*void sensorsRead(int *sensor_data = an_ir, int *pins = pins, int n = 8){  //Read sensor Data
    for(int i=0; i<n; i++){
      sensor_data[i] = analogRead(pins[i]);
      
      if(sensor_data[i]> 150){
        dig_ir[i]=1;
        
      }
      else{
        dig_ir[i]=0;
      }
    } 
  }*/

signed  int value_assign[8]={-4,-3,-2,-1,1,2,3,4};

  //Find position of line
float getPosition(const int sensor_data[]=dig_ir, const int n = 8, const int sensor_distances[] = sensor_distances){
  int number_of_high_sensors = 0; 
  float sum_of_high_sensors = 0;
  for(int i=0; i<n; i++){
    if(sensor_data[i]){
      number_of_high_sensors += 1; // number of on sensors
      sum_of_high_sensors += sensor_distances[i];
    }
  }
  if(!number_of_high_sensors){
    return 0;  // edge case handling -- if everything is zero, return 0 instead of NaN
  }
  float deviation = sum_of_high_sensors/number_of_high_sensors; // error value for pid
  return +deviation; // CONVENTION: ROBOT TURNING RIGHT IS POSITIVE
}

float getPID(float error)
{
  static float reset,prev_error;
  static unsigned long int prev_time;
  //Error is the difference between the postion of the bot and the position we want it to be
  unsigned long current_time=millis();
  double del_time=current_time-prev_time;                   //Steady state error
  reset += error*del_time;                                 //Reset-The small errors that get accumulated over time *reset gets added over time , hence global variable
  float rate_error= (error-prev_error)/del_time;    
  //float rate_error = error-prev_error;//Rate of change of error
  float pid=kp*(error) + ki*(reset) + kd*(rate_error);     //Calculate PID value

  if(pid!=0){
  //Serial.print(error);
  //Serial.print(" ");
  //Serial.print(rate_error);
  //Serial.print(" ");
  //Serial.print(pid);
  //Serial.print("\n");
  }
  prev_error=error;
  prev_time=current_time;

  return pid;

  
}
void printArray(int array[], int n=8){
  for(int i=0; i<n; ++i){
    Serial.print(array[i]);
    Serial.print(" ");
  }
  Serial.print("\n");
}

void sensorsRead(int sensor_data[], byte const pins[], int n = 8){
  for(int i=0; i<n; i++){
    sensor_data[i] = analogRead(pins[i]);
  }
}

void newCalibrate(int thresholds[], int const pins[], int const n = 8){
  startSpinning(HIGH);
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

void testingIncrementConstant(unsigned const int increment_delay){
  //kd = int(millis() / increment_delay);
}
 
void setup() {
    // put your setup code here, to run once
    
    pinMode(3, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(2,INPUT);

    newCalibrate(thresholds, pins, n);
    
    //Serial.begin(9600);
  }

/*
Direction heading;
bool isRetrace;

junction : <bool Left, bool Front, bool Right, Direction heading_at_junction, int iterations = 0>
path: stack(junction)

Turn checkJunction(){
  static bool at_junction;
}

void stack_loop(){
  readSensors();
  digitaliseData();
  Junction junction = getJunction(dig_ir, n);
  if(junction != nullptr){
    if(junction is end){
      if(heading == end){
        return;
      }
      stopMoving();
      save(path);
      heading = end;
      return;
    }
    Turn direction;
    if(isRetrace){
      assert path.top == junction;
      direction = path.next(&heading);
      return;
    }
    direction = path.push(junction, &heading);
    if(direction != front){
      makeTurn(direction);
      return;
    }
  }
  current_pos = getPosition();
  float pid = getPID(current_pos);
  writeMotors(pid);
}

*/

void pid(){
  current_pos = getPosition();
  float pid = getPID(current_pos);
  writeMotors(pid);
}

bool isAllSame(const bool check_high, const int sensor_data[], const int n){
  int sign = check_high ? +1 : -1;
  for(int i=0; i<n; ++i){
    if(sign*sensor_data[i]){
      return false;
    }
  }
  return true;
}

bool isEndAlike(const int sensor_data[], const int n=8){
  return isAllSame(black_line, sensor_data, n);
}

bool checkConditionToAction(bool (condition)(), void (action)()){
  static bool response_delay_flag;
  if(!condition()){
    response_delay_flag = false;
    return false;
  }
  static unsigned long int target_time;
  if(!response_delay_flag){
    target_time = millis() + response_delay;
    response_delay_flag = true;
  }
  if(millis()>target_time){
    action();
  }
  return true;
}

int countEndStreak(unsigned int mode, int const sensor_data[], int const n){
  int start_point, end_point, increment;
  if(mode==0){ // left
    start_point = 0;
    end_point = n;
    increment = 1;
  }
  else if(mode=1){ // right
    start_point = n-1;
    end_point = -1;
    increment = -1;
  }
  for(int i=0; i<n; ++i){
    if(!sensor_data[i]){
      return i;
    }
  }
  return n;
}

int longestStreak(const int sensor_data[], int const n){
  int streak = 0;
  int max_streak = 0;
  for(int i=0; i<n; ++i){
    if(sensor_data[i]){
      streak += 1;
    }
    else{
      max_streak = streak > max_streak ? streak : max_streak;
      streak = 0;
    }
  }
  return max_streak;
}

int* checkJunction(const int sensor_data[], int const n){
  int result[3] = {countEndStreak(0, sensor_data, n), longestStreak(sensor_data, n), countEndStreak(1, sensor_data, n)};
  // result format : [left_streak, highest_streak, right_streak];
  // left_streak : Number of sensors consecutively  on from the left
  // (If it's too complicated to get the streaks in one function call, we can do it in three)
  int *junction = new int[5]{
    result[0] > line_width,
    result[1] >= 0,
    result[2] > line_width,
    // add these while pushing to stack
    // heading,
    // -1
  };
  return junction;
}

int* getJunction(int const sensor_data[], int const n){
  // junction returned will exist only for one iteration. save it somewhere else (no need for wall hugger)
  static int *previous_junction = nullptr;
  int *junction = checkJunction(sensor_data, n);
  if(junction[1] && !junction[0] && !junction[2]){
    if(previous_junction != nullptr){
      delete [] previous_junction;
      previous_junction = nullptr;
      return junction;
    }
    return nullptr;
  }
  // assert previous_junction == junction || previous_junction == nullptr;
  delete [] previous_junction;
  previous_junction = junction;
  return nullptr;
}

void makeTurn(const int direction, const int n){
  int escape_index, catch_index, left_motor_pwm, right_motor_pwm;
  if(direction == right){
    escape_index = 0;
    catch_index = n-1;
    left_motor_pwm = base_pwm;
  }
  else if(direction == left){
    escape_index = n-1;
    catch_index = 0;
    left_motor_pwm = -base_pwm;
  }
  else{
    return;
  }
  right_motor_pwm = -left_motor_pwm;
  writeMotors(left_motor_pwm, right_motor_pwm);
  // Future Improvement: Asynchronise the sensor read
  while(!dig_ir[escape_index]){
    sensorsRead();
    digitaliseData(black_line);
  }
  while(dig_ir[catch_index]){
    sensorsRead();
    digitaliseData(black_line);
  }
}

// Task: Stop at end
void loop(){
  sensorsRead();
  digitaliseData(black_line);
  bool in_end = checkConditionToAction(
    [](){return isEndAlike(dig_ir, n);},
    [](){}
    // [](){stopMoving();}
  );
  if(in_end){
    return;
  }
  int  *junction = getJunction(dig_ir, n);
  int direction = junction[0] ? left : junction[1] ? front : junction[2] ? right : back;
  if(direction != front){
    makeTurn(direction, n);
    return;
  }
  pid();
}