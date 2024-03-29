/* TODO: Base speed, new algorithm, response delay, wheels, continuous calibration */

  constexpr float kp = 80,ki = 0,kd =0;
  constexpr int base_pwm = 0;
  constexpr unsigned int response_delay = 0;
   float calibration_ratio;  //0.2 or 0.05 response_delay at zero is almost the same:The same can be used

  constexpr int n = 8;
  //int thresholds[8] = {150, 150, 150, 150, 150, 150, 150, 150};
  int thresholds[8];
  
  int left_motor_pwm;
  int right_motor_pwm;
  float current_pos;  //PID error
  float set_pos=0;    //Sensor 4,5 go high
  float reset,prev_error;
  unsigned long int prev_time;

  
  // Pin list
  int lmotor = 3;
  int lmotorn = 9;
  int rmotor = 10;
  int rmotorn = 11;
  int ir1 = A0;
  int ir2 = A1;
  int ir3 = A2;
  int ir4 = A3;
  int ir5 = A4;
  int ir6 = A5;
  int ir7 = A6;
  int ir8 = A7;
  int swit=2;
//  int bsl = 40; deprecated analogWrite base speed for left motor
//  int bsr = 40; deprecated analogWrite base speed for right motor 
  int pins[8] = {ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};
  int an_ir[8];
  int dig_ir[8];
  
  

  void sensorsRead(int sensor_data[]=an_ir, int const pins[]=pins, int n = 8){
  for(int i=0; i<n; i++){
    sensor_data[i] = analogRead(pins[i]);
  }
}

void digitaliseData(int sensor_data[]=an_ir, int const thresholds[]=thresholds, int const  n = 8)
{
  for(int i=0; i<n; ++i){
    dig_ir[i] = sensor_data[i] > thresholds[i];
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
float getPosition(int sensor_data[]=dig_ir, int n = 8){
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
void startSpinning(bool const clockwise_flag){
  digitalWrite(lmotor, clockwise_flag);
  digitalWrite(lmotorn, !clockwise_flag);
  digitalWrite(rmotor, !clockwise_flag);
  digitalWrite(rmotorn, clockwise_flag);
}

void sensorsRead(int sensor_data[], byte const pins[], int n = 8){
  for(int i=0; i<n; i++){
    sensor_data[i] = analogRead(pins[i]);
  }
}
                                                                                                                                                                                                                                                                                                                                                                            

int capMotorPWM(int const unprocessed_pwm){
  int cappedPWM = unprocessed_pwm > 255 ? 255 : unprocessed_pwm < 0 ? 0 : unprocessed_pwm;
  return cappedPWM;
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

bool checkWhiteToStopMoving(int sensor_data[], unsigned int const response_delay, const int n=8){
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
    startSpinning(right_motor_pwm<left_motor_pwm);
    response_delay_flag = false;
  }
  return false;
}

void writeMotors(const int pid){
  left_motor_pwm = base_pwm+pid;
  right_motor_pwm = base_pwm-pid;
  left_motor_pwm = capMotorPWM(left_motor_pwm);
  right_motor_pwm = capMotorPWM(right_motor_pwm);
  analogWrite(lmotor,left_motor_pwm);          //Check pid values, direction of turning and adjust
  digitalWrite(lmotorn, HIGH);
  analogWrite(rmotor, right_motor_pwm);
  digitalWrite(rmotorn, HIGH);
}

void newCalibrate(int thresholds[], int const pins[], int const n = 8){
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
    pinMode(swit,INPUT_PULLUP);
    
    calibration_ratio = digitalRead(swit) ? 0.2 : 0.05;

    newCalibrate(thresholds, pins, n);
    
    //Serial.begin(9600);
  }

 void loop() {
    //testingIncrementConstant(100);
    //stopMoving();
    //delay(100);
    
  //printArray(thresholds);
  
    sensorsRead(); // get ir values
    //printArray(an_ir, n);
    digitaliseData();
    //printArray(dig_ir, n);
    if(!checkWhiteToStopMoving(dig_ir, response_delay))return;
    current_pos=getPosition();             //Calculate Position
    /*//Serisl.print(" Current pos");        //When all sensors detect low The function gives99.99
    ////Serial.println(current_pos);*/         //At which point it will turn either side in search of the line
    float  pid=getPID(current_pos);                   //Retrieve PID value
  //  //Serial.print(pid); 
  writeMotors(pid);
 }  
