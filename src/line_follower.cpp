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
  int bsl = 90; // analogWrite base speed for left motor
  int bsr = 90; // analogWrite base speed for right motor 
  byte pins[8] = {ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};
  int an_ir[8];
  int dig_ir[8];
  
  float kp,ki,kd;
  float KP,KI,KD;   //Variables for tuning
  int A;            //Same
  double B,Bp;      //Same
  double x,y,x1,y1,x2,y2; //Same
  
  
  
  
  
  
  // Task: Convert sensor reads to analog values --Done
  
  
  
  
  void sensorsRead(int sensor_data[], byte const pins[], int n = 8){
  for(int i=0; i<n; i++){
    sensor_data[i] = analogRead(pins[i]);
  }
}

void copyArray(int const a[], int b[], int const n = 8){
  for(int i=0; i<n; ++i){
    b[i] = a[i];
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
void copyArray(int const a[], int b[], int const n = 8){
  for(int i=0; i<n; ++i){
    sensor_data[i] = sensor_data[i] > thresholds[i];
    b[i] = a[i];
  }
}


int findEndOfStreak(int sensor_data[], bool *streak_present, int j, int const n = 8){
  for(++j; j<n-2; ++j){
    if(!sensor_data[j]){
      if(!sensor_data[++j]){
        *streak_present = true;
        return j+1;
        
        
void calibrate(int thresholds[], byte const pins[], int const n = 8){
  digitalWrite(lmotor,HIGH);
  digitalWrite(lmotorn,LOW);
  digitalWrite(rmotor,LOW);
  digitalWrite(rmotorn,HIGH);
  int sensorData[n];
  int minValues[n], maxValues[n];
  for(int i=0; i<200; ++i){
    sensorsRead(sensorData, pins, n);
    for(int j=0; j<n; ++j){
      if(sensorData[j]<minValues[j] || j==0){
        minValues[j] = sensorData[j];
      }
      if(sensorData[j]>maxValues[j] || j==0){
        maxValues[j] = sensorData[j];
      }
    }
  }
  for(int i=0; i<n; ++i){
    thresholds[i] = 0.75*maxValues[i] + 0.25*minValues[i] + 0.5;
  }
  digitalWrite(lmotor, HIGH);
  digitalWrite(lmotorn, HIGH);
  digitalWrite(rmotor, HIGH);
  digitalWrite(rmotorn, HIGH);
}
        

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
        thresholds[i] = 0.75*maxValues[i] + 0.25*minValues[i] + 0.5;
  }
  digitalWrite(lmotor, HIGH);
  digitalWrite(lmotorn, HIGH);
  digitalWrite(rmotor, HIGH);
  digitalWrite(rmotorn, HIGH);
}
  
  
  // Task : Stop at full black

signed  int value_assign[8]={-4,-3,-2,-1,1,2,3,4};

  //Find position of line
  float getPosition(){
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
    if(!number_of_high_sensors){
    return 0;  // edge case handling -- if everything is zero, return 0 instead of NaN
  }
  float deviation = -sum_of_high_sensors/number_of_high_sensors; // error value for pid
  return deviation;
}

// Task : Stop at full black

    
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
signed  int value_assign[8]={-4,-3,-2,-1,1,2,3,4};

float getPID(){
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
  return pid;

}

int thresholds[8];      
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


void writeMotors(const int pid, const int sensorStates[]){
    for(int i=0; i<8; ++i){
      if(sensorStates[i]){
        analogWrite(lmotor,bsl-pid);          //Check pid values, direction of turning and adjust
        analogWrite(rmotor,bsl+pid);
        break;
      }
    }
}



void printArray(int array[], int n=8){
  for(int i=0; i<n; ++i){
    Serial.print(array[i]);
    Serial.print(" ");
       }
  }

void setup(){
    // put your setup code here, to run once:
    pinMode(A0, INPUT);     
    pinMode(A1, INPUT);     
    pinMode(A2, INPUT);     
    pinMode(A3, INPUT);     
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);
    pinMode(A6, INPUT);
    pinMode(A7, INPUT);
    
    pinMode(3, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    
    Serial.begin(9600);
    calibrate(thresholds, pins);
    printArray(thresholds);
    
    
    kp=20.00;ki=0.00;kd=0.00;
    KP=1.00;KI=1.00;KD=1.00;
    y=1.00;y1=1.00;y2=1.00;
    //Calibaration sequence
}
 
 void loop() {
    sensorsRead(ir, pins); // get ir values
    for(int i=0;i<8;i++) {
    Serial.print(dig_ir[i]); // debugging sensor readings dig_ir for digital and an_ir for analog
    Serial.print(" ");
    }
    Serial.print(" ");
    for(int i=0;i<8;i++) {
    Serial.print(an_ir[i]); // debugging sensor readings dig_ir for digital and an_ir for analog
    Serial.print(" ");
    }
    Serial.print(" ");
    current_pos=getPosition();             //Calculate Position
    Serial.print("Current pos: ");        //When all sensors detect low The function gives99.99
    Serial.print(current_pos);         //At which point it will turn either side in search of the line
    float  pid=getPID();                   //Retrieve PID value
    Serial.print(" PID: ");
    Serial.print(pid); 
    pid=abs(pid);
    int mpid=int(pid+0.5);                 //Convert pid to integer values:Round it to nearest integer
    float er=set_pos-current_pos;
    int m1=bsl-mpid;
    int m2=bsl+mpid;
    Serial.print(" M1:");
    Serial.print(m1);
    Serial.print(" M2:");
    Serial.println(m2);
    writeMotors(pid, dig_ir);   
 }
