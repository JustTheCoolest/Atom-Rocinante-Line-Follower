#include <Arduino.h> // BEFORE EXECUTING ON ARDUINO IDE, REMOVE THIS LINE. IT WILL THROW ERRORS.


float error;


// Pin list
int lmotor = 3;
int lmotorn = 9;
int rmotor = 10;
int rmotorn = 11;
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
  pinMode(1, OUTPUT);     // Motor pins are 3,9 and 10,11.
  pinMode(2, OUTPUT);     // From Haridutt.
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

// Task: Convert sensor reads to analog values




void sensorsRead(int *sensor_data = ir, int *pins = pins, int n = 8){
  for(int i=0; i<n; i++){
    sensor_data[i] = digitalRead(pins[i]);
  }
}

/*
int checkStreakMembership(int sensor_data[], int i, int n = 8){
  for(int j=i+1; j<i+3; ++j){ // At least 2 bits ahead should be 0
    if(sensor_data[j]){
      while(j<n-2 && sensor_data[j] && !sensor_data[j+1]){ // To find end of streak + 2 consecutive 0s
        ++j;
      }
      return j+2; // Return the index from which to continue checking
    }
  }
  return -1; // Stray
}


void filterSensors(int sensor_data[] = ir, int n = 8){
  bool streak_flag = false;
  for(int i=0; i<n; ++i){
    if(!sensor_data[i]){
      continue;
    }

    if(streak_flag){
      if(!sensor_data[++i] && !sensor_data[++i]){
      }
      i+=2;
      continue;
    }
    
    int jump_index = checkStreakMembership(sensor_data, i);
    if(jump_index == -1){
      if(streak_flag){
        sensor_data[i] = 0;
        continue;
      }
      else{
        // Recurse
      }
    }
    streak_flag = true;
    i = jump_index;
  }
}
*/


int findEndOfStreak(int sensor_data[], int j, int n = 8){
  ++j;
  bool padding_check_flag;
  while(j<n-2){
    if(!sensor_data[i]){
      if(!sensor_data[++j]){
        return j+1;
      }
      continue;
    }
    ++j;
  }
  return n;
}

void filterSensors(int sensor_data[] = ir, int n = 8){
  bool streak_flag = false;
  for(int i=0; i<n-1; ++i){
    if(sensor_data[i] && sensor_data[i+1]){
      streak_flag = true;
      break;
    }
  }
  if(!streak_flag){
    return;
  }
  bool padding_check_flag;
  for(int i=0; i<n; ++i){
    if(!sensor_data[i]){
      continue;
    }
    padding_check_flag = true;
    for(int j=i; j<min(i+2, n); ++j){
      if(sensor_data[j]){
        padding_check_flag = false;
        i = findEndOfStreak(sensor_data, j);
        break;
      }
    }
    if(padding_check_flag){
      sensor_data[i] = 0;
      i+=2;
    }
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

// Task : Stop at full black



void loop() {
  sensorsRead(); // get ir values
  for(int i=0;i<8;i++) {
  Serial.print(ir[i]); // debugging
  }
  Serial.println(" ");
  error=getDeviation(ir);
  Serial.print("error:")
  Serial.println(error);  // also debugging
  Serial.print("pid value:");
  pid=getpid(); // pid error value
  analogWrite(lmotor,bsl+pid);
  analogWrite(lmotorn, 0);
  analogWrite(rmotor, bsl-pid);
  analogWrite(rmotorn, 0);
  
  // int op = digitalread(ir_pin);
  delay(250);
    // put your main code here, to run repeatedly:


}




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
