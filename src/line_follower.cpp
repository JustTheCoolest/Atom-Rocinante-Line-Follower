#include <Arduino.h>

float error;

// Pin list
int ir1 = 3;
int ir2 = 4;
int ir3 = 5;
int ir4 = 6;
int ir5 = 7;
int ir6 = 8;
int ir7 = 9;
int ir8 = 10;
int pins[8] = {ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};

int ir[8];


void setup() {
  // put your setup code here, to run once:
  
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


float getDeviation(int sensor_data[], int n = 8){ 
  float index_shift = n/2+0.5;
  int number_of_high_sensors = 0;
  float sum_of_high_sensors = 0;
  for(int i=0; i<n; i++){
    if(sensor_data[i]){
      number_of_high_sensors += 1;
      
      sum_of_high_sensors += i-(index_shift);
    }
  }
  if(!number_of_high_sensors){
    return 0;
  }
  float deviation = -sum_of_high_sensors/number_of_high_sensors;
  return deviation;
}

// Task : Stop at full black

void loop() {
  sensorsRead();
  for(int i=0;i<8;i++) {
  Serial.print(ir[i]);
  }
  Serial.println(" ");
  error=getDeviation(ir);
  Serial.println(error);
  // int op = digitalread(ir_pin);
  delay(250);
    // put your main code here, to run repeatedly:

}


int perr;
int p, i=0, d;
int kp=0, ki=0, kd=0;
int pid;

void getpid() {
  perr = error;
  p = error;
  i = i+error;
  d = error-perr;
  pid = (kp*p)+(ki*i)+(kd*d);
}

