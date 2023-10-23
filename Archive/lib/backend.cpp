void digitaliseData(int sensor_data[], int const thresholds[], int const  n = 8)
{
  for(int i=0; i<n; ++i){
    sensor_data[i] = sensor_data[i] > thresholds[i];
  }
}

// Alternative: Implement both hasStreak and findEndOfStreak as two instances of a sequence detector
int findEndOfStreak(const int sensor_data[], int j, int const n = 8){
  for(++j; j<n-2; ++j){
    if(!sensor_data[j]){
      if(!sensor_data[++j]){
        return j+1;
      }
      continue;
    }
  }
  return -1;
}


void filterSensors(int sensor_data[], int n = 8){
  bool streak_present = false;
  /*
  if(!hasStreak(sensor_data)){
    return; // If there is no streak, a single 1 is not treated as noise, it could be our true data
  }
  */
  for(int i=0; i<n; ++i){
    if(!sensor_data[i]){
      continue;
    }
    int j = findEndOfStreak(sensor_data, i);
    if(j<-1){
      return;
    }
    if(j==i+3){
      sensor_data[i] = 0;
    }
    i = j;
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
  float deviation = sum_of_high_sensors/number_of_high_sensors; // error value for pid
  return +deviation; // CONVENTION: ROBOT TURNING RIGHT IS POSITIVE
}

struct streak{
  int filled = false;
  int start_index;
  int end_index;
  int length;
};

class Line{
  public:
  struct streak streaks[8];
  void findBranches(int sensor_data[], int n){
    int streak_start = -1;
    int streaks_index = 0;
    for(int i=0; i<n; ++i){
      if(sensor_data[i]){
        if(streak_start==-1){
          streak_start = i;
        }
      }
      else{
        if(streak_start!=-1){
          streaks[streaks_index].filled = true;
          streaks[streaks_index].start_index = streak_start;
          streaks[streaks_index].end_index = i-1;
          streaks[streaks_index].length = i-streak_start;
          ++streaks_index;
          streak_start = -1;
        }
      }
    }
  }
  struct streak selectBranch(int sensor_data[], int n){
    int max_length = 0;
    int max_length_index = -1;
    for(int i=0; i<n; ++i){
      if(!streaks[i].filled){
        break;
      }
      if(streaks[i].length>max_length){
        max_length = streaks[i].length;
        max_length_index = i;
      }
    }
    return streaks[max_length_index];
  }
  float getDeviation(struct streak branch, const int n = 8){
    float step_size = 0.5;
    float index_shift = n/2-step_size;  // 3.5
    int first_value = branch.start_index-index_shift;
    int deviation = (first_value+step_size*(branch.length-1))/branch.length;
    return deviation; // CONVENTION: ROBOT TURNING RIGHT IS POSITIVE
  }
};

int numberOfHighSensors(int sensor_data[], int n){
  int number_of_high_sensors = 0;
  for(int i=0; i<n; ++i){
    if(sensor_data[i]){
      ++number_of_high_sensors;
    }
  }
  return number_of_high_sensors;
}

float correctPidValueForOrientation(float pid, int number_of_high_sensors){
  int constant_of_sync = 2;
  return pid*number_of_high_sensors/constant_of_sync;
}

int capMotorPWM(int const unprocessed_pwm){
  int cappedPWM = unprocessed_pwm > 255 ? 255 : unprocessed_pwm < 0 ? 0 : unprocessed_pwm;
  return cappedPWM;
}

void shiftMotorPWM(int *first_motor_pwm, int *second_motor_pwm){
  const int max_value = 255;
  int *higher_motor_pwm, *lower_motor_pwm;
  higher_motor_pwm = *first_motor_pwm > *second_motor_pwm ? first_motor_pwm : second_motor_pwm;
  if(*higher_motor_pwm > max_value){
    *lower_motor_pwm -= *higher_motor_pwm - max_value;
    *higher_motor_pwm = max_value;
  }
  if(*lower_motor_pwm < -max_value){
    *lower_motor_pwm = -max_value;
  }
}