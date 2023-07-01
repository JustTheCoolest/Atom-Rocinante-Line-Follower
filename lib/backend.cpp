void digitaliseData(int sensor_data[], int const thresholds[], int const  n = 8)
{
  for(int i=0; i<n; ++i){
    sensor_data[i] = sensor_data[i] > thresholds[i];
  }
}


// Alternative: Implement both hasStreak and findEndOfStreak as two instances of a sequence detector
int findEndOfStreak(const int sensor_data[], bool *streak_present, int j, int const n = 8){
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


void filterSensors(int sensor_data[], int n = 8){
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
