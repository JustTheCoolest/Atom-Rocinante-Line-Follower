#include <assert.h>
#include "../lib/backend.cpp"
#include "../lib/backend_non_arduino.cpp"

bool isEqual(int const a[], int const b[], int const n = 8){
  for(int i=0; i<n; ++i){
    if(a[i]!=b[i]){
      printArrayConsole(a, n);
      printArrayConsole(b, n);
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
    assert(isEqual(sensor_data, expected, n));
  }
}

void testGetDeviation(){
  int sensor_datas[5][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0}, 
    {1, 1, 1, 0, 0, 0, 0, 0}, 
    {0, 0, 0, 0, 1, 1, 1, 0},
    {0, 0, 0, 1, 1, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 1}
  };
  float expected[] = {0, -2.5, +1.5, +0.5, +3};
  for(int i=0; i<5; ++i){
    float calculated_value = getDeviation(sensor_datas[i]);
    if(!(calculated_value == expected[i])){
      printArrayConsole(sensor_datas[i], 8);
      cout << calculated_value << "!=" << expected[i] << endl;
    }
  }
}

bool isEqual(int a, int b){
  if(a!=b){
    cout << a << "!=" << b << endl;
    return false;
  }
  return true;
}

int testCapMotors(){
  assert(isEqual(capMotorPWM(260), 255));
  assert(isEqual(capMotorPWM(56), 56));
  assert(isEqual(capMotorPWM(78), 78));
  assert(isEqual(capMotorPWM(-17), 0));
}


int main(){
  testGetDeviation();
  //testFilterSensors();
  cout << "Testing complete";
}