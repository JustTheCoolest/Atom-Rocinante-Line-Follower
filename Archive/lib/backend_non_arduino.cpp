#include <iostream> 
using namespace std;

void printArrayConsole(int const array[], int const n){
    for(int i=0; i<n; ++i){
        cout << array[i] << " ";
    }
    cout << endl;
}