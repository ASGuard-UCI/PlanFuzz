#include "mutation_util.h"
#include <iostream>

int main(){
  int count = 0;
  for (int i=0; i<1000; i++){
    double x1 = 14.0;
    double x2 = 10.0;
    double y1 = 19.0;
    double y2 = 15.0;
    SBX(&x1, &x2);
    SBX(&y1, &y2);
    if (x1 > 14.0) count++;
    std::cout<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<std::endl;
  }
  std::cout<<count<<std::endl;
  return 0;
}
