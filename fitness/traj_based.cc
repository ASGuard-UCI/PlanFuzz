#include "modules/planning/proto/planning.pb.h"
using apollo::planning::ADCTrajectory; 


double evaluatePlanningTrajectory(ADCTrajectory *input){

//  if (input->decision().main_decision().has_stop())
 //   return 0;
  double init_v = input->trajectory_point(0).v();
  double fitness = 0.0; 
  for (int i=0; i<input->trajectory_point_size(); i++){
    if (input->trajectory_point(i).a() < fitness){
     // double temp_fitness = input->trajectory_point(i).v()/init_v;
     // if (temp_fitness < fitness)
     //   fitness = temp_fitness;
      fitness = input->trajectory_point(i).a();
    }
  }
  return fitness;
}
