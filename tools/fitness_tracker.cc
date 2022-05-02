#include "fitness_tracker.h"
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdio.h>

int shared_mem[500];
double shared_mem1[500];
double max_history[500];
int shared_mem2[500];
int* count_ptr = shared_mem;
double* min_ptr = shared_mem1;
int* br_count_ptr = shared_mem2;
int debug_flag = 0;
std::ofstream *log_file_;
int max_targeted_pos;
std::vector<struct inst_info> inst_info_list;
void print(){
  printf("Get into instrumentation Instructions\n");
}

void LoadInstInfo(std::string filename){

  inst_info_list.clear();
  std::ifstream input(filename);
  while (!input.eof()){
    struct inst_info new_inst_info;
    input>>new_inst_info.file_name
         >>new_inst_info.line_id
         >>new_inst_info.BB_dist
         >>new_inst_info.is_critical
         >>new_inst_info.closer_branch
         >>new_inst_info.t_dist
         >>new_inst_info.f_dist
         >>new_inst_info.is_fcmp
         >>new_inst_info.BB_name;


    inst_info_list.push_back(new_inst_info);
  }
}

void refresh_shared_mem(){
  for (int i=0; i<500; i++){
    shared_mem[i] = 0;
    shared_mem2[i] = 0;
    shared_mem1[i] = 100000000000.00;
  }
  return;
}

void init_shared_mem(){
  for (int i=0; i<500; i++){
    shared_mem[i] = 0;
    shared_mem2[i] = 0;
    shared_mem1[i] = 100000000000.00;
    max_history[i] = -1.0;
  }
  return;
}



/*double get_fitness_shared_mem(){
  double pred_fitness[20];
  for (int i=0; i< 22; i++)
    if (shared_mem[i]>0 && shared_mem1[i]> max_history[i])
      max_history[i] = shared_mem1[i];
  std::cout<<"+++Begin print shared_mem+++\n";
  for (int i=0; i<22; i++)
    std::cout<<shared_mem[i]<<" ";
  std::cout<<'\n';
  for (int i=0; i<22; i++)
    std::cout<<shared_mem1[i]<<" ";
  std::cout<<'\n';
  for (int i=0; i<22; i++)
    std::cout<<shared_mem2[i]<<" ";
  std::cout<<'\n';
  for (int i=0; i<22; i++)
    std::cout<<max_history[i]<<" ";
  std::cout<<'\n';

  if (shared_mem2[20] + shared_mem2[21]==0)
    pred_fitness[0] = 0.1;
  else
    pred_fitness[0] = 0.1 * 1 *((shared_mem2[20]*1.0/(shared_mem2[20]+ shared_mem2[21]))
		  *(shared_mem1[2]/max_history[2]) 
		  + (shared_mem2[21]*1.0/(shared_mem2[20]+shared_mem2[21]))
		  *(shared_mem1[3]/max_history[3])); 
 if (shared_mem2[18] + shared_mem2[19] ==0)
    pred_fitness[1] = 0.4;
  else  
    pred_fitness[1] = 0.1 * 4 *((shared_mem2[18]*1.0/(shared_mem2[18]+ shared_mem2[19]))
		  *(shared_mem1[0]/max_history[0]) 
		  + (shared_mem2[19]*1.0/(shared_mem2[18]+shared_mem2[19]))
		  *(shared_mem1[1]/max_history[1]));  
  if (shared_mem2[20] + shared_mem2[21]==0)
    pred_fitness[2] = 1;
  else 
    pred_fitness[2] = 1 *((shared_mem2[20]*1.0/(shared_mem2[20]+ shared_mem2[21]))
		  *(1.0) 
		  + (shared_mem2[21]*1.0/(shared_mem2[20]+shared_mem2[21]))
		  *(0.0));
 if (shared_mem2[18] + shared_mem2[19]==0)
    pred_fitness[3] = 4;
  else  
    pred_fitness[3] = 4 *((shared_mem2[18]*1.0/(shared_mem2[18]+ shared_mem2[19]))
		  *(1.0) 
		  + (shared_mem2[19]*1.0/(shared_mem2[18]+shared_mem2[19]))
		  *(0.0)); 
 if (shared_mem2[16] + shared_mem2[17]==0)
    pred_fitness[4] = 14.0;
  else  
    pred_fitness[4] = 14.0* ((shared_mem2[16]*1.0/(shared_mem2[16]+ shared_mem2[17]))
		  *(1.0) 
		  + (shared_mem2[17]*1.0/(shared_mem2[16]+shared_mem2[17]))
		  *(0.0)); 
 if (shared_mem2[14] + shared_mem2[15]==0)
    pred_fitness[5] = 2.0;
  else  
    pred_fitness[5] = 2.0* ((shared_mem2[14]*1.0/(shared_mem2[14]+ shared_mem2[15]))
		  *(1.0) 
		  + (shared_mem2[14]*1.0/(shared_mem2[14]+shared_mem2[15]))
		  *(0.0));
 if (shared_mem2[4] + shared_mem2[5]==0)
    pred_fitness[6] = 16.0;
  else  
    pred_fitness[6] = 16.0* ((shared_mem2[4]*1.0/(shared_mem2[4]+ shared_mem2[5]))
		  *(1.0) 
		  + (shared_mem2[5]*1.0/(shared_mem2[4]+shared_mem2[5]))
		  *(0.0));

 if (shared_mem2[2] + shared_mem2[3]==0)
    pred_fitness[7] = 17.0;
  else
    pred_fitness[7] = 17.0* ((shared_mem2[2]*1.0/(shared_mem2[2]+ shared_mem2[3]))
		  *(1.0) 
		  + (shared_mem2[3]*1.0/(shared_mem2[2]+shared_mem2[3]))
		  *(0.0));

 if (shared_mem2[0] + shared_mem2[1]==0)
    pred_fitness[8] = 18.0;
  else  
    pred_fitness[8] = 18.0* ((shared_mem2[0]*1.0/(shared_mem2[0]+ shared_mem2[1]))
		  *(0.0) 
		  + (shared_mem2[1]*1.0/(shared_mem2[0]+shared_mem2[1]))
		  *(1.0));
  std::cout<<"+++ Print fitness value +++\n";
  double sum = 0;
  for (int i=0; i<9; i++){
    std::cout<<pred_fitness[i]<<" ";
    sum+=pred_fitness[i];
  }
  std::cout<<"\n+++ end of fitness +++\n";
  //for (int i=0; i<9; i++)
  //  (*log_file_)<<pred_fitness[i]<<" ";
  //(*log_file_)<<sum<<'\n';
  return sum;
}*/

double critical_fitness(int i){
  
  double ret;
  if (inst_info_list[i].is_fcmp){
    if (shared_mem[2*i] + shared_mem[2*i+1] == 0)
      ret = inst_info_list[i].BB_dist;
    else if (shared_mem[2*i+1 - inst_info_list[i].closer_branch] == 0 )
      ret = (shared_mem1[2*i+ inst_info_list[i].closer_branch] * 1.0/
             max_history[2*i + inst_info_list[i].closer_branch])
          * inst_info_list[i].BB_dist;
    else 
      ret = (shared_mem[2*i + inst_info_list[i].closer_branch] * 1.0
             / (shared_mem[2*i] + shared_mem[2*i+1]))
           * inst_info_list[i].BB_dist;
  }
  else{
    if (shared_mem2[2*i] + shared_mem2[2*i+1] == 0)
      ret = inst_info_list[i].BB_dist;
    else if (shared_mem2[2*i+1 - inst_info_list[i].closer_branch] == 0 )
      ret = (shared_mem2[2*i + inst_info_list[i].closer_branch] * 1.0
             / (shared_mem2[2*i] + shared_mem2[2*i+1]))
           * inst_info_list[i].BB_dist;
  }
  std::cout<<"Fitness value of critical predicate "<<i<<" : "<<ret<<'\n';
  std::cout<<shared_mem[2*i]<<' '<<shared_mem[2*i+1]<<' '
           <<shared_mem1[2*i]<<' '<<shared_mem1[2*i+1]<<' '
  	   <<shared_mem2[2*i]<<' '<<shared_mem2[2*i+1]<<'\n';
  
  return ret;
}

double non_critical_fitness(int i){
  double ret;
    //std::cout<<"Non-critical runtime info: "<<inst_info_list[i].is_fcmp<<" "<<shared_mem[2*i]<<" "<<shared_mem[2*i+1]<<" "<<inst_info_list[i].BB_dist<<max_history[2*i]<<" "<<max_history[2*i+1]<<'\n';
  if (inst_info_list[i].is_fcmp){
    if (shared_mem[2*i] + shared_mem[2*i+1] == 0)
      ret = inst_info_list[i].BB_dist;
    else{
      double d1 = shared_mem1[2*i]/max_history[2*i];
      if (d1 < 0)  d1 = 1.0;
      double d2 = shared_mem1[2*i+1]/max_history[2*i+1];
      if (d2 < 0) d2 = 1.0;
      if (d1 < d2)
        ret = d1 * inst_info_list[i].BB_dist;
      else
        ret = d2 * inst_info_list[i].BB_dist;
    } 
  }
  else{
    if (shared_mem2[2*i] + shared_mem2[2*i+1] ==0)
      ret = inst_info_list[i].BB_dist;
    else{
        ret =  0.0;
    } 
  }
 /* std::cout<<"Fitness value of non_critical predicate "<<i<<" : "<<ret<<'\n';
  std::cout<<shared_mem[2*i]<<' '<<shared_mem[2*i+1]<<' '
           <<shared_mem1[2*i]<<' '<<shared_mem1[2*i+1]<<' '
           <<shared_mem2[2*i]<<' '<<shared_mem2[2*i+1]<<'\n';
 */
  return ret * 0.2;
}

double get_fitness_shared_mem(){
  double sum = 0;
  for (int i=0; i<500; i++){
    if (shared_mem1[i] >= 1000000) 
      continue;
    if (max_history[i] == 0.0)
      max_history[i] = shared_mem1[i];
    else if (shared_mem1[i] > max_history[i])
      max_history[i] = shared_mem1[i];
  }
  for (int i=0; i<inst_info_list.size(); i++){
    if (inst_info_list[i].is_critical){
      sum+=critical_fitness(i);
      std::cout<<"Get fitness for " <<i<<" th inst "<< critical_fitness(i)<<std::endl;
    }
    else{
      double delta=non_critical_fitness(i);
      sum+=delta;
      std::cout<<"Get non-critical fitness for "<<i<<" th inst "<< delta<<std::endl;

    }
  }
  std::cout<<"Get fitness value "<<sum<<'\n';
  return sum;
}
 

double get_fitness_shared_mem_AFLGO(){
  double sum = 0;
  int num = 0;
  for (int i=0; i<500; i++){
    if (inst_info_list[i].is_fcmp){
      if (shared_mem[2*i]+ shared_mem[2*i+1]!=0){
        num+=1;
        sum += 1.0/(inst_info_list[i].BB_dist + 1.0);
        //if (inst_info_list[i].BB_dist == 0)
        //  return 0;
      }
    }
    else{
      if (shared_mem2[2*i] + shared_mem2[2*i+1] != 0){
        num+=1;
        sum+=1.0/(inst_info_list[i].BB_dist+1.0);
        //if (inst_info_list[i].BB_dist == 0)
        //  return 0;
      }
    }
  }
  std::cout<<"Fitness "<<(1.0/sum)/(num*1.0)<<'\n';
  return (1.0/sum)/(num*1.0);
}

void FitnessTracker::Init(std::ofstream *log_file){
  log_file_ = log_file;
  //fitness = 100000000.0;
  for (int i=0; i<400; i++)
    shared_mem1[i] = 1000000000.0;
  for (int i=0; i<400; i++)
    max_history[i] = -1.0;
  return;
}


void FitnessTracker::Update(double a, double b, double dis, int index){
 // (*log_file_)<<a<<" "<<b<<" "<<dis<<std::endl;
  double operand_dis;
  if (a>b)
    operand_dis = a-b;
  else
    operand_dis = b-a;
  if (dis*operand_dis < min_value[index])
    min_value[index] = dis*operand_dis;
  
/*
  double temp_dis;
  if (std::abs(a)>std::abs(b))
    temp_dis = dis * operand_dis/(std::abs(a) - std::abs(b));
  else
    temp_dis = dis * operand_dis/ (std::abs(b)- std::abs(a));
  fitness = 1/(1/fitness + 1/temp_dis);
*/
  //if (temp_dis < fitness)
//    fitness = temp_dis;
  return;
}

double FitnessTracker::GetFitness(){
  for (int i=0; i<10; i++)
    (*log_file_)<<min_value[i]<<" ";
  (*log_file_)<<std::endl;
  double temp_sum = 0;
  for (int i=0; i<10; i++)
    temp_sum = temp_sum + 1.0/min_value[i];
  return 1.0/temp_sum;
}

void __attribute__ ((optnone)) mark_targeted_pos(int i){
  if (max_targeted_pos<i)
    max_targeted_pos = i;
}
