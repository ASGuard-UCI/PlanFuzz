#ifndef __FITNESS_TRACKER__
#define __FITNESS_TRACKER__
#include <iostream>
#include <fstream>
#include <vector>
#define MARK_TARGETED_POS(x) mark_targeted_pos(x);
extern int debug_flag;


extern int shared_mem[500];
extern double shared_mem1[500];
void print();
void init_shared_mem();
void refresh_shared_mem();
double non_critical_fitness(int);
double critical_fitness(int);
double get_fitness_shared_mem_AFLGO();
double get_fitness_shared_mem();
class FitnessTracker{
	public:
		void Init(std::ofstream *log_file);
		void Update(double a, double b, double dis, int id);
                double GetFitness();
    private:
	//std::ofstream *log_file_;
        double min_value[100];

};
extern FitnessTracker fitness_tracker;

struct inst_info{
  std::string file_name;
  int line_id;
  int BB_dist;
  bool is_critical;
  bool closer_branch;
  int t_dist;
  int f_dist;
  bool is_fcmp;
  std::string BB_name;

};

void LoadInstInfo(std::string filename);

extern int max_targeted_pos;
void  mark_targeted_pos(int i);
//extern int* count_ptr;
//extern double* min_ptr;
#endif
