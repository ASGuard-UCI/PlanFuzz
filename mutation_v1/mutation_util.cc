#include <cmath>
#include <iostream>
#include <fstream>
#include <random>
#include "boost/random/normal_distribution.hpp"
#include "boost/random/generate_canonical.hpp"
#include "boost/random/random_device.hpp"
#include "boost/random/uniform_real_distribution.hpp"
#include "mutation_util.h"
double kSBX_poly_index = 10.0;

void SBX(double *input1, double *input2){
  double prev_1 = *input1;
  double prev_2 = *input2;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);
  double mu = dis(gen);
  double belta;
  if (mu>0.5)
    belta = std::pow(2*mu, 1.0/(kSBX_poly_index+1));
  else
    belta = std::pow(1.0/(2*(1-mu)), 1.0/(kSBX_poly_index+1));
  *(input1) = 0.5*( (1+belta)*prev_1 + (1-belta)*prev_2 );
  *(input2) = 0.5*( (1-belta)*prev_1 + (1+belta)*prev_2 );
  return;
}

void FMutate(double *input){
  double prev = *input;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dis(0.0, 1.0);
  *input = prev + dis(gen);
  return;
}

void FMutate(double *input, double variance){
  double prev = *input;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dis(0.0, variance);
  *input = prev + dis(gen);
  return;
}


