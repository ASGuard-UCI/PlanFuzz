#ifndef FUNCTIONFILTER_H_
#define FUNCTIONFILTER_H_
#include "PDGUtils.hpp"
#include "ControlDependencyGraph.hpp"
#include "llvm/IR/Function.h"
using namespace llvm;
bool isFiltered(const Function *F);



#endif
