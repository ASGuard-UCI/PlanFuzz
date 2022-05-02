#ifndef __PLANNING_CONTEXT_UTIL__
#define __PLANNING_CONTEXT_UTIL__
#include "modules/tools/fuzz/vuln_test/planning_context/planning_context.pb.h"
#include "modules/planning/common/planning_context.h"

#include <string>
using apollo::tools::fuzz::planning::PlanningContextPb;
void StorePlanningContext(PlanningContextPb *x);
bool CompareContext(const PlanningContextPb a, const PlanningContextPb b);
void LoadPlanningContext(PlanningContextPb x);
void StorePlanningContext(std::string path);

void LoadPlanningContext(std::string path);



#endif
