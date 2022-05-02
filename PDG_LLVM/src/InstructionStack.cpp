#include "InstructionStack.hpp"
#include "llvm/Support/Debug.h"
#include "llvm/IR/DebugInfoMetadata.h"
using namespace llvm;


bool pdg::InstructionStack::isInStack(Instruction* inst){
  return instmap[inst];
}

void pdg::InstructionStack::addInst(Instruction* inst){
  if (instmap[inst] == false)
    inst_list.push_back(inst);
  errs()<<"Add inst to inst_stack "<<*inst<<'\n';
  instmap[inst] = true;
  distmap[inst] = 0;
  is_critical[inst] = true;
  closer_branch[inst] = true;
  return;
}

void pdg::InstructionStack::UpdateDist(Instruction* inst, int dist){
  if (distmap[inst]==0){
    distmap[inst] = dist;
  }
  else if (dist < distmap[inst])
    distmap[inst] = dist;
  return;
}
void pdg::InstructionStack::UpdateTDist(Instruction* inst, int dist){
  if (t_dist[inst]==0){
    t_dist[inst] = dist;
  }
  else if (dist < t_dist[inst])
    t_dist[inst] = dist;
  return;
}


void pdg::InstructionStack::UpdateFDist(Instruction* inst, int dist){
  if (f_dist[inst]==0){
    f_dist[inst] = dist;
  }
  else if (dist < f_dist[inst])
    f_dist[inst] = dist;
  return;
}
int pdg::InstructionStack::QueryFDist(Instruction* inst){
  return f_dist[inst];
}
int pdg::InstructionStack::QueryTDist(Instruction* inst){
  return t_dist[inst];
}



void pdg::InstructionStack::UpdateIsCritical(Instruction* inst, bool x){
  if (is_critical[inst]== true && x == false){
    is_critical[inst] = false;
  }
}

void pdg::InstructionStack::UpdateCloserBranch(Instruction* inst, bool x){
  closer_branch[inst] = x;
}




int pdg::InstructionStack::QueryDist(Instruction* inst){
  return distmap[inst];
}
bool pdg::InstructionStack::QueryIsCritical(Instruction* inst){
  return is_critical[inst];
}
bool pdg::InstructionStack::QueryCloserBranch(Instruction* inst){
  return closer_branch[inst];
}

void pdg::InstructionStack::printList(){
  errs()<<"--- Begin to print Instruction List ---\n";
  for (std::list<Instruction*>::iterator it = inst_list.begin(); it!=inst_list.end(); ++it){
    errs()<<*(*it)<<'\n';
  }
  errs()<<"--- End of printing instruction list ---\n";
}

void pdg::InstructionStack::printBrList(){
  errs()<<"--- Begin print Br list ---\n";
  for (std::list<Instruction*>::iterator it = inst_list.begin(); it!=inst_list.end(); ++it){
    if (llvm::BranchInst::classof(*it))
    if (DILocation *Loc = (*it)->getDebugLoc()){
         //(br_inst->isConditional()&& br_inst->getNumSuccessor()==2)){
      BranchInst* br_inst;
      br_inst = static_cast<BranchInst*> (*it);
      if (!br_inst->isConditional())
        continue;
      if (br_inst->getNumSuccessors()!=2)
        continue;
      int line = Loc->getLine();
      std::string  filename = Loc->getFilename().str();
      errs()<<filename<<" "<<line<<" "<<QueryDist(*it)<<' '<<QueryIsCritical(*it)<<' '
            <<QueryCloserBranch(*it)<<"\n";
      errs()<<QueryTDist(*it)<<' '<<QueryFDist(*it)<<' '
            <<llvm::FCmpInst::classof(br_inst->getCondition())<<'\n';
      errs()<<((*it)->getParent()->getName())<<'\n';
    }
  }
}
