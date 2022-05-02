#ifndef INSTRUCTIONSTACK_H_
#define INSTRUCTIONSTACK_H_

#include "llvm/IR/Module.h"
#include "llvm/PassAnalysisSupport.h"
#include "ControlDependencyGraph.hpp"
#include "DataDependencyGraph.hpp"
#include "llvm/IR/DebugInfo.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/Support/Debug.h"
#include "DependencyGraph.hpp"
#include "PDGCommandLineOptions.hpp"


namespace pdg{
class InstructionStack{
  public:
  bool isInStack(llvm::Instruction* inst);
  void addInst(llvm::Instruction* inst);
  void printList();
  void printBrList();
  void UpdateDist(llvm::Instruction* inst, int dist);
  int QueryDist(llvm::Instruction* inst);
  bool QueryCloserBranch(llvm::Instruction* inst);
  bool QueryIsCritical(llvm::Instruction* inst);
  void UpdateIsCritical(llvm::Instruction *inst, bool );
  void UpdateCloserBranch(llvm::Instruction *inst, bool);
  void UpdateFDist(llvm::Instruction*, int);
  int QueryFDist(llvm::Instruction*);
  void UpdateTDist(llvm::Instruction*, int);
  int QueryTDist(llvm::Instruction*);
 
  private:
  std::map<llvm::Instruction* , bool> instmap;
  std::map<llvm::Instruction*, int> distmap;
  std::map<llvm::Instruction*, bool> is_critical;
  std::map<llvm::Instruction*, bool> closer_branch;
  std::list<llvm::Instruction*> inst_list;
  std::map<llvm::Instruction*, int> t_dist;
  std::map<llvm::Instruction*, int> f_dist;
};


} //end of namespace pdg
#endif
