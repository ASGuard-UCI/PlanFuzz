#ifndef PROGRAMDEPENDENCYGRAPH_H_
#define PROGRAMDEPENDENCYGRAPH_H_
#include "llvm/IR/Module.h"
#include "llvm/PassAnalysisSupport.h"
#include "ControlDependencyGraph.hpp"
#include "DataDependencyGraph.hpp"
#include "llvm/IR/DebugInfo.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/Support/Debug.h"
#include "DependencyGraph.hpp"
#include "PDGCommandLineOptions.hpp"
#include "InstructionStack.hpp"
#include <stack>

namespace pdg
{
class ProgramDependencyGraph : public llvm::ModulePass
{
public:
  static char ID;
  ProgramDependencyGraph() : llvm::ModulePass(ID) { PDG = new DependencyGraph<InstructionWrapper>(); }
  ~ProgramDependencyGraph() { delete PDG; }
  bool runOnModule(llvm::Module &M);
  void getAnalysisUsage(llvm::AnalysisUsage &AU) const;
  llvm::StringRef getPassName() { return "Program Dependency Graph"; }
  // PDG processing
  bool processCallInst(InstructionWrapper *instW);
  bool processIndirectCallInst(llvm::CallInst *CI, InstructionWrapper *instW);
  void addNodeDependencies(InstructionWrapper *instW);
  // parameter tree building
  std::vector<llvm::Function *> collectIndirectCallCandidates(llvm::FunctionType *funcType);
  void buildActualParameterTrees(llvm::CallInst *CI);
  void drawActualParameterTree(llvm::CallInst *CI, TreeType treeTy);
  void buildFormalTreeForFunc(llvm::Function *Func);
  void buildFormalTreeForArg(llvm::Argument &arg, TreeType treeTy);
  bool hasRecursiveType(ArgumentWrapper *argW, tree<InstructionWrapper *>::iterator insert_loc);
  bool isFilePtrOrFuncTy(llvm::Type* ty);
  InstructionWrapper* buildPointerTypeNode(ArgumentWrapper *argW, InstructionWrapper *curTyNode, tree<InstructionWrapper *>::iterator);
  InstructionWrapper *buildPointerTypeNodeWithDI(ArgumentWrapper *argW, InstructionWrapper *curTyNode, tree<InstructionWrapper *>::iterator, llvm::DIType *dt);
  void buildTypeTree(llvm::Argument &arg, InstructionWrapper *treeTyW, TreeType TreeType);
  void drawFormalParameterTree(llvm::Function *Func, TreeType treeTy);
  void connectFunctionAndFormalTrees(llvm::Function *callee);
  bool connectAllPossibleFunctions(llvm::CallInst *CI, std::vector<llvm::Function *> indirect_call_candidates);
  bool connectCallerAndCallee(InstructionWrapper *instW, llvm::Function *callee);
  void connectActualTrees(InstructionWrapper *callInstW);
  void connectInOutTrees(ArgumentWrapper *CIArgW, ArgumentWrapper *funcArgW);
  // field sensitive related functions
  std::set<pdg::InstructionWrapper *> getAllRelevantGEP(llvm::Argument &arg);
  InstructionWrapper *getTreeNodeGEP(llvm::Argument &arg, unsigned field_offset, llvm::Type *treeNodeTy, llvm::Type *parentNodeTy);
  std::vector<llvm::Instruction *> getArgStoreInsts(llvm::Argument &arg);
  // tree building helper functions
  bool isFuncTypeMatch(llvm::FunctionType *funcTy1, llvm::FunctionType *funcTy2);
  tree<InstructionWrapper *>::iterator getInstInsertLoc(ArgumentWrapper *argW, InstructionWrapper *tyW, TreeType treeTy);
  //  dep printer related functions
  std::vector<DependencyNode<InstructionWrapper> *> getNodeSet() { return PDG->getNodeSet(); }
  DependencyGraph<InstructionWrapper> *_getPDG() { return PDG; }
  void growInsideFunc(std::set<llvm::Instruction*> &inst_set, llvm::Module &M, std::set<std::string> &invoked_func_list);
  typename DependencyNode<InstructionWrapper>::DependencyLinkList getNodeDepList(llvm::Instruction *inst);
  typename DependencyNode<InstructionWrapper>::DependencyLinkList getNodeReverseDepList(llvm::Instruction *inst);
  void expandDependency(std::stack<llvm::Instruction*> *expand_stack, llvm::Module &M);
  void expandCallInst(std::stack<llvm::Instruction*> *expand_stack,llvm::Function* Func ,llvm::Module &M);
  void expandRetInst(std::stack<llvm::Instruction*> *expand_stack, llvm::Function* Func, int, llvm::Module &M);
private:
  llvm::Module *module;
  DependencyGraph<InstructionWrapper> *PDG;
  ControlDependencyGraph *cdg;
  DataDependencyGraph *ddg;
  std::map<const llvm::Function*, DataDependencyGraph*> ddg_map;
  std::map<const llvm::Function*, ControlDependencyGraph*> cdg_map;
  InstructionStack *inst_stack;
  int BBDistance(llvm::Instruction* prev, llvm::Instruction* next, llvm::Module &M);
  std::map<llvm::Function*, std::set<llvm::Function*>> calledGraph;
  std::map<llvm::Function*, bool> visited; 
};
} // namespace pdg

#endif
