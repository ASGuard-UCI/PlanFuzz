#include "PDGUtils.hpp" 
#include "llvm/IR/InstIterator.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include <sstream>
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

void pdg::PDGUtils::constructInstMap(Function &F)
{
  for (inst_iterator I = inst_begin(F); I != inst_end(F); ++I)
  {
    if (G_instMap.find(&*I) == G_instMap.end())
    {
      InstructionWrapper *instW = new InstructionWrapper(&*I, GraphNodeType::INST);
      G_instMap[&*I] = instW;
      G_funcInstWMap[&F].insert(instW); 
    }
  }
}

void pdg::PDGUtils::constructFuncMap(Module &M)
{
  for (Module::iterator FI = M.begin(); FI != M.end(); ++FI)
  {
    if (FI->isDeclaration())
      continue;
    constructInstMap(*FI);
    if (G_funcMap.find(&*FI) == G_funcMap.end())
    {
      FunctionWrapper *funcW = new FunctionWrapper(&*FI);
      G_funcMap[&*FI] = funcW;
    }
  }
}

void pdg::printDebugInfo(Instruction &inst){
  unsigned line;
  std::string filename;
  errs()<<"*************Start of Debug\n";
  if (DILocation *Loc = inst.getDebugLoc()){
    line = Loc->getLine();
    filename = Loc->getFilename().str();
    errs()<<"    Pos of LLVM IR: "<<filename<<" : "<<line<<'\n';
  }
  errs()<<"******************End of Debug\n";
}

bool pdg::CDGPrune(Instruction &inst){
  if (auto c = dyn_cast<BranchInst>(&inst)){
  }
  else{ 
    errs()<<"Skip a CDG because of not a branch "<<inst;
    return true;
  }
  unsigned line;
  std::string filename;
  if (DILocation *Loc = inst.getDebugLoc()){
    line = Loc->getLine();
    filename = Loc->getFilename().str();
    if (filename.find("/usr/")!=std::string::npos)
      return true;
  }
  std::string init;
  init.empty();
  llvm::raw_string_ostream ss(init);
  inst.print(ss);
  std::string inst_str = ss.str();
  if (inst_str.find("gcov")!=std::string::npos)
    return true;
  return false;
}

bool pdg::DDGPrune(Instruction &inst){
  unsigned line;
  std::string filename;
  if (DILocation *Loc = inst.getDebugLoc()){
    line = Loc->getLine();
    filename = Loc->getFilename().str();
    if (filename.find( "/usr/")!=std::string::npos)
      return true;
  }
  std::string init;
  init.empty();
  llvm::raw_string_ostream ss(init);
  inst.print(ss);
  std::string inst_str = ss.str();
  if (inst_str.find("gcov")!=std::string::npos)
    return true;
  if (auto c = dyn_cast<StoreInst>(&inst)){
    return false;
  }
  if (inst.getType()==NULL)
    return true;
  if (inst.getType()->isIntegerTy()){
    if (inst.getType()->getPrimitiveSizeInBits()!=1)
      return true;
  }
  else 
    return true;
  return false;
}

void pdg::PDGUtils::collectGlobalInsts(Module &M)
{
  for (Module::global_iterator globalIt = M.global_begin(); globalIt != M.global_end(); ++globalIt)
  {
    InstructionWrapper *globalW = new InstructionWrapper(dyn_cast<Value>(&(*globalIt)), GraphNodeType::GLOBAL_VALUE);
    G_globalInstsSet.insert(globalW);
  }
}

void pdg::PDGUtils::categorizeInstInFunc(Function &F)
{
  // sort store/load/return/CallInst in function
  for (inst_iterator I = inst_begin(F), IE = inst_end(F); I != IE; ++I)
  {
    Instruction *inst = dyn_cast<Instruction>(&*I);
    if (isa<StoreInst>(inst))
      G_funcMap[&F]->addStoreInst(inst);

    if (isa<LoadInst>(inst))
      G_funcMap[&F]->addLoadInst(inst);

    if (isa<ReturnInst>(inst))
      G_funcMap[&F]->addReturnInst(inst);

    if (isa<CallInst>(inst))
      G_funcMap[&F]->addCallInst(inst);

    if (isa<CastInst>(inst))
      G_funcMap[&F]->addCastInst(inst);
  }
}
