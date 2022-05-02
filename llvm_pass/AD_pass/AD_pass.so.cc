
/*
  Copyright 2015 Google LLC All rights reserved.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at:

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

/*
   american fuzzy lop - LLVM-mode instrumentation pass
   ---------------------------------------------------

   Written by Laszlo Szekeres <lszekeres@google.com> and
              Michal Zalewski <lcamtuf@google.com>

   LLVM integration design comes from Laszlo Szekeres. C bits copied-and-pasted
   from afl-as.c are Michal's fault.

   This library is plugged into LLVM when invoking clang through afl-clang-fast.
   It tells the compiler to add code roughly equivalent to the bits discussed
   in ../afl-as.h.
*/

#define AFL_LLVM_PASS

#include "../config.h"
#include "../debug.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include "llvm/IR/DebugLoc.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/IR/User.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/Support/Debug.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
using namespace llvm;



struct inst_info{
  std::string file_name;
  int line_id;
  int BB_dist;
  bool is_critical;
  bool closer_branch;
  int t_dist; int f_dist;
  bool is_fcmp;
  std::string BB_name;
};

std::vector<struct inst_info> inst_info_list;
namespace {

  class AFLCoverage : public ModulePass {

    public:

      static char ID;
      AFLCoverage() : ModulePass(ID) { }

      bool runOnModule(Module &M) override;

      // StringRef getPassName() const override {
      //  return "American Fuzzy Lop Instrumentation";
      // }

  };

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

int findInstInfo(Instruction* x){
  std::string BBname = x->getParent()->getName();
  int line; std::string filename;
  if (DILocation *Loc = x->getDebugLoc()){
    line = Loc->getLine();
    filename = Loc->getFilename().str();
  }
  else
    return -1;
  for (int i=0; i< inst_info_list.size(); i++){
    if (line == inst_info_list[i].line_id 
      && filename == inst_info_list[i].file_name
      && BBname == inst_info_list[i].BB_name)
      return i;
  }
  return -1;
}
bool instrumentBr(Instruction* insert_pt, int index, Module* M, GlobalVariable* global_br_count_ptr){
  if (index == -1)
    return false;
  if (DILocation *loc = insert_pt->getDebugLoc()){
    errs()<<"Instrument br  "<<index<<" "<<loc->getFilename()<<" "<<loc->getLine()<<"\n";
  }
  LLVMContext &C = M->getContext();
  IntegerType *Int32Ty = IntegerType::getInt32Ty(C);
  auto br_inst = static_cast<llvm::BranchInst*>(insert_pt);
  TerminatorInst **if_term = new TerminatorInst*;
  TerminatorInst **else_term = new TerminatorInst*;
  SplitBlockAndInsertIfThenElse(br_inst->getCondition(), insert_pt, if_term, else_term);

  IRBuilder<> IRB1(*if_term);
  LoadInst *br_count_ptr = IRB1.CreateLoad(global_br_count_ptr);
  ConstantInt *const_index = ConstantInt::get(Int32Ty, index);
  ConstantInt *const_2 = ConstantInt::get(Int32Ty, 2);
  Value *count_index = IRB1.CreateGEP(br_count_ptr, IRB1.CreateMul(const_index, const_2));
  LoadInst* counter = IRB1.CreateLoad(count_index);
  Value *incr = IRB1.CreateAdd(counter, ConstantInt::get(Int32Ty, 1));
  IRB1.CreateStore(incr, count_index);


  IRBuilder<> IRB2(*else_term);
  br_count_ptr = IRB2.CreateLoad(global_br_count_ptr);
  count_index = IRB2.CreateGEP(br_count_ptr, IRB2.CreateAdd(
    IRB2.CreateMul(const_index, const_2), ConstantInt::get(Int32Ty, 1)));
  counter = IRB2.CreateLoad(count_index);
  incr = IRB2.CreateAdd(counter, ConstantInt::get(Int32Ty, 1));
  IRB2.CreateStore(incr, count_index);
  return true;
 
}


bool instrumentFcmp(Instruction* insert_pt, int index, Module* M, GlobalVariable* global_count_ptr, GlobalVariable* global_min_ptr, GlobalVariable* global_debug_flag){
  LLVMContext &C = M->getContext();
  if (index==-1)
    return false;
  if (DILocation *loc = insert_pt->getDebugLoc()){
    errs()<<"Instrument Fcmp  "<<index<<" "<<loc->getFilename()<<" "<<loc->getLine()<<"\n";
  }
  IntegerType *Int32Ty = IntegerType::getInt32Ty(C);
  Type* DoubleTy = Type::getDoubleTy(C);
  auto br_inst = static_cast<llvm::BranchInst*>(insert_pt);

  auto label_0 = br_inst->getSuccessor(0)->getName();
  auto label_1 = br_inst->getSuccessor(1)->getName();
  //errs()<<"Got 2 labels: "<<label_0<<" and "<<label_1<<'\n'; 
  //errs()<<"Label 0 = "<<label_0<<'\n';
  //errs()<<"Label 1 = "<<label_1<<'\n';
  /*
  std::string func_name = br_inst->getParent()->getParent()->getName();
  std::string dist_file_name = "/apollo/llvm_pass/integrate_test/dot_dir_v6/cfg." + func_name+ ".dot.distances.txt"; 
  std::map<std::string, double> bb_to_dis;
  std::vector<std::string> basic_blocks;
  std::ifstream cf(dist_file_name);
    if (cf.is_open()) {
      //errs()<<"Successfully open distance file"<<dist_file_name<<'\n';
      std::string line;
      while (getline(cf, line)) {

        std::size_t pos = line.find(",");
        std::string bb_name = line.substr(0, pos);
        double bb_dis =atof(line.substr(pos + 1, line.length()).c_str());
        bb_to_dis.emplace(bb_name, bb_dis);
        basic_blocks.push_back(bb_name);

      }
      cf.close();


    } else {
      //SAYF("Unable to find %s.", dist_file_name.c_str());
  }
  double dist_0 = -1.0;
  double dist_1 = -1.0;
  std::map<std::string, double>::iterator it;
  for (it = bb_to_dis.begin(); it!=bb_to_dis.end(); ++it){
    if (it->first.compare(label_0)==0){
      dist_0 = it->second;
      //errs()<<"Find a matching distance"<<it->second<<'\n';
    }
    if (it->first.compare(label_1)==0){
      dist_1 = it->second;
    }
  }
  if (dist_0 == -1.0 || dist_1 == -1.0){
    return false;
  }  
  errs()<<label_0<<" "<<label_1<<" "<<dist_0<<" "<<dist_1<<"\n";
*/
  
  IRBuilder<> IRB0(insert_pt);
//Start of debugging IR
 /* 
  Constant *hookFunc;
  hookFunc = M->getOrInsertFunction("print", Type::getVoidTy(M->getContext()), (Type*)0);            
  Function* hook= cast<Function>(hookFunc);
  IRB0.CreateCall(hook,None); */
  LoadInst* count_ptr;
  Value* count_index;
/*
  count_ptr = IRB0.CreateLoad(global_debug_flag);
  count_index = IRB0.CreateGEP(count_ptr, ConstantInt::get(Int32Ty, 214748323390));
  count_ptr = IRB0.CreateLoad(count_index);  
  IRB0.CreateStore(ConstantInt::get(Int32Ty, 8888), count_index);*/

//End of debugging IR
  Value* compare_fabs = IRB0.CreateFCmpOGE(
    ((Instruction*)br_inst->getCondition())->getOperand(0),
    ((Instruction*)br_inst->getCondition())->getOperand(1));
  TerminatorInst** bb0_if = new TerminatorInst*;
  TerminatorInst** bb0_else = new TerminatorInst*;
  SplitBlockAndInsertIfThenElse(compare_fabs, insert_pt, bb0_if, bb0_else);
  IRBuilder<> IRB0_0(*bb0_if);  
  IRBuilder<> IRB0_1(*bb0_else);
  Value* fabs_0 = IRB0_0.CreateFSub(((Instruction*)br_inst->getCondition())->getOperand(0), ((Instruction*)br_inst->getCondition())->getOperand(1));
  Value* fabs_1 = IRB0_1.CreateFSub(((Instruction*)br_inst->getCondition())->getOperand(1), ((Instruction*)br_inst->getCondition())->getOperand(0));
  
  IRBuilder<> IRB0_2(insert_pt);
  PHINode* fabs_phi = IRB0_2.CreatePHI(fabs_1->getType(), 2);
  fabs_phi->addIncoming(fabs_0, (*bb0_if)->getParent());
  fabs_phi->addIncoming(fabs_1, (*bb0_else)->getParent());

  TerminatorInst **if_term = new TerminatorInst*; 
  TerminatorInst **else_term = new TerminatorInst*;
  SplitBlockAndInsertIfThenElse(br_inst->getCondition(), insert_pt, if_term, else_term);      
  //Update count for True branch 
  IRBuilder<> IRB1(*if_term);
  count_ptr = IRB1.CreateLoad(global_count_ptr);
  ConstantInt *const_index = ConstantInt::get(Int32Ty, index);
  ConstantInt *const_2 = ConstantInt::get(Int32Ty,2);
  count_index = IRB1.CreateGEP(count_ptr, IRB1.CreateMul(const_index, const_2));
  
  LoadInst* counter = IRB1.CreateLoad(count_index);
  Value *incr = IRB1.CreateAdd(counter, ConstantInt::get(Int32Ty, 1));
  IRB1.CreateStore(incr, count_index);

  //Calculate fabs value and compare
  LoadInst *min_ptr = IRB1.CreateLoad(global_min_ptr);
  Value *min_index = IRB1.CreateGEP(DoubleTy, min_ptr, IRB1.CreateMul(const_index, const_2));
  LoadInst* min = IRB1.CreateLoad(DoubleTy, min_index);
  Value* cmp_fabs_min = IRB1.CreateFCmpOGE(min, fabs_phi);
  TerminatorInst** temp_if = new TerminatorInst*;
  TerminatorInst** temp_else = new TerminatorInst*;
  SplitBlockAndInsertIfThenElse(cmp_fabs_min, *if_term, temp_if, temp_else);
  IRBuilder<> IRB1_0(*temp_if);
  IRB1_0.CreateStore(fabs_phi, min_index);


  //Update coutn for False branch
  IRBuilder<> IRB2(*else_term);
  count_ptr = IRB2.CreateLoad(global_count_ptr);
  const_index  =ConstantInt::get(Int32Ty, index);
  count_index = IRB2.CreateGEP(count_ptr, IRB2.CreateAdd(
    IRB2.CreateMul(const_index, const_2), ConstantInt::get(Int32Ty, 1)));
  
  counter = IRB2.CreateLoad(count_index);
  incr = IRB2.CreateAdd(counter, ConstantInt::get(Int32Ty, 1));
  IRB2.CreateStore(incr, count_index);

  min_ptr = IRB2.CreateLoad(global_min_ptr);
  min_index = IRB2.CreateGEP(DoubleTy, min_ptr, IRB2.CreateAdd(
    IRB2.CreateMul(const_index, const_2), ConstantInt::get(Int32Ty, 1)));
  min = IRB2.CreateLoad(DoubleTy, min_index);
  cmp_fabs_min = IRB2.CreateFCmpOGE(min, fabs_phi);
  temp_if = new TerminatorInst*;
  temp_else = new TerminatorInst*;
  SplitBlockAndInsertIfThenElse(cmp_fabs_min, *else_term, temp_if, temp_else);
  IRBuilder<> IRB2_0(*temp_if);
  IRB2_0.CreateStore(fabs_phi, min_index);
  return true;
}
char AFLCoverage::ID = 0;


bool AFLCoverage::runOnModule(Module &M) {
  LLVMContext &C = M.getContext();
  LoadInstInfo("/apollo/vuln_test_bc/lane_follow_v1/inst_config");
  IntegerType *Int8Ty  = IntegerType::getInt8Ty(C);
  IntegerType *Int32Ty = IntegerType::getInt32Ty(C);

  /* Show a banner */

  char be_quiet = 0;

  if (isatty(2) && !getenv("AFL_QUIET")) {

    SAYF(cCYA "afl-llvm-pass " cBRI VERSION cRST " by <lszekeres@google.com>\n");

  } else be_quiet = 1;

  /* Decide instrumentation ratio */

  char* inst_ratio_str = getenv("AFL_INST_RATIO");
  unsigned int inst_ratio = 100;

  if (inst_ratio_str) {

    if (sscanf(inst_ratio_str, "%u", &inst_ratio) != 1 || !inst_ratio ||
        inst_ratio > 100)
      FATAL("Bad value of AFL_INST_RATIO (must be between 1 and 100)");

  }

  /* Get globals for the SHM region and the previous location. Note that
     __afl_prev_loc is thread-local. */
/*
  GlobalVariable *AFLMapPtr =
      new GlobalVariable(M, PointerType::get(Int8Ty, 0), false,
                         GlobalValue::ExternalLinkage, 0, "__afl_area_ptr");

  GlobalVariable *AFLPrevLoc = new GlobalVariable(
      M, Int32Ty, false, GlobalValue::ExternalLinkage, 0, "__afl_prev_loc",
      0, GlobalVariable::GeneralDynamicTLSModel, 0, false);
*/
  /* Instrument all the things! */

  Type* DoubleTy = Type::getDoubleTy(C);
 
  GlobalVariable *global_count_ptr = new GlobalVariable(M, PointerType::get(Int32Ty, 0), false,
    GlobalVariable::ExternalLinkage, 0, "count_ptr");
  GlobalVariable *global_min_ptr = new GlobalVariable(M, PointerType::get(DoubleTy, 0), false,
    GlobalVariable::ExternalLinkage, 0, "min_ptr");
  GlobalVariable *global_debug_flag = new GlobalVariable(M, PointerType::get(Int32Ty, 0), false,
    GlobalVariable::ExternalLinkage, 0, "debug_flag");
  GlobalVariable *global_br_count_ptr = new GlobalVariable(M, PointerType::get(Int32Ty, 0), false,
    GlobalVariable::ExternalLinkage, 0, "br_count_ptr");  

  int inst_blocks = 0;
  int br_inst_blocks = 0;
  for (auto &F : M){
    std::vector<Instruction*> inst_list;
    std::vector<Instruction*> br_inst_list;
    for (auto &BB : F) {
      llvm::Instruction *insert_pt;
      bool should_inst = false;
      llvm::BranchInst *br_inst;
      for (auto &I : BB)
        if (llvm::BranchInst::classof(&I)){
        int line; std::string filename;
        if (DILocation *Loc = I.getDebugLoc()){
          //errs()<<Loc->getLine()<<" "<<Loc->getFilename()<<" "<<I<<"\n";
          line = Loc->getLine();
          filename = Loc->getFilename().str();
          //errs()<<"Find a  br inst "<<filename<<" "<<line<<' '<<I.getParent()->getName()<<'\n';
          if (findInstInfo(&I) ==-1)
            continue;
        }
        else
          continue; 
        //errs()<<I<<"\n";
        br_inst = static_cast<llvm::BranchInst*>(&I);
        if (!(br_inst->isConditional()))
          continue;
        if (br_inst->isConditional() && br_inst->getNumSuccessors()==2){
          //errs()<<(*curr->getCondition())<<"\n";
          if (llvm::FCmpInst::classof(br_inst->getCondition())){
            insert_pt = &I;
            should_inst = true;
            //errs()<<"Capture a predicate "<<I<<"  depends on "<<(*br_inst->getCondition())<<"\n";
          }
        }
        br_inst_list.push_back(&I);
      }
      if (should_inst == false)
        continue;
      inst_list.push_back(insert_pt);

    }
    for (auto I : inst_list){
      if (instrumentFcmp(I, findInstInfo(I), &M, global_count_ptr, global_min_ptr, global_debug_flag))
        inst_blocks++;
    }
    for (auto I : br_inst_list){
      if (instrumentBr(I, findInstInfo(I), &M, global_br_count_ptr))
        br_inst_blocks++;
    }
  /* Say something nice. */
  }
  if (!be_quiet) {

    if (!inst_blocks) WARNF("No instrumentation targets found.");
    else OKF("Instrumented %u locations (%s mode, ratio %u%%).",
             inst_blocks, getenv("AFL_HARDEN") ? "hardened" :
             ((getenv("AFL_USE_ASAN") || getenv("AFL_USE_MSAN")) ?
              "ASAN/MSAN" : "non-hardened"), inst_ratio);

  }

  return true;

}


static void registerAFLPass(const PassManagerBuilder &,
                            legacy::PassManagerBase &PM) {

  PM.add(new AFLCoverage());

}

static RegisterPass<AFLCoverage> X("AFL", "AFL instrumentation Pass", false, false);

static RegisterStandardPasses RegisterAFLPass(
    PassManagerBuilder::EP_ModuleOptimizerEarly, registerAFLPass);

static RegisterStandardPasses RegisterAFLPass0(
    PassManagerBuilder::EP_EnabledOnOptLevel0, registerAFLPass);
