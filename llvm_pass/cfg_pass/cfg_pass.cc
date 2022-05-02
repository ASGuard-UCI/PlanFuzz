
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
#include <iostream>
#include <list>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "llvm/IR/User.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Instruction.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Analysis/CFGPrinter.h"
#include <fstream>
using namespace llvm;

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

static bool isBlacklisted(const Function *F){
  static const SmallVector<std::string, 8> Blacklist = {
    "asan.",
    "llvm.",
    "sancov.",
    "__ubsan_handle_",
    "free",
    "malloc",
    "calloc",
    "realloc"
  };
  for (auto const &BlacklistFunc : Blacklist) {
    if (F->getName().startswith(BlacklistFunc)) {
      return true;
    }
  }

  return false;
}


bool AFLCoverage::runOnModule(Module &M) {
  //std::cout<<"Start the pass to generate target position in BB"<<std::endl;
  SAYF("Start the pass to build target pos in BB!\n");
  LLVMContext &C = M.getContext();

  std::list<std::string> targets;
  
  std::ofstream targetsfile("./target.txt", std::ofstream::out | std::ofstream::trunc);
  std::string line;
 
  IntegerType *Int8Ty  = IntegerType::getInt8Ty(C);
  IntegerType *Int32Ty = IntegerType::getInt32Ty(C);
  int inst_blocks = 0;
  
  std::ofstream bbnames("./BBnames.txt", std::ofstream::out | std::ofstream::trunc);
  std::ofstream bbcalls("./BBcalls.txt", std::ofstream::out | std::ofstream::trunc);
  std::ofstream fnames("./fnames.txt", std::ofstream::out | std::ofstream::trunc);
  std::ofstream ftargets("./ftargets.txt", std::ofstream::out | std::ofstream::trunc);

  std::string dotfiles("./dot-files");

  std::ofstream debugout("./debugloc.txt", std::ofstream::out | std::ofstream::trunc);
  for (auto &F : M){
    for (auto &BB : F)
      errs()<<BB.getName()<<'\n';
    std::string funcName = F.getName();
    if (isBlacklisted(&F))
      continue;
    bool has_BBs = true;
    if (has_BBs){
      if (F.getName().str().find("apollo")!=std::string::npos && F.getName().str().find("PathBoundsDecider")!=std::string::npos){
        std::string cfgFileName = "./dot_dir_v4/cfg."+ funcName + ".dot";
        std::error_code EC;
        raw_fd_ostream cfgFile(cfgFileName, EC, sys::fs::F_None);
        if (!EC){
          WriteGraph(cfgFile, &F, true);
        }
      }						              
    }
  }  //end of each function
  return true;
}

char AFLCoverage::ID = 0;

static void registerAFLPass(const PassManagerBuilder &,
                            legacy::PassManagerBase &PM) {

  PM.add(new AFLCoverage());

}

static RegisterPass<AFLCoverage> X("AFL", "AFL instrumentation Pass", false, false);

static RegisterStandardPasses RegisterAFLPass(
    PassManagerBuilder::EP_ModuleOptimizerEarly, registerAFLPass);

static RegisterStandardPasses RegisterAFLPass0(
    PassManagerBuilder::EP_EnabledOnOptLevel0, registerAFLPass);
