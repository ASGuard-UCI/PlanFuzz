
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
  


  for (auto &F : M){
    std::string funcName = F.getName();
    if (isBlacklisted(&F))
      continue;
    bool is_target = false;
    for (auto &BB : F) {
      bool BB_is_target = false;
      std::string bb_name("");
      std::string filename;
      unsigned line;
      for (auto &I : BB){
        if (DILocation *Loc = I.getDebugLoc()){
          line = Loc->getLine();
          filename = Loc->getFilename().str();
        }
        if (filename =="" || line == 0)
          continue;
        if (bb_name.empty()){
          std::size_t found = filename.find_last_of("/\\");
          if (found !=std::string::npos)
            filename = filename.substr(found+1);
          bb_name = filename + ":" + std::to_string(line);
        }
      }
      
      if (!bb_name.empty()) {
          BB.setName(bb_name + ":");
          errs()<<"set name "<<bb_name<<'\n';
          if (!BB.hasName()) {
            SAYF("BB does not have name!");
            std::string newname = bb_name + ":";
            Twine t(newname);
            SmallString<256> NameData;
            StringRef NameRef = t.toStringRef(NameData);
            BB.setValueName(ValueName::Create(NameRef));
          }

        }
      //if (!bb_name.empty()){
         
      //}
    }  //end of each BB
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
