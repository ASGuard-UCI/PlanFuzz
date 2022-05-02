#include "FunctionFilter.hpp"
#include "llvm/ADT/SmallVector.h"
#include "PDGUtils.hpp"
using namespace llvm;
bool isFiltered(const Function *F){

  static const SmallVector<std::string, 8> Blacklist = {
    "asan.",
    "llvm.",
    "sancov.",
    "__ubsan_handle_",
    "free",
    "calloc",
    "realloc",
  };
  for (auto const &BlacklistFunc : Blacklist){
    if (F->getName().startswith(BlacklistFunc)){
      return true;
    }
  }
  static const SmallVector<std::string, 2> Whitelist = {
    "apollo",
    "planning",
  };
  for (auto const &WhitelistFunc : Whitelist){
    if (F->getName().find(WhitelistFunc)==-1){
      return true;
    }
  }
  //if (F->getName().find("ZN6apollo8planning17PathBoundsDecider30GetBoundaryFromStatic")!=-1
	//	||F->getName().find("ZN6apollo8planning17PathBoundsDecider31Update")!=-1
  //)
//    return false;
  return false;
}
