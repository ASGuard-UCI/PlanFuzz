# PlanFuzz_private

PlanFuzz is a novel dynamic testing tool to automatically discover semantic DoS vulnerabilities in Autonomous Driving (AD) behavioral planning. Semantic DoS vulnerability refers to the vulnerabilities inside AD planning which can be exploited by external attacker and lead to overly-conservative decisions.

## Paper
Too Afraid to Drive: Systematic Discovery of Semantic DoS Vulnerability in Autonomous Driving Planning under Physical-World Attacks

Author: Ziwen Wan, Junjie Shen, Jalen Chuang, Xin Xia, Joshua Garcia, Jiaqi Ma, Qi Alfred Chen

Website: https://sites.google.com/view/cav-sec/planfuzz

## Set up

### Mark unexpected decision in source code

The first step of identifying the semantic DoS vulnerability is to mark the unexpected planning decision in the source code. An example of this could be:
```C
MARK_TARGETED_POS()
make_stop_decision();
```
The first line is a pre-defined macro in the following static analysis program. This basic block will be further marked as a targeted block by the analysis phase. Multiple blocks are allowed.

### Off-line static analysis & 

Necessary requirement:
- Installing LLVM 6.0
- Installing wllvm (whole-program llvm)
- Compile Apollo (or Autoware) with wllvm

#### Step 1
Use rename pass to mark the basic block and find the basic block marked as targeted block.
```bash
opt -load /apollo/llvm_pass/rename_pass/rename_pass.so -AFL <$1".a.bc" >$1"_rename.bc"
```

opt is the LLVM optimizer. $1".a.bc" is the bitcode of the program under test. For example, if we want to reach some decision in lane changing part, we can compile the whole planning in Apollo and use extract-bc (wllvm) to extract "liblane_changing.a.bc" here.

#### Step 2
Use pdg pass to analyze the data/control dependency in the program and generate instrumentation profile.
```bash
opt -load /apollo/PDG_LLVM/build/libpdg.so  -PDG <$1".rename.bc" >null 2>inst_config
```
Here the inst_config is some runtime log generated in the pass. It contains necessary information to instrument the program such as which predicate should be instrumented and the way is should be instrumented. 

#### Step 3
Instrument the program and recompile.
```bash
opt -load /apollo/llvm_pass/AD_pass/AD_pass.so -AFL <$1".rename.bc" >$1".inst.bc"
llc -filetyoe=obj -relocation-model=pic -O0 $1".inst.bc" -o $1".a"
```
After that, replace the original source code in BUILD (or Makefile) with the newly generated ".a" file. Recompile the whole planning, we can get an instrumented binary which can generate the BP vulnerability distance at runtime. 

### Data pre-processing

Our fuzzing interface is specalized designed which includes inputs to planning including localization, prediction, and routing results. The defination of such input is inside the ./planning/proto/on_lane_planning_msg.proto. Also, some basic interface of planning module is included in the same folder.

### Building testing tool

Our testing tool is tightly coupled with the existing AD software and is built as a target in Bazel (for Apollo) or CMake (for Autoware.AI).  


