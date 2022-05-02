#include "modules/tools/fuzz/vuln_test/tools/bpfuzz_gflag.h"

#include <limits>


DEFINE_bool(enable_PI, true, "Enable the Planning invariant-aware mutatiion");

DEFINE_string(test_apollo_map_path, "", "The path to map file for testing");

DEFINE_string(test_inst_path, "", "The inst config file for testing");

DEFINE_double(init_range, 100, "The range of init");

DEFINE_int(obs_num, 2, "The number of generated obstacles");

DEFINE_bool(enable_data, true, "Enable the data distance calculation");

DEFINE_string(context_path, " ", "The path to the context file (pb binary)");

DEFINE_string(input_path, "", "The path to the planning input path (pb binary)");

DEFINE_string(benign_behavior, "", "The path to the benign planning behavior (pb binary)");

DEFINE_string(saved_input_prefix, "", "The prefix of the saved input files");

DEFINE_string(saved_output_prefix, "", "The prefix of the saved output files");

DEFINE_int(generation_size, 50, "The size of evolutionary generation size");
