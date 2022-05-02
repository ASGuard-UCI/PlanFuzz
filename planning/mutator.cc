#include "src/mutator.h"
#include "src/libfuzzer/libfuzzer_mutator.h"
#include "modules/tools/fuzz/planning/proto/on_lane_planning_msg.pb.h"

#define MUTATIONS 32

extern "C" {
    size_t mutator(uint8_t **data, size_t size, size_t max_size, unsigned int seed) {
        assert(size <= max_size);
        uint8_t *mutated_out = *data;
        //Msg msg;
        apollo::tools::fuzz::planning::OnLanePlanningFuzzMessage msg;
        msg.ParseFromArray(mutated_out, size);
        protobuf_mutator::RandomEngine re(seed);
        protobuf_mutator::libfuzzer::Mutator pbm(&re);
	for(unsigned int i = 0; i < MUTATIONS; i++)
        	pbm.Mutate(&msg, max_size);
        //assert(size <= max_size);
	size = msg.ByteSizeLong();
        msg.SerializeToArray(mutated_out, max_size);
//	size = protobuf_mutator::libfuzzer::CustomProtoMutator(1, mutated_out, size, max_size, seed, &msg);
        return size;
    }
    
}
