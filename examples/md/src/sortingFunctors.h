#ifndef _SORTING_FUNCTORS_
#define _SORTING_FUNCTORS_

#include "common_structs.h"
#include "descriptorVoteProcessing.h"

struct sortVotesByVoteCount : public std::binary_function<notFilteredVotes, notFilteredVotes, bool>
{
    bool 
    operator()( notFilteredVotes a, notFilteredVotes b)
    {
        return (a.voteCount > b.voteCount);
    }
};

        
struct sortPerfDataByComputedRotation : public std::binary_function<perfData, perfData, bool>
{
    bool 
    operator()( perfData a, perfData b)
    {
        return (a.computedRotation < b.computedRotation);
    }
};

#endif