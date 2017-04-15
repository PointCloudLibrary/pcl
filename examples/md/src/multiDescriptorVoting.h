#ifndef _MULTI_DESCRIPTOR_VOTING_
#define _MULTI_DESCRIPTOR_VOTING_

#include "common_structs.h"
#include "descriptorVoteProcessing.h"

class CMultiDescriptorVoting
{
public:

    CMultiDescriptorVoting(inputArgs & inputParams, outputPerf * pGlobalPerf);
    void computeVotes();

    void displayVotingResults();
    void displayRuntimePerformance();

    void logFinalResults() { descriptorVote_.logFinalResults(); }

    struct sortPerfDataByEstErr : public std::binary_function<perfData, perfData, bool>
    {
      bool 
      operator()( perfData a, perfData b)
      {
        return (a.rotEstError < b.rotEstError );
      }
    };

    struct sortPerfDataByRuntime : public std::binary_function<perfData, perfData, bool>
    {
      bool 
      operator()( perfData a, perfData b)
      {
        return (a.processingTime < b.processingTime );
      }
    };

private:
    outputPerf			*	pGlobalPerf_;
    bool				enableSorting_;
    vector<perfData>			voteList_;
    CDescriptorVoteProcessing	        descriptorVote_;
    ofstream			*	logResults_;
    inputArgs			&	inputParams_;
};

#endif
