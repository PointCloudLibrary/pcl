#ifndef _CDESCRIPTOR_VOTE_PROCESSING_
#define _CDESCRIPTOR_VOTE_PROCESSING_

#include "common_structs.h"
#include "descriptorVoteHelper.h"

class CDescriptorVoteProcessing
{

public:
    CDescriptorVoteProcessing(inputArgs & inputParams, outputPerf * pGlobalPerf )
    :   inputParams_(inputParams)
    ,	pGlobalPerf_(pGlobalPerf)
    ,	logResults_(inputParams.logFile2)
    ,	threshold_(inputParams.vote_threshold)
    {
        minErr_ = 1000.0; //## some large positive number
        maxErr_ = 0;
        notFilteredVotes_.resize(1);
    }

    void mySort();
    void vote(perfData & pd);
    void insertElement(votingObject & vtob);

    void computeStats();
    void computeVotingResults();
    void logFinalResults();

    void getVotingResults(votingResult & ofo) 
         { 
            ofo.estErr	= chosenVote_.estErr; 
            ofo.average	= chosenVote_.average;
            ofo.numVotes	= chosenVote_.numVotes;
         }

private:
    perfData				currentPD_;	// pd currently being processed
    vector<votingObject *>		votingHist_;

    float				threshold_;
    float				minErr_;
    float				maxErr_;

    votingResult			chosenVote_;

    vector<notFilteredVotes>		notFilteredVotes_;

    inputArgs			&	inputParams_;
    outputPerf			*	pGlobalPerf_;
    ofstream			*	logResults_;

};

#endif
