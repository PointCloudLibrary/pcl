#include "descriptorVoteProcessing.h"
#include "sortingFunctors.h"

void CDescriptorVoteProcessing::vote(perfData & pd)
{
    currentPD_ = pd;
    bool bDone = false;

    for ( unsigned i = 0; !bDone && i < votingHist_.size() && votingHist_[i]->Votes.size() ; i++ )
    {
        if ( abs( abs(currentPD_.computedRotation) - abs(votingHist_[i]->Votes[0]->computedRotation) ) < threshold_ )
        {
            insertElement(*votingHist_[i]);
            bDone = true;
        }
    }

    if ( !bDone)
    {
        votingObject * pVotingObject(new votingObject);

        votingHist_.push_back(pVotingObject);

        insertElement(*pVotingObject);
    }
}

void CDescriptorVoteProcessing::insertElement(votingObject & vtob)
{
    vtob.setElement(currentPD_);

    //## not in use ## collect more stats
    float err =  abs(abs(currentPD_.computedRotation) - abs(currentPD_.actualRotation));
    if (  err  < minErr_ )
    {
        minErr_ = err;	// keep the min
    }
    if ( err > maxErr_ )
    {
        maxErr_ = err;	// keep the max
    }
}

void CDescriptorVoteProcessing::computeVotingResults()
{
    notFilteredVotes_.resize(votingHist_.size());

    for ( unsigned i = 0; i < votingHist_.size(); i++ )
    {
        notFilteredVotes_[i].idx = i;
        notFilteredVotes_[i].voteCount = votingHist_[i]->Votes.size();
        notFilteredVotes_[i].modality2D = votingHist_[i]->modality2D;
        notFilteredVotes_[i].modality3D = votingHist_[i]->modality3D;
        notFilteredVotes_[i].modality2D3D = votingHist_[i]->modality2D3D;
        notFilteredVotes_[i].topComputedRotation = votingHist_[i]->average;
    }

    std::sort (notFilteredVotes_.begin (), notFilteredVotes_.end (),
                sortVotesByVoteCount ());

    // initialize the default vote
    
    chosenVote_.average		= votingHist_[notFilteredVotes_[0].idx]->average;
    chosenVote_.estErr		= votingHist_[notFilteredVotes_[0].idx]->voteErr;
    chosenVote_.numVotes	= votingHist_[notFilteredVotes_[0].idx]->Votes.size();

    bool bFound = false;

    for ( unsigned i = 0; !bFound && i < notFilteredVotes_.size(); i++ )
    {
        float computedAverage = abs(notFilteredVotes_[i].topComputedRotation);

        if ( 0 ) 
        {
            // use only 2D descriptors
            if ( notFilteredVotes_[i].modality2D )
            {
                chosenVote_.average		= votingHist_[notFilteredVotes_[i].idx]->average2D;
                chosenVote_.estErr		= votingHist_[notFilteredVotes_[i].idx]->voteErr2D;
                chosenVote_.numVotes	= votingHist_[notFilteredVotes_[i].idx]->Votes.size();
                bFound = true;
            }
        }
        else if (  true )
        {
            // prefer 2D&3D
            if ( notFilteredVotes_[i].modality2D3D )
            {
                chosenVote_.average		= votingHist_[notFilteredVotes_[i].idx]->average;
                chosenVote_.estErr		= votingHist_[notFilteredVotes_[i].idx]->voteErr;
                chosenVote_.numVotes	= votingHist_[notFilteredVotes_[i].idx]->Votes.size();
                bFound = true;
            }
        }

        else	// prefer 3D
        {
            if ( notFilteredVotes_[i].modality3D )
            {
                chosenVote_.average		= votingHist_[notFilteredVotes_[i].idx]->average3D;
                chosenVote_.estErr		= votingHist_[notFilteredVotes_[i].idx]->voteErr3D;
                chosenVote_.numVotes	= votingHist_[notFilteredVotes_[i].idx]->Votes.size();
                bFound = true;
            }
        }
    }

    if ( inputParams_.debug_level > OUPUT_RESULT_SUMMARY )
    {
        pcl::console::print_info("^^^^ ChosenVote: Rot: %8.3f , Err: %8.3f, Count: %d\n", 
                      chosenVote_.average, chosenVote_.estErr, chosenVote_.numVotes);
    }

    // append the voting result to the perf data descriptor list

    perfData & pd = pGlobalPerf_->pd[DESC_MDv];

    pd.descID = DESC_MDv;
    pd.actualRotation = inputParams_.rot_deg;
    pd.computedRotation = chosenVote_.average;
    pd.rotEstError	=  chosenVote_.estErr;
}

void CDescriptorVoteProcessing::computeStats()
{
    for ( unsigned i = 0; i < votingHist_.size(); i++ )
    {
        votingHist_[i]->computeVoteElements();

        if ( inputParams_.debug_level > OUPUT_RESULT_SUMMARY )
        {
            pcl::console::print_info("Vote List: %d, avg: %8.3f, ",i,votingHist_[i]->average);
            pcl::console::print_info("Vote Count: %d, err: %8.3f, desIDs:  ", 
                                      votingHist_[i]->Votes.size(), votingHist_[i]->voteErr);

            for ( unsigned j = 0; j < votingHist_[i]->Votes.size(); j++ )
            {
                pcl::console::print_info(" %d ",votingHist_[i]->Votes[j]->descID);
            }

            pcl::console::print_info("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
        }
    }
    
    // compute the voting result
    computeVotingResults();
}

//## for matlab graphics, not tested after code re-write
void CDescriptorVoteProcessing::logFinalResults()
{

    char buf[1024];

    //log to file: Estimated Rotation, Estimated Error	

    sprintf(buf,"%8.3f,%8.3f,",
                chosenVote_.average, chosenVote_.estErr);
    (*logResults_) << buf;
}
