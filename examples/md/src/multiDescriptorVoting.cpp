#include "multiDescriptorVoting.h"
#include "sortingFunctors.h"

CMultiDescriptorVoting::CMultiDescriptorVoting(inputArgs & inputParams, outputPerf * pGlobalPerf)
    : pGlobalPerf_(pGlobalPerf)
    , enableSorting_(inputParams.vote_sorting)  
    , logResults_(inputParams.logFile2)
    , inputParams_(inputParams)
    , descriptorVote_(inputParams,pGlobalPerf)
{
    vector<perfData> & pd = pGlobalPerf->pd;

    for ( unsigned i = 0; i < pd.size(); i++ )
    {
        if ( pd[i].descID >= DESC_SIFT  &&  // within range of valid descriptors
             pd[i].descID < DESC_MDv    &&  // MDv not computed yet!
             pd[i].inlierCount >= MIN_CORRESPONDENCE_COUNT // above min count
           )						//## should be in inputParams!
            {
                voteList_.push_back(pd[i]);
            }
    }
}

void CMultiDescriptorVoting::computeVotes()
{

    /*	if ( enableSorting_ ) // no difference wrt results
    {
        std::sort (voteList_.begin (), voteList_.end (),
                       sortPerfDataByComputedRotation ());
    } */

    for ( unsigned i = 0; i < voteList_.size(); i++ )
    {
        descriptorVote_.vote(voteList_[i]);
    }

    descriptorVote_.computeStats();
}

void CMultiDescriptorVoting::displayVotingResults()
{
    if ( inputParams_.debug_level <= OUPUT_RESULT_SUMMARY )
    {	// do nothing
        return;
    }
    
    votingResult	ofo;
    descriptorVote_.getVotingResults(ofo);

    voteList_.push_back(pGlobalPerf_->pd[DESC_MDv]);	// add the chosenVote to the display list

    std::sort (voteList_.begin (), voteList_.end (),
                sortPerfDataByEstErr ());

    pcl::console::print_info("==============================================================================\n");
    pcl::console::print_info("DescID   ComputedRot   EstimatedError   ObjFuncEstErr   ObjFuncRot   numVotes \n");
    pcl::console::print_info("==============================================================================\n");
    for ( unsigned i = 0; i < voteList_.size(); i++ )
    {
        pcl::console::print_info("%4d	%8.3f	%8.3f	%8.3f	%8.3f  %d\n",
                voteList_[i].descID, 
                voteList_[i].computedRotation,
                voteList_[i].rotEstError, 
                ofo.estErr, ofo.average, ofo.numVotes);
    }
    pcl::console::print_info("==============================================================================\n");
}

void CMultiDescriptorVoting::displayRuntimePerformance()
{ 
    if ( inputParams_.debug_level != OUTPUT_RUNTIME_PERF )
    {	// do nothing
        return;
    }

    std::sort (voteList_.begin (), voteList_.end (),
        sortPerfDataByRuntime ());

    pcl::console::print_info("================================================\n");
    pcl::console::print_info("       Runtime Performance Summary              \n");
    pcl::console::print_info("================================================\n");
    for ( unsigned i = 0; i < DESC_MD2dR ; i++ )
    {
        pcl::console::print_info("DescID: %3d, Total Time: %8.5f\n",
                voteList_[i].descID, voteList_[i].processingTime);
    }
    pcl::console::print_info("================================================\n");
}

