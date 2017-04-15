#ifndef _CDESCRIPTOR_VOTE_HELPER_
#define _CDESCRIPTOR_VOTE_HELPER_

#define MAX_2D_DESC_ID	 3

/*
 * helper structure for md voting evaluation
 *
 */
typedef struct notFilteredVotes
{
    unsigned     idx;
    unsigned     voteCount;
    float	 topComputedRotation;
    bool	 modality2D;
    bool	 modality3D;
    bool	 modality2D3D;

    notFilteredVotes()
    {
        idx = -1;
        voteCount = -1;
        modality2D = false;
        modality3D = false;
        modality2D3D = false;
    }

} notFilteredVotes;


struct votingObject
{
    votingObject()
    {
        average		= 0;
        average2D	= 0;
        average3D	= 0;

        voteCount	= 0;
        voteCount2D     = 0;
        voteCount3D     = 0;

        voteErr		= 0;
        voteErr2D	= 0;
        voteErr3D	= 0;

        modality2D	= false;
        modality3D	= false;
        modality2D3D	= false;
    }

    void setElement(perfData & pd)
    {
        perfData *	pVote(new perfData);

        if ( !pVote ) { printf("mem failed\n"); throw (-1); }

        pVote->descID			= pd.descID;
        pVote->actualRotation	= pd.actualRotation;
        pVote->computedRotation = pd.computedRotation;
        pVote->rotEstError		= pd.rotEstError;

        Votes.push_back(pVote);
    }

    void computeVoteElements()
    {
        if ( !Votes.size() )	// don't compute if no votes
             return;

        // compute the average per modality
        float sum = 0;	float sum2D = 0;  float sum3D   = 0;

        for ( unsigned i = 0; i < Votes.size(); i++ )
        {
            sum = sum + Votes[i]->computedRotation;

            if ( Votes[i]->descID <= MAX_2D_DESC_ID )
            {
                voteCount2D++;
                sum2D = sum2D + Votes[i]->computedRotation;
                modality2D = Votes[i]->descID <= MAX_2D_DESC_ID ? true : false;
            }

            if ( Votes[i]->descID > MAX_2D_DESC_ID  )
            {
                voteCount3D++;
                sum3D = sum3D + Votes[i]->computedRotation;
                modality3D = Votes[i]->descID > MAX_2D_DESC_ID ? true : false;
            }
        }
        
        modality2D3D = modality2D && modality3D;
        voteCount = Votes.size();

        average = sum / voteCount;

        if ( voteCount2D )
        {
            average2D = sum2D / voteCount2D;
        }
        if ( voteCount3D ) 
        {
            average3D = sum3D / voteCount3D;
        }

        voteErr	  = abs(abs(average)   - abs(Votes[0]->actualRotation));	// actualRotation is same for all	
        voteErr2D = abs(abs(average2D) - abs(Votes[0]->actualRotation));	// actualRotation is same for all	
        voteErr3D = abs(abs(average3D) - abs(Votes[0]->actualRotation));	// actualRotation is same for all	
    }

    float			average;
    float			average2D;
    float			average3D;
    
    float			voteErr;
    float			voteErr2D;
    float			voteErr3D;

    unsigned			voteCount;
    unsigned			voteCount2D;
    unsigned			voteCount3D;

    bool			modality2D;
    bool			modality3D;
    bool			modality2D3D;
    vector<perfData *>	Votes;
};

#endif
