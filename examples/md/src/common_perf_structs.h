#ifndef _COMMON_PERF_STRUCTS_
#define _COMMON_PERF_STRUCTS_

/*
 *	common data structures needed for the performance evaluation
 *
 */

typedef struct perfData
{
    unsigned	descID;

    // perf analysis
    float		actualRotation;
    float		computedRotation;
    float		rotEstError;

    float		averageDistance;
    unsigned	        inlierCount;
    float		inlierRate;
    float		confidenceFactor;

    double		processingTime;

    perfData()
    {
        descID             = DESC_INVALID;
        actualRotation     = -1000.0;	// some large number
        computedRotation   = 0.0;
        rotEstError        = 1000.0;	// some large number
        averageDistance    = 0.0;
        inlierCount        = 0;
        inlierRate         = 0.0;
        confidenceFactor   = 0.0;

        processingTime     = 10000.0;   // some large number
    }

} perfData;

/*
 *	A vector containing perf data for all evaluations
 *
 */

typedef struct outputPerf
{
    vector<perfData>	pd;
    unsigned		count;

    outputPerf()
    {
        pd.resize(TOTAL_DESC_COUNT); 
        count = 0;
    }

}ouputPerf;

/*
 *	Final voting Results
 *
 */

typedef struct votingResult
{
    float	 average;
    float	 estErr;
    unsigned     numVotes;

    votingResult()
    {
        average = 0;
        estErr  = 0;
        numVotes= 0;
    }

} votingResult;

#endif
