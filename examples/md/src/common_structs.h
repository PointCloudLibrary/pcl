#ifndef _COMMON_STRUCTS_H_
#define _COMMON_STRUCTS_H_

#include "pcl_includes.h"
#include "common_params.h"
#include "common_typedefs.h"
#include "common_perf_structs.h"

/*
 *	Input arguments and parameters that are needed throughout.
 *
 */
typedef struct inputArgs
{	
    unsigned	thread_count;
    float	rot_deg;
    bool	live_sensor;
    unsigned    min_keypoint_count;
    unsigned	min_correspondence_count;
    bool	b_enable_graphics;
    int		debug_level;


    // 2D parameters
    unsigned	        best_k_sorted_2D;
    float		ransac_inlier_threshold_2D;

    // 3D parameters
    float		leaf_size;
    float		normals_estimation_radius;

    float		descriptor_radius;
    float		max_matching_distance; // some large number, differs by desc, will be squared, not critical!
    unsigned            best_k_sorted_3D;
    float		ransac_inlier_threshold_3D;
    float		ransac_max_iterations_3D;
    
    // voting parameters
    float		vote_threshold;
    bool		vote_sorting;

    pcPtr	        pModelCloud;
    pcPtr		pQueryCloud;
    ofstream	*	logFile1;
    ofstream	*	logFile2;
    
    inputArgs(pcPtr	 pModel_Cloud,
              pcPtr      pQuery_Cloud,
              ofstream * log_file_1,
              ofstream * log_file_2)

  : pModelCloud(pModel_Cloud)
  , pQueryCloud(pQuery_Cloud)
  , logFile1(log_file_1)
  , logFile2(log_file_2)
    {
        thread_count		 = 2;  // 0: auto, 1: single thread, 2: two threads only, > 2: up to 5 threads.
        rot_deg			 = 0.0;
        live_sensor		 = false;
        min_keypoint_count	 = 10;
        min_correspondence_count = MIN_CORRESPONDENCE_COUNT;

        // 2D parameters
        best_k_sorted_2D	        = 0;  // default: no sorting
        ransac_inlier_threshold_2D	= 3.0; // not critical, too small (0.1) makes for unstable result!

        // 3D parameters
        leaf_size			= 0.04;
        normals_estimation_radius	= 0.05; // fpfh is very sensitive to this param
        best_k_sorted_3D	        = 10;

        descriptor_radius = 0.1;

        max_matching_distance	   = 1000;	// result will be sorted & resized, differs by desc, will be squared, not critical
        ransac_inlier_threshold_3D = 0.01; 
        ransac_max_iterations_3D   = 10000;

        // vote parameters
        vote_threshold		= 0.3;	
        vote_sorting		= true;

        // graphics & text output
        b_enable_graphics	= false;
        debug_level		= 0;
    };

} inputArgs;

#endif
