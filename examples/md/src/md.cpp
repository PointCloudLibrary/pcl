#include "common_structs.h"	

#include "CTCDriver.h"

void RunCTCDriver(inputArgs & inputParams)
{
        CTCDriver	perfObject(inputParams);

        perfObject.init2D();

        perfObject.init2DFeatureObject(MDO_KP2D_SIFT,DESC_SIFT); 
        perfObject.init2DFeatureObject(MDO_KP2D_SURF,DESC_SURF); 
        perfObject.init2DFeatureObject(MDO_KP2D_BRISK,DESC_ORB); 
        perfObject.init2DFeatureObject(MDO_KP2D_ORB,DESC_BRISK); 

        perfObject.init3DFeatureObject(); 

        perfObject.computeAll(); 
        perfObject.initiateVoteProcessing();
}


int main(int argc, char ** argv)
{
    if ( argc < 16 )
    {
        pcl::console::print_info("Usage: %s	inputFile1.pcd inputFile2.pcd\n",argv[0]);
        pcl::console::print_info("argv[3] float	    max_rotation: max simulated rotation in degrees\n");
        pcl::console::print_info("argv[4] float	    rotation_resolution: increments in degrees \n");
        pcl::console::print_info("argv[5] unsigned	thread_count [0: auto-configure (HW depdendent), 1: single threaded, 2: 2 threads, >2: up to 5 threads]\n");

        pcl::console::print_info("argv[6]  unsigned best_k_sorted_2D: 0: no sorting(use full set), > 8 to enable sorting\n");
        pcl::console::print_info("argv[7]  unsigned best_k_sorted_3D: 0: no sorting(use full set), > 8 to enable sorting	\n");
        pcl::console::print_info("argv[8]  float	leaf_size: pcl leaf size for downsampling\n");
        pcl::console::print_info("argv[9]  float	vote_threshold: best values [0.3-0.8]\n");
        pcl::console::print_info("argv[10] float	ransac_inlier_threshold_2D: best values [0.3-0.5]\n");
        pcl::console::print_info("argv[11] float	ransac_inlier_threshold_3D: best values [0.008-0.1]\n");
        pcl::console::print_info("argv[12] bool		enable_graphics [0/1]\n");
        pcl::console::print_info("argv[13] bool		debug_level	[0: no console output, 1: result summary output, 2: 1+transformations, 3:Detailed output]\n");
        pcl::console::print_info("argv[14] bool		live_sensor  [0: simulate rotations, 1: Horizontal rotation of the sensor by the user\n");
        pcl::console::print_info("argv[15] string	logFile1.txt \n");
        pcl::console::print_info("argv[16] string	logFile2.txt \n");
                            
        return -1;
    }

    ofstream logFile1(argv[15]);	// logFile1, for matlab graphics
    ofstream logFile2(argv[16]);	// logFile2, for matlab graphics

    pcPtr pModelCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcPtr pQueryCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile(argv[1],*pModelCloud); // input file1: organized cloud
    pcl::io::loadPCDFile(argv[2],*pQueryCloud); // input file2: organized cloud

    if ( !pModelCloud || !pQueryCloud )
    {
        pcl::console::print_info ("Failed to load clouds, exiting\n");
        throw(-1);
    }	
    else
    {
        pcl::console::print_info("Loaded clouds: size: %d, size: %d\n",pModelCloud->size(),pQueryCloud->size());
    }
    
    inputArgs	inputParams(pModelCloud, pQueryCloud, &logFile1, &logFile2);

    float		max_rotation	= atof(argv[3]);
    float		rotation_res	= atof(argv[4]);
    unsigned	thread_count		= atoi(argv[5]);

    inputParams.best_k_sorted_2D = atoi(argv[6]);
    inputParams.best_k_sorted_3D = atoi(argv[7]);
    inputParams.leaf_size	 = atof(argv[8]);
    inputParams.vote_threshold   = atof(argv[9]);
    inputParams.ransac_inlier_threshold_2D = atof(argv[10]);
    inputParams.ransac_inlier_threshold_3D = atof(argv[11]);
    inputParams.b_enable_graphics = atoi(argv[12]);	//  0: disabled, 1: enabled
    inputParams.debug_level	= atoi(argv[13]);	// -1: runtime perf data, 0: none, 
                                                        //  1: result summary only, 2: 1+computed transforms,  >2: 1+2+details
    inputParams.live_sensor	= atoi(argv[14]);	//  0: use built-in rotation, 1: use 2 images obtained from a horizontal sensor rotation.

    if ( thread_count == 0 )
    {
        inputParams.thread_count = boost::thread::hardware_concurrency();
    }
    else
    {
        inputParams.thread_count = thread_count;
    }

    pcl::console::print_info("================================================\n");
    pcl::console::print_info ("Configured Parameters: \n");
    pcl::console::print_info ("max_rotation: %8.3f\n",max_rotation);
    pcl::console::print_info ("rotation_res: %8.3f\n",rotation_res);
    pcl::console::print_info ("thread_count: %d\n",inputParams.thread_count);

    pcl::console::print_info ("best_k_sorted_2D: %d\n",inputParams.best_k_sorted_2D);
    pcl::console::print_info ("best_k_sorted_3D: %d\n",inputParams.best_k_sorted_3D);
    pcl::console::print_info ("leaf_size: %4.2f\n",inputParams.leaf_size);
    pcl::console::print_info ("vote_threshold: %4.2f\n",inputParams.vote_threshold);
    pcl::console::print_info ("ransac_inlier_threshold_2D: %8.3f\n",inputParams.ransac_inlier_threshold_2D);
    pcl::console::print_info ("ransac_inlier_threshold_3D: %8.3f\n",inputParams.ransac_inlier_threshold_3D);
    pcl::console::print_info ("enable_graphics: %d\n",inputParams.b_enable_graphics);
    pcl::console::print_info ("debug_level: %d\n",inputParams.debug_level);
    pcl::console::print_info ("live_sensor: %d\n",inputParams.live_sensor);
    pcl::console::print_info("================================================\n");

    for ( float i = 0.0; i < max_rotation; i = i + rotation_res )
    {
        inputParams.rot_deg = i;

        RunCTCDriver(inputParams);		
    }

    logFile1.close();
    logFile2.close();
}
