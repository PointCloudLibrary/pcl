#include "common_structs.h"
#include "utilityFuncs3D.h"

pcPtr
spinPointCloud(pcPtr & sourcePc, float deg = 0)
{
    pcl::PointCloud<PointT>::Ptr pTransformedCloud (new pc);
    Eigen::Matrix4f tform; 
    tform.setIdentity ();

    if ( deg == 0 )
    {
        pTransformedCloud = sourcePc;
        return pTransformedCloud;
    }

    const float& ax = 0; 
    const float& ay = 0; 
    const float& az = 1; 
    const float& theta = pcl::deg2rad(deg);	

    tform.topLeftCorner (3, 3) = Eigen::Matrix3f (Eigen::AngleAxisf (theta, Eigen::Vector3f (ax, ay, az)));
  
    pcl::transformPointCloud( *sourcePc, *pTransformedCloud, tform);

    return pTransformedCloud;
}

pcPtr downSampleCloud(pcPtr & sourcePc, float leafSize)
{
    pcPtr downSampledCloud(new pc);

    if ( leafSize == 0.0 )
    {
        downSampledCloud = sourcePc;
    }
    else
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(sourcePc);
        vg.setLeafSize(leafSize, leafSize, leafSize);
        vg.setDownsampleAllData(true);
        vg.filter(*downSampledCloud);
    }

    return downSampledCloud;
}

void computeRANSACRegistrationModel(ransacRegParams & in)
{
     pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> rejector;

     rejector.setInputSource(in.pModel);
     rejector.setInputTarget(in.pQuery);
     rejector.setInlierThreshold(in.inlierThreshold); 
     rejector.setMaximumIterations(in.maxIterations); 

     Eigen::Matrix4f trf;

     rejector.setInputCorrespondences(in.corr2D);
     rejector.getCorrespondences(in.corr2DOut);
     trf = rejector.getBestTransformation();

     Eigen::Matrix3f rotation = trf.block<3,3>(0, 0);
     Eigen::Vector3f translation = trf.block<3,1>(0, 3);

     // init perf struture
     perfData & pd = in.pd; 

     if ( in.inputParams.live_sensor ) // assumes horizontal rotation
     {
         pd.computedRotation = pcl::rad2deg(atan(rotation (2,0)/rotation(0,0)));  
         // if live_sensor, perfData_[i].actualRotation is ignored.
     }
     else // simulated rotation
     {
         pd.computedRotation = pcl::rad2deg(atan(rotation (1,0)/rotation(0,0)));
     }

     pd.rotEstError = abs(abs(pd.computedRotation) - abs(in.pd.actualRotation));

     pd.inlierCount = in.corr2DOut.size();	
     pd.inlierRate  = (float)in.corr2DOut.size()/in.corr2D->size();	

     if ( in.inputParams.debug_level > OUTPUT_TRANSFORMATIONS )
     {
         pcl::console::print_info ("----------------------------------------------------------------------------\n");
         pcl::console::print_info ("3D DescID: %d, Computed rotation: %8.3f, actual rotation: %8.3f\n",
                                    pd.descID,pd.computedRotation,pd.actualRotation);
         pcl::console::print_info ("inlierCount: %d, inlierRate: %8.3f\n",
                                    pd.inlierCount, pd.inlierRate );

         pcl::console::print_info ("\n");
         pcl::console::print_info ("    | %8.3f %8.3f %8.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
         pcl::console::print_info ("R = | %8.3f %8.3f %8.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
         pcl::console::print_info ("    | %8.3f %8.3f %8.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
         pcl::console::print_info ("\n");
         pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
      }
}

