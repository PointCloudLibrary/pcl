#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void printMatix4f(const Eigen::Matrix4f & matrix) {

	printf ("Rotation matrix :\n");
	printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0,0), matrix (0,1), matrix (0,2));
	printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1,0), matrix (1,1), matrix (1,2));
	printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2,0), matrix (2,1), matrix (2,2));
	printf ("Translation vector :\n");
	printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0,3), matrix (1,3), matrix (2,3));
}

void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

int
main (int argc, char* argv[])
{
	// The point clouds we will be using
	PointCloudT::Ptr cloud_in 	(new PointCloudT); // Original point cloud
	PointCloudT::Ptr cloud_tr	(new PointCloudT); // Transformed point cloud
	PointCloudT::Ptr cloud_icp	(new PointCloudT); // ICP output point cloud

	// Checking program arguments
	if (argc < 2) {
		printf ("Usage :\n");
		printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
		PCL_ERROR("Provide one ply file.\n");
		return -1;
	}

	if (pcl::io::loadPLYFile (argv[1], *cloud_in) < 0)	{
		PCL_ERROR("Error loading cloud %s.\n", argv[1]);
		return -1;
	}

	int iterations = 1;
	// If the user passed the number of iteration as an argument
	if (argc > 2) {
		iterations = atoi(argv[2]);
	}

	if (iterations < 1) {
		PCL_ERROR("Number of initial iterations must be >= 1\n");
		return -1;
	}

	printf ("\nLoaded file %s with %d points successfully\n\n", argv[1], (int)cloud_in->size());

	// Defining a rotation matrix and translation vector
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

	// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	float theta = M_PI/8; // The angle of rotation in radians
	transformation_matrix (0,0) = cos(theta);
	transformation_matrix (0,1) = -sin(theta);
	transformation_matrix (1,0) = sin(theta);
	transformation_matrix (1,1) = cos(theta);

	// A translation on Z axis (0.4 meters)
	transformation_matrix (2,3) = 0.4;

	// Display in terminal the transformation matrix
	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	printMatix4f(transformation_matrix);

	// Executing the transformation
	pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
	*cloud_tr = *cloud_icp; // We backup cloud_icp into cloud_tr for later use

	// The Iterative Closest Point algorithm
	std::cout << "Initial iterations number is set to : " << iterations;
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(iterations);
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_in);
	icp.align(*cloud_icp);
	icp.setMaximumIterations(1); // For the next time we will call .align() function

	if (icp.hasConverged()) {
		printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation();
		printMatix4f(transformation_matrix);
	} else {
		PCL_ERROR ("\nICP has not converged.\n");
		return -1;
	}

	// Visualization
	pcl::visualization::PCLVisualizer viewer ("ICP demo");
	// Create two verticaly separated viewports
	int v1(0); int v2(1);
	viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

	// The color we will be using
	float bckgr_gray_level = 0.0; // Black
	float txt_gray_lvl = 1.0-bckgr_gray_level; 

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int)255* txt_gray_lvl, (int)255* txt_gray_lvl, (int)255* txt_gray_lvl);
	viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
	viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
	viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
	viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

	// Adding text descriptions in each viewport
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	std::stringstream ss; ss << iterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024); // Visualiser window size

	// Register keyboard callback :
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);

	// Display the visualiser
	while (!viewer.wasStopped ()) {
		viewer.spinOnce ();
		
		// The user pressed "space" :
		if (next_iteration) {
			icp.align(*cloud_icp);

			if (icp.hasConverged()) {
				printf("\033[11A"); // Go up 11 lines in terminal output.
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation();	// This is not very accurate !
				printMatix4f(transformation_matrix);					// Print the transformation between original pose and current pose

				ss.str (""); ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
			} else {
				PCL_ERROR ("\nICP has not converged.\n");
				return -1;
			}
		}
		next_iteration = false;
	}
 return 0;
}
