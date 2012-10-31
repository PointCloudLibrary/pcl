
#include "pcl_demo.h"
#include "myFunctions.h"

#define  LOG_TAG    "KiwiViewer"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

extern  std::string  file1, file2;
extern timespec time1, time2;
extern void ShowPCLtoKiwi(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int r,int g, int b, float x_transl);

/**  Creates point cloud with random points, save it on sdcard, shows result on the screen */
void loadPCLPointCloud(){
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    // Fill in the cloud data
    cloud->width  = 10050;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    LOGI ("WriteFileStart");
    pcl::io::savePCDFileASCII ("/sdcard/others/outputKiwi.pcd", *cloud);
    LOGI ("WriteFile");

ShowPCLtoKiwi(cloud,255,0,0,0);

}

/** Creates point cloud with random points, 
    transforms created point cloud with known translate matrix, 
    adds noise to transformed point cloud,
    estimates rotation and translation matrix. output matrix to ANDROID LOG */
void simplePclRegistration()
{
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the CloudIn data
  cloud_in->width    = 5;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
/*  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"  << std::endl;
    for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
      cloud_in->points[i].z << std::endl;
  */

  *cloud_out = *cloud_in;

//  std::cout << "size:" << cloud_out->points.size() << std::endl;
 
 for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {  cloud_out->points[i].x = cloud_in->points[i].x + 0.7f+ 0.01f*rand()/(RAND_MAX + 1.0f);
        cloud_out->points[i].y = cloud_in->points[i].y + 0.2f;
  }

// std::cout << "Transformed " << cloud_in->points.size () << " data points:"  << std::endl;
 /* for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " << cloud_out->points[i].x << " " <<    cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
*/
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
//  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//  icp.getFitnessScore() << std::endl;
//  std::cout << icp.getFinalTransformation() << std::endl;

string outputstr;
ostringstream out_message;
out_message << icp.getFinalTransformation();
outputstr=out_message.str();
LOGI("%s", outputstr.c_str());
}


/** classic ICP using PCL Library*/
void ICP_PCL()
{
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file1, *cloud1) == -1) //* load the file
  {
   LOGI("Couldn't read file1");
    return;
  }
  LOGI("Loaded %d data points from file1",cloud1->width * cloud1->height);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file2, *cloud2) == -1) //* load the file 
  {
     LOGI("Couldn't read file2");
    return;
  }
 LOGI("Loaded %d data points from file2",cloud2->width * cloud2->height);

 clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

	MyReg* mr=new MyReg();
	PointCloud::Ptr src (new PointCloud(mr->width_ds,mr->height_ds));
	PointCloud::Ptr tgt (new PointCloud(mr->width_ds,mr->height_ds));
	LOGI("Start Downsampling...");
	mr->DownSampling(cloud1,cloud2,src,tgt);

	for (size_t i = 0; i < src->points.size (); ++i){
	   if(isnan(src->points[i].x)) {src->points[i].x=0;src->points[i].y=0;src->points[i].z=0;}
	}

	for (size_t i = 0; i < tgt->points.size (); ++i){
	   if(isnan(tgt->points[i].x)) {tgt->points[i].x=0;tgt->points[i].y=0;tgt->points[i].z=0;}
	}

	pcl::PointCloud<pcl::PointXYZ> Final;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(src);
	icp.setInputTarget(tgt);
	LOGI("Start Aligning...");

	icp.align(Final);

	string outputstr;
	ostringstream out_message;
	out_message << icp.getFinalTransformation();
	outputstr=out_message.str();
	LOGI("Final Transform: \n %s", outputstr.c_str());

	Eigen::Matrix4f GlobalTransf=icp.getFinalTransformation();
	pcl::PointCloud<pcl::PointXYZ>::Ptr transf (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud (*src, *transf,  GlobalTransf);

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
	LOGI("Time of registration: :%d:%d",diff(time1,time2).tv_sec,diff(time1,time2).tv_nsec);

ShowPCLtoKiwi(transf,255,0,0,-1.5);
ShowPCLtoKiwi(tgt,0,255,0,-1.5);

ShowPCLtoKiwi(src,255,0,0,1.5);
ShowPCLtoKiwi(tgt,0,255,0,1.5);

}


/** registration two clouds using ICP with projective correspondence */
void ICP()
{
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);


  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file1, *cloud1) == -1) //* load the file
  {
    LOGI("Couldn't read file1");
    return;
  }
  LOGI("Loaded %d data points from file2",cloud1->width * cloud1->height);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file2, *cloud2) == -1) //* load the file
  {
    LOGI("Couldn't read file2");
    return;
  }
  LOGI("Loaded %d data points from file2",cloud1->width * cloud1->height);


	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
	
	///////////////////ICP - REGISTRATION ////////////////
	MyReg* mr=new MyReg();
	PointCloud::Ptr src (new PointCloud(mr->width_ds,mr->height_ds));
	PointCloud::Ptr tgt (new PointCloud(mr->width_ds,mr->height_ds));
	mr->DownSampling(cloud1,cloud2,src,tgt);
        LOGI("Start Downsampling...");
	pcl::PointCloud<PointNormalT>::Ptr points_with_normals_src (new pcl::PointCloud<PointNormalT>);
	pcl::PointCloud<PointNormalT>::Ptr points_with_normals_tgt (new pcl::PointCloud<PointNormalT>);
	LOGI("Start Normals estimation...");	
	mr->Normals(points_with_normals_src,points_with_normals_tgt,src,tgt);
	
	LOGI("Start Matrix estimation...");
	Eigen::Matrix4f GlobalTransf;
	mr->MatrixEstimation(points_with_normals_src, points_with_normals_tgt, GlobalTransf);

		string outputstr;
		ostringstream out_message;
		out_message << GlobalTransf;
		outputstr=out_message.str();
		LOGI("%s", outputstr.c_str());

	PointCloud::Ptr transf (new PointCloud);
    // 	pcl::transformPointCloud (*cloud1, *transf,  GlobalTransf);
	pcl::transformPointCloud (*src, *transf,  GlobalTransf);

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
	LOGI("Time of registration: :%d:%d",diff(time1,time2).tv_sec,diff(time1,time2).tv_nsec);
    //  LOGI("Transform");
    //	std::cout<< GlobalTransf <<endl;
    //  LOGI("point-size: %d", transf->points.size());


ShowPCLtoKiwi(src,255,0,0,-1.5);
ShowPCLtoKiwi(tgt,0,255,0,-1.5);

ShowPCLtoKiwi(transf,255,0,0,1.5);
ShowPCLtoKiwi(tgt,0,255,0,1.5);

}


void ICP( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 )
{
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
	///////////////////ICP - REGISTRATION ////////////////
	 	

 LOGI("Start ICP...%d,%d,%d,%d",cloud1->width,cloud1->height,cloud2->width,cloud2->height);

	MyReg* mr=new MyReg();
	PointCloud::Ptr src (new PointCloud(mr->width_ds,mr->height_ds));
	PointCloud::Ptr tgt (new PointCloud(mr->width_ds,mr->height_ds));
	LOGI("Start Downsampling...");	
	mr->DownSampling(cloud1,cloud2,src,tgt);
      
	pcl::PointCloud<PointNormalT>::Ptr points_with_normals_src (new pcl::PointCloud<PointNormalT>);
	pcl::PointCloud<PointNormalT>::Ptr points_with_normals_tgt (new pcl::PointCloud<PointNormalT>);
	LOGI("Start Normals estimation...");	
	mr->Normals(points_with_normals_src,points_with_normals_tgt,src,tgt);
	
	LOGI("Start Matrix estimation...");
	Eigen::Matrix4f GlobalTransf;
	mr->MatrixEstimation(points_with_normals_src, points_with_normals_tgt, GlobalTransf);

		string outputstr;
		ostringstream out_message;
		out_message << GlobalTransf;
		outputstr=out_message.str();
		LOGI("%s", outputstr.c_str());

	PointCloud::Ptr transf (new PointCloud);
    // 	pcl::transformPointCloud (*cloud1, *transf,  GlobalTransf);
	pcl::transformPointCloud (*src, *transf,  GlobalTransf);

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
	LOGI("Time of registration: :%d:%d",diff(time1,time2).tv_sec,diff(time1,time2).tv_nsec);
    //  LOGI("Transform");
    //	std::cout<< GlobalTransf <<endl;
    //  LOGI("point-size: %d", transf->points.size());


/*ShowPCLtoKiwi(cloud1,255,0,0,-1.5);
ShowPCLtoKiwi(src,0,255,0,-1.6);
//ShowPCLtoKiwi(cloud2,0,255,0,1.5);
*/

ShowPCLtoKiwi(src,255,0,0,-1.5);
ShowPCLtoKiwi(tgt,0,255,0,-1.5);

ShowPCLtoKiwi(transf,255,0,0,1.5);
ShowPCLtoKiwi(tgt,0,255,0,1.5);


}



void CreatePCLFileFromKinect(short * buf_int, int frame_width, int frame_height, int maxZ) 
{
//////TODO implement correct create PCL file from kinect
/*
frame_width=frame_width;
frame_height=frame_height;

int height_ds=48*4;
int width_ds=64*4;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    // Fill in the cloud data
    cloud->width  = height_ds*width_ds;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

for(int  i=0;i<height_ds;i++)
        {
            for(int j=0;j<width_ds;j++)
            {
                int iold=i*frame_height/height_ds;
                int jold=j*frame_width/width_ds;

		short depth = *(buf_int+iold*frame_width+jold);
	//	LOGI("width%d, height%d, depth%d",iold, jold,(short) depth);

		cloud->points[i*width_ds+j].x=(short)((j-frame_width/2)*depth/525);
		cloud->points[i*width_ds+j].y=(short)(-(i-frame_height/2)*depth/525);	
		cloud->points[i*width_ds+j].z=(short)(-depth);
						//(-depth*255/maxZ);//(-depth*depth*255/maxZ/maxZ);               
	if (depth==0) {
 		cloud->points[i*width_ds+j].x=(short)0;
		cloud->points[i*width_ds+j].y=(short)0;
		cloud->points[i*width_ds+j].z=(short)0;
		}

             }
        }
    LOGI ("WriteFileStart");
    pcl::io::savePCDFileASCII ("/sdcard/others/outputOpenNI.pcd", *cloud);
    LOGI ("WriteFile");
*/
}



 
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
 
//pcl::PointCloud<pcl::PointXYZ> cloud_1;
//pcl::PointCloud<pcl::PointXYZ> cloud_2;
int cloud_status=0;


void CreatePointCloud(short * buf_int, int frame_width, int frame_height, int maxZ) 
{ 
//if(cloud_status==10) { return;}
frame_width=frame_width;
frame_height=frame_height;

float scale=1.0f;
int height_ds=frame_height*scale;
int width_ds=frame_width*scale;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

if(cloud_status==0) 
{
  cloud=cloud_1;
 LOGI ("Save First Cloud");
 
}
else
{
 pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
 cloud_2=tmp;
 cloud=cloud_2;
 LOGI ("Save new Cloud");

}
    
    // Fill in the cloud data
    cloud->width = width_ds;
    cloud->height = height_ds;
    cloud->points.resize (cloud->width * cloud->height);

for(int  i=0;i<height_ds;i++)
        {
            for(int j=0;j<width_ds;j++)
            {
                int iold=i*frame_height/height_ds;
                int jold=j*frame_width/width_ds;

		short depth = *(buf_int+iold*frame_width+jold);

		cloud->points[i*width_ds+j].x=((j-width_ds/2)*depth/(525*scale))/1000.f;  
		cloud->points[i*width_ds+j].y=((i-height_ds/2)*depth/(525*scale))/1000.f;
		cloud->points[i*width_ds+j].z=(depth)/1000.f;
		if (depth==0) {
	 		cloud->points[i*width_ds+j].x=(short)0;
			cloud->points[i*width_ds+j].y=(short)0;
			cloud->points[i*width_ds+j].z=(short)0;
		}
		//else  {LOGI("i=%d,j=%d,depth=%d,x=%f y=%f z=%f",i,j,depth,cloud->points[i*width_ds+j].x,cloud->points[i*width_ds+j].y,cloud->points[i*width_ds+j].z);	}

             }
        }
   
 LOGI ("DATA LOADED");

//// Execute ICP for cloud_1 cloud_2
if(cloud_status==0) 
{
cloud_status++;
return;
}

cloud_status++;


ICP(cloud_1,cloud_2);
//usleep(1000000);

 LOGI ("ICP");
cloud_1=cloud_2;
 LOGI ("cloud1=cloud2");

}


