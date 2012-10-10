#include "PCLWrapper.h"

//Contructor
PCLWrapper::PCLWrapper() {
//	//allocate memory or init other things here
//	this->init(IMAGE_WIDTH, IMAGE_HEIGHT);
	NE10_init();
}
//Destructor
PCLWrapper::~PCLWrapper() {
	//free memory here
}
void PCLWrapper::init(int width, int height) {
//	//initialize the size of these point data
//	input->width = width;
//	input->height = height;
//	input->is_dense = false;
//	input->points.resize(input->width * input->height);
//
//	filtered->width = width;
//	filtered->height = height;
//	filtered->is_dense = false;
//	filtered->points.resize(filtered->width * filtered->height);
}
void PCLWrapper::setData(unsigned short *depth_map) {
//	for (int x = 0; x < IMAGE_WIDTH; x++) {
//		for (int y = 0; y < IMAGE_HEIGHT; y++) {
//			int i = x + y * IMAGE_WIDTH;
//			input->points[i].x = x;
//			input->points[i].y = y;
//			input->points[i].z = *depth_map;
//			depth_map++;
//		}
//	}
}
void PCLWrapper::benchmark_raw(){
	int count = 0;
	//generate a set of random points
	int width = 640;
	int height = 480;
	unsigned short *test1 = (unsigned short *)malloc(width*height*sizeof(unsigned short)); //depth
	unsigned char *test2 = (unsigned char *)malloc(width*height*sizeof(unsigned char)*3); //rgb

	//load the input points with some random numbers
	//worst case for the image compression
	for (size_t i = 0; i < width*height*3; ++i) {
		*(test2+i)=rand();
	}
	for (size_t i = 0; i < width*height; ++i) {
		*(test1+i)=rand();
	}
	static int counter=0;
	int iterations = 100;
	int average = 20;
	char buf[512];
	double total = 0;
	while (iterations > 0){
		struct timeval start, end;
		double t1, t2;
		static double elapsed_sec = 0;
		const int MAX_COUNT = average; //average 10 results before printing
		gettimeofday(&start, NULL);

		//do some work here.
		char my_path[512];
		sprintf(my_path, "/mnt/sdcard2/pcl/out_rgb_%06d.raw", counter);
		writeImageRAW_RGB(my_path, width, height, 3, test2, 0);
		//sprintf(my_path, "/data/ni/out_depth_%06d.png", counter);
		//writeImageRAW(my_path, width, height, test1);

		counter++;
		gettimeofday(&end, NULL);
		t1 = start.tv_sec + (start.tv_usec / 1000000.0);
		t2 = end.tv_sec + (end.tv_usec / 1000000.0);
		elapsed_sec += (t2 - t1);
		total +=(t2 - t1);
		count++;
		if (count >= MAX_COUNT) {
			sprintf(buf, "Number of Points: %d, Total Runtime: %f of %d runs , Average: %f (s)\n",
					width*height, elapsed_sec, MAX_COUNT, (elapsed_sec) / MAX_COUNT);
			elapsed_sec = 0;
			count = 0;
			__android_log_write(ANDROID_LOG_INFO, "PCL Benchmark:", buf);
		}
		iterations--;
	}

	sprintf(buf, "Total Runtime: %lf (s)\n", total);
	__android_log_write(ANDROID_LOG_INFO, "PCL Benchmark:", buf);

	free(test1);
	free(test2);
}

void PCLWrapper::benchmark_png(){
	int count = 0;
	//generate a set of random points
	int width = 640;
	int height = 480;
	unsigned short *test1 = (unsigned short *)malloc(width*height*sizeof(unsigned short)); //depth
	unsigned char *test2 = (unsigned char *)malloc(width*height*sizeof(unsigned char)*3); //rgb

	//load the input points with some random numbers
	//worst case for the image compression
	for (size_t i = 0; i < width*height*3; ++i) {
		*(test2+i)=rand();
	}
	for (size_t i = 0; i < width*height; ++i) {
		*(test1+i)=rand();
	}
	int counter=0;
	int iterations = 50;
	int average = 25;
	while (iterations > 0){
		struct timeval start, end;
		double t1, t2;
		static double elapsed_sec = 0;
		const int MAX_COUNT = average; //average 10 results before printing
		gettimeofday(&start, NULL);

		//do some work here.
		char my_path[512];
		sprintf(my_path, "/data/ni/out_rgb_%06d.png", counter);
		writeImageRGB(my_path, width, height, test2, my_path);
		sprintf(my_path, "/data/ni/out_depth_%06d.png", counter);
		writeImageDepth(my_path, width, height, test1, my_path);

		gettimeofday(&end, NULL);
		t1 = start.tv_sec + (start.tv_usec / 1000000.0);
		t2 = end.tv_sec + (end.tv_usec / 1000000.0);
		elapsed_sec += (t2 - t1);
		count++;
		counter++;
		if (count >= MAX_COUNT) {
			char buf[512];
			sprintf(buf, "Number of Points: %d, Runtime: %f (s)\n", width*height, (elapsed_sec) / MAX_COUNT);
			elapsed_sec = 0;
			count = 0;
			__android_log_write(ANDROID_LOG_INFO, "PCL Benchmark:", buf);
		}
		iterations--;
	}
	free(test1);
	free(test2);
}

//this will be our benchmark testing
void PCLWrapper::benchmark(int iterations, int average){
	int count = 0;
	//generate a set of random points
	int width = 100;
	int height = 1000;
	unsigned short *test1 = (unsigned short *)malloc(width*height*sizeof(unsigned short)); //depth
	unsigned char *test2 = (unsigned char *)malloc(width*height*sizeof(unsigned char)*3); //rgb
	float *test3 = (float*)malloc(width*height*sizeof(float)*3); //xyz

	//begin our testing.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = width;
	cloud->height = height;
	cloud->points.resize(cloud->width * cloud->height);

	//results
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_filtered->width = width;
	cloud_filtered->height = height;
	cloud_filtered->points.resize(cloud_filtered->width * cloud_filtered->height);

	//unsigned short version
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_xyzrgb->width = width;
	cloud_xyzrgb->height = height;
	cloud_xyzrgb->points.resize(cloud_xyzrgb->width * cloud_xyzrgb->height);

	//load the input points with some random numbers
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	while (iterations > 0){
		struct timeval start, end;
		double t1, t2;
		static double elapsed_sec = 0;
		const int MAX_COUNT = average; //average 10 results before printing
		gettimeofday(&start, NULL);

		//do some work here.
		// Create the filtering object
		pcl::PassThrough < pcl::PointXYZ > pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 1.0);
		//pass.setFilterLimitsNegative (true);
		pass.filter(*cloud_filtered);

		gettimeofday(&end, NULL);
		t1 = start.tv_sec + (start.tv_usec / 1000000.0);
		t2 = end.tv_sec + (end.tv_usec / 1000000.0);
		elapsed_sec += (t2 - t1);
		count++;
		if (count >= MAX_COUNT) {
			char buf[512];
			sprintf(buf, "Number of Points: %d, Runtime: %f (s)\n", width*height, (elapsed_sec) / MAX_COUNT);
			elapsed_sec = 0;
			count = 0;
			__android_log_write(ANDROID_LOG_INFO, "PCL Benchmark:", buf);
		}
		iterations--;
	}
	free(test1);
	free(test2);
	free(test3);
}
int PCLWrapper::voxel_filter(unsigned short *depth_data, float *point_cloud_data){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data
	cloud->width = IMAGE_WIDTH;
	cloud->height = IMAGE_HEIGHT;
	cloud->points.resize(cloud->width * cloud->height);

	//copy the data...? slow but for testing now.
	int i=0;

	const float fx_d = 1.0/5.5879981950414015e+02;
	const float fy_d = 1.0/5.5874227168094478e+02;
	const float cx_d = 3.1844162327317980e+02;
	const float cy_d = 2.4574257294583529e+02;
	unsigned short *my_depth_data = depth_data;
	float *my_point_cloud_data = point_cloud_data;
	double sum=0;
	for(int y=0; y<IMAGE_HEIGHT; y++){
		for(int x=0; x<IMAGE_WIDTH; x++){
			//copy the data to the point cloud struc object.
			unsigned short d_val = *my_depth_data;
		    float my_z = d_val*0.001f;
		    sum=sum+my_z;
		    float my_x = my_z * (x-cx_d) * fx_d;
		    float my_y = my_z * (y-cy_d) * fy_d;
		    cloud->points[i].x = my_x;
		    cloud->points[i].y = my_y;
		    cloud->points[i].z = my_z;
		    my_depth_data++;
			i++;
		}
	}
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered);

//	pcl::PassThrough < pcl::PointXYZ > pass;
//	pass.setInputCloud(cloud);
//	pass.setFilterFieldName("z");
//	pass.setFilterLimits(0.0, 3.0);
//	//pass.setFilterLimitsNegative (true);
//	pass.filter(*cloud_filtered);

	size_t new_size = cloud_filtered->width * cloud_filtered->height;
	char buf[1024];
	sprintf(buf, "Original: %d, Filtered: %d, Ratio: %f, Sum %f \n", IMAGE_WIDTH*IMAGE_HEIGHT, new_size, ((float)new_size)/(IMAGE_WIDTH*IMAGE_HEIGHT), sum);
	__android_log_write(ANDROID_LOG_INFO, "PCL FILTER TESTING:", buf);

	//save the results to the pointer
	for(int i=0; i<cloud_filtered->width*cloud_filtered->height;i++){
		float x = cloud_filtered->points[i].x;
		float y = cloud_filtered->points[i].y;
		float z = cloud_filtered->points[i].z;

		*my_point_cloud_data=x;
		*(my_point_cloud_data+1)=y;
		*(my_point_cloud_data+2)=z;
		my_point_cloud_data+=3;
//		sprintf(buf, "x: %f y: %f z:%f \n", x,y,z);
//		__android_log_write(ANDROID_LOG_INFO, "PCL DATA:", buf);
	}
	i =0;
//	for (float x=-5; x<5; x+=0.5){
//		for (float y=-10; y<10; y+=0.5){
//			*(my_point_cloud_data+0)=x;
//			*(my_point_cloud_data+1)=y;
//			*(my_point_cloud_data+2)=5;
//			my_point_cloud_data+=3;
//			i++;
//		}
//	}
	return new_size+i;
}

int PCLWrapper::test_registration(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
			new pcl::PointCloud<pcl::PointXYZ>);
	char buf[1024];
	sprintf(buf, "Testing Registration\n");
	__android_log_write(ANDROID_LOG_INFO, "PCL FILTER TESTING:", buf);

	// Fill in the CloudIn data
	cloud_in->width = 5;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);
	for (size_t i = 0; i < cloud_in->points.size(); ++i) {
		cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	std::cout << "Saved " << cloud_in->points.size() << " data points to input:"
			<< std::endl;

	for (size_t i = 0; i < cloud_in->points.size(); ++i)
		std::cout << "    " << cloud_in->points[i].x << " "
				<< cloud_in->points[i].y << " " << cloud_in->points[i].z
				<< std::endl;

	*cloud_out = *cloud_in;

	std::cout << "size:" << cloud_out->points.size() << std::endl;

	for (size_t i = 0; i < cloud_in->points.size(); ++i)
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;

	std::cout << "Transformed " << cloud_in->points.size() << " data points:"
			<< std::endl;
	sprintf(buf, "Transformed %d\n", cloud_in->points.size());
	__android_log_write(ANDROID_LOG_INFO, "PCL FILTER TESTING:", buf);

	for (size_t i = 0; i < cloud_out->points.size(); ++i)
		std::cout << "    " << cloud_out->points[i].x << " "
				<< cloud_out->points[i].y << " " << cloud_out->points[i].z
				<< std::endl;

	pcl::IterativeClosestPoint < pcl::PointXYZ, pcl::PointXYZ > icp;
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud < pcl::PointXYZ > Final;
	icp.align(Final);

	std::cout << "has converged:" << icp.hasConverged() << " score: "
			<< icp.getFitnessScore() << std::endl;

	sprintf(buf, "has converged: %d, score: %lf\n ", icp.hasConverged(), icp.getFitnessScore());
	__android_log_write(ANDROID_LOG_INFO, "PCL FILTER TESTING:", buf);

	std::cout << icp.getFinalTransformation() << std::endl;

//	sprintf(buf, "Final Transformation: %s\n ", icp.getFinalTransformation());
//	__android_log_write(ANDROID_LOG_INFO, "PCL FILTER TESTING:", buf);
}

int PCLWrapper::filter(unsigned short *depth_data, float *point_cloud_data){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data
	cloud->width = IMAGE_WIDTH;
	cloud->height = IMAGE_HEIGHT;
	cloud->points.resize(cloud->width * cloud->height);

	//copy the data...? slow but for testing now.
	int i=0;
	const float fx_d = 1.0/5.5879981950414015e+02;
	const float fy_d = 1.0/5.5874227168094478e+02;
	const float cx_d = 3.1844162327317980e+02;
	const float cy_d = 2.4574257294583529e+02;
	unsigned short *my_depth_data = depth_data;
	float *my_point_cloud_data = point_cloud_data;
	double sum=0;
	for(int y=0; y<IMAGE_HEIGHT; y++){
		for(int x=0; x<IMAGE_WIDTH; x++){
			//copy the data to the point cloud struc object.
			unsigned short d_val = *my_depth_data;
		    float my_z = d_val*0.001f;
		    sum=sum+my_z;
		    float my_x = my_z * (x-cx_d) * fx_d;
		    float my_y = my_z * (y-cy_d) * fy_d;
		    cloud->points[i].x = my_x;
		    cloud->points[i].y = my_y;
		    cloud->points[i].z = my_z;
		    my_depth_data++;
			i++;
		}
	}
	// Create the filtering object
	pcl::PassThrough < pcl::PointXYZ > pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 100.0);
//	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_filtered);

	size_t new_size = cloud_filtered->width * cloud_filtered->height;
//	char buf[1024];
//	sprintf(buf, "Original: %d, Filtered: %d, Ratio: %f, Sum %f \n", IMAGE_WIDTH*IMAGE_HEIGHT, new_size, ((float)new_size)/(IMAGE_WIDTH*IMAGE_HEIGHT), sum);
//	__android_log_write(ANDROID_LOG_INFO, "PCL FILTER TESTING:", buf);

	//save the results to the pointer
	for(int i=0; i<cloud_filtered->width*cloud_filtered->height;i++){
		float x = cloud_filtered->points[i].x;
		float y = cloud_filtered->points[i].y;
		float z = cloud_filtered->points[i].z;

		*my_point_cloud_data=x;
		*(my_point_cloud_data+1)=y;
		*(my_point_cloud_data+2)=z;
		my_point_cloud_data+=3;
	}
	return new_size;
}

