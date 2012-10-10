#include "FingerTracker.h"

FingerTracker::FingerTracker(int w, int h)
{
	this->width = w;
	this->height = h;
	this->depth_map = cvCreateImage(cvSize(w, h), IPL_DEPTH_16U, 1);
	this->depth_map8 = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
	this->rgb_image = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);
	this->g_storage = cvCreateMemStorage(0);
	this->hand_storage = cvCreateMemStorage(0);
	this->hullStorage = cvCreateMemStorage(0);
	this->grasp=0;
	x = 0;
	y = 0;
}

void FingerTracker::myThreshold(IplImage *depth_raw, IplImage *depth_binary, int min, int max){
	unsigned short *depth_raw_ptr = (unsigned short*)depth_raw->imageData;
	unsigned char *depth_binary_ptr = (unsigned char*)depth_binary->imageData;
	for(int i = this->width*this->height; --i >= 0;){
		unsigned short depth_value = *depth_raw_ptr;
		if(depth_value < max && depth_value>min)
			*depth_binary_ptr = 255;
		else
			*depth_binary_ptr = 0;
		depth_binary_ptr++;
		depth_raw_ptr++;
	}
}
int FingerTracker::isGrasp(){
	return this->grasp;

}
void FingerTracker::getPosition(float *x, float *y){
	*x = this->x;
	*y = this->y;
}
void FingerTracker::runTracking(unsigned short *raw_depth, unsigned char *rgb_buffer, int min_range, int max_range){
	double AREA_THRESHOLD = 3000;
	//copy the data and get ready
	cvSetData(depth_map, raw_depth, depth_map->widthStep);
	cvSetData(rgb_image, rgb_buffer, rgb_image->widthStep);

	//create a binary image that we can use for contour finding
	//adaptive to the last hand track??
	static int last_range = 1000;
	this->myThreshold(depth_map, depth_map8, min_range+last_range, max_range+last_range);


	CvSeq* contours = 0;
	cvFindContours( depth_map8, g_storage, &contours);
	cvClearMemStorage(g_storage);
	cvClearMemStorage(hand_storage);
    cvClearMemStorage(hullStorage);
	this->grasp = 0; //reset variables

	if(contours){
		char buf[512];
		CvSeq *contours_list = contours;
		int i = 0;

		double max_area=0;
		CvSeq *max_area_hand = NULL;
		while(contours_list){
			double area = cvContourArea(contours_list);
			if(area>AREA_THRESHOLD){
				if(area>max_area){
					max_area = area;
					max_area_hand = contours_list;
				}
//			    sprintf(buf,"Area for contour %d is %lf\n", i, area);
//			    __android_log_print(ANDROID_LOG_DEBUG, "CONTOURS",  buf);
			    i++;

				//show that as real hand if it satisify some of our requirements.
			}
			contours_list=contours_list->h_next;
			//iterate through all contours (no inner contours).
		}
		if(max_area_hand != NULL){
			CvSeq *hand_contour = cvApproxPoly( max_area_hand, sizeof(CvContour), hand_storage, CV_POLY_APPROX_DP, 15.0, 1 );
			CvRect rect = cvBoundingRect(max_area_hand, 1 );

			this->grasp = 1;
			this->x = rect.x+rect.width/2.0;
			this->y = rect.y+rect.height/2.0;

			if(cvCheckContourConvexity(hand_contour)){
				cvRectangle( rgb_image, cvPoint(rect.x, rect.y + rect.height), cvPoint(rect.x + rect.width, rect.y), CV_RGB(255, 255, 0), 3, 8, 0 );
			}else{
				cvRectangle( rgb_image, cvPoint(rect.x, rect.y + rect.height), cvPoint(rect.x + rect.width, rect.y), CV_RGB(0, 0, 255), 3, 8, 0 );
			}

			//draw a mask for the contour map.
			cvZero(depth_map8);
			cvDrawContours(depth_map8, hand_contour, cvScalar(255), cvScalar(255), 1, CV_FILLED, 8);
			//cvCvtColor(depth_map8, rgb_image, CV_GRAY2BGR);

			//set the bounding box that we do the depth map average, this will adaptively follow the last tracked object.
//			cvSetImageROI(depth_map, rect);
//			cvSetImageROI(depth_map8, rect);
//			last_range=cvAvg(depth_map, depth_map8).val[0];
//			cvResetImageROI(depth_map);
//			cvResetImageROI(depth_map8);
//			if(last_range > 2000){
//				last_range = 2000;
//			}if(last_range < 800){
//				last_range = 800;
//			}

			cvDrawContours(rgb_image, hand_contour, cvScalar(0, 255, 0), cvScalar(255,255,0), 1, 2, 8);
			//cvCopy(rgb_image, rgb_image, depth_map8);
			cvSet(rgb_image, cvScalar(0, 255, 0), depth_map8);

			CvFont font;
			cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 2.0, 2.0, 0, 2, CV_AA);
			char buf_out[512];
			double area = cvContourArea(hand_contour);

			//int i=0;
			const float fx_d = 1.0/5.5879981950414015e+02;
			const float fy_d = 1.0/5.5874227168094478e+02;
			const float cx_d = 3.1844162327317980e+02;
			const float cy_d = 2.4574257294583529e+02;
//			unsigned short *my_depth_data = depth_data;
//			float *my_point_cloud_data = point_cloud_data;
//			double sum=0;
//			for(int y=0; y<IMAGE_HEIGHT; y++){
//				for(int x=0; x<IMAGE_WIDTH; x++){
//					//copy the data to the point cloud struc object.
//					unsigned short d_val = *my_depth_data;
//				    float my_z = d_val*0.001f;
//				    sum=sum+my_z;
//				    float my_x = my_z * (x-cx_d) * fx_d;
//				    float my_y = my_z * (y-cy_d) * fy_d;
//				    cloud->points[i].x = my_x;
//				    cloud->points[i].y = my_y;
//				    cloud->points[i].z = my_z;
//				    my_depth_data++;
//					i++;
//				}
//			}
			double area_x = (last_range)/1000.0 * area * fx_d;
			sprintf(buf_out, "%.03f m^2", area_x);
			cvPutText(rgb_image, buf_out, cvPoint(rect.x, rect.y + rect.height), &font, cvScalar(255, 0, 0, 0));


			CvSeq* hulls=cvConvexHull2( hand_contour, hullStorage, CV_CLOCKWISE, 1 );
			for (int i = 0; i < hulls->total; i++) {
				CvPoint *p = (CvPoint*)cvGetSeqElem ( hulls, i );
				//these are all the tips of the convex hulls
				cvCircle(rgb_image, *p, 8, cvScalar(255,0,255), 2, 8, 0);
				//TODO: use their relationship to know if we have fingers or not. (i.e., orientations)

			}
		}
	}

	//cvCircle(rgb_image, cvPoint(100, 100), 30, cvScalar(0,255,0));
}

FingerTracker::~FingerTracker(){
	cvReleaseImage(&depth_map);
	cvReleaseImage(&depth_map8);
	cvReleaseImage(&rgb_image);
}
