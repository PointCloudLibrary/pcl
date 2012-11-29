#include "FingerTracker.h"

#define SHOWIMAGE 1
FingerTracker::FingerTracker(){

}
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
	hasSeeds = 0;

	//tracking only 1 hand (history for the hand tracking)
	for(int i=0; i<HISTORY_TRACK_SIZE; i++){
		x_track_his[i]=-1;
		y_track_his[i]=-1;
		z_track_his[i]=-1;
	}

	//a list of images for motion estimation (especially for push and
	//pull operations
	this->his_frame_buf=(IplImage**)malloc(sizeof(IplImage*)*NUM_HIS_FRAME);
	for(int i=0; i<NUM_HIS_FRAME; i++){
		this->his_frame_buf[i] = cvCreateImage(cvSize(w/DOWN_SAMPLE_SIZE, h/DOWN_SAMPLE_SIZE), IPL_DEPTH_16U, 1);
	}
	this->tmp_depth = cvCreateImage(cvSize(w/DOWN_SAMPLE_SIZE, h/DOWN_SAMPLE_SIZE), IPL_DEPTH_16U, 1);
	this->pushmask = cvCreateImage(cvSize(w/DOWN_SAMPLE_SIZE, h/DOWN_SAMPLE_SIZE), IPL_DEPTH_8U, 1);
	this->tmpmask = cvCreateImage(cvSize(w/DOWN_SAMPLE_SIZE, h/DOWN_SAMPLE_SIZE), IPL_DEPTH_8U, 1);
	this->handmask = cvCreateImage(cvSize(w/DOWN_SAMPLE_SIZE, h/DOWN_SAMPLE_SIZE), IPL_DEPTH_8U, 1);
	this->handcheckmask = cvCreateImage(cvSize(w/DOWN_SAMPLE_SIZE, h/DOWN_SAMPLE_SIZE), IPL_DEPTH_8U, 1);

	//kalman filter
	KF = cv::KalmanFilter(2, 1, 0);
    state = cv::Mat(2, 1, CV_32F); /* (phi, delta_phi) */
    processNoise = cv::Mat(2, 1, CV_32F);
    measurement = cv::Mat::zeros(1, 1, CV_32F);
    KF.transitionMatrix = *(cv::Mat_<float>(2, 2) << 1, 1, 0, 1);

    cv::setIdentity(KF.measurementMatrix);
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));

    cv::randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
}
void FingerTracker::resetSeed(){
	hasSeeds = 0;
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
void FingerTracker::updateHistory(int x, int y, int z){
	//tracking only 1 hand (history for the hand tracking)
	for(int  i=HISTORY_TRACK_SIZE-1; i>0; i--){
		x_track_his[i]=x_track_his[i-1];
		y_track_his[i]=y_track_his[i-1];
		z_track_his[i]=z_track_his[i-1];
	}
	x_track_his[0]=x;
	y_track_his[0]=y;
	z_track_his[0]=z;
//	for(int i=0; i<HISTORY_TRACK_SIZE; i++){
//		printf("%d, %d, %d |", x_track_his[i], y_track_his[i], z_track_his[i]);
//	}
//	printf("\n");
}
void FingerTracker::getPosition(int  *x, int  *y, int *z){
	*x = x_track_his[0];
	*y = y_track_his[0];
	*z = z_track_his[0];
}

//generate a mask that shows the slope of the frames
//by performing linear regression on the position time (buf frames)

//NOTE: step size of the function can affect the slope
//i.e., a 15 fps camera shall halfen the slope! in compare to the 30fps.

void FingerTracker::pushMask(int threshold){
	int xAvg = 0;
    int yAvg = 0;
    int v2 = 0;

    //this will remove noisy pixels and retain the pixels that are blob like.
    //we need a relatively large kernel size to remove the unwanted pixels.
//    cvErode(pushmask,tmpmask,NULL,8/DOWN_SAMPLE_SIZE);
//    cvDilate(tmpmask,pushmask,NULL,8/DOWN_SAMPLE_SIZE);

    //precompute
    for(int x=0; x<NUM_HIS_FRAME; x++){
    	//linear regression!
        xAvg += x;
    }
    xAvg = xAvg / NUM_HIS_FRAME;
    for(int x=0; x<NUM_HIS_FRAME; x++){
    	v2 += (x - xAvg) * (x - xAvg);
    }

    unsigned char *push_val = (unsigned char*)pushmask->imageData;
   	//for every pixel in all stacks
    for(int i=0; i<his_frame_buf[0]->width*his_frame_buf[0]->height;i++){
        int v1 = 0;
    	//we first compute the average of y, depth
    	for (int x = 0; x < NUM_HIS_FRAME; x++)
    	{
    		unsigned short *val = (unsigned short*)his_frame_buf[x]->imageData;
    		yAvg += *(val+i) ;
    	}
    	yAvg = yAvg / NUM_HIS_FRAME;

    	for (int x = 0; x < NUM_HIS_FRAME; x++)
    	{
    		unsigned short *val = (unsigned short*)his_frame_buf[x]->imageData;
    		v1 += (x - xAvg) * (*(val+i) - yAvg);
    	}
    	if(v1>threshold && v1<threshold*2)
    		*(push_val+i) = 255.0;
    	else
    		*(push_val+i) = 0;
    	//printf("v1 is %lf", v1);
    }
    cvErode(pushmask,tmpmask,NULL,8/DOWN_SAMPLE_SIZE);
    cvDilate(tmpmask,pushmask,NULL,8/DOWN_SAMPLE_SIZE);

    //show the results
#ifdef SHOWIMAGE
    cvShowImage("Push Mask", pushmask);
#endif
}

/**
 * Two rect matches if and only if the origin of the rectangles are within some thresholds
 * The threshold is set to the size of the rectangle
 */
bool matchRect(CvRect *a, CvRect *b){
	if(a == NULL)
		return true;
	if(b == NULL)
		return true;

	//later
	return true;
}
/**
 * Find the region that has the push event
 * TODO: this algorithm is critical for the algorithm to run correctly. i.e., it initialize the
 * seed that will be used for the region growing, and a bad seed is going to hurt the performance of the
 * algorithm significantly.
 *
 * Updates: all x_his_track and y_his_track to that point for the first time.
 */

int FingerTracker::getSeeds(){
	int x, y, z;
	//extract the push point from the mask
	CvSeq* contours = 0;
	cvFindContours( pushmask, g_storage, &contours);
	cvClearMemStorage(g_storage);
	cvClearMemStorage(hand_storage);
    cvClearMemStorage(hullStorage);

    static int stable_counts = 0;
    CvRect rect;
    static CvRect prev_rect;

    //add to the seed if and only if we have it stable
    //(*i.e., occurred over a few frames).
    if(contours){
    	cvDrawContours(tmpmask, contours, cvScalar(255), cvScalar(128), 1, 2, 8);
    	CvSeq *contour_list = contours;
    	int count=0;
    	while(contour_list){
			rect = cvBoundingRect(contour_list, 1 );
			cvRectangle( tmpmask, cvPoint(rect.x, rect.y + rect.height),
					cvPoint(rect.x + rect.width, rect.y), cvScalar(128), 3, 8, 0 );
			count++;
			contour_list=contour_list->h_next;
    	}
    	if(count==1 && matchRect(&rect, &prev_rect)){
    		if(rect.width*rect.height > 800/DOWN_SAMPLE_SIZE){ //ignore small blocks
    			prev_rect = rect;
    			stable_counts++;
    		}
    	}else{
    		stable_counts = 0;
    	}
    }
#ifdef SHOWIMAGE
    cvShowImage("contour of hands", tmpmask);
#endif

    //we only take the position if and only if it is stable
    if(stable_counts>2){
    	IplImage *current_frame = his_frame_buf[0];
    	x = rect.x+rect.width/2;
    	y = rect.y+rect.height/2;

    	//get the average depth
    	unsigned short *frame_ptr = (unsigned short*)current_frame->imageData;
    	int widthStep = current_frame->widthStep;
    	int depth = 0;
    	int my_count=0;
    	int size = current_frame->width*current_frame->height;
		for(int u=-2;u<=2;u++){
			for(int v=-2;v<=2;v++){
				int cx = x+u;
				int cy = y+v;
				int i = (cx) + (cy)*current_frame->width;
				if(i<0 || i>=size)
					continue;
				if(frame_ptr[i]>0){
					depth += frame_ptr[i];
					my_count++;
				}
			}
		}
		if(my_count == 0 || depth == 0 ){
			stable_counts = 0;
			return 0; //something wesubsnt wrong
		}
		depth=depth/my_count;

		//set the history to the initial point
		for(int i=0; i<HISTORY_TRACK_SIZE; i++){
			x_track_his[i]=x;
			y_track_his[i]=y;
			z_track_his[i]=depth;
		}
		printf("Initialized Seed to %d %d %d\n", x, y, depth);
		stable_counts = 0;
		return 1;
    }
	return 0;
}

//need low pass filter next.

/**
 * Shall handle the boundary case outside of this. This makes an assumption that
 * the x+1, x-1, y-1, z-1 are within the bound
 */
int FingerTracker::connect(int x, int y, int z, int direction, int delta){
	IplImage *current_frame = his_frame_buf[0];
	unsigned char *handmask_ptr = (unsigned char*)handmask->imageData;
	unsigned char *handcheck_ptr = (unsigned char*)handcheckmask->imageData;
	unsigned short *current_frame_ptr = (unsigned short*)current_frame->imageData;
	int widthStep = current_frame->widthStep;


	int my_z = current_frame_ptr[(x+0)+(y+0)*widthStep];
	int z0 = current_frame_ptr[(x+1)+(y+0)*widthStep];
	int z1 = current_frame_ptr[(x+1)+(y+1)*widthStep];
	int z2 = current_frame_ptr[(x+0)+(y-1)*widthStep];
	int z3 = current_frame_ptr[(x-1)+(y-1)*widthStep];
	int z4 = current_frame_ptr[(x-1)+(y+0)*widthStep];
	int z5 = current_frame_ptr[(x-1)+(y+1)*widthStep];
	int z6 = current_frame_ptr[(x+0)+(y+1)*widthStep];
	int z7 = current_frame_ptr[(x+1)+(y+1)*widthStep];

	//my neighbours

	//reconstruct the x,y position based on the z is usually required for better
	//accurracy, now we only consider the depth differences.



	return direction;
}

/**
 * Generate a lookup table for the spiral lookup, i.e., how the
 * array shall be walked. box_size must be an ODD number for this to behave normally
 * right now we only go clockwise direction
 */
int **FingerTracker::spiralLookup(int box_size){
	int **spiral_lookup = (int**)malloc(box_size*box_size*sizeof(int*));
	int **checkmap = (int **)malloc(box_size*sizeof(int*));
	for(int i=0; i < box_size*box_size; i++){
		spiral_lookup[i] = (int*)malloc(2*sizeof(int));
		spiral_lookup[i][0] = 0;
		spiral_lookup[i][1] = 0;
	}
	for(int i=0; i<box_size; i++){
		checkmap[i] = (int *)malloc(box_size*sizeof(int));
		memset(checkmap[i], 0, box_size*sizeof(int));
	}

	int direction = 0; //up right down left
	int x_pos = box_size/2;
	int y_pos = box_size/2;
	int x=0;
	int y=0;
	checkmap[x_pos+x][y_pos+y] = 1;

	//now let's do the walk in a spiral fashion and record the x and y position to the spiral look up table
	for(int i=0; i<box_size*box_size; i++){
		checkmap[x_pos+x][y_pos+y] = 1;
		spiral_lookup[i][0]=x;
		spiral_lookup[i][1]=y;
		//printf("(%d, %d)", x, y);
		if(direction == 0){
			y=y-1;
			//change direction to right
			if(checkmap[x_pos+x+1][y_pos+y]==0){
				direction = 1;
			}
		}else if (direction == 1){
			x=x+1;
			//change direction to down
			if(checkmap[x_pos+x][y_pos+y+1]==0){
				direction = 2;
			}
		}else if (direction == 2){
			y=y+1;
			//change direction to down
			if(checkmap[x_pos+x-1][y_pos+y]==0){
				direction = 3;
			}
		}else if (direction == 3){
			x=x-1;
			//change direction to up
			if(checkmap[x_pos+x][y_pos+y-1]==0){
				direction = 0;
			}
		}
//		printf("\n");
//
//		for(int v=0; v<box_size; v++){
//			for(int u=0; u<box_size; u++){
//				printf("%d ", checkmap[u][v]);
//			}
//			printf("\n");
//		}
//		printf("============================================");

	}
	free(checkmap);
	return spiral_lookup;
}
/**
 * This will perform the kalman filter's prediction step
 * We shall initialize the kalman filter before we run this function
 */
void FingerTracker::predictHistoryTrack(int *results){
//	//CREATE THE KALMAN FILTER FOR TRACKING 3D MOTION OF A POINT
//	// state is (x,y,z,x',y',z') where "'" = derivative (velocity)
//	// measurement observation is x,y,z
//	CvKalman* Kalman = cvCreateKalman(6, 3); //system has 6 dynamic params, 6 measurement
//	CvMat Phi = cvMat(6, 6, CV_MAT32F, Kalman->DynamMatr); //row,col
//	CvMat H = cvMat(3, 6, CV_MAT32F, Kalman->MeasurementMatr);
//	CvMat Q = cvMat(6, 6, CV_MAT32F, Kalman->PNCovariance);
//	CvMat R = cvMat(3, 3, CV_MAT3x3_32F, Kalman->MNCovariance);
//	CvMat Pminus = cvMat(6, 6, CV_MAT32F, Kalman->PriorErrorCovariance);
//	CvMat P = cvMat(6, 6, CV_MAT32F, Kalman->PosterErrorCovariance);
//	CvMat xminus = cvMat(6, 1, CV_MAT32F, Kalman->PriorState); //note vects are column
//	float Measurement[3];
//	CvMat z = cvMat(3, 1, CV_MAT3x1_32F, Measurement); //vects are column oriented
//	cvmSetIdentity(&Phi); //dynamics
//	cvmSet(&Phi, 0, 3, 1.0);
//	cvmSet(&Phi, 1, 4, 1.0);
//	cvmSet(&Phi, 2, 5, 1.0);
//	cvmSetZero(&H);
//	//Measure to State matrix
//	cvmSet(&H, 0, 0, 1.0);
//	cvmSet(&H, 1, 1, 1.0);
//	cvmSet(&H, 2, 2, 1.0);
//	cvmSetZero(&Q);
//	//Prediction error covariance
//	cvmSet(&Q, 0, 0, 0.1);
//	// cvmSet(&Q,1,1,0.1);
//	cvmSet(&Q, 2, 2, 0.1);
//	cvmSet(&Q, 3, 3, 0.2);
//	cvmSet(&Q, 4, 4, 0.2);
//	cvmSet(&Q, 5, 5, 0.2);
//	cvmSetZero(&R);
//	//Measurement noise cov
//	cvmSet(&R, 0, 0, 0);
//	cvmSet(&R, 1, 1, 0);
//	cvmSet(&R, 2, 2, 0);
//	cvmSetIdentity(&Pminus);
//	//Estimation error covariance
//	cvmScale(&Pminus, &Pminus, 20.0);
//	//Just make it large compared to what it normally can be
//	cvmSetIdentity(&P); //Posterior error covariance
//
//	//READ AND TRACK DATA
//
//	cvReleaseKalman(&Kalman);


	//simple prediction based on the last few history (very bad idea)
	int v1 = x_track_his[0]-x_track_his[1];
	if(v1 > SPEED_LIMIT)
		v1=SPEED_LIMIT;
	if(v1 < -SPEED_LIMIT)
			v1=-SPEED_LIMIT;
	predictHand[0]=x_track_his[0]+v1;

	v1 = y_track_his[0]-y_track_his[1];
	if(v1 > SPEED_LIMIT)
		v1=SPEED_LIMIT;
	if(v1 < -SPEED_LIMIT)
			v1=-SPEED_LIMIT;
	predictHand[1]=y_track_his[0]+v1;


	v1 = z_track_his[0]-z_track_his[1];
	if(v1 > SPEED_LIMIT)
		v1=SPEED_LIMIT;
	if(v1 < -SPEED_LIMIT)
			v1=-SPEED_LIMIT;
	predictHand[2]=z_track_his[0]+v1;

	if(predictHand[0] > handmask->width)
		predictHand[0] = handmask->width - 3; //border
	if(predictHand[1] > handmask->height)
			predictHand[1] = handmask->height - 3; //border

	if(predictHand[0] < 3)
		predictHand[0] = 3;
	if(predictHand[1] < 3)
			predictHand[1] = 3;

	//max range for the depth sensor
	if(predictHand[2]<500){
		predictHand[2] = 500;
	}
	if(predictHand[2]>10000){
		predictHand[2] = 10000;
	}
	//need kalman filter or we will run into problem with noisy data
}


/**
 * This will extract location of the center of the blob
 * from the segmented image of the hand in depth map
 * Any ideas:? TODO: what shall we use to define a HAND? curvature? or? ...
 */
void FingerTracker::locateTrackPoint(int *result, int box_size){
	//use the bounding box for speed up
	int m_x = predictHand[0];
	int m_y = predictHand[1];
	int m_z = predictHand[2];
	int box_width = box_size/2;

	IplImage *current_frame = his_frame_buf[0];
	//cvCopy(current_frame, tmp_depth, 0);
	unsigned short *frame_ptr = (unsigned short *)tmp_depth->imageData;
	unsigned short *original_frame_ptr = (unsigned short *)current_frame->imageData;
	unsigned char *handmask_ptr = (unsigned char *)handmask->imageData;
	unsigned char *tmpmask_ptr = (unsigned char *)tmpmask->imageData;
	int widthStep = handmask->widthStep;
	int widthStep_short = tmp_depth->widthStep;
	int im_w = handmask->width;
	int im_h = handmask->height;

	//cvLaplace(handmask, tmpmask);
	IplImage *image_box = cvCreateImage(cvSize(box_width, box_width),IPL_DEPTH_16U, 1);
	cvZero(tmp_depth);
	int min_val=255*255;
	int max_val=0;
	for(int y=-box_width+1; y<box_width-1; y++){
		for(int x=-box_width+1; x<box_width-1; x++){
			int i = (m_x+x)+(y+m_y)*widthStep;
			int value = *(original_frame_ptr+i);
			if(value < min_val){
				min_val = value;
			}
			if(value > max_val){
				max_val = value;
			}
		}
	}
	//invert the image for locating the centroid
	for(int y=-box_width+1; y<box_width-1; y++){
		for(int x=-box_width+1; x<box_width-1; x++){
			int i = (m_x+x)+(y+m_y)*widthStep;

			if(i >= im_w*im_w || i<0)
				continue;

			int value = *(original_frame_ptr+i);
			int mask = *(handmask_ptr+i);
			//unsigned char laplace_mask = *(tmpmask_ptr+i);
			if(value > 0 && mask > 0){
				int my_val = 0;
				if(mask == 1){
					//my_val = 255*255; //bad idea (but should use it to compute the curvature or so)
				}
				else{
					my_val = max_val - value;
				}
				*(frame_ptr+i) = my_val;
			}
			else{
				*(frame_ptr+i) = 0;
			}
		}
	}
    //label the seed as checked

	//shall use the derivatives image or something with the curvature information
	//it is now going around like crazy.

	//increase the weight of the edges.


#ifdef SHOWIMAGE
	//cvShowImage("locate Track point 1", handmask);
	cvShowImage("locate Track point 2", tmp_depth);
	//cvShowImage("locate Track point 3", tmpmask);
#endif

	//use cvMoments to find the centre of gravity! need to refine this
	//if the tracking fails to locate the region of interest.

	//we use use ROI here to minimize the RUNTIME overheads
	
	
//	int left_x = m_x - box_width;
//	int left_y = m_y - box_width;
	
//	if(left_x < 0)
//		left_x = 0;
//	if(left_y < 0)
//		left_y = 0;


	//CvRect rect = cvRect(left_x, left_y, box_size, box_size);
	
	//cvSetImageROI(tmp_depth, rect);
	CvMoments moments;
	cvMoments(tmp_depth, &moments);
	//cvResetImageROI(tmp_depth);
	
	double m00, m10, m01;

	//can get better performance if we use only integer here (SLOW)
	m00 = cvGetSpatialMoment(&moments, 0,0);
	m10 = cvGetSpatialMoment(&moments, 1,0);
	m01 = cvGetSpatialMoment(&moments, 0,1);

	// TBD check that m00 != 0
	if(m00==0){
		//printf("Something is wrong with m00\n");
		return;
	}
	int center_x = m10/m00;
	int center_y = m01/m00;
	//printf("%lf, %lf\n",center_x, center_y);
	
	int avg_z = 0;
	//this will be used for the next part of the tracking.
	result[0]=center_x;
	result[1]=center_y;
	int x = center_x;
	int y = center_y;
	int my_count=0;
	int depth=0;
	int size = current_frame->width*current_frame->height;
	for(int u=-4;u<=4;u++){
		for(int v=-4;v<=4;v++){
			int cx = x+u;
			int cy = y+v;
			int i = (x+u) + (y+v)*widthStep;
			if(i < 0 || i >= size)
				continue;
			if(original_frame_ptr[i]>0){
				int val = original_frame_ptr[i];
				if((val-m_z)*(val-m_z) < 100*100){
					depth += val;
					my_count++;
				}
			}
		}
	}
	if(depth>0){
		result[2]=depth/my_count;
		updateHistory(result[0], result[1], result[2]);
	}

	cvReleaseImage(&image_box);
}
/*
 * delta define the maximum distance between the points before we consider it as a connected set
 * box_size defines the maximum size the bounding box can be, a larger bounding box may be used if
 * the subject is closer to the camera (TODO)
 * The seed is critical for these techniques and must be chosen very carefully!
 */
int FingerTracker::growHand(int delta, int depth_bound, int box_size){
	if(box_size % 2==0)
		box_size+=1;
	//expand the mask based on the initial seed point
	int x = predictHand[0];
	int y = predictHand[1];
	int z = predictHand[2];

	//RULES:
	//up, right, down, left. 0, 1, 2, 3
	//we have the right hand rule, check if the right neighbor is checked, if not
	//we change the direction to right, else go up again.
	//this will ensure that it will go in a spiral fashion!
	//i.e.,
	//
	// 0 0 0
	// 0 1 0	direction: up
	// 0 0 0
	//
	// 0 1 0
	// 0 1 0 --> check right, if not checked, then change direction
	// 0 0 0 (i.e., turn to right by changing direction right)
	//
	// 0 1 1
	// 0 1 1 --> check down (which is +1 of the direction) it not checked then change direction
	// 0 0 0
	//
	// etc...

	//use a lookup table for the spiral lookup! with different box size
	static	int **spiral_lookup = spiralLookup(box_size);

	//clear the old history mask
	cvZero(handmask);
	cvZero(handcheckmask);
	cvZero(tmp_depth);
	//grow by minimizing the distance
	//search in all direction and stops when we hit the boundary or the maximum bounding box

	int im_w = handmask->width;
	int im_h = handmask->height;
	int widthStep = handmask->widthStep;

	//the smooth function will remove some of the speckle noise from the depth sensor
	IplImage *current_frame = his_frame_buf[0];
	//cvCopy(current_frame, tmp_depth, 0);
    //cvDilate(tmp_depth,tmp_depth,NULL,2);
    //cvShowImage("Depth Blurred",tmp_depth);

	unsigned short *frame_ptr = (unsigned short *)current_frame->imageData;
	unsigned char *handmask_ptr = (unsigned char *)handmask->imageData;

	//label the seed as checked
	for(int u=-1;u<=1;u++){
		for(int v=-1;v<=1;v++){
			if(x+u < 1 || x+u > im_w-1 || y+v < 1 || y+v > im_h-1){
				continue;
			}else{
				handmask_ptr[(x+u)+(y+v)*widthStep]=255;
			}
		}
	}

	//this is critical for connecting the hand and so. it is a BUG for now
	//need to figure out how to extract the proper depth
	int ref_z=frame_ptr[x+y*widthStep];
//
//	int has_matches = 0;
	if(ref_z == 0){
		for(int u=-5;u<=5;u++){
			for(int v=-5;v<=5;v++){
				if(x+u < 1 || x+u > im_w-1 || y+v < 1 || y+v > im_h-1){
					continue;
				}else{
					int f = frame_ptr[(x+u)+(y+v)*widthStep];
					if(f>0 && (ref_z - z)*(ref_z - z) < depth_bound){
						ref_z = f;
						x = x+u;
						y = y+u;
						break;
						//handmask_ptr[(x+u)+(y+v)*widthStep]=255;
					}
				}
			}
		}
	}
//	//the region is off the tracking box, we need to restart or wait for the object to come back
//	if(!has_matches)
//		return 1;
	//printf("z %d vs ref_z: %d\n", z, ref_z);

	int size = 0;
	int has_connection = 0;
	/**
	 * NOTE: The size of the bounding box can be dynamic
	 */
	for(int i=0; i<box_size*box_size-1; i++){
		//check if the current point is connected to the neighbors
		//TODO:

		//use the spiral_lookup for the position
		int pos_x=spiral_lookup[i][0];
		int pos_y=spiral_lookup[i][1];

		int c_x = (x + pos_x);
		int c_y = (y + pos_y);

		//check out of bound (including the boarders - 1 pixels)
		//ignore if it is out of frame
		if(c_x < 1 || c_x > im_w-1 || c_y < 1 || c_y > im_h-1){
			continue;
		}

		int c_z = frame_ptr[c_x + c_y*widthStep];

		//check if any of the points are withint the estimated bound
		if((ref_z-c_z)*(ref_z-c_z)<depth_bound*depth_bound){
			has_connection++;
		}
		//debug
//		if(i%box_size==0){
//			printf("\n");
//		}
//		printf("(%d, %d, %d) ", c_x, c_y, c_z);
		//compare the new pos with the ref
		//if good then add to the mask


		//connect the points to the set
		//RULE: the distance between the points is less than `delta`, but
		//contains at least 1 votes to be considered as a connect set (i.e., 2

		//neighbour points
		//basically the sum of the distance error < delta from the reference point
		//too all connected dots are < delta
		//then we consider that as a connected point
		//at least 1 point connection is required

		//can hardcode the conditions for performance (SLOW)
		int vote = 0;
		for(int u=-1;u<=1;u++){
			for(int v=-1;v<=1;v++){
				if(u==0 && v==0)
					continue;
				int f = frame_ptr[(c_x+u) + (c_y+v)*widthStep];
				int f1= f - c_z;
				int z1=handmask_ptr[(c_x+u) + (c_y+v)*widthStep];
				//if the distance from the test point is too far, reject
				if((f-ref_z)*(f-ref_z) > depth_bound*depth_bound)
					continue;
				//if the distance is within the delta and also within the set
				//we add a vote
				if(f1*f1*z1 < delta*delta*255 && z1 > 0){
					vote++;
				}
			}
		}

		//label the point as a part of the set, i.e., we have more than 1 set nearby
		if(vote==1){
			handmask_ptr[c_x + c_y*widthStep] = 1;
			size++;
		}
		else if(vote>1){
			handmask_ptr[c_x + c_y*widthStep] = 32;
			size++;
		}
		//now for each of the point, check for the neighbours and connect the points as we go
	}
#ifdef SHOWIMAGE
	cvShowImage("Hand Mask", handmask);
#endif
	if(size<box_size*box_size*0.2 || has_connection == 0)
		return 1;

	return 0;
}
/**
 * This will compute the area for tracking and the location of the
 * tracker.
 * updates: x_his_track, y_his_track, the size of the area
 */
//void FingerTracker::meanShift(){
//
//}

void FingerTracker::blobTracking(unsigned short *raw_depth){
	int box_size = 80;
	int depth_bound = 100;
	int link = 20;

	//save the history with a circular buffer
	//the oldest image will be replaced by the new one
	cvSetData(depth_map, raw_depth, depth_map->widthStep);
	//cvSetData(rgb_image, rgb_buffer, rgb_image->widthStep);

	IplImage *buf_tmp;
	buf_tmp = his_frame_buf[NUM_HIS_FRAME-1]; //oldest frame, zero is the newest

	//shifting to the right, and now the first buffer will be free
	//i.e., 0 1 2 3 4 5 will be x 0 1 2 3 4
	int i;
	for(i=(NUM_HIS_FRAME-1); i>=1; i--){
		his_frame_buf[i] = his_frame_buf[i-1];
	}

	his_frame_buf[0] = buf_tmp;
	cvResize(depth_map, his_frame_buf[0], CV_INTER_AREA); //overwrite the first buffer

	//spatial filter for noise reduction
	cvSmooth(his_frame_buf[0], tmp_depth, CV_MEDIAN, 3, 3);
	cvSmooth(tmp_depth, his_frame_buf[0], CV_GAUSSIAN, 3, 3);

	//now we have updated the frame buffers and the sequence is
	//0 1 2 3 4 5 (newest --> oldest)
	pushMask(SLOPE_THRESHOLD);
	int error = 0;
	static int fail = 0;
	//get seeds, only support 1 hand tracking for now, we can extend this
	//capability by adding multiple seeds and using multiple track history (TODO)
	if(!hasSeeds){
		hasSeeds = getSeeds(); //this will initialize all history to the seed
                if (hasSeeds)
                HandFound();
                main_found=1;	
       }else{

		/**
		 * PREDICT the SEED!
		 */
		predictHistoryTrack(predictHand);

		/**
		 * SEGEMENTATION
		 * the core of the next step is to segment out the hand from the rest of the
		 * image based on the seed (again, the seed is critical!)
		 */
		error=growHand(link, depth_bound, box_size/DOWN_SAMPLE_SIZE); //this will create a mask which has all the points connected
		//delta=40 is the minimum distance between to points to be considered as connected
		//depth_bound=300, the maximum distance we allow for connecting the set
		//100 is the bounding box size for the search, and can be reduced for better performance
		if(error){
			fail++;
			printf("Tracking failed...\n");
		}else{
			//reduce the weight of the fail if we can get back to tracking properly
			fail--;
			if(fail < 0)
				fail = 0;

			//extract the exact location of the new blob, and update the tracking
			//history
			int xyz[3];
			//now extract the orientation for the hand, to refine our tracking
			//update the tracker history (0th element is the most recent)!
			//shall we filter the result?! another critical design issue!
			//also there is a problem of shifting. we need to address that
			//provide a ROI for performance!!
			CvRect rect = cvRect(10, 20, 50, 60);
			locateTrackPoint(xyz, box_size);
		}
		for(int i=0; i<HISTORY_TRACK_SIZE-1; i++){
			CvPoint pt1 = cvPoint(x_track_his[i]*DOWN_SAMPLE_SIZE,y_track_his[i]*DOWN_SAMPLE_SIZE);
			CvPoint pt2 = cvPoint(x_track_his[i+1]*DOWN_SAMPLE_SIZE,y_track_his[i+1]*DOWN_SAMPLE_SIZE);
			//cvCircle(depth_map,pt1,(HISTORY_TRACK_SIZE-i)*2,cvScalar(255*255),3,8);
			cvLine(depth_map,pt1,pt2,cvScalar(255*128),2,8);
		}
		CvPoint pt2 = cvPoint(x_track_his[i+1]*DOWN_SAMPLE_SIZE,y_track_his[i+1]*DOWN_SAMPLE_SIZE);
		cvCircle(depth_map,pt2,(HISTORY_TRACK_SIZE),cvScalar(255*128),3,8);
		//TODO:

		//if we lose track, we will take out the seed by initialize all history to -1
		//*usually when the area for tracking become too small or the tracker was stationary for
		//3-4 seconds (preset these). (i.e., the tracker was locked onto the background)

		if(fail > 10){
			printf("Tracking Failed... Resetting\n");
			hasSeeds = 0;
			fail = 0;
                        main_fail=1;
                        HandLost();
		}
		//TODO:
	}
}

void FingerTracker::showHistory(){
	char buf[512];
	for(int i=0; i<NUM_HIS_FRAME; i++){
		sprintf(buf, "Frame Buf %d", i);
		cvShowImage(buf, his_frame_buf[i]);
	}
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
	for(int i=0; i<NUM_HIS_FRAME; i++){
		cvReleaseImage(&his_frame_buf[i]);
	}
	cvReleaseImage(&tmp_depth);
	cvReleaseImage(&pushmask);
	cvReleaseImage(&tmpmask);
	cvReleaseImage(&handmask);
	cvReleaseImage(&handcheckmask);
}
