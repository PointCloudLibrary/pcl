#include <android/log.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <pthread.h>
#include <math.h>
#include <unistd.h>

#include <libfreenect.h>

pthread_t freenect_thread;
volatile int die = 0;

int g_argc;
char **g_argv;

int window;

pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;

// back: owned by libfreenect (implicit for depth)
// mid: owned by callbacks, "latest frame ready"
// front: owned by GL, "currently being drawn"
uint8_t *depth_mid, *depth_front;
uint16_t *depth_true;
uint8_t *rgb_back, *rgb_mid, *rgb_front;

freenect_context *f_ctx;
freenect_device *f_dev;
int freenect_angle = 0;
int freenect_led;

freenect_video_format requested_format = FREENECT_VIDEO_RGB;
freenect_video_format current_format = FREENECT_VIDEO_RGB;

pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
int got_rgb = 0;
int got_depth = 0;

uint16_t t_gamma[2048];

int max_x = 0.0;
int max_y = 0.0;



void change_angle(double angle){
	freenect_set_tilt_degs(f_dev,angle);
}
static int got_new_frame=0;
static int got_new_frame_rgb=0;
static int got_new_frame_depth_true=0;


int new_frame(){
	pthread_mutex_lock(&gl_backbuf_mutex);
	return got_new_frame;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}
void getDepthData(uint8_t *rgb){
	//lock this and make copy of the memory
	pthread_mutex_lock(&gl_backbuf_mutex);
	if(got_new_frame){
		memcpy(rgb, depth_mid, 640*480*4*sizeof(uint8_t));
		got_new_frame=0;
	}
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}
void getDepthDataShort(uint16_t *depth_float){
	//lock this and make copy of the memory
	pthread_mutex_lock(&gl_backbuf_mutex);
	if(got_new_frame_depth_true){
		//copy to the depth_float
		memcpy(depth_float, depth_true, 640*480*sizeof(uint16_t));
		got_new_frame_depth_true=0;
	}
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
	  const float k1 = 1.1863;
	  const float k2 = 2842.5;
	  const float k3 = 0.1236;
	  const float depth = k3 * tan(depthValue/k2 + k1);
	  return depth;
    //return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}
float *depthLookUp=0;

float dot(float *x, float *y){
	return x[0]*y[0]+x[1]*y[1]+x[2]*y[2];
}

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

void getRGBDataCalibrated(uint8_t *rgb){
	//lock this and make copy of the memory
	pthread_mutex_lock(&gl_backbuf_mutex);
	if(got_new_frame_rgb){
		float *depthLookUpHere=0;
		depthLookUpHere=(float*)malloc(sizeof(float)*2048);
		for (int i = 0; i < 2048; i++) {
			depthLookUpHere[i] = rawDepthToMeters(i);
		}
		//constants here
		//KINECT COLOR CAMERA
		float fx_rgb = 5.2921508098293293e+02;
		float fy_rgb = 5.2556393630057437e+02;
		float cx_rgb = 3.2894272028759258e+02;
		float cy_rgb = 2.6748068171871557e+02;
		float k1_rgb = 2.6451622333009589e-01;
		float k2_rgb = -8.3990749424620825e-01;
		float p1_rgb = -1.9922302173693159e-03;
		float p2_rgb = 1.4371995932897616e-03;
		float k3_rgb = 9.1192465078713847e-01;
		//DEPTH CAMERA
		float fx_d = 5.9421434211923247e+02;
		float fy_d = 5.9104053696870778e+02;
		float cx_d = 3.3930780975300314e+02;
		float cy_d = 2.4273913761751615e+02;
		float k1_d = -2.6386489753128833e-01;
		float k2_d = 9.9966832163729757e-01;
		float p1_d = -7.6275862143610667e-04;
		float p2_d = 5.0350940090814270e-03;
		float k3_d = -1.3053628089976321e+00;
		//transformation between cameras
		float R1[] = {9.9984628826577793e-01, 1.2635359098409581e-03,-1.7487233004436643e-02};
		float R2[] = {-1.4779096108364480e-03, 9.9992385683542895e-01, -1.2251380107679535e-02};
		float R3[] = {1.7470421412464927e-02, 1.2275341476520762e-02, 9.9977202419716948e-01};
		float T[] = {1.9985242312092553e-02, -7.4423738761617583e-04,-1.0916736334336222e-02};
		//Size of the image, use to transform from texture coord to image coord..

		unsigned char *result_rgb_ptr = rgb;
		unsigned char *input_rgb_ptr = rgb_mid;
		unsigned short *depth_ptr = depth_true;
		//for each depth pixel, we find the color...
		//process each row
		for(int x_d=0; x_d<480; x_d++){
			for(int y_d=0; y_d<640; y_d++){
				short rawDepth = *depth_ptr;
				float P3D[3];
				//depth in meter!
				float depth = depthLookUpHere[rawDepth];
				P3D[0] = (x_d - cy_d) * depth / fy_d;
				P3D[1]= (y_d - cx_d) * depth / fx_d;
				P3D[2]= depth;

				float P3D_1[3];
				float P2D_rgb[2];
				//transformation here
				P3D_1[0]=dot(R1, P3D)+T[0];
				P3D_1[1]=dot(R2, P3D)+T[1];
				P3D_1[2]=dot(R3, P3D)+T[2];

				//location of the color pixel
				P2D_rgb[0] = (P3D_1[0] * fy_rgb / P3D_1[2]) + cy_rgb;
				P2D_rgb[1] = (P3D_1[1] * fx_rgb / P3D_1[2]) + cx_rgb;
				int my_x, my_y;
				my_x = MAX((int)P2D_rgb[0], 0);
				my_y = MAX((int)P2D_rgb[1], 0);
				my_x = MIN((int)P2D_rgb[0], 480);
				my_y = MIN((int)P2D_rgb[1], 640);

				unsigned char r;
				unsigned char g;
				unsigned char b;
				//if(my_x > 480 || my_y > 640 || my_x < 0 || my_y < 0){
				//	char buf[512];
				//	sprintf(buf, "%d %d %d %d\n", x_d, y_d, my_x, my_y);
				//	__android_log_write(ANDROID_LOG_INFO, "Kinect", buf);
				//}else{
				r = input_rgb_ptr[my_x*640*3+my_y*3+0];
				g = input_rgb_ptr[my_x*640*3+my_y*3+1];
				b = input_rgb_ptr[my_x*640*3+my_y*3+2];
				//}
				*(result_rgb_ptr+0)=r;
				*(result_rgb_ptr+1)=g;
				*(result_rgb_ptr+2)=b;
				float mydepth=128*(depth-0.35);
				mydepth = MAX((int)mydepth, 0);
				mydepth = MIN((int)mydepth, 255);
				*(result_rgb_ptr+3) = mydepth;
				depth_ptr++;
				result_rgb_ptr+=4;
			}
		}

		free(depthLookUpHere);
		//memcpy(rgb, rgb_mid, 640*480*3*sizeof(uint8_t));
		got_new_frame_rgb=0;
	}
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}
int get_max_xy(int *x, int *y){
	*x = max_x;
	*y = max_y;
}
//need to convert it to ARB
void getRGBData(uint8_t *rgb){
	//lock this and make copy of the memory
	float max_depth=-1;
	float min_depth=999999;
	//pthread_mutex_lock(&gl_backbuf_mutex);
	if(got_new_frame_rgb){

		//initialize the lookup table for the first time use
		if(depthLookUp==0){
			depthLookUp=(float*)malloc(sizeof(float)*2048);
			for (int i = 0; i < 2048; i++) {
			  depthLookUp[i] = 32*(rawDepthToMeters(i)-0.35);
			  if(depthLookUp[i]>255){
				  depthLookUp[i]=0;
			  }
			  if(depthLookUp[i]<0){
				  depthLookUp[i]=0;
			  }
			}
		}

		unsigned char *rgb_ptr = rgb;
		unsigned char *rgb_ptr_mid = rgb_mid;
		unsigned short *depth_ptr = depth_true;
		int max_depth = -1;

		for(int i=0;i<640*480;i++){
			short rawDepth = *depth_ptr;
			*rgb_ptr=*rgb_ptr_mid;
			*(rgb_ptr+1)=*(rgb_ptr_mid+1);
			*(rgb_ptr+2)=*(rgb_ptr_mid+2);
			if(rawDepth < 2047 && rawDepth > 0){
				*(rgb_ptr+3)=(depthLookUp[rawDepth]);
//				if((depthLookUp[rawDepth])>max_depth)
//					max_depth = depthLookUp[rawDepth];
//				if((depthLookUp[rawDepth])<min_depth)
//					min_depth = depthLookUp[rawDepth];
//				if(*(rgb_ptr+3)<2){
//					max_x = i%640;
//					max_y = i/640;
//				}
			}
			else{
				*(rgb_ptr+3)=0;
			}
			rgb_ptr+=4;
			depth_ptr++;
			rgb_ptr_mid+=3;
		}
		//memcpy(rgb, rgb_mid, 640*480*3*sizeof(uint8_t));
		got_new_frame_rgb=0;
		//char buf[512];
		//sprintf(buf, "Max: %lf \t Min %lf\n", max_depth, min_depth);
		//__android_log_write(ANDROID_LOG_INFO, "Kinect", buf);
	}
	//pthread_cond_signal(&gl_frame_cond);
	//pthread_mutex_unlock(&gl_backbuf_mutex);
}
void depth_cb_true(freenect_device *dev, void *v_depth, uint32_t timestamp){
	uint16_t *depth = (uint16_t*)v_depth;
	pthread_mutex_lock(&gl_backbuf_mutex);
	memcpy(depth_true, depth, 640*480*sizeof(uint16_t));
	got_new_frame=1;
	got_new_frame_depth_true=1;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}
void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	int i;
	uint16_t *depth = (uint16_t*)v_depth;
	pthread_mutex_lock(&gl_backbuf_mutex);
	got_new_frame=1;
	unsigned char* depth_mid_ptr=depth_mid;
	for (i=0; i<640*480; i++) {
		int pval = t_gamma[depth[i]];
		int lb = pval & 0xff;
		switch (pval>>8) {
			case 0:
				//should be more efficient then the old way way. (optimized for mobile!)
				*depth_mid_ptr=255;
				*(depth_mid_ptr+1)=255-lb;
				*(depth_mid_ptr+2)=255-lb;
				//*(depth_mid_ptr+3)=255;
				depth_mid_ptr+=3;
//				depth_mid[3*i+0] = 255;
//				depth_mid[3*i+1] = 255-lb;
//				depth_mid[3*i+2] = 255-lb;
				break;
			case 1:
				*depth_mid_ptr=255;
				*(depth_mid_ptr+1)=lb;
				*(depth_mid_ptr+2)=0;
				//*(depth_mid_ptr+3)=255;
				depth_mid_ptr+=3;
//				depth_mid[3*i+0] = 255;
//				depth_mid[3*i+1] = lb;
//				depth_mid[3*i+2] = 0;
				break;
			case 2:
				*depth_mid_ptr=255-lb;
				*(depth_mid_ptr+1)=255;
				*(depth_mid_ptr+2)=0;
//				*(depth_mid_ptr+3)=255;
				depth_mid_ptr+=3;
//				depth_mid[3*i+0] = 255-lb;
//				depth_mid[3*i+1] = 255;
//				depth_mid[3*i+2] = 0;
				break;
			case 3:
				*depth_mid_ptr=0;
				*(depth_mid_ptr+1)=255;
				*(depth_mid_ptr+2)=lb;
				//*(depth_mid_ptr+3)=255;
				depth_mid_ptr+=3;
//				depth_mid[3*i+0] = 0;
//				depth_mid[3*i+1] = 255;
//				depth_mid[3*i+2] = lb;
				break;
			case 4:
				*depth_mid_ptr=0;
				*(depth_mid_ptr+1)=255-lb;
				*(depth_mid_ptr+2)=255;
				//*(depth_mid_ptr+3)=255;
				depth_mid_ptr+=3;
//				depth_mid[3*i+0] = 0;
//				depth_mid[3*i+1] = 255-lb;
//				depth_mid[3*i+2] = 255;
				break;
			case 5:
				*depth_mid_ptr=0;
				*(depth_mid_ptr+1)=0;
				*(depth_mid_ptr+2)=255-lb;
				//*(depth_mid_ptr+3)=255;
				depth_mid_ptr+=3;
//				depth_mid[3*i+0] = 0;
//				depth_mid[3*i+1] = 0;
//				depth_mid[3*i+2] = 255-lb;
				break;
			default:
				*depth_mid_ptr=0;
				*(depth_mid_ptr+1)=0;
				*(depth_mid_ptr+2)=0;
				depth_mid_ptr+=3;
//				depth_mid[3*i+0] = 0;
//				depth_mid[3*i+1] = 0;
//				depth_mid[3*i+2] = 0;
				break;
		}
	}
	//got_depth++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	pthread_mutex_lock(&gl_backbuf_mutex);

	// swap buffers
	assert (rgb_back == rgb);
	rgb_back = rgb_mid;
	freenect_set_video_buffer(dev, rgb_back);
	rgb_mid = (uint8_t*)rgb;
	got_new_frame_rgb=1;
	//got_rgb++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void *freenect_threadfunc(void *arg)
{
	int accelCount = 0;

	freenect_set_tilt_degs(f_dev,freenect_angle);
	freenect_set_led(f_dev,LED_RED);
	//using the true value instead of the remapping.
	freenect_set_depth_callback(f_dev, depth_cb_true);
	freenect_set_video_callback(f_dev, rgb_cb);
	freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, current_format));
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
	freenect_set_video_buffer(f_dev, rgb_back);

	freenect_start_depth(f_dev);
	freenect_start_video(f_dev);

	//printf("'w'-tilt up, 's'-level, 'x'-tilt down, '0'-'6'-select LED mode, 'f'-video format\n");
	char buf[512];
	__android_log_write(ANDROID_LOG_INFO, "Kinect","Threaded...\n");

	die=0;

	while (!die && freenect_process_events(f_ctx) >= 0) {
		//Throttle the text output
//		if (accelCount++ >= 2000)
//		{
//			accelCount = 0;
//			freenect_raw_tilt_state* state;
//			freenect_update_tilt_state(f_dev);
//			state = freenect_get_tilt_state(f_dev);
//			double dx,dy,dz;
//			freenect_get_mks_accel(state, &dx, &dy, &dz);
//			printf("\r raw acceleration: %4d %4d %4d  mks acceleration: %4f %4f %4f", state->accelerometer_x, state->accelerometer_y, state->accelerometer_z, dx, dy, dz);
//			fflush(stdout);
//		}
//
//		if (requested_format != current_format) {
//			freenect_stop_video(f_dev);
//			freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, requested_format));
//			freenect_start_video(f_dev);
//			current_format = requested_format;
//		}
		usleep(100);
	}
	__android_log_write(ANDROID_LOG_INFO, "Kinect","Shutting down...\n");

	printf("\nshutting down streams...\n");

	freenect_stop_depth(f_dev);
	freenect_stop_video(f_dev);

	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);

	free(depth_front);
	free(depth_mid);
	free(depth_true);
	free(rgb_back);
	free(rgb_mid);
	free(rgb_front);
	printf("-- done!\n");
	__android_log_write(ANDROID_LOG_INFO, "Kinect","Done...\n");
	return NULL;
}
void *xtion_threadfunc(void *arg)
{
	//using the true value instead of the remapping.
	freenect_set_depth_callback(f_dev, depth_cb_true);
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_10BIT));

	freenect_start_depth(f_dev);

	char buf[512];
	__android_log_write(ANDROID_LOG_INFO, "Xtion","Threaded...\n");

	die=0;

	while (!die && freenect_process_events(f_ctx) >= 0) {
		//Throttle the text output
		usleep(100);
	}
	__android_log_write(ANDROID_LOG_INFO, "Xtion","Shutting down...\n");

	printf("\nshutting down streams...\n");

	freenect_stop_depth(f_dev);
	//freenect_stop_video(f_dev);

	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);

	free(depth_front);
	free(depth_mid);
	free(depth_true);
	free(rgb_back);
	free(rgb_mid);
	free(rgb_front);
	printf("-- done!\n");
	__android_log_write(ANDROID_LOG_INFO, "Kinect","Done...\n");
	return NULL;
}

void stop_kinect(){
	die=1;
}
int try_kinect()
{
	__android_log_write(ANDROID_LOG_INFO, "Kinect", "message here");

	int res;

	depth_mid = (uint8_t*)malloc(640*480*3);
	depth_front = (uint8_t*)malloc(640*480*3);
	depth_true= (uint16_t*)malloc(640*480*sizeof(uint16_t));
	rgb_back = (uint8_t*)malloc(640*480*3);
	rgb_mid = (uint8_t*)malloc(640*480*3);
	rgb_front = (uint8_t*)malloc(640*480*3);

	__android_log_write(ANDROID_LOG_INFO, "Kinect","Kinect camera test\n");

	int i;
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = pow(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}

	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		__android_log_write(ANDROID_LOG_INFO, "Kinect","Kinect init() failed\n");
		return 1;
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);

	char my_log[512];
	sprintf(my_log, "Number Devices found %d\n", nr_devices);
	__android_log_write(ANDROID_LOG_INFO, "Kinect", my_log);

	int user_device_number = 0;

	if (nr_devices < 1)
		return 1;

	if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		__android_log_write(ANDROID_LOG_INFO, "Kinect", "Cannot Open device\n");
		return 1;
	}

	res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
	if (res) {
		printf("pthread_create failed\n");
		__android_log_write(ANDROID_LOG_INFO, "Kinect", "pthread_create failed...\n");
		return 1;
	}

	return 0;
}
int try_xtion()
{
	__android_log_write(ANDROID_LOG_INFO, "Xtion", "message here");

	int res;

	depth_mid = (uint8_t*)malloc(640*480*3);
	depth_front = (uint8_t*)malloc(640*480*3);
	depth_true= (uint16_t*)malloc(640*480*sizeof(uint16_t));
	rgb_back = (uint8_t*)malloc(640*480*3);
	rgb_mid = (uint8_t*)malloc(640*480*3);
	rgb_front = (uint8_t*)malloc(640*480*3);

	__android_log_write(ANDROID_LOG_INFO, "Kinect","Kinect camera test\n");

	int i;
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}

	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		__android_log_write(ANDROID_LOG_INFO, "Kinect","Kinect init() failed\n");
		return 1;
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_CAMERA));

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);

	char my_log[512];
	sprintf(my_log, "Number Devices found %d\n", nr_devices);
	__android_log_write(ANDROID_LOG_INFO, "Kinect", my_log);

	int user_device_number = 0;

	if (nr_devices < 1)
		return 1;

	if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		__android_log_write(ANDROID_LOG_INFO, "Kinect", "Cannot Open device\n");
		return 1;
	}

	res = pthread_create(&freenect_thread, NULL, xtion_threadfunc, NULL);
	if (res) {
		printf("pthread_create failed\n");
		__android_log_write(ANDROID_LOG_INFO, "Kinect", "pthread_create failed...\n");
		return 1;
	}

	return 0;
}
