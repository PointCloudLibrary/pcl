/**
 * OpenNI Wrapper. Simplified the work flow to only a few function calls
 */

#include "OpenniWrapper.h"

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define CHECK_RC(rc, what)											 	\
	if (rc != XN_STATUS_OK)										  		\
	{	char buf[1024];													\
		sprintf(buf, "%s failed: %s\n", what, xnGetStatusString(rc)); 	\
		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);			\
		return rc;														\
	}

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
//private function
XnBool fileExists(const char *fn)
{
	XnBool exists;
	xnOSDoesFileExist(fn, &exists);
	return exists;
}
//Contructor
OpenniWrapper::OpenniWrapper(){
	//allocate memory or init other things here

}
//Destructor
OpenniWrapper::~OpenniWrapper(){
	//free memory here

}

//this will update the local buffer
bool OpenniWrapper::WaitAndUpdate(){ //we will wait and process the RGB and depth data
	XnStatus rc = XN_STATUS_OK;
	// Read a new frame
	//g_depth.StartGenerating();

//	g_image.WaitAndUpdateData();
//	g_depth.WaitAndUpdateData();

	rc = g_context.WaitAndUpdateAll(); //this was buggy! keep getting seg faults here when using other updates
//
//
	//usleep(300000);
	if (rc != XN_STATUS_OK)
	{
		sprintf(buf,"Read failed: %s\n", xnGetStatusString(rc));
		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
		return false;
	}

	if(hasDepth){
		g_depth.GetMetaData(g_depthMD);
		const XnDepthPixel* my_pDepth = g_depthMD.Data();
		//TODO: maybe redundant? however it seems like a better idea to make a copy of that pointer?
		//prepare the depth map and rgbd data
		if(my_pDepth==NULL){
			//sprintf(buf,"Read failed: Null pointer from the depth data\n");
			//__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
		}else{
			memcpy(pDepth, my_pDepth, width*height*sizeof(unsigned short));
		}
	}

	if(hasRGB){
		g_image.GetMetaData(g_imageMD);
		const XnUInt8* my_rgbImage = g_imageMD.Data();
		//memcpy sig fault?
		if(my_rgbImage==NULL){
			//sprintf(buf,"Read failed: Null pointer from the rgb data\n");
			//__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
		}else{
			memcpy(rgbImage, my_rgbImage, width*height*3*sizeof(unsigned char));
		}
	}

	return true;
}

//we shall only call this if the buffer is ready, user have to check if the buffer is large enough no error handling here
void OpenniWrapper::getRGB(unsigned char *rgb_buffer){
	if(rgb_buffer!=NULL)
		memcpy(rgb_buffer, rgbImage, width*height*3*sizeof(unsigned char));
}

//the raw depth data from OpenNI in millimeter (mm).
void OpenniWrapper::getDepth(unsigned short *depth_buffer){
	//should be 1 channel
	if(depth_buffer!=NULL)
		memcpy(depth_buffer, pDepth, width*height*1*sizeof(unsigned short));
}

//create the RGBD for display
void OpenniWrapper::getRGBD(unsigned char *rgbd_buffer){
	unsigned char *rgb_ptr = rgbd_buffer;
	//from OpenNI
	unsigned char *rgb_ptr_mid = rgbImage;
	//unsigned short *depth_ptr = pDepth;
	//int max_depth = -1;

	//convert to our format RGBD
	for(int i=0;i<width*height;i++){
		//short rawDepth = *depth_ptr;
		*rgb_ptr=*rgb_ptr_mid;
		*(rgb_ptr+1)=*(rgb_ptr_mid+1);
		*(rgb_ptr+2)=*(rgb_ptr_mid+2);
		//clear out the images that's out of range
//		if(rawDepth> 10000 || rawDepth < 600){
//			*(rgb_ptr+0)=0;
//			*(rgb_ptr+1)=0;
//			*(rgb_ptr+2)=0;
//		}
		//fix the depth for now for display purposes.
		*(rgb_ptr+3)=255;
		rgb_ptr+=4;
		//depth_ptr++;
		rgb_ptr_mid+=3;
	}
}
void OpenniWrapper::release(){
	//?what to free here?

	//Must free the resources or it won't resume!
	if(hasDepth)
		g_depth.Release();
	if(hasRGB)
		g_image.Release();
	g_scriptNode.Release();
	g_context.Release();
	//TODO: ?do i need to release the images and depth images as well?? I guess not?
}
int OpenniWrapper::getWidth(){
	return width;
}
int OpenniWrapper::getHeight(){
	return height;
}
int OpenniWrapper::start()
{
	XnStatus rc;
	EnumerationErrors errors;
	hasRGB = 0;
	hasDepth = 0;

	rc = g_context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
	if (rc == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		sprintf(buf, "%s\n", strError);
		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
		return (rc);
	}
	else if (rc != XN_STATUS_OK)
	{
		sprintf(buf, "Open failed: %s\n", xnGetStatusString(rc));
		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
		return (rc);
	}

	rc = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	hasDepth = 1;
	if (rc != XN_STATUS_OK)
	{
		sprintf(buf, "No depth node exists! Check your XML.");
		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
		hasDepth = 0;
		//return 1;
	}

	rc = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
	hasRGB=1;
	if (rc != XN_STATUS_OK)
	{
		sprintf(buf, "No image node exists! Check your XML.");
		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
		hasRGB=0;
		//return 1;
	}

//	rc = g_depth.SetIntProperty ("OutputFormat", 0);
//	if (rc != XN_STATUS_OK)
//	{
//		sprintf(buf, "Cannot set depth generator property");
//		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
//		return 1;
//	}
	//for ASUS XTION ONLY
	//see: http://dev.pointclouds.org/projects/pcl/wiki/MacOSX
	rc = g_depth.SetIntProperty ("RegistrationType", 1);
	if (rc != XN_STATUS_OK)
	{
		sprintf(buf, "Cannot set depth generator property: RegistrationType");
		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
		return 1;
	}

	//obviously Kinect doesn't support anything else!?
//	XnMapOutputMode outputMode;
//
//	outputMode.nXRes = 640;
//	outputMode.nYRes = 480;
//	outputMode.nFPS = 30;
////
//	rc = g_depth.SetMapOutputMode(outputMode);
//	if (rc != XN_STATUS_OK)
//	{
//		sprintf(buf, "Cannot set depth generator property");
//		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
//		return 1;
//	}
////
//	rc = g_image.SetMapOutputMode(outputMode);
//	if (rc != XN_STATUS_OK)
//	{
//		sprintf(buf, "Cannot set image generator property");
//		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
//		return 1;
//	}
	// TODO: check error code
	if(hasDepth)
		g_depth.GetMetaData(g_depthMD);
	if(hasRGB)
		g_image.GetMetaData(g_imageMD);

	// Hybrid mode isn't supported in this sample
//	if (g_imageMD.FullXRes() != g_depthMD.FullXRes() || g_imageMD.FullYRes() != g_depthMD.FullYRes())
//	{
//		sprintf (buf, "The device depth and image resolution must be equal!\n");
//		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
//		return 1;
//	}

	//show some info of the device...
//	sprintf(buf, "Image Resolution: %d x %d", g_imageMD.FullXRes(), g_imageMD.FullYRes());
//	__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);

//	// RGB is the only image format supported.
//	if (g_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
//	{
//		sprintf(buf, "The device image format must be RGB24\n");
//		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
//		return 1;
//	}

	//configure the global variable
	width  = g_depthMD.FullXRes();
	height  = g_depthMD.FullYRes();

	//g_image.StopGenerating();

	//init the tmp storage for the frames
	rgbImage = (unsigned char*)malloc(width*height*3*sizeof(unsigned char));
	pDepth = (unsigned short*)malloc(width*height*sizeof(unsigned short));

	//this will map the depth map to the rgb image by default. Of course we can turn this off
	//if we would like to process the depth map independently.
	//TODO: turning this off will cause the RGB image to corrupt? why?.glo
	if(g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image)!=XN_STATUS_OK)
	{
		sprintf(buf, "Cannot set GetAlternativeViewPointCap() ");
		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
	}
	if(g_image.GetAlternativeViewPointCap().SetViewPoint(g_depth)!=XN_STATUS_OK){
		sprintf(buf, "Cannot set GetAlternativeViewPointCap() for g_image ");
		__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
	}
	//show some info of the device...
	sprintf(buf, "Finished OpenNI Initialization");
	__android_log_print(ANDROID_LOG_DEBUG, "OPENNI",  buf);
	return 0;
}
