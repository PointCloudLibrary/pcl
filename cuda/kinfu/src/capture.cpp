#include <XnOS.h>
#include <XnCppWrapper.h>

#include "pcl/gpu/kinfu/openni_capture.hpp"
#include "pcl/gpu/containers/initialization.hpp"

using namespace cv;
using namespace std;
using namespace pcl;
using namespace xn;

//const std::string XMLConfig =
//"<OpenNI>"
//        "<Licenses>"
//        "<License vendor=\"PrimeSense\" key=\"0KOIk2JeIBYClPWVnMoRKn5cdY4=\"/>"
//        "</Licenses>"
//        "<Log writeToConsole=\"false\" writeToFile=\"false\">"
//                "<LogLevel value=\"3\"/>"
//                "<Masks>"
//                        "<Mask name=\"ALL\" on=\"true\"/>"
//                "</Masks>"
//                "<Dumps>"
//                "</Dumps>"
//        "</Log>"
//        "<ProductionNodes>"
//                "<Node type=\"Image\" name=\"Image1\">"
//                        "<Configuration>"
//                                "<MapOutputMode xRes=\"640\" yRes=\"480\" FPS=\"30\"/>"
//                                "<Mirror on=\"false\"/>"
//                        "</Configuration>"
//                "</Node> "
//                "<Node type=\"Depth\" name=\"Depth1\">"
//                        "<Configuration>"
//                                "<MapOutputMode xRes=\"640\" yRes=\"480\" FPS=\"30\"/>"
//                                "<Mirror on=\"false\"/>"
//                        "</Configuration>"
//                "</Node>"
//        "</ProductionNodes>"
//"</OpenNI>";


#define REPORT_ERROR(msg) pcl::gpu::error((msg), __FILE__, __LINE__)

struct pcl::gpu::CaptureOpenNI::Impl
{
	Context context;
	ScriptNode scriptNode;
	DepthGenerator depth;
	ImageGenerator image;
	ProductionNode node;
	DepthMetaData depthMD;
	ImageMetaData imageMD;
	XnChar strError[1024];

	bool has_depth;
	bool has_image;
};

pcl::gpu::CaptureOpenNI::~CaptureOpenNI() 
{
	impl->context.StopGeneratingAll();    
	impl->context.Release();
}

pcl::gpu::CaptureOpenNI::CaptureOpenNI(int device) : depth_focal_length_VGA(0), baseline(0.f), 
	shadow_value(0), no_sample_value(0), pixelSize(0.0), impl( new Impl() )
{
	XnMapOutputMode mode; 
	mode.nXRes = XN_VGA_X_RES; 
	mode.nYRes = XN_VGA_Y_RES; 
	mode.nFPS = 30; 
	
	XnStatus rc;
	rc = impl->context.Init();
	if (rc != XN_STATUS_OK)
	{
		sprintf(impl->strError, "Init failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}

	xn::NodeInfoList devicesList;
	rc = impl->context.EnumerateProductionTrees( XN_NODE_TYPE_DEVICE, NULL, devicesList, 0 );
	if (rc != XN_STATUS_OK)
	{
		sprintf(impl->strError, "Init failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}

	xn::NodeInfoList::Iterator it = devicesList.Begin();
	for( int i = 0; i < device; ++i ) it++;
			
	rc = impl->context.CreateProductionTree( *it, impl->node );
	if (rc != XN_STATUS_OK)
	{
		sprintf(impl->strError, "Init failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}
	
	XnLicense license;
	const char* vendor="PrimeSense";
	const char* key="0KOIk2JeIBYClPWVnMoRKn5cdY4=";	
	sprintf(license.strKey, key);
	sprintf(license.strVendor, vendor);

	rc = impl->context.AddLicense(license);
	if (rc != XN_STATUS_OK)
	{
		sprintf(impl->strError, "licence failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}
	
	rc = impl->depth.Create(impl->context);	
	if (rc != XN_STATUS_OK)		
	{	
		sprintf(impl->strError, "Depth generator  failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}		
	
	rc = impl->image.Create(impl->context);	
	if (rc != XN_STATUS_OK)		
	{	
		sprintf(impl->strError, "Image generator reate failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}		

    impl->has_depth = true;
    impl->has_image = true;

	//rc = impl->depth.SetIntProperty("HoleFilter", 1);
	rc = impl->depth.SetMapOutputMode(mode);
	rc = impl->image.SetMapOutputMode(mode);
		
	rc = impl->context.StartGeneratingAll();
	if (rc != XN_STATUS_OK)
	{
		sprintf(impl->strError, "Start failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}	

	getParams();
}


pcl::gpu::CaptureOpenNI::CaptureOpenNI(const string& filename) : depth_focal_length_VGA(0), baseline(0.f), 
		shadow_value(0), no_sample_value(0), pixelSize(0.0), impl( new Impl() )
{
	XnStatus rc;

	rc = impl->context.Init();
	if (rc != XN_STATUS_OK)
	{
		sprintf(impl->strError, "Init failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}
		
	rc = impl->context.OpenFileRecording(filename.c_str(), impl->node);
	if (rc != XN_STATUS_OK)
	{
		sprintf(impl->strError, "Open failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}

	rc = impl->context.FindExistingNode(XN_NODE_TYPE_DEPTH, impl->depth);
	impl->has_depth = (rc == XN_STATUS_OK);    
	
	rc = impl->context.FindExistingNode(XN_NODE_TYPE_IMAGE, impl->image);
	impl->has_image = (rc == XN_STATUS_OK);	

	if (!impl->has_image && impl->has_depth)
		REPORT_ERROR("Not depth and image nodes. Check your configuration");

	if(impl->has_depth)
		impl->depth.GetMetaData(impl->depthMD);

	if(impl->has_image)
		impl->image.GetMetaData(impl->imageMD);	

	// RGB is the only image format supported.
	if (impl->imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
		REPORT_ERROR("Image format must be RGB24\n");

	getParams();
}

bool pcl::gpu::CaptureOpenNI::grab(Mat& depth, Mat& rgb24)
{
	depth.release();
	rgb24.release();

	XnStatus rc = XN_STATUS_OK;
	
	rc = impl->context.WaitAndUpdateAll();
	if (rc != XN_STATUS_OK)
		return printf("Read failed: %s\n", xnGetStatusString(rc)), false;
	
	if(impl->has_depth)
	{
		impl->depth.GetMetaData(impl->depthMD);
		const XnDepthPixel* pDepth = impl->depthMD.Data();
		int x = impl->depthMD.FullXRes();
		int y = impl->depthMD.FullYRes();
        Mat(y, x,  CV_16U, (void*)pDepth).copyTo(depth);		
	}
    else
        printf("no depth\n");

	if (impl->has_image)
	{	
		impl->image.GetMetaData(impl->imageMD);	
		const XnRGB24Pixel* pImage = impl->imageMD.RGB24Data();	
		int x = impl->imageMD.FullXRes();
		int y = impl->imageMD.FullYRes();	
		rgb24 = Mat(y, x, CV_8UC3, (void*)pImage);
	}
    else
        printf("no image\n");

	return impl->has_image || impl->has_depth;	
}

void pcl::gpu::CaptureOpenNI::getParams()
{
	XnStatus rc = XN_STATUS_OK;	

	max_depth = impl->depth.GetDeviceMaxDepth();

    rc = impl->depth.GetRealProperty( "ZPPS", pixelSize ); // in mm
	if (rc != XN_STATUS_OK)
	{
		sprintf(impl->strError, "ZPPS failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}

	XnUInt64 depth_focal_length_SXGA_mm; //in mm
    rc = impl->depth.GetIntProperty("ZPD", depth_focal_length_SXGA_mm);
	if (rc != XN_STATUS_OK)
	{
		sprintf(impl->strError, "ZPD failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}

    XnDouble baseline_local;
    rc = impl->depth.GetRealProperty ("LDDIS", baseline_local);
    if (rc != XN_STATUS_OK)
	{
		sprintf(impl->strError, "ZPD failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}

	XnUInt64 shadow_value_local;
    rc = impl->depth.GetIntProperty ("ShadowValue", shadow_value_local);    
    if (rc != XN_STATUS_OK)
	{
		sprintf(impl->strError, "ShadowValue failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}
	shadow_value = (int)shadow_value_local;

	XnUInt64 no_sample_value_local;
    rc = impl->depth.GetIntProperty ("NoSampleValue", no_sample_value_local);    
    if (rc != XN_STATUS_OK)
	{
		sprintf(impl->strError, "NoSampleValue failed: %s\n", xnGetStatusString(rc));
		REPORT_ERROR(impl->strError);
	}
	no_sample_value = (int)no_sample_value_local;


    // baseline from cm -> mm
    baseline = (float)(baseline_local * 10);

    //focal length from mm -> pixels (valid for 1280x1024)    
    float depth_focal_length_SXGA = static_cast<float>(depth_focal_length_SXGA_mm / pixelSize);
	depth_focal_length_VGA = depth_focal_length_SXGA/2;
}