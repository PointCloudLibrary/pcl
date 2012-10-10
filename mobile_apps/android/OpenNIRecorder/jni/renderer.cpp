#include "renderer.h"
#include "common.h"
#include "PCLWrapper.h"
#include <android/log.h>

#define MODULE "multi"
#define USE_OPENNI
#define NUM_BUTTONS 12

Renderer::Renderer(int _width, int _height){
	//constructor
	image_width = _width;
	image_height = _height;
	user_input_x = 0;
	user_input_y = 0;

	iXangle = 0;
	iYangle = 90;
	iZangle = 0;

	uiWidth = _width;
	uiHeight = _height;
	animParam = 1.0;
}
Renderer::~Renderer() {
	//distructor?
	if (gProgram)
		glDeleteProgram(gProgram);
	gProgram = NULL;

	if (overlayProgram)
		glDeleteProgram(overlayProgram);
	overlayProgram = NULL;

	if (gDirectProgram) {
		glDeleteProgram(gDirectProgram);
	}
	gDirectProgram = NULL;
	free(depth_info);
	free(depth_short_info);
	free(processed_data);
    delete(fingerTracker);
}

void Renderer::touch(){
	this->isTouched = !this->isTouched;
}

void Renderer::updateMousePointer(float x, float y){
	user_input_x = x;
	user_input_y = y;
}
void Renderer::setXYZ(double x, double y, double z){
	iAccx = x;
	iAccy = y;
	iAccz = z;
}
void Renderer::pressButton(int x, int y){
//	if(y>screen_height || y<screen_height-120){
//		return;
//	}
	int	button_id = x / 160;
	int button_id_y = (screen_height-y) / 120;
	if(button_id>NUM_BUTTONS || button_id<0){
		return;
	}
	int b_id = button_id+button_id_y*4;
	buttons[b_id]=!buttons[b_id];

	show_depth=buttons[0];
	show_rgb=buttons[1];
	record_depth = buttons[2];
	record_rgb = buttons[3];
	trackFinger = buttons[4];

	char buf [512];
    __android_log_print(ANDROID_LOG_DEBUG, MODULE,  "BUTTONS: %d", b_id);
}

void Renderer::savePNG(){
	this->isSaving = !this->isSaving;
}
bool Renderer::init() {
	isTouched = false;
	isSaving = true;
	trackFinger = false;
    fingerTracker = new FingerTracker(IMAGE_WIDTH, IMAGE_HEIGHT);

//	gp = new GpuImageProcessor();
//	gb_in = new GpuImageProcessorBuffer(IMAGE_WIDTH, IMAGE_HEIGHT);
//	gb_out = new GpuImageProcessorBuffer(IMAGE_WIDTH, IMAGE_HEIGHT);

	char buf [512];
    __android_log_print(ANDROID_LOG_DEBUG, MODULE,  "GL_VENDOR: %s", glGetString(GL_VENDOR));
    __android_log_print(ANDROID_LOG_DEBUG, MODULE,  "GL_VERSION: %s", glGetString(GL_VERSION));
    __android_log_print(ANDROID_LOG_DEBUG, MODULE,  "GL_RENDERER: %s", glGetString(GL_RENDERER));
    __android_log_print(ANDROID_LOG_DEBUG, MODULE,  "GL_EXTENSIONS: %s", glGetString(GL_EXTENSIONS));
    __android_log_print(ANDROID_LOG_DEBUG, MODULE,  "GL_IMPLEMENTATION_COLOR_READ_FORMAT: %s", glGetString(GL_IMPLEMENTATION_COLOR_READ_FORMAT));
    __android_log_print(ANDROID_LOG_DEBUG, MODULE,  "GL_IMPLEMENTATION_COLOR_READ_TYPE: %s", glGetString(GL_IMPLEMENTATION_COLOR_READ_TYPE));

    GLint ext_format, ext_type;
    glGetIntegerv(GL_IMPLEMENTATION_COLOR_READ_FORMAT, &ext_format);
    glGetIntegerv(GL_IMPLEMENTATION_COLOR_READ_TYPE, &ext_type);

    sprintf(buf,"Supported GL_IMPLEMENTATION_COLOR_READ_FORMAT: %d", ext_format);
    __android_log_print(ANDROID_LOG_DEBUG, MODULE,  buf);

    sprintf(buf,"Supported GL_IMPLEMENTATION_COLOR_READ_TYPE: %d", ext_type);
    __android_log_print(ANDROID_LOG_DEBUG, MODULE, buf);

    //TODO: a bit of performance hit here again... remove flags here if not needed later
    // set up depth test
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST); //depth function is GL_LESS
   	glEnable(GL_CULL_FACE);  //cull face is GL_BACK
   	glEnable(GL_BLEND);      //we are leaving blending on
   	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //we compile the shader the fragment program here.
    //read from the files, multi.frag and multi.vert
    //nv_shader_init(app->activity->assetManager);
    gProgram = nv_load_program("multi");
    if (!gProgram) {
    	sprintf(buf, "Unable to load program MULTI\n");
    	__android_log_print(ANDROID_LOG_DEBUG, MODULE,  buf);
        return false;
    }
    gvPositionHandle = glGetAttribLocation(gProgram, "a_position");
    gvTexCoordHandle = glGetAttribLocation(gProgram, "a_texCoord");
    gvSamplerRGBHandle = glGetAttribLocation(gProgram, "s_texture_rgb");
    gvSamplerDEPTHHandle = glGetAttribLocation(gProgram, "s_texture_depth");
    gvDepthHandle = glGetAttribLocation(gProgram, "a_depth");

    //for the direct output using texture
    gDirectProgram = nv_load_program("direct");
    if (!gDirectProgram) {
        	sprintf(buf, "Unable to load program MULTI\n");
        	__android_log_print(ANDROID_LOG_DEBUG, MODULE,  buf);
            return false;
    }
    gvDirectPositionHandle = glGetAttribLocation(gDirectProgram, "a_position");
    gvDirectTexCoordHandle = glGetAttribLocation(gDirectProgram, "a_texCoord");
    gvDirectSamplerRGBHandle = glGetAttribLocation(gDirectProgram, "s_texture_rgb");

    overlayProgram = nv_load_program("overlay");
    if (!overlayProgram) {
		sprintf(buf, "Unable to load program overlay\n");
		__android_log_print(ANDROID_LOG_DEBUG, MODULE,  buf);
		return false;
    }

    glBindAttribLocation(overlayProgram, 0, "vPosition");
    overlayvPositionHandle = glGetAttribLocation(overlayProgram, "vPosition");
    overlayvTexCoordHandle = glGetAttribLocation(overlayProgram, "a_texCoord");
    overlayvSamplerHandle = glGetAttribLocation(overlayProgram, "s_texture_rgb");
    overlayaColorHandle = glGetAttribLocation(overlayProgram, "a_color");

    // Use tightly packed data
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    //glViewport(0, 0, IMAGE_WIDTH*2, IMAGE_HEIGHT);
    //glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    //create a buffer that can store the input image texture
    fb = new FrameBuffer(IMAGE_WIDTH, IMAGE_HEIGHT);

    //create the Point Cloud Data here.
    myVertices = (GLfloat*)malloc(IMAGE_WIDTH*IMAGE_HEIGHT*3*sizeof(GLfloat)); //location for the vertices
    myColor = (GLfloat*)malloc(IMAGE_WIDTH*IMAGE_HEIGHT*4*sizeof(GLfloat));

    sprintf(buf,"Initialize Vertices Size: %d", DEPTH_VERTEX_SIZE);
    __android_log_print(ANDROID_LOG_DEBUG, "Vertices:",  buf);
	__android_log_write(ANDROID_LOG_INFO, "Init:", "Initialization Finished\n");

	depth_info = (unsigned char*)malloc(IMAGE_HEIGHT*IMAGE_WIDTH*COLOR_CHANNEL);      //RGBD (faster than RGB)
	processed_data = (unsigned char*)malloc(IMAGE_WIDTH*IMAGE_HEIGHT*COLOR_CHANNEL);
	depth_short_info = (unsigned short*)malloc(IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(unsigned short));      //RGBA (faster than RGB)
	depth_float_info = (float*)malloc(IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(float));
    lookupDepth = (float*)malloc(2048*sizeof(float));
    rgb_info = (unsigned char*)malloc(IMAGE_HEIGHT*IMAGE_WIDTH*3);      //RGB - for opencv processing

	if(!processed_data){
		__android_log_write(ANDROID_LOG_INFO, "Init:", "Malloc Processed Data Failed\n");
		return false;
	}
	if(!depth_info){
		__android_log_write(ANDROID_LOG_INFO, "Init:", "Malloc Depth Data Failed\n");
		return false;
	}
	if(!depth_short_info){
		__android_log_write(ANDROID_LOG_INFO, "Init:", "Malloc Depth Short Data Failed\n");
		return false;
	}
	if(!depth_float_info){
		__android_log_write(ANDROID_LOG_INFO, "Init:", "Malloc Depth Float Data Failed\n");
		return false;
	}
	memset(processed_data, 0, IMAGE_WIDTH*IMAGE_HEIGHT*COLOR_CHANNEL*sizeof(unsigned char));
	__android_log_write(ANDROID_LOG_INFO, "Init:", "Memset Finished\n");
	//sleep(5);
    for(int i=0; i<2048; i++){
    	lookupDepth[i]=rawDepthToMeters(i);
        //sprintf(buf,"%lf ", lookupDepth[i]);
    	//__android_log_write(ANDROID_LOG_INFO, "Init:", buf);
    }
	__android_log_write(ANDROID_LOG_INFO, "Init:", "Malloc Finished\n");

	button_names = (char**)malloc(sizeof(char*)*NUM_BUTTONS);
	for(int i=0; i<NUM_BUTTONS; i++){
		buttons[i]=0;
		button_names[i]=(char*)malloc(sizeof(char)*128);
		sprintf(button_names[i], "");
	}

	buttons[0]=1;
	buttons[1]=1;
	show_rgb=1;
	show_depth=1;
	record_rgb = 0;
	record_depth = 0;
	sprintf(button_names[0], "Display Depth");
	sprintf(button_names[1], "Display RGB");
	sprintf(button_names[2], "Record Depth");
	sprintf(button_names[3], "Record RGB");
	sprintf(button_names[4], "Tracking");

    return true;
}
bool Renderer::recordRGB(){
	return record_rgb;
}
bool Renderer::recordDepth(){
	return record_depth;
}
bool Renderer::showDepth(){
	return show_depth;
}
bool Renderer::showRGB(){
	return show_rgb;
}

void Renderer::setMessage(char *my_msg, int size){
	memcpy(msg, my_msg, size);
}
void Renderer::setMessage2(char *my_msg, int size){
	memcpy(msg2, my_msg, size);
}
/**
 *
 */
int Renderer::copyToDevice(unsigned char *depth_info) {
   	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, IMAGE_WIDTH, IMAGE_HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, depth_info);
    return 0;
}

/**
 * Write the memory back to the pointer (TODO: very slow)
 */
void Renderer::copyToHost(unsigned char *processed_data){
	//read the image back from GPU!
	glReadPixels(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, processed_data);
}
/**
 *
 */
void RGBToRGBD(unsigned char *rgb, unsigned char *rgbd){
        for(int i = IMAGE_WIDTH*IMAGE_HEIGHT; --i >= 0;){
                *(rgbd+0)=*(rgb+0);
                *(rgbd+1)=*(rgb+1);
                *(rgbd+2)=*(rgb+2);
                *(rgbd+3)=255;
                rgb+=3;
                rgbd+=4;
        }
}
void Renderer::displayTextureDirect(int offset_x, int offset_y, float scale, float rotx, float roty){
	glViewport(offset_x, offset_y, IMAGE_WIDTH*scale, IMAGE_HEIGHT*scale);
	//float	aspect_ratio_w = screenWidth/640.0;
	//float	aspect_ratio_h = screenHeight/480.0;
	float aspect_ratio_w=1;
	float aspect_ratio_h=1;

//	GLfloat vVertices[] = { -1.0f/aspect_ratio_w, 1.0f/aspect_ratio_h, 0.0f, // Position 0
//			0.0f, 0.0f, // TexCoord 0
//			-1.0f/aspect_ratio_w, -1.0f/aspect_ratio_h, 0.0f, // Position 1
//			0.0f, 1.0f, // TexCoord 1
//			1.0f/aspect_ratio_w, -1.0f/aspect_ratio_h, 0.0f, // Position 2
//			1.0f, 1.0f, // TexCoord 2
//			1.0f/aspect_ratio_w, 1.0f/aspect_ratio_h, 0.0f, // Position 3
//			1.0f, 0.0f // TexCoord 3
//			};
	GLfloat vVertices[] = { -1.0f/aspect_ratio_w, 1.0f/aspect_ratio_h, 0.0f, // Position 0
			0.0f, 0.0f, // TexCoord 0
			-1.0f/aspect_ratio_w, -1.0f/aspect_ratio_h, 0.0f, // Position 1
			0.0f, 1.0f, // TexCoord 1
			1.0f/aspect_ratio_w, -1.0f/aspect_ratio_h, 0.0f, // Position 2
			1.0f, 1.0f, // TexCoord 2
			1.0f/aspect_ratio_w, 1.0f/aspect_ratio_h, 0.0f, // Position 3
			1.0f, 0.0f // TexCoord 3
			};
	//flip!
//	GLfloat vVertices[] = { -1.0f/aspect_ratio_w, 1.0f/aspect_ratio_h, 0.0f, // Position 0
//			0.0f, 1.0f, // TexCoord 0
//			-1.0f/aspect_ratio_w, -1.0f/aspect_ratio_h, 0.0f, // Position 1
//			0.0f, 0.0f, // TexCoord 1
//			1.0f/aspect_ratio_w, -1.0f/aspect_ratio_h, 0.0f, // Position 2
//			1.0f, 0.0f, // TexCoord 2
//			1.0f/aspect_ratio_w, 1.0f/aspect_ratio_h, 0.0f, // Position 3
//			1.0f, 1.0f // TexCoord 3
//			};
	GLushort indices[] = { 0, 1, 2, 0, 2, 3 };
	GLsizei stride = 5 * sizeof(GLfloat); // 3 for position, 2 for texture

	//glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	//checkGlError("glClear");
	//glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	//glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	//checkGlError("glClearColor");

	glUseProgram(gDirectProgram);
	//checkGlError("glUseProgram");

	rotate_then_translate_matrix(0,1,1,1, 0,0,0, aModelView);

	//translate it
	aModelView[12] = 0;
	aModelView[13] = 0;
	//aModelView[14] = -0.5;

	//reset the aModelView
	rotate_matrix(rotx, 1.0, 0.0, 0.0, aRotate);
	multiply_matrix(aRotate, aModelView, aModelView);
	rotate_matrix(roty, 0.0, 1.0, 0.0, aRotate);
	multiply_matrix(aRotate, aModelView, aModelView);
	rotate_matrix(0, 0.0, 0.0, 1.0, aRotate);
	multiply_matrix(aRotate, aModelView, aModelView);


	//Pull the camera back from the geometry
	aModelView[14] = -2.0;
	//translate_matrix(1,0,-50,aModelView);

	//use very little FOV ~ 5 degree... we should use orthogonal instead...
	perspective_matrix(_PI/3.0, 1, 0.01, 100.0, aPerspective);
	multiply_matrix(aPerspective, aModelView, aMVP);

	glUniformMatrix4fv(glGetUniformLocation(gDirectProgram, "MVPMat"), (GLsizei)1, GL_FALSE, aMVP);


	// Load the vertex position
	glVertexAttribPointer(gvDirectPositionHandle, 3, GL_FLOAT, GL_FALSE, stride,
			vVertices);
	// Load the texture coordinate
	glVertexAttribPointer(gvDirectTexCoordHandle, 2, GL_FLOAT, GL_FALSE, stride,
			&vVertices[3]);

	glEnableVertexAttribArray(gvDirectPositionHandle);
	glEnableVertexAttribArray(gvDirectTexCoordHandle);

	// Bind the texture
	glActiveTexture(GL_TEXTURE0);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, fb->getTextureId());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	//Set the sampler texture unit to 0
	glUniform1i(gvDirectSamplerRGBHandle, 0);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, indices);
}
void Renderer::displayTexture() {
	//glViewport(screen_width/2-image_width/2, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
	glViewport(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
	//glViewport(-IMAGE_WIDTH, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
	//glViewport(0, 0, screen_width, screen_height);

//	float	aspect_ratio_w = screen_width/image_width;
//	float	aspect_ratio_h = screen_height/image_height;
	float aspect_ratio_w=1;
	float aspect_ratio_h=1;
	//flip for later use
	GLfloat vVertices[] = { -1.0f/aspect_ratio_w, 1.0f/aspect_ratio_h, 0.0f, // Position 0
			0.0f, 1.0f, // TexCoord 0
			-1.0f/aspect_ratio_w, -1.0f/aspect_ratio_h, 0.0f, // Position 1
			0.0f, 0.0f, // TexCoord 1
			1.0f/aspect_ratio_w, -1.0f/aspect_ratio_h, 0.0f, // Position 2
			1.0f, 0.0f, // TexCoord 2
			1.0f/aspect_ratio_w, 1.0f/aspect_ratio_h, 0.0f, // Position 3
			1.0f, 1.0f // TexCoord 3
			};

	GLushort indices[] = { 0, 1, 2, 0, 2, 3 };
	GLsizei stride = 5 * sizeof(GLfloat); // 3 for position, 2 for texture

	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	//checkGlError("glClear");

	//glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	//checkGlError("glClearColor");

	glUseProgram(gProgram);
	//checkGlError("glUseProgram");

	// Load the vertex position
	glVertexAttribPointer(gvPositionHandle, 3, GL_FLOAT, GL_FALSE, stride,
			vVertices);
	// Load the texture coordinate
	glVertexAttribPointer(gvTexCoordHandle, 2, GL_FLOAT, GL_FALSE, stride,
			&vVertices[3]);

	glEnableVertexAttribArray(gvPositionHandle);
	glEnableVertexAttribArray(gvTexCoordHandle);

	// Bind the texture
	glActiveTexture(GL_TEXTURE0);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, fb->getTextureId());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);glViewport(0, 0, screen_width, screen_height);

	//Set the sampler texture unit to 0
	glUniform1i(gvSamplerRGBHandle, 0);
	//glUniform1i(gvSamplerDEPTHHandle, 1);

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, indices);
}

void Renderer::setScreenSize(int x, int y){
	screen_width = x;
	screen_height = y;
}
void Renderer::displayOverlayPointCloud(float *point_cloud_data, const int NUM_POINT_CLOUD_DATA){
		glViewport(0, 0, screen_width, screen_height);
		double max_range = 5.0; //5 meters = max range?

		//load the depth map
		//GLfloat *point_vertices = myVertices;
		//float *depth_float_ptr = point_cloud_data;
	//	const float fx_d = 5.5879981950414015e+02;
	//    const float fy_d = 5.5874227168094478e+02;
	//    const float cx_d = 3.1844162327317980e+02;
	//    const float cy_d = 2.4574257294583529e+02;
		//now initialize the x,y variables... update z later only
	//    float fx_d_1 = 1.0 / fx_d;
	//    float fy_d_1 = 1.0 / fy_d;
//		for(int i=0;i<size; i++){
//			*(point_vertices+
//		}
		//overwrite the depth buffer
//	    for (float j = 0; j < IMAGE_HEIGHT; j++) {
//			for (float i = 0; i < IMAGE_WIDTH; i++) {
//				//unsigned short raw_depth = *point_depth;
//				float real_depth=*depth_float_ptr; //this range from...?
//				if(real_depth<=0)
//					real_depth=9999;
//				*(point_vertices + 0) = i;
//				*(point_vertices + 1) = j;
//				*(point_vertices + 2) = real_depth;
//				point_vertices += 3;
//				depth_float_ptr++;
//			}
//	    }
		//copy the color
	//	unsigned char *processed_ptr = processed_data;
	//	float *color_ptr = myColor;
	//	//color_ptr += 4*(IMAGE_WIDTH*IMAGE_HEIGHT-1);
	//	for (int k = 0; k <IMAGE_WIDTH*IMAGE_HEIGHT; k++){
	//		*(color_ptr+0) = *(processed_ptr+0)/255.0f;
	//		*(color_ptr+1) = *(processed_ptr+1)/255.0f;
	//		*(color_ptr+2) = *(processed_ptr+2)/255.0f;
	//		*(color_ptr+3) = 1.0f;
	//		processed_ptr+=4;
	//		color_ptr+=4;
	//	}

		//glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
			//checkGlError("glClearColor");

		//glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	    //glClear(GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);
		//
		//checkGlError("glClear");
		// Clear the color buffer
		//glClear(GL_COLOR_BUFFER_BIT);
		// Use the program object
		glUseProgram(overlayProgram);

		iXangle=0;
		iYangle=180;
		iZangle=180;

		//rotate - begin
		rotate_matrix(iXangle, 1.0, 0.0, 0.0, aModelView);
		rotate_matrix(iYangle, 0.0, 1.0, 0.0, aRotate);
		multiply_matrix(aRotate, aModelView, aModelView);
		rotate_matrix(iZangle, 0.0, 0.0, 1.0, aRotate);
		multiply_matrix(aRotate, aModelView, aModelView);

		//translate_matrix(user_input_x,user_input_y,-5,aModelView);
		//Pull the camera back from the geometry

		aModelView[12] = 0;
		aModelView[13] = 0;
		aModelView[14] -= 50;

		//use very little FOV ~ 5 degree... we should use orthogonal instead...
		perspective_matrix(_PI/36.0, (double)screen_width/(double)screen_height, 0.01, 300.0, aPerspective);
		multiply_matrix(aPerspective, aModelView, aMVP);


	//	glUniform1f(gAHandle, -1.0*(1.0 - animParam));
	//	glUniform1f(gThetaHandle, animParam*_HALF_PI);

	//	animParam -= 0.01;
	//	if(animParam < 0.0) animParam = 1.0f;

		glUniformMatrix4fv(glGetUniformLocation(overlayProgram, "MVPMat"), (GLsizei)1, GL_FALSE, aMVP);
		// Load the vertex data
		//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, vVertices2);
		//glEnableVertexAttribArray(0);
		//glDrawArrays(GL_TRIANGLES, 0, 3);
	    //glColorMask(GL_TRUE, GL_FALSE, GL_FALSE, GL_FALSE);
		//glDrawArrays(GL_LINE_LOOP, 0, 3);
		glVertexAttribPointer(glGetAttribLocation(overlayProgram, "vPosition"), 3, GL_FLOAT, GL_FALSE, 0, point_cloud_data);
		glVertexAttribPointer(glGetAttribLocation(overlayProgram, "a_color"), 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, depth_info);
		//attColor = glGetAttribLocation(__programObject, "a_color");

		glEnableVertexAttribArray(glGetAttribLocation(overlayProgram, "vPosition"));
		glEnableVertexAttribArray(glGetAttribLocation(overlayProgram, "a_color"));
		glDrawArrays(GL_POINTS, 0, NUM_POINT_CLOUD_DATA);

//		//this will get transformed by the vertex program
		float cube_depth = 0.5;
		float cube_width = 0.5;
		float cube_height = 0.5;
		float min_front = 0.6;

		GLfloat cubeVertex[]={
				//front
				cube_width, cube_height, 0.0,
				0, cube_height, 0.0,
				0, 0, 0.0,
				cube_width, 0, 0.0,
				cube_width, cube_height, 0.0,
				//right
				cube_width, cube_height, 0.0,
				cube_width, cube_height, cube_depth,
				cube_width, 0, cube_depth,
				cube_width, 0, 0.0,
				cube_width, cube_height, 0.0,
				//left
				0, cube_height, 0.0,
				0, cube_height, cube_depth,
				0, 0, cube_depth,
				0, 0, 0.0,
				0, cube_height, 0.0,
				//up
				cube_width, cube_height, 0.0,
				0, cube_height, 0.0,
				0, 0, 0.0,
				cube_width, 0, 0.0,
				cube_width, cube_height, 0.0,
				//down
				cube_width, cube_height, 0.0,
				0, cube_height, 0.0,
				0, 0, 0.0,
				cube_width, 0, 0.0,
				cube_width, cube_height, 0.0,
				//back
				cube_width, cube_height, cube_depth,
				0, cube_height, cube_depth,
				0, 0, cube_depth,
				cube_width, 0, cube_depth,
				cube_width, cube_height, cube_depth,
		};
		GLfloat cubeColor[]={
				1.0, 0.0, 0.0, 1.0,
				0.0, 1.0, 0.0, 1.0,
				0.0, 0.0, 1.0, 1.0,
				1.0, 1.0, 0.0, 1.0,
				1.0, 0.0, 1.0, 1.0,

				1.0, 0.0, 0.0, 1.0,
				0.0, 1.0, 0.0, 1.0,
				0.0, 0.0, 1.0, 1.0,
				1.0, 1.0, 0.0, 1.0,
				1.0, 0.0, 1.0, 1.0,

				1.0, 0.0, 0.0, 1.0,
				0.0, 1.0, 0.0, 1.0,
				0.0, 0.0, 1.0, 1.0,
				1.0, 1.0, 0.0, 1.0,
				1.0, 0.0, 1.0, 1.0,

				1.0, 0.0, 0.0, 1.0,
				0.0, 1.0, 0.0, 1.0,
				0.0, 0.0, 1.0, 1.0,
				1.0, 1.0, 0.0, 1.0,
				1.0, 0.0, 1.0, 1.0,

				1.0, 0.0, 0.0, 1.0,
				0.0, 1.0, 0.0, 1.0,
				0.0, 0.0, 1.0, 1.0,
				1.0, 1.0, 0.0, 1.0,
				1.0, 0.0, 1.0, 1.0,

				1.0, 0.0, 0.0, 1.0,
				0.0, 1.0, 0.0, 1.0,
				0.0, 0.0, 1.0, 1.0,
				1.0, 1.0, 0.0, 1.0,
				1.0, 0.0, 1.0, 1.0
		};
		glVertexAttribPointer(glGetAttribLocation(overlayProgram, "a_color"), 4, GL_FLOAT, GL_FALSE, 0, cubeColor);
		glVertexAttribPointer(glGetAttribLocation(overlayProgram, "vPosition"), 3, GL_FLOAT, GL_FALSE, 0, cubeVertex);
		glEnableVertexAttribArray(glGetAttribLocation(overlayProgram, "vPosition"));
		glEnableVertexAttribArray(glGetAttribLocation(overlayProgram, "a_color"));
		glDrawArrays(GL_LINE_STRIP, 0, 5*6); //6 faces

		glDisable(GL_DEPTH_TEST);
}
void Renderer::displayOverlay2(){
	glViewport(0, 0, screen_width, screen_height);
	double max_range = 5.0; //5 meters = max range?

	//load the depth map
	GLfloat *point_vertices = myVertices;
	float *depth_float_ptr = depth_float_info;
//	const float fx_d = 5.5879981950414015e+02;
//    const float fy_d = 5.5874227168094478e+02;
//    const float cx_d = 3.1844162327317980e+02;
//    const float cy_d = 2.4574257294583529e+02;
	//now initialize the x,y variables... update z later only
//    float fx_d_1 = 1.0 / fx_d;
//    float fy_d_1 = 1.0 / fy_d;

	//overwrite the depth buffer
    for (float j = 0; j < IMAGE_HEIGHT; j++) {
		for (float i = 0; i < IMAGE_WIDTH; i++) {
			//unsigned short raw_depth = *point_depth;
			float real_depth=*depth_float_ptr; //this range from...?
			if(real_depth<=0)
				real_depth=9999;
			*(point_vertices + 0) = i;
			*(point_vertices + 1) = j;
			*(point_vertices + 2) = real_depth;
			point_vertices += 3;
			depth_float_ptr++;
		}
    }
	//copy the color
//	unsigned char *processed_ptr = processed_data;
//	float *color_ptr = myColor;
//	//color_ptr += 4*(IMAGE_WIDTH*IMAGE_HEIGHT-1);
//	for (int k = 0; k <IMAGE_WIDTH*IMAGE_HEIGHT; k++){glViewport(0, 0, screen_width, screen_height);
//		*(color_ptr+0) = *(processed_ptr+0)/255.0f;
//		*(color_ptr+1) = *(processed_ptr+1)/255.0f;
//		*(color_ptr+2) = *(processed_ptr+2)/255.0f;
//		*(color_ptr+3) = 1.0f;
//		processed_ptr+=4;
//		color_ptr+=4;
//	}

	//glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		//checkGlError("glClearColor");

	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    //glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	//
	//checkGlError("glClear");
	// Clear the color buffer
	//glClear(GL_COLOR_BUFFER_BIT);
	// Use the program object
	glUseProgram(overlayProgram);

	//Comment the lines below to disable rotation
//	iXangle = 5.0+user_input_x/5.0;
//	//iYangle += 5;
//	iZangle = 180;
//	iYangle += 1;
//	//reset it
//	if(iYangle>270){
//		iYangle=90;
//	}
	//rotate - begin
	rotate_matrix(iXangle, 1.0, 0.0, 0.0, aModelView);
	rotate_matrix(iYangle, 0.0, 1.0, 0.0, aRotate);
	multiply_matrix(aRotate, aModelView, aModelView);
	rotate_matrix(iZangle, 0.0, 0.0, 1.0, aRotate);
	multiply_matrix(aRotate, aModelView, aModelView);

	//translate_matrix(user_input_x,user_input_y,-5,aModelView);
	//Pull the camera back from the geometry

	aModelView[12] = 0;
	aModelView[13] = -0.5;
	aModelView[14] -= 50;

	//use very little FOV ~ 5 degree... we should use orthogonal instead...
	perspective_matrix(_PI/36.0, (double)screen_width/(double)screen_height, 0.01, 300.0, aPerspective);
	multiply_matrix(aPerspective, aModelView, aMVP);


//	glUniform1f(gAHandle, -1.0*(1.0 - animParam));
//	glUniform1f(gThetaHandle, animParam*_HALF_PI);

//	animParam -= 0.01;
//	if(animParam < 0.0) animParam = 1.0f;

	glUniformMatrix4fv(glGetUniformLocation(overlayProgram, "MVPMat"), (GLsizei)1, GL_FALSE, aMVP);
	// Load the vertex data
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, vVertices2);
	//glEnableVertexAttribArray(0);
	//glDrawArrays(GL_TRIANGLES, 0, 3);
    //glColorMask(GL_TRUE, GL_FALSE, GL_FALSE, GL_FALSE);
	//glDrawArrays(GL_LINE_LOOP, 0, 3);,

	glVertexAttribPointer(glGetAttribLocation(overlayProgram, "vPosition"), 3, GL_FLOAT, GL_FALSE, 0, myVertices);
	glVertexAttribPointer(glGetAttribLocation(overlayProgram, "a_color"), 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, processed_data);
	//attColor = glGetAttribLocation(__programObject, "a_color");

	glEnableVertexAttribArray(glGetAttribLocation(overlayProgram, "vPosition"));
	glEnableVertexAttribArray(glGetAttribLocation(overlayProgram, "a_color"));
	glDrawArrays(GL_POINTS, 0, NUM_VERTICES);

	//this will get transformed by the vertex program
	float cube_depth = 7;
	float cube_width = 640;
	float cube_height = 480;
	float min_front = 0.6;

	GLfloat cubeVertex[]={
			//front
			cube_width, cube_height, 0.0,
			0, cube_height, 0.0,
			0, 0, 0.0,
			cube_width, 0, 0.0,
			cube_width, cube_height, 0.0,
			//right
			cube_width, cube_height, 0.0,
			cube_width, cube_height, cube_depth,
			cube_width, 0, cube_depth,
			cube_width, 0, 0.0,
			cube_width, cube_height, 0.0,
			//left
			0, cube_height, 0.0,
			0, cube_height, cube_depth,
			0, 0, cube_depth,
			0, 0, 0.0,
			0, cube_height, 0.0,
			//up
			cube_width, cube_height, 0.0,
			0, cube_height, 0.0,
			0, 0, 0.0,
			cube_width, 0, 0.0,
			cube_width, cube_height, 0.0,
			//down
			cube_width, cube_height, 0.0,
			0, cube_height, 0.0,
			0, 0, 0.0,
			cube_width, 0, 0.0,
			cube_width, cube_height, 0.0,
			//back
			cube_width, cube_height, cube_depth,
			0, cube_height, cube_depth,
			0, 0, cube_depth,
			cube_width, 0, cube_depth,
			cube_width, cube_height, cube_depth,
	};
	GLfloat cubeColor[]={
			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0,
			1.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 1.0, 1.0,

			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0,
			1.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 1.0, 1.0,

			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0,
			1.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 1.0, 1.0,

			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0,
			1.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 1.0, 1.0,

			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0,
			1.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 1.0, 1.0,

			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0,
			1.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 1.0, 1.0
	};
	glVertexAttribPointer(glGetAttribLocation(overlayProgram, "a_color"), 4, GL_FLOAT, GL_FALSE, 0, cubeColor);
	glVertexAttribPointer(glGetAttribLocation(overlayProgram, "vPosition"), 3, GL_FLOAT, GL_FALSE, 0, cubeVertex);
	glEnableVertexAttribArray(glGetAttribLocation(overlayProgram, "vPosition"));
	glEnableVertexAttribArray(glGetAttribLocation(overlayProgram, "a_color"));
	glDrawArrays(GL_LINE_STRIP, 0, 5*6); //6 faces

	glDisable(GL_DEPTH_TEST);
}
unsigned char *Renderer::getDepthInfoPtr(){
	return depth_info;
}

unsigned short *Renderer::getDepthShortInfoPtr(){
	return depth_short_info;
}
unsigned char *Renderer::getRGBInfoPtr(){
        return rgb_info;
}

CvMemStorage* storage = cvCreateMemStorage(0);
void processImage(unsigned char *my_process_data, unsigned short *my_depth_data){
	static IplImage *input = cvCreateImageHeader(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 4);
	static IplImage *input_gray = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 1);

	cvSetData(input, my_process_data, input->widthStep);
	cvCvtColor(input,input_gray,CV_RGBA2GRAY); //discard the alpha
    cvCanny( input_gray, input_gray, 50, 200, 3 );
	cvCvtColor(input_gray,input,CV_GRAY2RGBA);
}

void saveImage(unsigned char *rgb_data, char *path){
	static IplImage *input = cvCreateImageHeader(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
	static IplImage *output = cvCreateImage(cvSize(IMAGE_WIDTH/2, IMAGE_HEIGHT/2), IPL_DEPTH_8U, 3);
	cvSetData(input, rgb_data, input->widthStep);
	cvResize(input, output, CV_INTER_LINEAR);
    cvCvtColor(output, output, CV_BGR2RGB);
	cvSaveImage(path, output);
}

void Renderer::convertToDepth(float *depth, unsigned short *depth_s){
	for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++){
		*depth=lookupDepth[*depth_s];
		depth++;
		depth_s++;
	}
}
void DepthToRGBD(unsigned short *depth, unsigned char *rgbd){
        for(int i = IMAGE_WIDTH*IMAGE_HEIGHT; --i >= 0;){
        		unsigned short depth_v = *depth;
        		//use bit shifting instead of dividing.
                *(rgbd+0)=(depth_v)>>4;
                *(rgbd+1)=(depth_v)>>2;
                *(rgbd+2)=(depth_v)>>1;
                *(rgbd+3)=100;
                depth+=1;
                rgbd+=4;
        }
}
void Renderer::render_pointcloud(float x, float y, float w, float h){
    glViewport(x, y, w, h);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    struct timeval start, end;
    double t1,t2;
    static double elapsed_sec=0;
    static int count=0;
    const int MAX_COUNT=30;
    gettimeofday(&start, NULL);
    int i;

    //glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    PCLWrapper *pcl_tester = new PCLWrapper();
    int size=pcl_tester->voxel_filter(depth_short_info, myVertices);
    displayOverlayPointCloud(myVertices, size);

    gettimeofday(&end, NULL);
    t1=start.tv_sec+(start.tv_usec/1000000.0);
    t2=end.tv_sec+(end.tv_usec/1000000.0);
    elapsed_sec += (t2-t1);
    count++;
    if(count>=MAX_COUNT){
		char buf[512];
    	sprintf(buf, "Display loop %f (s)\n", (elapsed_sec)/MAX_COUNT);
    	elapsed_sec=0;
    	count=0;
		__android_log_write(ANDROID_LOG_INFO, "Render Loop:", buf);
    }
}
void Renderer::renderButtons(unsigned char *rgba){
	IplImage *button_map = cvCreateImageHeader(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), IPL_DEPTH_8U, 4);
	cvSetData(button_map, rgba, button_map->widthStep);

	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2, CV_AA);
	char buf_out[1024];
	sprintf(buf_out, "PCL Demo: %s", &msg);
	cvPutText(button_map, buf_out, cvPoint(30, 30), &font, cvScalar(255, 255, 255, 0));
	sprintf(buf_out, " %s", &msg2);
	cvPutText(button_map, buf_out, cvPoint(30, 60), &font, cvScalar(255, 0, 0, 0));
	sprintf(buf_out, " %0.5lf, %0.5lf, %0.5lf", iAccx, iAccy, iAccz);
	cvPutText(button_map, buf_out, cvPoint(30, 90), &font, cvScalar(0, 255, 0, 0));

	for (int i=0; i<NUM_BUTTONS; i++){
		int j=i%4;
		int k=i/4;
		if(buttons[i]==0){
		/*draw a white box*/
			cvRectangle(button_map,              /* the dest image */
				cvPoint(160*j, 480-120*(k+1)),        /* top left point */
				cvPoint(160*(j+1)-5, 480-120*(k)),       /* bottom right point */
				cvScalar(255, 255, 255, 255), /* the color; blue */
				2, 8, 0);
		}else{
			/*draw a green box*/
			cvRectangle(button_map,              /* the dest image */
				cvPoint(160*j, 480-120*(k+1)),        /* top left point */
				cvPoint(160*(j+1)-5, 480-120*(k)),       /* bottom right point */
				cvScalar(0, 255, 0, 255), /* the color; green */
				2, 8, 0);
		}
		CvFont font2;
		cvInitFont(&font2, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2, CV_AA);
		cvPutText(button_map, button_names[i], cvPoint(160*j+5, 480-60-120*(k)), &font2, cvScalar(255, 255, 255, 0));
	}
	//draw text on them
	cvReleaseImageHeader(&button_map);
}

//void Renderer::plot(float *data, int x, int y, int width, int height, int N, float *color){
//	float x, dx = 1.0/N;
//	glPushMatrix(); /* GL_MODELVIEW is default */
//	glScalef(width, height, 1.0);
//	glTranslatef(x, y, 0.0);
//	glColor3f(color[0], color[1], color[2]);
//	glBegin(GL_LINE_STRIP);
//	for(x = 0; x < N; x += dx)
//	{
//		float y = *(data+x);
//		glVertex2f(x, y);
//	}
//	glEnd();
//	glPopMatrix();
//}
void Renderer::showBar(float x, float y){

	glViewport(0, 0, screen_width, screen_height);
	double max_range = 5.0; //5 meters = max range?
	//glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		//checkGlError("glClearColor");

	//glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	// Use the program object
	glUseProgram(overlayProgram);

	//Comment the lines below to disable rotation
//	iXangle = 45;
//	iZangle = 90;
//	iYangle += my_x/10;
//	//reset it
//	if(iYangle>360){
//		iYangle=0;
//	}
//	iXangle=0;
//	iYangle=180;
//	iZangle=180;

	//use this to reset the aModelView
	rotate_then_translate_matrix(0,1,1,1, 0,0,0, aModelView);

	//translate it
	aModelView[12] = -0.5;
	aModelView[13] = -0.5;
	aModelView[14] = -0.5;

	//reset the aModelView
	rotate_matrix(iXangle, 1.0, 0.0, 0.0, aRotate);
	multiply_matrix(aRotate, aModelView, aModelView);
	rotate_matrix(iYangle, 0.0, 1.0, 0.0, aRotate);
	multiply_matrix(aRotate, aModelView, aModelView);
	rotate_matrix(iZangle, 0.0, 0.0, 1.0, aRotate);
	multiply_matrix(aRotate, aModelView, aModelView);



	//Pull the camera back from the geometry
	aModelView[14] = -30;
	aModelView[12] = x/320 - 1;
	aModelView[13] = -y/240+0.5;

	//translate_matrix(1,0,-50,aModelView);


	//use very little FOV ~ 5 degree... we should use orthogonal instead...
	perspective_matrix(_PI/36.0, (double)screen_width/(double)screen_height, 0.01, 100.0, aPerspective);
	multiply_matrix(aPerspective, aModelView, aMVP);


	glUniformMatrix4fv(glGetUniformLocation(overlayProgram, "MVPMat"), (GLsizei)1, GL_FALSE, aMVP);


	//this will get transformed by the vertex program
	float cube_depth = 1.0;//*my_x/50;
	float cube_width = 1.0;//*my_x/50;
	float cube_height = 1.0;//*my_x/50;
	float min_front = 1;

	GLfloat cubeVertex[]={
			//front
			cube_width, cube_height, 0.0,
			0, cube_height, 0.0,
			0, 0, 0.0,
			cube_width, 0, 0.0,
			cube_width, cube_height, 0.0,
			//right
			cube_width, cube_height, 0.0,
			cube_width, cube_height, cube_depth,
			cube_width, 0, cube_depth,
			cube_width, 0, 0.0,
			cube_width, cube_height, 0.0,
			//left
			0, cube_height, 0.0,
			0, cube_height, cube_depth,
			0, 0, cube_depth,
			0, 0, 0.0,
			0, cube_height, 0.0,
			//up
			cube_width, cube_height, 0.0,
			0, cube_height, 0.0,
			0, 0, 0.0,
			cube_width, 0, 0.0,
			cube_width, cube_height, 0.0,
			//down
			cube_width, cube_height, 0.0,
			0, cube_height, 0.0,
			0, 0, 0.0,
			cube_width, 0, 0.0,
			cube_width, cube_height, 0.0,
			//back
			cube_width, cube_height, cube_depth,
			0, cube_height, cube_depth,
			0, 0, cube_depth,
			cube_width, 0, cube_depth,
			cube_width, cube_height, cube_depth,
	};
	GLfloat cubeColor[]={
			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0,
			1.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 1.0, 1.0,

			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0,
			1.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 1.0, 1.0,

			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0,
			1.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 1.0, 1.0,

			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0,
			1.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 1.0, 1.0,

			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0,
			1.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 1.0, 1.0,

			1.0, 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			0.0, 0.0, 1.0, 1.0,
			1.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 1.0, 1.0
	};
	glVertexAttribPointer(overlayaColorHandle, 4, GL_FLOAT, GL_FALSE, 0, cubeColor);
	glVertexAttribPointer(overlayvPositionHandle, 3, GL_FLOAT, GL_FALSE, 0, cubeVertex);
	glEnableVertexAttribArray(glGetAttribLocation(overlayProgram, "vPosition"));
	glEnableVertexAttribArray(glGetAttribLocation(overlayProgram, "a_color"));
//
////	// Load the texture coordinate
	//flip for later use
	GLfloat vVertices[] = { -1.0f, 1.0f, 0.0f, // Position 0
			0.0f, 1.0f, // TexCoord 0
			-1.0f, -1.0f, 0.0f, // Position 1
			0.0f, 0.0f, // TexCoord 1
			1.0f, -1.0f, 0.0f, // Position 2
			1.0f, 0.0f, // TexCoord 2
			1.0f, 1.0f, 0.0f, // Position 3
			1.0f, 1.0f // TexCoord 3
			};
	GLfloat texVert[]={
			0.0f, 1.0f, // TexCoord 0
			0.0f, 0.0f, // TexCoord 0
			1.0f, 0.0f, // TexCoord 0
			1.0f, 1.0f, // TexCoord 0
			0.0f, 1.0f, // TexCoord 0
			0.0f, 0.0f, // TexCoord 0
			1.0f, 0.0f, // TexCoord 0
			1.0f, 1.0f, // TexCoord 0
			0.0f, 1.0f, // TexCoord 0
			0.0f, 0.0f, // TexCoord 0
			1.0f, 0.0f, // TexCoord 0
			1.0f, 1.0f, // TexCoord 0
			0.0f, 1.0f, // TexCoord 0
			0.0f, 0.0f, // TexCoord 0
			1.0f, 0.0f, // TexCoord 0
			1.0f, 1.0f, // TexCoord 0
			0.0f, 1.0f, // TexCoord 0
			0.0f, 0.0f, // TexCoord 0
			1.0f, 0.0f, // TexCoord 0
			1.0f, 1.0f, // TexCoord 0
			0.0f, 1.0f, // TexCoord 0
			0.0f, 0.0f, // TexCoord 0
			1.0f, 0.0f, // TexCoord 0
			1.0f, 1.0f // TexCoord 0
	};
	GLushort indices[] = { 0, 1, 2, 0, 2, 3 };
	GLsizei stride = 5 * sizeof(GLfloat); // 3 for position, 2 for texture
	// Load the vertex position
//	glVertexAttribPointer(overlayvPositionHandle, 3, GL_FLOAT, GL_FALSE, stride,
//			vVertices);
	// Load the texture coordinate
	glVertexAttribPointer(overlayvTexCoordHandle, 2, GL_FLOAT, GL_FALSE, 0, texVert);
	glEnableVertexAttribArray(overlayvTexCoordHandle);
//
//	// Bind the texture
	glActiveTexture(GL_TEXTURE0);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, fb->getTextureId());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	//glViewport(0, 0, screen_width, screen_height);

	//Set the sampler texture unit to 0
	glUniform1i(overlayvSamplerHandle, 0);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 5*6); //6 faces

	glDisable(GL_DEPTH_TEST);
}
void Renderer::render(float x, float y, float w, float h){
    glViewport(x, y, w, h);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
//    //glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
#ifdef USE_OPENNI
    float scale = 1.1;
    static float rotate = 0;
    static float diff_r = 0.5;
//	rotate += diff_r;
//	if(rotate>10){
//		rotate=10;
//		diff_r = -0.5;
//	}
//	if(rotate < -10){
//		rotate=-10;
//		diff_r = 0.5;
//	}

    if(isTouched)
    	scale=0.5;
    if(trackFinger){
    	fingerTracker->runTracking(depth_short_info, rgb_info, -200, 400);
    	fingerTracker->getPosition(&my_x, &my_y);
    	float isGrasp=fingerTracker->isGrasp();
    	RGBToRGBD(rgb_info, depth_info);
    	rotate = -20.0*(my_x-320)/480.0;
    }
    //so slow... ;( we need to create RGBD in a much more efficient way or it won't work in real-time.

    //load the texture!
    if(!show_rgb){
    	memset(depth_info, 0, IMAGE_WIDTH*IMAGE_HEIGHT*4*sizeof(unsigned char));
    }
    else{
    	copyToDevice(depth_info);
    	//display the thresholded version on the left
    	displayTextureDirect(screen_width-IMAGE_WIDTH*scale, 0, scale, 0, -rotate);
    }
    //display the depth map on the right

    if(show_depth&&show_rgb){ //slow
    	DepthToRGBD(depth_short_info, depth_info);
    	renderButtons(depth_info);
        //overwrite the texture
        copyToDevice(depth_info);
        //display it side by side
        displayTextureDirect(0, 0, scale, 0, -rotate);
    }else{
    	DepthToRGBD(depth_short_info, depth_info);
    	renderButtons(depth_info);
        //overwrite the texture
        copyToDevice(depth_info);
        //display it side by side
        displayTextureDirect(0, 0, scale, 0, -rotate);
    }
    if(trackFinger)
    	showBar(my_x, my_y);

    if(isTouched){
    	PCLWrapper *pcl_tester = new PCLWrapper();
    	int size=pcl_tester->voxel_filter(depth_short_info, myVertices);
		memset (depth_info, 255, size*3);
    	displayOverlayPointCloud(myVertices, size);
    }
#else
    copyToDevice(depth_info);
    convertToDepth(depth_float_info, depth_short_info);
    displayTexture();
    //copyToHost(processed_data); //crazy slow... 0.08s! per call! Have to plan out carefully when to use this, and how to make this async call.
    //display the 3D volume..
    //displayOverlay2();
#endif

    //test benchmark...
    //glFinish();
}
