#include "test_png.h"
#include <android/log.h>
#include <sys/stat.h>
#include <time.h>

const char *PCL_SDCARD_DIR = "/mnt/sdcard2/pcl";
static char pcl_path[512];
static time_t t = 0;

int setup_pcl_dir(){
	if(t==0){
		//this will get initialized only the first time, and stayed till the program is restarted?
		t = time(0);
		sprintf(pcl_path, "%s/%d", PCL_SDCARD_DIR, t);
	}
	struct stat st;
	if(stat(pcl_path,&st) == 0){
		//printf(" /tmp is present\n");
		return 0;
	}
	//create the directory if it does not exist, return the error code
	mkdir(PCL_SDCARD_DIR, S_IRWXU | S_IRWXG | S_IRWXO); //just to make sure PCL path is always there
	return mkdir(pcl_path, S_IRWXU | S_IRWXG | S_IRWXO);
}

//DO NOT ACCESS these directly from other programs (use the thread manager for better performance).
int writeImageDepth(char* filename, int width, int height, unsigned short *buffer, char* title)
{
	char buf[1024];
	int error_code=0;
	if(error_code=setup_pcl_dir()){
		sprintf(buf, "Could not open SDCARD path %s for writing: error code %d \n", pcl_path, error_code);
		__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);
		return error_code;
	}

	//walk through the buffer?
//	unsigned short max = 0;
//	unsigned short min = 99999;
//	int i=0;
//	for(i=0; i<width*height; i++){
//		if(buffer[i]>max){
//			max=buffer[i];
//		}
//		if(buffer[i]<min){
//			min = buffer[i];
//		}
//	}
//	sprintf(buf, "Max %d, Min %d\n", max, min);
//	__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);

	int code = 0;
	FILE *fp;
	png_structp png_ptr;
	png_infop info_ptr;
	//png_bytep row;
	// Open file for writing (binary mode)
	char filepath[1024];
	sprintf(filepath,"%s/%s", pcl_path, filename);
	fp = fopen(filepath, "wb");
	if (fp == NULL) {
		//fprintf(stder	r, "Could not open file %s for writing\n", filename);
		sprintf(buf, "Could not open file %s for writing\n", filepath);
		__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);

		code = 1;
		goto finalise;
	}

	// Initialize write structure
	png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (png_ptr == NULL) {
		//fprintf(stderr, "Could not allocate write struct\n");
		sprintf(buf, "Could not allocate write struct\n");
		__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);

		code = 1;
		goto finalise;
	}

	// Initialize info structure
	info_ptr = png_create_info_struct(png_ptr);
	if (info_ptr == NULL) {
		//fprintf(stderr, "Could not allocate info struct\n");

		sprintf(buf, "Could not allocate info struct\n");
		__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);
		code = 1;
		goto finalise;
	}

	// Setup Exception handling
	if (setjmp(png_jmpbuf(png_ptr))) {
		//fprintf(stderr, "Error during png creation\n");

		sprintf(buf, "Error during png creation\n");
		__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);
		code = 1;
		goto finalise;
	}
	png_set_swap(png_ptr); //LITTLE ENDIAN!? or BIG ENDIAN?

	png_init_io(png_ptr, fp);
	// Write header (16 bit colour depth, greyscale)
	png_set_IHDR(png_ptr, info_ptr, width, height,
			16, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
			PNG_COMPRESSION_TYPE_DEFAULT, PNG_COMPRESSION_TYPE_DEFAULT);

	//fine tuned parameter for speed!
    png_set_filter(png_ptr, PNG_FILTER_TYPE_BASE, PNG_FILTER_SUB);
    png_set_compression_level(png_ptr, 1); //1 is Z_BEST_SPEED in zlib.h!
    png_set_compression_strategy(png_ptr, 3); //3 is Z_RLE

	// Set title
	if (title != NULL) {
		png_text title_text;
		title_text.compression = PNG_TEXT_COMPRESSION_NONE;
		title_text.key = "Title";
		title_text.text = title;
		png_set_text(png_ptr, info_ptr, &title_text, 1);
	}

	png_write_info(png_ptr, info_ptr);

	// Write image data
	int x, y;
	for (y=0 ; y<height ; y++) {
		png_write_row(png_ptr, (png_byte*)(buffer+y*width));
	}

	// End write
	png_write_end(png_ptr, NULL);

	finalise:
	if (fp != NULL) fclose(fp);
	if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
	if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);

	return code;
}

int writeImageRAW_RGB(char* filename, int width, int height, int num_channels, unsigned char *buffer, int compress){
	char buf[1024];
	int error_code=0;
	if(error_code=setup_pcl_dir()){
		sprintf(buf, "Could not open SDCARD path %s for writing: error code %d \n", pcl_path, error_code);
		__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);
		return error_code;
	}

	int code=0;
	FILE *fp;
	// Open file for writing (binary mode)
	char filepath[1024];
	sprintf(filepath,"%s/%s", pcl_path, filename);
	fp = fopen(filepath, "wb");
//	size_t len, len2;
//	unsigned char *src, *dst;

//	len = width*height*num_channels; //rgb
//	len2 = len; //without compression these two should be the same

	if (fp == NULL) {
		//fprintf(stderr, "Could not open file %s for writing\n", filename);
		sprintf(buf, "Could not open file %s for writing\n", filepath);
		__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);

		code = 1;
		goto finalise;
	}

//	if(compress){
//		src = buffer;
//		qlz_state_compress *state_compress = (qlz_state_compress *)malloc(sizeof(qlz_state_compress));
//		// allocate "uncompressed size" + 400 for the destination buffer
//		dst = (char*) malloc(len + 400);
//		//memset(buffer, 0, len);
//		// compress and write result
//		len2 = qlz_compress(src, dst, len, state_compress);
//		sprintf(buf, "COMPRESSOIN: %d to %d", len, len2);
//		__android_log_write(ANDROID_LOG_INFO, "RAW IMAGE TESTING:", buf);
//		free(state_compress);
//	}

	size_t out_size = fwrite(buffer, sizeof(unsigned char), width*height*num_channels, fp);

	//fprintf(stderr, "Could not open file %s for writing\n", filename);
	//sprintf(buf, "Image Size: %d, Output Image size: %d", width*height*3*sizeof(unsigned char), out_size);
	//__android_log_write(ANDROID_LOG_INFO, "RAW IMAGE TESTING:", buf);

	//focus I/O sync with the disk
	//fsync(fileno(fp));

	finalise:
	if (fp != NULL) fclose(fp);
//	if (dst != NULL) free(dst);

	return code;
}
int writeImageRGB(char* filename, int width, int height, unsigned char *buffer, char* title)
{
	char buf[1024];
	int error_code=0;
	if(error_code=setup_pcl_dir()){
		sprintf(buf, "Could not open SDCARD path %s for writing: error code %d \n", pcl_path, error_code);
		__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);
		return error_code;
	}

	int code = 0;
	FILE *fp;
	png_structp png_ptr;
	png_infop info_ptr;
	//png_bytep row;
	// Open file for writing (binary mode)
	// Open file for writing (binary mode)
	char filepath[1024];
	sprintf(filepath,"%s/%s", pcl_path, filename);
	fp = fopen(filepath, "wb");

	if (fp == NULL) {
		//fprintf(stderr, "Could not open file %s for writing\n", filename);
		sprintf(buf, "Could not open file %s for writing\n", filepath);
		__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);

		code = 1;
		goto finalise;
	}

	// Initialize write structure
	png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (png_ptr == NULL) {
		//fprintf(stderr, "Could not allocate write struct\n");
		sprintf(buf, "Could not allocate write struct\n");
		__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);

		code = 1;
		goto finalise;
	}

	// Initialize info structure
	info_ptr = png_create_info_struct(png_ptr);
	if (info_ptr == NULL) {
		//fprintf(stderr, "Could not allocate info struct\n");

		sprintf(buf, "Could not allocate info struct\n");
		__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);
		code = 1;
		goto finalise;
	}

	// Setup Exception handling
	if (setjmp(png_jmpbuf(png_ptr))) {
		//fprintf(stderr, "Error during png creation\n");

		sprintf(buf, "Error during png creation\n");
		__android_log_write(ANDROID_LOG_INFO, "PNG IMAGE TESTING:", buf);
		code = 1;
		goto finalise;
	}

	png_init_io(png_ptr, fp);

	// Write header (8 bit colour depth)
	png_set_IHDR(png_ptr, info_ptr, width, height,
			8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
			PNG_COMPRESSION_TYPE_DEFAULT, PNG_COMPRESSION_TYPE_DEFAULT);
	//fine tuned parameter for speed!
    png_set_filter(png_ptr, PNG_FILTER_TYPE_BASE, PNG_FILTER_SUB);
    png_set_compression_level(png_ptr, 1); //1 is Z_BEST_SPEED in zlib.h!
    png_set_compression_strategy(png_ptr, 3); //3 is Z_RLE


	// Set title
	if (title != NULL) {
		png_text title_text;
		title_text.compression = PNG_TEXT_COMPRESSION_NONE;
		title_text.key = "Title";
		title_text.text = title;
		png_set_text(png_ptr, info_ptr, &title_text, 1);
	}

	png_write_info(png_ptr, info_ptr);


	// Write image data
	int x, y;
	for (y=0 ; y<height ; y++) {
		png_write_row(png_ptr, (png_byte*)(buffer+y*width*3));
	}

	// End write
	png_write_end(png_ptr, NULL);

	finalise:
	if (fp != NULL) fclose(fp);
	if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
	if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);

	return code;
}
