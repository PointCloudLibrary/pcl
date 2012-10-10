#include "capturescr.h"


#include <android/log.h>

typedef struct {
     long filesize;
     char reserved[2];
     long headersize;
     long infoSize;
     long width;
     long depth;
     short biPlanes;
     short bits;
     long biCompression;
     long biSizeImage;
     long biXPelsPerMeter;
     long biYPelsPerMeter;
     long biClrUsed;
     long biClrImportant;
} BMPHEAD;

//copyright text
char cprght[255]="Copyright(C) 2006-2009 Teksoft SRL, (C)2009 Motisan Radu PFA, All rights reserved.\nwww.teksoftco.com";
//surface pointer
//handler
//v screen info
struct fb_var_screeninfo vi;
//f screen info
struct fb_fix_screeninfo fi;

void dumpinfo(struct fb_fix_screeninfo *fi,
                     struct fb_var_screeninfo *vi);

void dumpinfo(struct fb_fix_screeninfo *fi, struct fb_var_screeninfo *vi)
{
	char buf[512];
	sprintf(buf,"vi.xres = %d\n", vi->xres);
    __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
    sprintf(buf,"vi.yres = %d\n", vi->yres);
    __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
    sprintf(buf,"vi.xresv = %d\n", vi->xres_virtual);
    __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
    sprintf(buf,"vi.yresv = %d\n", vi->yres_virtual);
    __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
    sprintf(buf,"vi.xoff = %d\n", vi->xoffset);
    __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
    sprintf(buf,"vi.yoff = %d\n", vi->yoffset);
    __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
    sprintf(buf, "vi.bits_per_pixel = %d\n", vi->bits_per_pixel);
    __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
    sprintf(buf, "fi.line_length = %d\n", fi->line_length);
    __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
}
void get_buffer(unsigned char *buffer)
{
  GGLSurface fb_data[2];
  GGLSurface *fb = fb_data;
  //get screen capture
  int fd;
  void *bits;

  fd = open("/dev/graphics/fb0", O_RDWR);

  if(fd < 0) {
  	  char buf[512];
  	  sprintf(buf,"Cannot open %s\n", "/dev/graphics/fb0");
      __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
      close(fd);
      return;
  }

  if(ioctl(fd, FBIOGET_FSCREENINFO, &fi) < 0) {
      perror("failed to get fb0 info");
  	  char buf[512];
  	  sprintf(buf,"failed to get fb0 info\n");
      __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
      close(fd);
      return;
  }

  if(ioctl(fd, FBIOGET_VSCREENINFO, &vi) < 0) {
      perror("failed to get fb0 info");
  	  char buf[512];
  	  sprintf(buf,"failed to get fb0 info\n");
      __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
      close(fd);
      return;
  }
  /*D/ScreenFrameBuffer( 4793): vi.xres = 1366
D/ScreenFrameBuffer( 4793): vi.yres = 768
D/ScreenFrameBuffer( 4793): vi.xresv = 1366
D/ScreenFrameBuffer( 4793): vi.yresv = 1536
D/ScreenFrameBuffer( 4793): vi.xoff = 0
D/ScreenFrameBuffer( 4793): vi.yoff = 0
   */

  //dumpinfo(&fi, &vi);

  bits = mmap(0, fi.smem_len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  //char buf[512];
  //sprintf(buf,"Got bits? %d\n", bits);
  //__android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);

  if(bits == MAP_FAILED) {
  	 char buf[512];
  	 sprintf(buf,"failed to mmap\n");
     __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
     close(fd);
     return;
  }

  fb->version = sizeof(*fb);
  fb->width = vi.xres;
  fb->height = vi.yres;
  fb->stride = fi.line_length / (vi.bits_per_pixel);
  fb->data = bits;
  fb->format = GGL_PIXEL_FORMAT_RGBA_8888;

  fb++;

  fb->version = sizeof(*fb);
  fb->width = vi.xres;
  fb->height = vi.yres;
  fb->stride = fi.line_length / (vi.bits_per_pixel);
  fb->data = (void*) (((unsigned) bits) + vi.yres * vi.xres * 2);
  fb->format = GGL_PIXEL_FORMAT_RGBA_8888;

  int w = vi.xres, h = vi.yres, depth = vi.bits_per_pixel;
  //convert pixel data
  //uint8_t *rgb24;
  if (depth == 32)
  {
	  memcpy((unsigned char *) fb_data[0].data, buffer, vi.xres*vi.yres*4);
	  memcpy((unsigned char *) fb->data, buffer, vi.xres*vi.yres*4);
  }
  if(munmap (bits, fi.smem_len)){
	  char buf[512];
	  sprintf(buf,"munmap error %d\n", bits);
	  __android_log_print(ANDROID_LOG_DEBUG, "ScreenFrameBuffer", buf);
	  return;
  }
  close(fd);
  return;
}
