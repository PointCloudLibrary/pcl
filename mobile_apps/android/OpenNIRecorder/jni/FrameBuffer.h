//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

#ifndef __INCLUDED_PHOTOVIEWER_FRAMEBUFFER_H
#define __INCLUDED_PHOTOVIEWER_FRAMEBUFFER_H

#include <GLES2/gl2.h>

class FrameBuffer {
public:
    FrameBuffer(int width=800, int height=480, bool useStencil = false);
    ~FrameBuffer();

    GLuint getTextureId();
    GLuint getFrameBufferId();

    int getWidth();
    int getHeight();

    void release();

    int inUse;

private:
    int width;
    int height;
    GLuint textureId;
    GLuint framebufferId;
};

#endif
