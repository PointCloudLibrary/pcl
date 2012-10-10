//----------------------------------------------------------------------------------
//
//----------------------------------------------------------------------------------

#include "FrameBuffer.h"

FrameBuffer::FrameBuffer(int w, int h, bool useStencil) : inUse(0), width(w), height(h)
{
	const GLuint MY_PIXEL_FORMAT = GL_RGBA;
	const GLuint MY_PIXEL_TYPE = GL_UNSIGNED_BYTE;

    glGenTextures(1, &textureId);
    glGenFramebuffers(1, &framebufferId);

    GLint curr_fbo;
    GLint curr_stencil;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &curr_fbo);

    glBindFramebuffer(GL_FRAMEBUFFER, framebufferId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexImage2D(GL_TEXTURE_2D, 0, MY_PIXEL_FORMAT, width, height, 0,
    		MY_PIXEL_FORMAT, MY_PIXEL_TYPE, 0);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureId, 0);

    if (useStencil)
    {
        GLuint stencil_rb;
        glGetIntegerv(GL_RENDERBUFFER, &curr_stencil);

        glGenRenderbuffers(1, &stencil_rb);
        glBindRenderbuffer(GL_RENDERBUFFER, stencil_rb);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_STENCIL_INDEX8, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, stencil_rb);
        glBindRenderbuffer(GL_RENDERBUFFER, curr_stencil);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, curr_fbo);
}

FrameBuffer::~FrameBuffer()
{
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteTextures(1, &textureId);
    glDeleteFramebuffers(1, &framebufferId);
}

int FrameBuffer::getWidth()
{
    return width;
}

int FrameBuffer::getHeight()
{
    return height;
}

GLuint FrameBuffer::getTextureId()
{
    return textureId;
}

GLuint FrameBuffer::getFrameBufferId()
{
    return framebufferId;
}


void FrameBuffer::release()
{
    inUse = 0;
}
