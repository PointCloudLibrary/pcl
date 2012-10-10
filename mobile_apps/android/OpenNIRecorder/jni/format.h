/*
 * Copyright (C) 2005 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ANDROID_PIXELFLINGER_FORMAT_H
#define ANDROID_PIXELFLINGER_FORMAT_H

#include <stdint.h>
#include <sys/types.h>

enum GGLPixelFormat {
    // these constants need to match those
    // in graphics/PixelFormat.java, ui/PixelFormat.h, BlitHardware.h
    GGL_PIXEL_FORMAT_UNKNOWN    =   0,
    GGL_PIXEL_FORMAT_NONE       =   0,

    GGL_PIXEL_FORMAT_RGBA_8888   =   1,  // 4x8-bit ARGB
    GGL_PIXEL_FORMAT_RGBX_8888   =   2,  // 3x8-bit RGB stored in 32-bit chunks
    GGL_PIXEL_FORMAT_RGB_888     =   3,  // 3x8-bit RGB
    GGL_PIXEL_FORMAT_RGB_565     =   4,  // 16-bit RGB
    GGL_PIXEL_FORMAT_RGBA_5551   =   6,  // 16-bit RGBA
    GGL_PIXEL_FORMAT_RGBA_4444   =   7,  // 16-bit RGBA

    GGL_PIXEL_FORMAT_A_8         =   8,  // 8-bit A
    GGL_PIXEL_FORMAT_L_8         =   9,  // 8-bit L (R=G=B = L)
    GGL_PIXEL_FORMAT_LA_88       = 0xA,  // 16-bit LA
    GGL_PIXEL_FORMAT_RGB_332     = 0xB,  // 8-bit RGB (non paletted)

    // YCbCr formats
    GGL_PIXEL_FORMAT_YCbCr_422_SP= 0x10,
    GGL_PIXEL_FORMAT_YCbCr_420_SP= 0x11,

    // reserved/special formats
    GGL_PIXEL_FORMAT_Z_16       =  0x18,
    GGL_PIXEL_FORMAT_S_8        =  0x19,
    GGL_PIXEL_FORMAT_SZ_24      =  0x1A,
    GGL_PIXEL_FORMAT_SZ_8       =  0x1B,
};

enum GGLFormatComponents {
	GGL_STENCIL_INDEX		= 0x1901,
	GGL_DEPTH_COMPONENT		= 0x1902,
	GGL_ALPHA				= 0x1906,
	GGL_RGB					= 0x1907,
	GGL_RGBA				= 0x1908,
	GGL_LUMINANCE			= 0x1909,
	GGL_LUMINANCE_ALPHA		= 0x190A,
	GGL_Y_CB_CR             = 0x8000,
};

enum GGLFormatComponentIndex {
    GGL_INDEX_ALPHA   = 0,
    GGL_INDEX_RED     = 1,
    GGL_INDEX_GREEN   = 2,
    GGL_INDEX_BLUE    = 3,
    GGL_INDEX_STENCIL = 0,
    GGL_INDEX_DEPTH   = 1,
    GGL_INDEX_Y       = 0,
    GGL_INDEX_CB      = 1,
    GGL_INDEX_CR      = 2,
};

typedef struct {
#ifdef __cplusplus
    enum {
        ALPHA   = GGL_INDEX_ALPHA,
        RED     = GGL_INDEX_RED,
        GREEN   = GGL_INDEX_GREEN,
        BLUE    = GGL_INDEX_BLUE,
        STENCIL = GGL_INDEX_STENCIL,
        DEPTH   = GGL_INDEX_DEPTH,
        LUMA    = GGL_INDEX_Y,
        CHROMAB = GGL_INDEX_CB,
        CHROMAR = GGL_INDEX_CR,
    };
    inline uint32_t mask(int i) const {
            return ((1<<(c[i].h-c[i].l))-1)<<c[i].l;
    }
    inline uint32_t bits(int i) const {
            return c[i].h - c[i].l;
    }
#endif
	uint8_t     size;	// bytes per pixel
    uint8_t     bitsPerPixel;
    union {    
        struct {
            uint8_t     ah;		// alpha high bit position + 1
            uint8_t     al;		// alpha low bit position
            uint8_t     rh;		// red high bit position + 1
            uint8_t     rl;		// red low bit position
            uint8_t     gh;		// green high bit position + 1
            uint8_t     gl;		// green low bit position
            uint8_t     bh;		// blue high bit position + 1
            uint8_t     bl;		// blue low bit position
        };
        struct {
            uint8_t h;
            uint8_t l;
        } __attribute__((__packed__)) c[4];        
    } __attribute__((__packed__));
	uint16_t    components;	// GGLFormatComponents
} GGLFormat;


#ifdef __cplusplus
extern "C" const GGLFormat* gglGetPixelFormatTable(size_t* numEntries = 0);
#else
const GGLFormat* gglGetPixelFormatTable(size_t* numEntries);
#endif


// ----------------------------------------------------------------------------

#endif // ANDROID_PIXELFLINGER_FORMAT_H
