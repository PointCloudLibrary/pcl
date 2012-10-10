/*
 * Copyright (C) 2007 The Android Open Source Project
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

#ifndef ANDROID_PIXELFLINGER_H
#define ANDROID_PIXELFLINGER_H

#include <stdint.h>
#include <sys/types.h>

#include "format.h"

// GGL types

typedef int8_t			GGLbyte;		// b
typedef int16_t			GGLshort;		// s
typedef int32_t			GGLint;			// i
typedef ssize_t			GGLsizei;		// i
typedef int32_t			GGLfixed;		// x
typedef int32_t			GGLclampx;		// x
typedef float			GGLfloat;		// f
typedef float			GGLclampf;		// f
typedef double			GGLdouble;		// d
typedef double			GGLclampd;		// d
typedef uint8_t			GGLubyte;		// ub
typedef uint8_t			GGLboolean;		// ub
typedef uint16_t		GGLushort;		// us
typedef uint32_t		GGLuint;		// ui
typedef unsigned int	GGLenum;		// ui
typedef unsigned int	GGLbitfield;	// ui
typedef void			GGLvoid;
typedef int32_t         GGLfixed32;
typedef	int32_t         GGLcolor;
typedef int32_t         GGLcoord;

// ----------------------------------------------------------------------------

#define GGL_MAX_VIEWPORT_DIMS           4096
#define GGL_MAX_TEXTURE_SIZE            4096
#define GGL_MAX_ALIASED_POINT_SIZE      0x7FFFFFF
#define GGL_MAX_SMOOTH_POINT_SIZE       2048
#define GGL_MAX_SMOOTH_LINE_WIDTH       2048

// ----------------------------------------------------------------------------

// All these names are compatible with their OpenGL equivalents
// some of them are listed only for completeness
enum GGLNames {
	GGL_FALSE						= 0,
	GGL_TRUE						= 1,

	// enable/disable
    GGL_SCISSOR_TEST                = 0x0C11,
	GGL_TEXTURE_2D					= 0x0DE1,
	GGL_ALPHA_TEST					= 0x0BC0,
	GGL_BLEND						= 0x0BE2,
	GGL_COLOR_LOGIC_OP				= 0x0BF2,
	GGL_DITHER						= 0x0BD0,
	GGL_STENCIL_TEST				= 0x0B90,
	GGL_DEPTH_TEST					= 0x0B71,
    GGL_AA                          = 0x80000001,
    GGL_W_LERP                      = 0x80000004,
    GGL_POINT_SMOOTH_NICE           = 0x80000005,

    // buffers, pixel drawing/reading
    GGL_COLOR                       = 0x1800,
    
    // fog
    GGL_FOG                         = 0x0B60,
    
	// shade model
	GGL_FLAT						= 0x1D00,
	GGL_SMOOTH						= 0x1D01,

	// Texture parameter name
	GGL_TEXTURE_MIN_FILTER			= 0x2801,
	GGL_TEXTURE_MAG_FILTER			= 0x2800,
	GGL_TEXTURE_WRAP_S				= 0x2802,
	GGL_TEXTURE_WRAP_T				= 0x2803,
	GGL_TEXTURE_WRAP_R				= 0x2804,

	// Texture Filter	
	GGL_NEAREST						= 0x2600,
	GGL_LINEAR						= 0x2601,
	GGL_NEAREST_MIPMAP_NEAREST		= 0x2700,
	GGL_LINEAR_MIPMAP_NEAREST		= 0x2701,
	GGL_NEAREST_MIPMAP_LINEAR		= 0x2702,
	GGL_LINEAR_MIPMAP_LINEAR		= 0x2703,

	// Texture Wrap Mode
	GGL_CLAMP						= 0x2900,
	GGL_REPEAT						= 0x2901,
    GGL_CLAMP_TO_EDGE               = 0x812F,

	// Texture Env Mode
	GGL_REPLACE						= 0x1E01,
	GGL_MODULATE					= 0x2100,
	GGL_DECAL						= 0x2101,
	GGL_ADD							= 0x0104,

	// Texture Env Parameter
	GGL_TEXTURE_ENV_MODE			= 0x2200,
	GGL_TEXTURE_ENV_COLOR			= 0x2201,

	// Texture Env Target
	GGL_TEXTURE_ENV					= 0x2300,

    // Texture coord generation
    GGL_TEXTURE_GEN_MODE            = 0x2500,
    GGL_S                           = 0x2000,
    GGL_T                           = 0x2001,
    GGL_R                           = 0x2002,
    GGL_Q                           = 0x2003,
    GGL_ONE_TO_ONE                  = 0x80000002,
    GGL_AUTOMATIC                   = 0x80000003,

    // AlphaFunction
    GGL_NEVER                       = 0x0200,
    GGL_LESS                        = 0x0201,
    GGL_EQUAL                       = 0x0202,
    GGL_LEQUAL                      = 0x0203,
    GGL_GREATER                     = 0x0204,
    GGL_NOTEQUAL                    = 0x0205,
    GGL_GEQUAL                      = 0x0206,
    GGL_ALWAYS                      = 0x0207,

    // LogicOp
    GGL_CLEAR                       = 0x1500,   // 0
    GGL_AND                         = 0x1501,   // s & d
    GGL_AND_REVERSE                 = 0x1502,   // s & ~d
    GGL_COPY                        = 0x1503,   // s
    GGL_AND_INVERTED                = 0x1504,   // ~s & d
    GGL_NOOP                        = 0x1505,   // d
    GGL_XOR                         = 0x1506,   // s ^ d
    GGL_OR                          = 0x1507,   // s | d
    GGL_NOR                         = 0x1508,   // ~(s | d)
    GGL_EQUIV                       = 0x1509,   // ~(s ^ d)
    GGL_INVERT                      = 0x150A,   // ~d
    GGL_OR_REVERSE                  = 0x150B,   // s | ~d
    GGL_COPY_INVERTED               = 0x150C,   // ~s 
    GGL_OR_INVERTED                 = 0x150D,   // ~s | d
    GGL_NAND                        = 0x150E,   // ~(s & d)
    GGL_SET                         = 0x150F,   // 1

	// blending equation & function
	GGL_ZERO                        = 0,		// SD
	GGL_ONE                         = 1,		// SD
	GGL_SRC_COLOR                   = 0x0300,	//  D
	GGL_ONE_MINUS_SRC_COLOR         = 0x0301,	//	D
	GGL_SRC_ALPHA                   = 0x0302,	// SD
	GGL_ONE_MINUS_SRC_ALPHA			= 0x0303,	// SD
	GGL_DST_ALPHA					= 0x0304,	// SD
	GGL_ONE_MINUS_DST_ALPHA			= 0x0305,	// SD
	GGL_DST_COLOR					= 0x0306,	// S
	GGL_ONE_MINUS_DST_COLOR			= 0x0307,	// S
	GGL_SRC_ALPHA_SATURATE			= 0x0308,	// S
    
    // clear bits
    GGL_DEPTH_BUFFER_BIT            = 0x00000100,
    GGL_STENCIL_BUFFER_BIT          = 0x00000400,
    GGL_COLOR_BUFFER_BIT            = 0x00004000,

    // errors
    GGL_NO_ERROR                    = 0,
    GGL_INVALID_ENUM                = 0x0500,
    GGL_INVALID_VALUE               = 0x0501,
    GGL_INVALID_OPERATION           = 0x0502,
    GGL_STACK_OVERFLOW              = 0x0503,
    GGL_STACK_UNDERFLOW             = 0x0504,
    GGL_OUT_OF_MEMORY               = 0x0505
};

// ----------------------------------------------------------------------------

typedef struct {
    GGLsizei    version;    // always set to sizeof(GGLSurface)
    GGLuint     width;      // width in pixels
    GGLuint     height;     // height in pixels
    GGLint      stride;     // stride in pixels
    GGLubyte*   data;       // pointer to the bits
    GGLubyte    format;     // pixel format
    GGLubyte    rfu[3];     // must be zero
    // these values are dependent on the used format
    union {
        GGLint  compressedFormat;
        GGLint  vstride;
    };
    void*       reserved;
} GGLSurface;


typedef struct {
    // immediate rendering
    void (*pointx)(void *con, const GGLcoord* v, GGLcoord r);
    void (*linex)(void *con, 
            const GGLcoord* v0, const GGLcoord* v1, GGLcoord width);
    void (*recti)(void* c, GGLint l, GGLint t, GGLint r, GGLint b); 
    void (*trianglex)(void* c,
            GGLcoord const* v0, GGLcoord const* v1, GGLcoord const* v2);

    // scissor
    void (*scissor)(void* c, GGLint x, GGLint y, GGLsizei width, GGLsizei height);

    // Set the textures and color buffers
    void (*activeTexture)(void* c, GGLuint tmu);
    void (*bindTexture)(void* c, const GGLSurface* surface);
    void (*colorBuffer)(void* c, const GGLSurface* surface);
    void (*readBuffer)(void* c, const GGLSurface* surface);
    void (*depthBuffer)(void* c, const GGLSurface* surface);
    void (*bindTextureLod)(void* c, GGLuint tmu, const GGLSurface* surface);

    // enable/disable features
    void (*enable)(void* c, GGLenum name);
    void (*disable)(void* c, GGLenum name);
    void (*enableDisable)(void* c, GGLenum name, GGLboolean en);

    // specify the fragment's color
    void (*shadeModel)(void* c, GGLenum mode);
    void (*color4xv)(void* c, const GGLclampx* color);
    // specify color iterators (16.16)
    void (*colorGrad12xv)(void* c, const GGLcolor* grad);

    // specify Z coordinate iterators (0.32)
    void (*zGrad3xv)(void* c, const GGLfixed32* grad);

    // specify W coordinate iterators (16.16)
    void (*wGrad3xv)(void* c, const GGLfixed* grad);

    // specify fog iterator & color (16.16)
    void (*fogGrad3xv)(void* c, const GGLfixed* grad);
    void (*fogColor3xv)(void* c, const GGLclampx* color);

    // specify blending parameters
    void (*blendFunc)(void* c, GGLenum src, GGLenum dst);
    void (*blendFuncSeparate)(void* c,  GGLenum src, GGLenum dst,
                                        GGLenum srcAlpha, GGLenum dstAplha);

    // texture environnement (REPLACE / MODULATE / DECAL / BLEND)
    void (*texEnvi)(void* c,    GGLenum target,
                                GGLenum pname,
                                GGLint param);

    void (*texEnvxv)(void* c, GGLenum target,
            GGLenum pname, const GGLfixed* params);

    // texture parameters (Wrapping, filter)
    void (*texParameteri)(void* c,  GGLenum target,
                                    GGLenum pname,
                                    GGLint param);

    // texture iterators (16.16)
    void (*texCoord2i)(void* c, GGLint s, GGLint t);
    void (*texCoord2x)(void* c, GGLfixed s, GGLfixed t);
    
    // s, dsdx, dsdy, scale, t, dtdx, dtdy, tscale
    // This api uses block floating-point for S and T texture coordinates.
    // All values are given in 16.16, scaled by 'scale'. In other words,
    // set scale to 0, for 16.16 values.
    void (*texCoordGradScale8xv)(void* c, GGLint tmu, const int32_t* grad8);
    
    void (*texGeni)(void* c, GGLenum coord, GGLenum pname, GGLint param);

    // masking
    void (*colorMask)(void* c,  GGLboolean red,
                                GGLboolean green,
                                GGLboolean blue,
                                GGLboolean alpha);

    void (*depthMask)(void* c, GGLboolean flag);

    void (*stencilMask)(void* c, GGLuint mask);

    // alpha func
    void (*alphaFuncx)(void* c, GGLenum func, GGLclampx ref);

    // depth func
    void (*depthFunc)(void* c, GGLenum func);

    // logic op
    void (*logicOp)(void* c, GGLenum opcode); 

    // clear
    void (*clear)(void* c, GGLbitfield mask);
    void (*clearColorx)(void* c,
            GGLclampx r, GGLclampx g, GGLclampx b, GGLclampx a);
    void (*clearDepthx)(void* c, GGLclampx depth);
    void (*clearStencil)(void* c, GGLint s);

    // framebuffer operations
    void (*copyPixels)(void* c, GGLint x, GGLint y,
            GGLsizei width, GGLsizei height, GGLenum type);
    void (*rasterPos2x)(void* c, GGLfixed x, GGLfixed y);
    void (*rasterPos2i)(void* c, GGLint x, GGLint y);
} GGLContext;

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

// construct / destroy the context
ssize_t gglInit(GGLContext** context);
ssize_t gglUninit(GGLContext* context);

GGLint gglBitBlti(
        GGLContext* c,
        int tmu,
        GGLint crop[4],
        GGLint where[4]);

#ifdef __cplusplus
};
#endif

// ----------------------------------------------------------------------------

#endif // ANDROID_PIXELFLINGER_H

