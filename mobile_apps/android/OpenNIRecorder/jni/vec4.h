//
// ==[ XPGL: eXPerimental Graphics Library ]== 
//
// Copyright 2006 JeGX / oZone3D.Net
// http://www.oZone3D.Net - jegx@ozone3d.net
//
// This SOFTWARE is distributed in the hope that it will be useful.
// TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW, THIS SOFTWARE IS PROVIDED
// *AS IS* AND oZone3D.Net DISCLAIM ALL WARRANTIES, EITHER EXPRESS
// OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY
// AND FITNESS FOR A PARTICULAR PURPOSE.  IN NO EVENT SHALL oZone3D.Net 
// BE LIABLE FOR ANY SPECIAL, INCIDENTAL, INDIRECT, OR CONSEQUENTIAL DAMAGES
// WHATSOEVER (INCLUDING, WITHOUT LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS,
// BUSINESS INTERRUPTION, LOSS OF BUSINESS INFORMATION, OR ANY OTHER PECUNIARY LOSS)
// ARISING OUT OF THE USE OF OR INABILITY TO USE THIS SOFTWARE, EVEN IF oZone3D.Net HAS
// BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
//


#ifndef _VEC4_H
#define _VEC4_H

//===================================================================
/**
 Helper utils
*/
//===================================================================

#ifndef NULL
	#define NULL 0
#endif

#define _PI				3.1415926535897932384626433832795f
#define _HALF_PI			1.5707963267948966192313216916398f
#define _PI_OVER_180		1.74532925199432957692369076848e-2f
#define _PI_OVER_360		0.0087266462599716478846184538424431f
#define _180_OVER_PI		57.295779513082320876798154814105f
#define _EPSILON			10e-6f
#define _BIG_FLOAT		1e30f

#define SAFE_DELETE( ptr ) \
	if(ptr) \
	{ \
		delete ptr; \
		ptr = NULL; \
	} \

#define SAFE_DELETE_ARRAY( ptr ) \
	if(ptr) \
	{ \
		delete [] ptr; \
		ptr = NULL; \
	} \


//===================================================================
/**
cVec4: a 4D vector class with floating point values.
*/
//===================================================================
class cVec4
{

public:

	union
	{
		struct
		{
			float x;
			float y;
			float z;
			float w;
		};
		
		float v[4];
	};


	inline cVec4()
		: x(0.0), y(0.0), z(0.0), w(1.0)
	{
	
	};

	inline cVec4( float _x, float _y, float _z, float _w=1.0 )
		: x(_x), y(_y), z(_z), w(_w)
	{

	};
	

	inline void zero()
	{
		x = y = z = w = 0.0; 
	};


	inline void set( float _x, float _y, float _z, float _w=1.0 )
	{
		x = _x, y = _y, z = _z,  w = _w;
	};

	inline cVec4& operator=(const cVec4 &vec)
	{
		x = vec.x;
		y = vec.y;
		z = vec.z;
		w = vec.w;
		return *this;
	}

	inline float dot( const cVec4 &vec )
	{
		return( x*vec.x + y*vec.y + z*vec.z );
	};

	inline float dot( cVec4 *vec )
	{
		return( x*vec->x + y*vec->y + z*vec->z );
	};

	inline cVec4 cross( const cVec4 &vec )
	{
		return( cVec4( y*vec.z - z*vec.y,
					   z*vec.x - x*vec.z,
					   x*vec.y - y*vec.x ) );
	};

	inline cVec4 cross(  cVec4 *vec )
	{
		return( cVec4( y*vec->z - z*vec->y,
					   z*vec->x - x*vec->z,
					   x*vec->y - y*vec->x ) );
	};

	inline void scalarMult( float scalar )
	{
		x *= scalar;
		y *= scalar;
		z *= scalar;
	};

	inline void normalize()
	{
		float norme;
		float root;
		
		norme = x*x + y*y + z*z;
		if( norme<0.0001 )
		{
			x = 0.0;
			y = 0.0;
			z = 0.0;
			return;
		}

		root =(float)( 1.0/(float)sqrt(norme) );
		x *= root;
		y *= root;
		z *= root;
	};


	inline float length()
	{
		return( (float)sqrt(x*x + y*y + z*z) );
	};


	inline void rotateX( float angle )
	{
		float theta = angle*_PI_OVER_180;
		float yTmp, zTmp;
		yTmp = y*(float)cos(theta) - z*(float)sin(theta);
		zTmp = y*(float)sin(theta) + z*(float)cos(theta);
		y=yTmp;
		z=zTmp;
	}

	inline void rotateY( float angle )
	{
		float theta = angle*_PI_OVER_180;
		float xTmp, zTmp;
		xTmp = x*(float)cos(theta) + z*(float)sin(theta);
		zTmp = -x*(float)sin(theta) + z*(float)cos(theta);
		x=xTmp;
		z=zTmp;
	}

	inline void rotateZ( float angle )
	{
		float theta = angle*_PI_OVER_180;
		float xTmp, yTmp;
		xTmp = x*(float)cos(theta) - y*(float)sin(theta);
		yTmp = x*(float)sin(theta) + y*(float)cos(theta);
		x = xTmp;
		y = yTmp;
	};

	
	inline cVec4 operator += ( const cVec4 &vec )
	{
		x += vec.x;
		y += vec.y;
		z += vec.z;
		return *this;
	};

	inline cVec4 operator -= ( const cVec4 &vec )
	{
		x -= vec.x;
		y -= vec.y;
		z -= vec.z;
		return *this;
	};

};

//--------------------------------------------
// Basic operations with overloaded operators.
//--------------------------------------------

// Vector addition: vResult = vVec1 + vVec2
inline cVec4 operator + ( const cVec4 &vec1, const cVec4 &vec2 )
{
	return cVec4( vec1.x + vec2.x,
 				  vec1.y + vec2.y, 
				  vec1.z + vec2.z );
}

// Vector substraction:  vResult = vec1 - vec2
inline cVec4 operator - ( const cVec4 &vec1, const cVec4 &vec2 )
{
	return cVec4( vec1.x - vec2.x,
 		 		  vec1.y - vec2.y, 
				  vec1.z - vec2.z );
}

// Scalar multiplication: vResult = vVec * scalar
inline cVec4 operator * ( const cVec4 &vec, const float scalar )
{
	return cVec4( vec.x * scalar,
 				  vec.y * scalar, 
				  vec.z * scalar );
}

// Scalar multiplication: vResult = vVec * scalar
inline cVec4 operator * ( const float scalar, const cVec4 &vec )
{
	return cVec4( vec.x * scalar,
 				  vec.y * scalar, 
				  vec.z * scalar );
}

// Dot product: result = vec1 DOT vec2
inline float operator * ( const cVec4 &vec1, const cVec4 &vec2 )
{
	return( vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z );
}

// Cross product: vResult = vec1 CROSS vec2
inline cVec4 operator ^ ( const cVec4 &vec1, const cVec4 &vec2 )
{
	return cVec4(  vec1.y*vec2.z - vec1.z*vec2.y,
 				  -vec1.x*vec2.z + vec1.z*vec2.x, 
				   vec1.x*vec2.y - vec1.y*vec2.x );
}


//===================================================================
/**
cVec4i: a 4D vector class with integer values.
*/
//===================================================================
class cVec4i
{

public:

	union
	{
		struct
		{
			int x;
			int y;
			int z;
			int w;
		};
		
		int v[4];
	};

	inline cVec4i()
		: x(0), y(0), z(0), w(1)
	{
	
	};

	inline cVec4i( int _x, int _y, int _z, int _w=1 )
		: x(_x), y(_y), z(_z), w(_w)
	{

	};

	inline void zero()
	{
		x = y = z = w = 0; 
	};


	inline void set( int _x, int _y, int _z, int _w=1 )
	{
		x = _x, y = _y, z = _z,  w = _w;
	};

	inline cVec4i operator = ( const cVec4i &vec )
	{
		x = vec.x;
		y = vec.y;
		z = vec.z;
		w = vec.w;
		return *this;
	}
};








#endif
