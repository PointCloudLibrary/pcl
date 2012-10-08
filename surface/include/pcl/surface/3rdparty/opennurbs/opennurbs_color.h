/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2012 Robert McNeel & Associates. All rights reserved.
// OpenNURBS, Rhinoceros, and Rhino3D are registered trademarks of Robert
// McNeel & Associates.
//
// THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY.
// ALL IMPLIED WARRANTIES OF FITNESS FOR ANY PARTICULAR PURPOSE AND OF
// MERCHANTABILITY ARE HEREBY DISCLAIMED.
//				
// For complete openNURBS copyright information see <http://www.opennurbs.org>.
//
////////////////////////////////////////////////////////////////
*/

#if !defined(OPENNURBS_COLOR_INC_)
#define OPENNURBS_COLOR_INC_

///////////////////////////////////////////////////////////////////////////////
//
// Class ON_Color
// 
class ON_CLASS ON_Color
{
public:
	// Constructors & Conversions -     also default copy and assignment	

  static const ON_Color UnsetColor; // 0xFFFFFFFF

  // Default is R = 0, G = 0, B = 0, A = 0
	ON_Color();

  // Sets A = 0
	ON_Color(
    int red,   // ( 0 to 255 )
    int green, // ( 0 to 255 )
    int blue   // ( 0 to 255 )
    );

	ON_Color(
    int red,   // ( 0 to 255 )
    int green, // ( 0 to 255 )
    int blue,  // ( 0 to 255 )
    int alpha  // ( 0 to 255 )  (0 = opaque, 255 = transparent)
    );

  // Construct from Windows COLORREF
	ON_Color(unsigned int);

	// Conversion to Windows COLORREF
  operator unsigned int() const;	

  /*
  Description:
    Call this function when the color is needed in a 
    Windows COLORREF format with alpha = 0;
  Returns
    A Windows COLOREF with alpha = 0.
  */
  unsigned int WindowsRGB() const;

  // < 0 if this < arg, 0 ir this==arg, > 0 if this > arg
  int Compare( const ON_Color& ) const; 

	int Red()   const; // ( 0 to 255 )
	int Green() const; // ( 0 to 255 )
	int Blue()  const; // ( 0 to 255 )
  int Alpha() const; // ( 0 to 255 ) (0 = opaque, 255 = transparent)

	double FractionRed()   const; // ( 0.0 to 1.0 )
	double FractionGreen() const; // ( 0.0 to 1.0 )
	double FractionBlue()  const; // ( 0.0 to 1.0 )
	double FractionAlpha() const; // ( 0.0 to 1.0 ) (0.0 = opaque, 1.0 = transparent)

  void SetRGB(
    int red,   // red in range 0 to 255
    int green, // green in range 0 to 255
    int blue   // blue in range 0 to 255
    );

  void SetFractionalRGB(
    double red,   // red in range 0.0 to 1.0
    double green, // green in range 0.0 to 1.0
    double blue   // blue in range 0.0 to 1.0
    );

  void SetAlpha(
    int alpha // alpha in range 0 to 255 (0 = opaque, 255 = transparent)
    );

  void SetFractionalAlpha(
    double alpha // alpha in range 0.0 to 1.0 (0.0 = opaque, 1.0 = transparent)
    );

  void SetRGBA(
    int red,   // red in range 0 to 255
    int green, // green in range 0 to 255
    int blue,  // blue in range 0 to 255
    int alpha  // alpha in range 0 to 255 (0 = opaque, 255 = transparent)
    );

  // input args
  void SetFractionalRGBA(
    double red,   // red in range 0.0 to 1.0
    double green, // green in range 0.0 to 1.0
    double blue,  // blue in range 0.0 to 1.0
    double alpha  // alpha in range 0.0 to 1.0 (0.0 = opaque, 1.0 = transparent)
    );

  // Hue() returns an angle in the range 0 to 2*pi 
  //
  //           0 = red, pi/3 = yellow, 2*pi/3 = green, 
  //           pi = cyan, 4*pi/3 = blue,5*pi/3 = magenta,
  //           2*pi = red
  double Hue() const;

  // Returns 0.0 (gray) to 1.0 (saturated)
  double Saturation() const;

  // Returns 0.0 (black) to 1.0 (white)
  double Value() const;

  void SetHSV( 
         double h, // hue in radians 0 to 2*pi
         double s, // satuation 0.0 = gray, 1.0 = saturated
         double v // value     
         );

private:
  // m_color is in Windows COLORREF format.
  //
  //  0xaabbggrr,  rr= red component 0-255, etc. (little endian order)
  //               aa=0 means opaque, aa=255 means transparent.
	unsigned int m_color;
};

#endif
