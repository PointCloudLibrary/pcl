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

#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"

const ON_Color ON_Color::UnsetColor(ON_UNSET_COLOR);

ON_Color::ON_Color() : m_color(0) 
{}

ON_Color::ON_Color(unsigned int colorref) : m_color(colorref) 
{}

ON_Color::ON_Color(int r, int g, int b) : m_color(0) 
{
  SetRGB(r,g,b);
}

ON_Color::ON_Color(int r, int g, int b, int a) : m_color(0) 
{
  SetRGBA(r,g,b,a);
}

unsigned int ON_Color::WindowsRGB() const
{
  unsigned int RGB = ON_Color(Red(),Green(),Blue());
  return RGB;
}

ON_Color::operator unsigned int() const
{
  return m_color;
}

int ON_Color::Compare( const ON_Color& b ) const
{
  unsigned int bc = b;
  return (((int)m_color) - ((int)bc));
}

int ON_Color::Red() const
{ return m_color & 0xFF;}

int ON_Color::Green() const
{ return (m_color>>8) & 0xFF;}

int ON_Color::Blue() const
{ return (m_color>>16) & 0xFF;}

int ON_Color::Alpha() const
{ return (m_color>>24) & 0xFF;}

double ON_Color::FractionRed() const
{ 
  //return Red()/255.0;
  return (m_color & 0xFF)*0.003921568627450980392156862745; // better fodder for optimizer
}

double ON_Color::FractionGreen() const       
{ 
  //return Green()/255.0;
  return ((m_color>>8) & 0xFF)*0.003921568627450980392156862745; // better fodder for optimizer
}

double ON_Color::FractionBlue() const       
{ 
  //return Blue()/255.0;
  return ((m_color>>16) & 0xFF)*0.003921568627450980392156862745; // better fodder for optimizer
}

double ON_Color::FractionAlpha() const       
{ 
  //return Alpha()/255.0;
  return ((m_color>>24) & 0xFF)*0.003921568627450980392156862745; // better fodder for optimizer
}

void ON_Color::SetRGB(int r,int g,int b) // 0 to 255
{
  SetRGBA(r,g,b,0);
}

void ON_Color::SetFractionalRGB(double r,double g,double b)
{
  SetFractionalRGBA(r,g,b,0.0);
}

void ON_Color::SetAlpha(int alpha)
{
	if (alpha < 0 ) alpha = 0; else if ( alpha > 255 ) alpha = 255;	
	m_color = (m_color & 0x00FFFFFF) | (alpha << 24 );
}

void ON_Color::SetFractionalAlpha(double alpha)
{
	if (alpha < 0.0 ) alpha = 0.0; else if ( alpha > 1.0 ) alpha = 1.0;	
  SetAlpha((int)(alpha*255.0));
}

void
ON_Color::SetRGBA( int red, int green, int blue, int alpha )
{
	if (red   < 0 ) red   = 0; else if ( red   > 255 ) red   = 255;	
	if (green < 0 ) green = 0; else if ( green > 255 ) green = 255;	
	if (blue  < 0 ) blue  = 0; else if ( blue  > 255 ) blue  = 255;	
	if (alpha < 0 ) alpha = 0; else if ( alpha > 255 ) alpha = 255;	
	m_color = (alpha << 24 ) | (blue << 16) | (green << 8) | red;
}

void
ON_Color::SetFractionalRGBA( double red, double green, double blue, double alpha )
{
  int r,g,b,a;
	if (red   < 0.0 ) red   = 0.0; else if ( red   > 1.0 ) red   = 1.0;	
	if (green < 0.0 ) green = 0.0; else if ( green > 1.0 ) green = 1.0;	
	if (blue  < 0.0 ) blue  = 0.0; else if ( blue  > 1.0 ) blue  = 1.0;	
	if (alpha < 0.0 ) alpha = 0.0; else if ( alpha > 1.0 ) alpha = 1.0;

  red   *= 255.0;
  green *= 255.0;
  blue  *= 255.0;
  alpha *= 255.0;

  r = (int)red;
  g = (int)green;
  b = (int)blue;
  a = (int)alpha;

  // round to closest int
  if( (red-r)>=0.5 ) r++;
  if( (green-g)>=0.5 ) g++;
  if( (blue-b)>=0.5 ) b++;
  if( (alpha-a)>=0.5 ) a++;

  SetRGBA( r, g, b, a );
}

double ON_Color::Hue() const
{
  // returns 0 to 2*pi 
  // 0    = red,  pi/3   = yellow, 2*pi/3 = green, 
  // pi   = cyan, 4*pi/3 = blue,   5*pi/3 = magenta,
  // 2*pi = red
  double h;
  int r = Red();
  int g = Green();
  int b = Blue();
  int minrgb, maxrgb;
  if ( r <= g ) {minrgb = r; maxrgb = g;} else {minrgb = g; maxrgb = r;}
  if (minrgb > b) minrgb = b; else if (maxrgb < b ) maxrgb = b;
  if ( maxrgb != minrgb ) {
    double d = 1.0/(maxrgb - minrgb);
    if ( r == maxrgb) {
      h = (g - b)*d;
      if ( h < 0.0 )
        h += 6.0;
    }
    else if ( g == maxrgb)
      h = 2.0 + (b - r)*d;
    else 
      h = 4.0 + (r - g)*d;
    h *= ON_PI/3.0;
  }
  else
    h = 0.0;
  return h;
}

double ON_Color::Saturation() const
{
  // 0.0 to 1.0    0.0 = gray,  1.0 = saturated
  double s;
  int r = Red();
  int g = Green();
  int b = Blue();
  int minrgb, maxrgb;
  if ( r <= g ) {minrgb = r; maxrgb = g;} else {minrgb = g; maxrgb = r;}
  if (minrgb > b) minrgb = b; else if (maxrgb < b ) maxrgb = b;
  if ( maxrgb > 0 ) {
    s = ((double)(maxrgb - minrgb))/((double)maxrgb);
  }
  else
    s = 0.0;
  return s;
}

double ON_Color::Value() const
{
  // 0.0 to 1.0    0.0 = black, 1.0 = white
  int r = Red();
  int g = Green();
  int b = Blue();
  int maxrgb = ( r <= g ) ? g : r; if ( maxrgb < b ) maxrgb = b;
  return (maxrgb/255.0);
}

void ON_Color::SetHSV( 
       double hue,         // hue in radians 
       double saturation, // satuation 0.0 = gray, 1.0 = saturated
       double value       // value     
       )
{
  int i;
  double f, p, q, t, r, g, b;
  if ( saturation <= 1.0/256.0 ) {
    r = value;
    g = value;
    b = value;
  }
  else  {
    hue *= 3.0 / ON_PI;  // (6.0 / 2.0 * ON_PI);
    i = (int)floor(hue);
    if ( i < 0 || i > 5 ) {
      hue = fmod(hue,6.0);
      if ( hue < 0.0 )
        hue += 6.0;
      i = (int)floor(hue);
    }    
    f = hue - i;    
    p = value * ( 1.0 - saturation);
    q = value * ( 1.0 - ( saturation * f) );
    t = value * ( 1.0 - ( saturation * ( 1.0 - f) ) );
    switch( i)
    {
    case 0:
      r = value; g = t; b = p; break;      
    case 1:
      r = q; g = value; b = p; break;
    case 2:
      r = p; g = value; b = t; break;
    case 3:
      r = p; g = q; b = value; break;
    case 4:
      r = t; g = p; b = value; break;
    case 5:
      r = value; g = p; b = q; break;      
    default:
      r = 0; g = 0; b = 0; break; // to keep lint quiet
    }    
  }
  SetFractionalRGB(r,g,b);
}

