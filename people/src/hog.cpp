/*
 * Software License Agreement (Simplified BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 * Copyright (c) 2012, Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 *
 * Derived from Piotr Dollar's MATLAB Image&Video Toolbox      Version 3.00.
 *
 * hog.cpp
 * Created on: Nov 30, 2012
 * Authors: Matteo Munaro, Stefano Ghidoni, Stefano Michieletto
 */

#include <pcl/people/hog.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>

#if defined(__SSE2__)
#include <pcl/sse.h>
#else
#include <cstdlib>
#endif

/** \brief Constructor. */
pcl::people::HOG::HOG () 
{
  // set default values for optional parameters:
  h_ = 128;
  w_ = 64;
  n_channels_ = 3;
  bin_size_ = 8;
  n_orients_ = 9;
  soft_bin_ = true;
  clip_ = 0.2;
}  

/** \brief Destructor. */
pcl::people::HOG::~HOG () {}

void 
pcl::people::HOG::gradMag( float *I, int h, int w, int d, float *M, float *O ) const
{
#if defined(__SSE2__)
  int x, y, y1, c, h4, s; float *Gx, *Gy, *M2; __m128 *_Gx, *_Gy, *_M2, _m;
  float *acost = acosTable(), acMult=25000/2.02f;

  // allocate memory for storing one column of output (padded so h4%4==0)
  h4=(h%4==0) ? h : h-(h%4)+4; s=d*h4*sizeof(float);

  M2=(float*) alMalloc(s,16);
  _M2=(__m128*) M2;
  Gx=(float*) alMalloc(s,16); _Gx=(__m128*) Gx;
  Gy=(float*) alMalloc(s,16); _Gy=(__m128*) Gy;

  // compute gradient magnitude and orientation for each column
  for( x=0; x<w; x++ ) {
  // compute gradients (Gx, Gy) and squared magnitude (M2) for each channel
  for( c=0; c<d; c++ ) grad1( I+x*h+c*w*h, Gx+c*h4, Gy+c*h4, h, w, x );
  for( y=0; y<d*h4/4; y++ ) _M2[y]=pcl::sse_add(pcl::sse_mul(_Gx[y],_Gx[y]),pcl::sse_mul(_Gy[y],_Gy[y]));
  // store gradients with maximum response in the first channel
  for(c=1; c<d; c++) {
    for( y=0; y<h4/4; y++ ) {
    y1=h4/4*c+y; _m = pcl::sse_cmpgt( _M2[y1], _M2[y] );
    _M2[y] = pcl::sse_or( pcl::sse_and(_m,_M2[y1]), pcl::sse_andnot(_m,_M2[y]) );
    _Gx[y] = pcl::sse_or( pcl::sse_and(_m,_Gx[y1]), pcl::sse_andnot(_m,_Gx[y]) );
    _Gy[y] = pcl::sse_or( pcl::sse_and(_m,_Gy[y1]), pcl::sse_andnot(_m,_Gy[y]) );
    }
  }
  // compute gradient magnitude (M) and normalize Gx
  for( y=0; y<h4/4; y++ ) {
    _m = pcl::sse_min( pcl::sse_rcpsqrt(_M2[y]), pcl::sse_set(1e10f) );
    _M2[y] = pcl::sse_rcp(_m);
    _Gx[y] = pcl::sse_mul( pcl::sse_mul(_Gx[y],_m), pcl::sse_set(acMult) );
    _Gx[y] = pcl::sse_xor( _Gx[y], pcl::sse_and(_Gy[y], pcl::sse_set(-0.f)) );
  };

  memcpy( M+x*h, M2, h*sizeof(float) );
  // compute and store gradient orientation (O) via table lookup
  if(O!=0) for( y=0; y<h; y++ ) O[x*h+y] = acost[(int)Gx[y]];
  }
  alFree(Gx); alFree(Gy); alFree(M2); 
#else
  int x, y, y1, c, h4, s; float *Gx, *Gy, *M2; 
  float *acost = acosTable(), acMult=25000/2.02f;

  // allocate memory for storing one column of output (padded so h4%4==0)
  h4=(h%4==0) ? h : h-(h%4)+4; s=d*h4*sizeof(float);

  M2=(float*) alMalloc(s,16);
  Gx=(float*) alMalloc(s,16); 
  Gy=(float*) alMalloc(s,16); 
  float m;

  // compute gradient magnitude and orientation for each column
  for( x=0; x<w; x++ ) {
  // compute gradients (Gx, Gy) and squared magnitude (M2) for each channel
  for( c=0; c<d; c++ ) grad1( I+x*h+c*w*h, Gx+c*h4, Gy+c*h4, h, w, x );
  for( y=0; y<d*h4; y++ ) 
  {
    M2[y] = Gx[y] * Gx[y] + Gy[y] * Gy[y];
  }  
  
  // store gradients with maximum response in the first channel
  for(c=1; c<d; c++) 
  {
  for( y=0; y<h4/4; y++ ) 
    {
      y1=h4/4*c+y; 

    for (int ii = 0; ii < 4; ++ii)
    {
      if (M2[y1 * 4 + ii] > M2[y * 4 + ii])
      {
        M2[y * 4 + ii] = M2[y1 * 4 + ii];
        Gx[y * 4 + ii] = Gx[y1 * 4 + ii];
        Gy[y * 4 + ii] = Gy[y1 * 4 + ii];
      }
    }
    }
  }
  // compute gradient magnitude (M) and normalize Gx
  for( y=0; y<h4; y++ ) 
  {
  m = 1.0f/sqrtf(M2[y]);
  m = m < 1e10f ? m : 1e10f;
    M2[y] = 1.0f / m;
    Gx[y] = ((Gx[y] * m) * acMult);
    if (Gy[y] < 0)
    Gx[y] = -Gx[y];
  }
  
  memcpy( M+x*h, M2, h*sizeof(float) );
  // compute and store gradient orientation (O) via table lookup
  if(O!=0) for( y=0; y<h; y++ ) O[x*h+y] = acost[(int)Gx[y]];
  }
  alFree(Gx); alFree(Gy); alFree(M2);
#endif  
}

void 
pcl::people::HOG::gradHist( float *M, float *O, int h, int w, int bin_size, int n_orients, bool soft_bin, float *H ) const
{
  const int hb=h/bin_size, wb=w/bin_size, h0=hb*bin_size, w0=wb*bin_size, nb=wb*hb;
  const float s=(float)bin_size, sInv=1/s, sInv2=1/s/s;
  float *H0, *H1, *M0, *M1; int x, y; int *O0, *O1;
  O0=(int*)alMalloc(h*sizeof(int),16); M0=(float*) alMalloc(h*sizeof(float),16);
  O1=(int*)alMalloc(h*sizeof(int),16); M1=(float*) alMalloc(h*sizeof(float),16);

  // main loop
  float xb = 0;
  float init = 0;
  for( x=0; x<w0; x++ ) {
    // compute target orientation bins for entire column - very fast
    gradQuantize( O+x*h, M+x*h, O0, O1, M0, M1, n_orients, nb, h0, sInv2 );

    if( !soft_bin || bin_size==1 ) {
      // interpolate w.r.t. orientation only, not spatial bin_size
      H1=H+(x/bin_size)*hb;
      #define GH H1[O0[y]]+=M0[y]; H1[O1[y]]+=M1[y]; y++;
      if( bin_size==1 )      for(y=0; y<h0;) { GH; H1++; }
      else if( bin_size==2 ) for(y=0; y<h0;) { GH; GH; H1++; }
      else if( bin_size==3 ) for(y=0; y<h0;) { GH; GH; GH; H1++; }
      else if( bin_size==4 ) for(y=0; y<h0;) { GH; GH; GH; GH; H1++; }
      else for( y=0; y<h0;) { for( int y1=0; y1<bin_size; y1++ ) { GH; } H1++; }
      #undef GH

    } else {
      // interpolate using trilinear interpolation
#if defined(__SSE2__)
      float ms[4], xyd, yb, xd, yd; __m128 _m, _m0, _m1;
      bool hasLf, hasRt; int xb0, yb0;
      if( x==0 ) { init=(0+.5f)*sInv-0.5f; xb=init; }
      hasLf = xb>=0; xb0 = hasLf?(int)xb:-1; hasRt = xb0 < wb-1;
      xd=xb-xb0; xb+=sInv; yb=init; y=0;
      // macros for code conciseness
      #define GHinit yd=yb-yb0; yb+=sInv; H0=H+xb0*hb+yb0; xyd=xd*yd; \
      ms[0]=1-xd-yd+xyd; ms[1]=yd-xyd; ms[2]=xd-xyd; ms[3]=xyd;
      #define GH(H,ma,mb) H1=H; pcl::sse_stru(*H1,pcl::sse_add(pcl::sse_ldu(*H1),pcl::sse_mul(ma,mb)));
      // leading rows, no top bin_size
      for( ; y<bin_size/2; y++ ) {
        yb0=-1; GHinit;
        if(hasLf) { H0[O0[y]+1]+=ms[1]*M0[y]; H0[O1[y]+1]+=ms[1]*M1[y]; }
        if(hasRt) { H0[O0[y]+hb+1]+=ms[3]*M0[y]; H0[O1[y]+hb+1]+=ms[3]*M1[y]; }
      }
      // main rows, has top and bottom bins, use SSE for minor speedup
      for( ; ; y++ ) {
        yb0 = (int) yb; if(yb0>=hb-1) break; GHinit;
        _m0=pcl::sse_set(M0[y]); _m1=pcl::sse_set(M1[y]);
        if(hasLf) { _m=pcl::sse_set(0,0,ms[1],ms[0]);
        GH(H0+O0[y],_m,_m0); GH(H0+O1[y],_m,_m1); }
        if(hasRt) { _m=pcl::sse_set(0,0,ms[3],ms[2]);
        GH(H0+O0[y]+hb,_m,_m0); GH(H0+O1[y]+hb,_m,_m1); }
      }      
      // final rows, no bottom bin_size
      for( ; y<h0; y++ ) {
        yb0 = (int) yb; GHinit;
        if(hasLf) { H0[O0[y]]+=ms[0]*M0[y]; H0[O1[y]]+=ms[0]*M1[y]; }
        if(hasRt) { H0[O0[y]+hb]+=ms[2]*M0[y]; H0[O1[y]+hb]+=ms[2]*M1[y]; }
      }       
      #undef GHinit
      #undef GH
#else
      float ms[4], xyd, yb, xd, yd;  
      bool hasLf, hasRt; int xb0, yb0;
      if( x==0 ) { init=(0+.5f)*sInv-0.5f; xb=init; }
      hasLf = xb>=0; xb0 = hasLf?(int)xb:-1; hasRt = xb0 < wb-1;
      xd=xb-xb0; xb+=sInv; yb=init; y=0;
      // macros for code conciseness
      #define GHinit yd=yb-yb0; yb+=sInv; H0=H+xb0*hb+yb0; xyd=xd*yd; \
      ms[0]=1-xd-yd+xyd; ms[1]=yd-xyd; ms[2]=xd-xyd; ms[3]=xyd;
      // leading rows, no top bin_size
      for( ; y<bin_size/2; y++ ) {
        yb0=-1; GHinit;
        if(hasLf) { H0[O0[y]+1]+=ms[1]*M0[y]; H0[O1[y]+1]+=ms[1]*M1[y]; }
        if(hasRt) { H0[O0[y]+hb+1]+=ms[3]*M0[y]; H0[O1[y]+hb+1]+=ms[3]*M1[y]; }
      }
      // main rows, has top and bottom bins
      for( ; ; y++ ) {
        yb0 = (int) yb;
        if(yb0>=hb-1) 
          break; 
        GHinit;
  
        if(hasLf) 
        {
          H0[O0[y]+1]+=ms[1]*M0[y]; 
          H0[O1[y]+1]+=ms[1]*M1[y];
          H0[O0[y]]+=ms[0]*M0[y]; 
          H0[O1[y]]+=ms[0]*M1[y];
        }
            if(hasRt) 
        {
          H0[O0[y]+hb+1]+=ms[3]*M0[y];
          H0[O1[y]+hb+1]+=ms[3]*M1[y]; 
          H0[O0[y]+hb]+=ms[2]*M0[y];
          H0[O1[y]+hb]+=ms[2]*M1[y];
        }
      }      
      // final rows, no bottom bin_size
      for( ; y<h0; y++ ) {
        yb0 = (int) yb; GHinit;
        if(hasLf) { H0[O0[y]]+=ms[0]*M0[y]; H0[O1[y]]+=ms[0]*M1[y]; }
        if(hasRt) { H0[O0[y]+hb]+=ms[2]*M0[y]; H0[O1[y]+hb]+=ms[2]*M1[y]; }
      }       
      #undef GHinit
#endif     
    }
  }

  alFree(O0); alFree(O1); alFree(M0); alFree(M1);
}
      
void 
pcl::people::HOG::normalization (float *H, int h, int w, int bin_size, int n_orients, float clip, float *G) const
{
  float *N, *N1, *H1; int o, x, y, hb=h/bin_size, wb=w/bin_size, nb=wb*hb;
  float eps = 1e-4f/4/bin_size/bin_size/bin_size/bin_size; // precise backward equality
  // compute 2x2 block normalization values
  N = (float*) calloc(nb,sizeof(float));
  for( o=0; o<n_orients; o++ ) for( x=0; x<nb; x++ ) N[x]+=H[x+o*nb]*H[x+o*nb];
  for( x=0; x<wb-1; x++ ) for( y=0; y<hb-1; y++ ) {
    N1=N+x*hb+y; *N1=1/float(sqrt( N1[0] + N1[1] + N1[hb] + N1[hb+1] +eps )); }
  // perform 4 normalizations per spatial block (handling boundary regions)
  #define U(a,b) Gs[a][y]=H1[y]*N1[y-(b)]; if(Gs[a][y]>clip) Gs[a][y]=clip;
  for( o=0; o<n_orients; o++ ) for( x=0; x<wb; x++ ) {
    H1=H+o*nb+x*hb; N1=N+x*hb; float *Gs[4]; Gs[0]=G+o*nb+x*hb;
    for( y=1; y<4; y++ ) Gs[y]=Gs[y-1]+nb*n_orients;
    bool lf, md, rt; lf=(x==0); rt=(x==wb-1); md=(!lf && !rt);
    y=0; if(!rt) U(0,0); if(!lf) U(2,hb);
    if(lf) for( y=1; y<hb-1; y++ ) { U(0,0); U(1,1); }
    if(md) for( y=1; y<hb-1; y++ ) { U(0,0); U(1,1); U(2,hb); U(3,hb+1); }
    if(rt) for( y=1; y<hb-1; y++ ) { U(2,hb); U(3,hb+1); }
    y=hb-1; if(!rt) U(1,1); if(!lf) U(3,hb+1);
  } free(N);
  #undef U
}
      
void
pcl::people::HOG::compute (float *I, int h, int w, int n_channels, int bin_size, int n_orients, bool soft_bin, float *descriptor)
{
  h_ = h;
  w_ = w;
  n_channels_ = n_channels;
  bin_size_ = bin_size;
  n_orients_ = n_orients;
  soft_bin_ = soft_bin;
  
  compute (I, descriptor);
}

void
pcl::people::HOG::compute (float *I, float *descriptor) const
{
  // HOG computation:
  float *M, *O, *G, *H;
  M = new float[h_ * w_];
  O = new float[h_ * w_];
  H = new float[(w_ / bin_size_) * (h_ / bin_size_) * n_orients_]();
  G = new float[(w_ / bin_size_) * (h_ / bin_size_) * n_orients_ *4]();

  // Compute gradient magnitude and orientation at each location (uses sse):
  gradMag (I, h_, w_, n_channels_, M, O );
 
  // Compute n_orients gradient histograms per bin_size x bin_size block of pixels:
  gradHist ( M, O, h_, w_, bin_size_, n_orients_, soft_bin_, H );

  // Apply normalizations:
  normalization ( H, h_, w_, bin_size_, n_orients_, clip_, G );
 
  // Select descriptor of internal part of the image (remove borders):
  int k = 0;    
  for (int l = 0; l < (n_orients_ * 4); l++)
  {
    for (int j = 1; j < (w_ / bin_size_ - 1); j++)
    {
      for (int i = 1; i < (h_ / bin_size_ - 1); i++)
      {
        descriptor[k] = G[i + j * h_ / bin_size_ + l * (h_ / bin_size_) * (w_ / bin_size_)];
        k++;
      }
    }
  }
  delete[] M; delete[] O; delete[] H; delete[] G;
}

void 
pcl::people::HOG::grad1 (float *I, float *Gx, float *Gy, int h, int w, int x) const
{
#if defined(__SSE2__)
  int y, y1; float *Ip, *In, r; __m128 *_Ip, *_In, *_G, _r;
  // compute column of Gx
  Ip=I-h; In=I+h; r=.5f;
  if(x==0) { r=1; Ip+=h; } else if(x==w-1) { r=1; In-=h; }
  if( h<4 || h%4>0 || (size_t(I)&15) || (size_t(Gx)&15) ) {
  for( y=0; y<h; y++ ) *Gx++=(*In++-*Ip++)*r;
  } else {
  _G=(__m128*) Gx; _Ip=(__m128*) Ip; _In=(__m128*) In; _r = pcl::sse_set(r);
  for(y=0; y<h; y+=4) *_G++=pcl::sse_mul(pcl::sse_sub(*_In++,*_Ip++),_r);
  }
  // compute column of Gy
  #define GRADY(r) *Gy++=(*In++-*Ip++)*r;
  Ip=I; In=Ip+1;
  // GRADY(1); Ip--; for(y=1; y<h-1; y++) GRADY(.5f); In--; GRADY(1);
  y1=((~((size_t) Gy) + 1) & 15)/4; if(y1==0) y1=4; if(y1>h-1) y1=h-1;
  GRADY(1); Ip--; for(y=1; y<y1; y++) GRADY(.5f);
  _r = pcl::sse_set(.5f); _G=(__m128*) Gy;
  for(; y+4<h-1; y+=4, Ip+=4, In+=4, Gy+=4)
  *_G++=pcl::sse_mul(pcl::sse_sub(pcl::sse_ldu(*In),pcl::sse_ldu(*Ip)),_r);
  for(; y<h-1; y++) GRADY(.5f); In--; GRADY(1);
  #undef GRADY
#else
  int y, y1;
  float *Ip, *In, r;
  
  // compute column of Gx
  Ip = I - h;
  In = I + h;
  r = .5f;
  
  if(x == 0)
  {
  r = 1;
  Ip += h;
  }
  else if (x == w-1)
  {
  r = 1;
  In -= h;
  }

  for (y = 0; y < h; y++)
  *Gx++=(*In++ - *Ip++)*r;
  
  // compute column of Gy
  #define GRADY(r) *Gy++=(*In++-*Ip++)*r;
  Ip=I; In=Ip+1;
  // GRADY(1); Ip--; for(y=1; y<h-1; y++) GRADY(.5f); In--; GRADY(1);
  y1=((~((size_t) Gy) + 1) & 15)/4; if(y1==0) y1=4; if(y1>h-1) y1=h-1;
  GRADY(1); Ip--; for(y=1; y<y1; y++) GRADY(.5f);
  
  r = 0.5f;
  for(; y<h-1; y++)
    GRADY(.5f); In--; GRADY(1);
  #undef GRADY
#endif
}
      
float* 
pcl::people::HOG::acosTable () const
{
  int i, n=25000, n2=n/2; float t, ni;
  static float a[25000]; static bool init=false;
  if( init ) return a+n2; ni = 2.02f/(float) n;
  for( i=0; i<n; i++ ) {
    t = i*ni - 1.01f;
    t = t<-1 ? -1 : (t>1 ? 1 : t);
    t = (float) acos( t );
    a[i] = (t <= M_PI-1e-5f) ? t : 0;
  }
  init=true; return a+n2;
}
      
void 
pcl::people::HOG::gradQuantize (float *O, float *M, int *O0, int *O1, float *M0, float *M1, int n_orients, int nb, int n, float norm) const
{
#if defined(__SSE2__)
  // assumes all *OUTPUT* matrices are 4-byte aligned
  int i, o0, o1; float o, od, m;
  __m128i _o0, _o1, *_O0, *_O1; __m128 _o, _o0f, _m, *_M0, *_M1;
  // define useful constants
  const float oMult=(float)n_orients/M_PI; const int oMax=n_orients*nb;
  const __m128 _norm=pcl::sse_set(norm), _oMult=pcl::sse_set(oMult), _nbf=pcl::sse_set((float)nb);
  const __m128i _oMax=pcl::sse_set(oMax), _nb=pcl::sse_set(nb);

  // perform the majority of the work with sse
  _O0=(__m128i*) O0; _O1=(__m128i*) O1; _M0=(__m128*) M0; _M1=(__m128*) M1;
  for( i=0; i<=n-4; i+=4 ) {
  _o=pcl::sse_mul(pcl::sse_ldu(O[i]),_oMult); _o0f=pcl::sse_cvt(pcl::sse_cvt(_o)); _o0=pcl::sse_cvt(pcl::sse_mul(_o0f,_nbf));
  _o1=pcl::sse_add(_o0,_nb); _o1=pcl::sse_and(pcl::sse_cmpgt(_oMax,_o1),_o1);
  *_O0++=_o0; *_O1++=_o1; _m=pcl::sse_mul(pcl::sse_ldu(M[i]),_norm);
  *_M1=pcl::sse_mul(pcl::sse_sub(_o,_o0f),_m); *_M0=pcl::sse_sub(_m,*_M1); _M0++; _M1++;
  }

  // compute trailing locations without sse
  for( ; i<n; i++ ) {
  o=O[i]*oMult; m=M[i]*norm; o0=(int) o; od=o-o0;
  o0*=nb; o1=o0+nb; if(o1==oMax) o1=0;
  O0[i]=o0; O1[i]=o1; M1[i]=od*m; M0[i]=m-M1[i];
  }
#else
  int i, o0, o1;
  float o, od, m;

  // define useful constants
  const float oMult=(float)n_orients/M_PI; const int oMax=n_orients*nb;

  // compute trailing locations without sse
  for( i = 0; i<n; i++ )
  {
    o=O[i]*oMult; m=M[i]*norm; o0=(int) o; od=o-o0;
    o0*=nb; o1=o0+nb; if(o1==oMax) o1=0;
    O0[i]=o0; O1[i]=o1; M1[i]=od*m; M0[i]=m-M1[i];
  }
#endif
}

inline void* 
pcl::people::HOG::alMalloc (size_t size, int alignment) const
{
  const size_t pSize = sizeof(void*), a = alignment-1;
  void *raw = malloc(size + a + pSize);
  void *aligned = (void*) (((size_t) raw + pSize + a) & ~a);
  *(void**) ((size_t) aligned-pSize) = raw;
  return aligned;
}

inline void 
pcl::people::HOG::alFree (void* aligned) const
{
  void* raw = *(void**)((char*)aligned-sizeof(void*));
  free(raw);
}

