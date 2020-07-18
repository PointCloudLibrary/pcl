/*
Copyright (c) 2007, Michael Kazhdan
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef MAT_INCLUDED
#define MAT_INCLUDED
#include "geometry.h"


namespace pcl
{
  namespace poisson
  {

    template <class Real>
    class MinimalAreaTriangulation
    {
        Real* bestTriangulation;
        int* midPoint;
        Real GetArea(const std::size_t& i,const std::size_t& j,const std::vector<Point3D<Real> >& vertices);
        void GetTriangulation(const std::size_t& i,const std::size_t& j,const std::vector<Point3D<Real> >& vertices,std::vector<TriangleIndex>& triangles);
      public:
        MinimalAreaTriangulation(void);
        ~MinimalAreaTriangulation(void);
        Real GetArea(const std::vector<Point3D<Real> >& vertices);
        void GetTriangulation(const std::vector<Point3D<Real> >& vertices,std::vector<TriangleIndex>& triangles);
    };

  }
}

#include "mat.hpp"



#endif // MAT_INCLUDED
