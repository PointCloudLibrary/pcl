/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (C) 2009-2010, NVIDIA Corporation, all rights reserved.
 *  Third party copyrights are property of their respective owners.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id:  $
 * Ported to PCL by Koen Buys : Attention Work in progress!
 */


#ifndef _ncvpyramid_hpp_
#define _ncvpyramid_hpp_

#include <memory>
#include <vector>
#include "NCV.hpp"

#if 0 //def _WIN32

template <class T>
class NCV_EXPORTS NCVMatrixStack
{
public:
    NCVMatrixStack() {this->_arr.clear();}
    ~NCVMatrixStack()
    {
        const Ncv32u nElem = this->_arr.size();
        for (Ncv32u i=0; i<nElem; i++)
        {
            pop_back();
        }
    }
    void push_back(NCVMatrix<T> *elem) {this->_arr.push_back(std::tr1::shared_ptr< NCVMatrix<T> >(elem));}
    void pop_back() {this->_arr.pop_back();}
    NCVMatrix<T> * operator [] (int i) const {return this->_arr[i].get();}
private:
    std::vector< std::tr1::shared_ptr< NCVMatrix<T> > > _arr;
};


template <class T>
class NCV_EXPORTS NCVImagePyramid
{
public:

    NCVImagePyramid(const NCVMatrix<T> &img,
                    Ncv8u nLayers,
                    INCVMemAllocator &alloc,
                    cudaStream_t cuStream);
    ~NCVImagePyramid();
    NcvBool isInitialized() const;
    NCVStatus getLayer(NCVMatrix<T> &outImg,
                       NcvSize32u outRoi,
                       NcvBool bTrilinear,
                       cudaStream_t cuStream) const;

private:

    NcvBool _isInitialized;
    const NCVMatrix<T> *layer0;
    NCVMatrixStack<T> pyramid;
    Ncv32u nLayers;
};

#endif //_WIN32

#endif //_ncvpyramid_hpp_
