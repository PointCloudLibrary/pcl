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

#ifndef _ncvruntimetemplates_hpp_
#define _ncvruntimetemplates_hpp_
#if _MSC_VER >= 1200
#pragma warning( disable: 4800 )
#endif


#include <stdarg.h>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
// The Loki Library
// Copyright (c) 2001 by Andrei Alexandrescu
// This code accompanies the book:
// Alexandrescu, Andrei. "Modern C++ Design: Generic Programming and Design 
//     Patterns Applied". Copyright (c) 2001. Addison-Wesley.
// Permission to use, copy, modify, distribute and sell this software for any 
//     purpose is hereby granted without fee, provided that the above copyright 
//     notice appear in all copies and that both that copyright notice and this 
//     permission notice appear in supporting documentation.
// The author or Addison-Welsey Longman make no representations about the 
//     suitability of this software for any purpose. It is provided "as is" 
//     without express or implied warranty.
// http://loki-lib.sourceforge.net/index.php?n=Main.License
////////////////////////////////////////////////////////////////////////////////

namespace Loki
{
    //==============================================================================
    // class NullType
    // Used as a placeholder for "no type here"
    // Useful as an end marker in typelists 
    //==============================================================================

    class NullType {};

    //==============================================================================
    // class template Typelist
    // The building block of typelists of any length
    // Use it through the LOKI_TYPELIST_NN macros
    // Defines nested types:
    //     Head (first element, a non-typelist type by convention)
    //     Tail (second element, can be another typelist)
    //==============================================================================

    template <class T, class U>
    struct Typelist
    {
        typedef T Head;
        typedef U Tail;
    };

    //==============================================================================
    // class template Int2Type
    // Converts each integral constant into a unique type
    // Invocation: Int2Type<v> where v is a compile-time constant integral
    // Defines 'value', an enum that evaluates to v
    //==============================================================================

    template <int v>
    struct Int2Type
    {
        enum { value = v };
    };

    namespace TL
    {
        //==============================================================================
        // class template TypeAt
        // Finds the type at a given index in a typelist
        // Invocation (TList is a typelist and index is a compile-time integral 
        //     constant):
        // TypeAt<TList, index>::Result
        // returns the type in position 'index' in TList
        // If you pass an out-of-bounds index, the result is a compile-time error
        //==============================================================================

        template <class TList, unsigned int index> struct TypeAt;

        template <class Head, class Tail>
        struct TypeAt<Typelist<Head, Tail>, 0>
        {
            typedef Head Result;
        };

        template <class Head, class Tail, unsigned int i>
        struct TypeAt<Typelist<Head, Tail>, i>
        {
            typedef typename TypeAt<Tail, i - 1>::Result Result;
        };
    }
}


////////////////////////////////////////////////////////////////////////////////
// Runtime boolean template instance dispatcher
// Cyril Crassin <cyril.crassin@icare3d.org>
// NVIDIA, 2010
////////////////////////////////////////////////////////////////////////////////

namespace NCVRuntimeTemplateBool
{
    //This struct is used to transform a list of parameters into template arguments
    //The idea is to build a typelist containing the arguments
    //and to pass this typelist to a user defined functor
    template<typename TList, int NumArguments, class Func>
    struct KernelCaller
    {
        //Convenience function used by the user
        //Takes a variable argument list, transforms it into a list
        static void call(Func *functor, ...)
        {
            //Vector used to collect arguments
            std::vector<int> templateParamList;

            //Variable argument list manipulation
            va_list listPointer;
            va_start(listPointer, functor);
            //Collect parameters into the list
            for(int i=0; i<NumArguments; i++)
            {
                int val = va_arg(listPointer, int);
                templateParamList.push_back(val);
            }
            va_end(listPointer);

            //Call the actual typelist building function
            call(*functor, templateParamList);
        }

        //Actual function called recursively to build a typelist based
        //on a list of values
        static void call( Func &functor, std::vector<int> &templateParamList)
        {
            //Get current parameter value in the list
            NcvBool val = templateParamList[templateParamList.size() - 1];
            templateParamList.pop_back();

            //Select the compile time value to add into the typelist
            //depending on the runtime variable and make recursive call.
            //Both versions are really instantiated
            if (val)
            {
                KernelCaller<
                    Loki::Typelist<typename Loki::Int2Type<1>, TList >,
                    NumArguments-1, Func >
                    ::call(functor, templateParamList);
            }
            else
            {
                KernelCaller<
                    Loki::Typelist<typename Loki::Int2Type<0>, TList >,
                    NumArguments-1, Func >
                    ::call(functor, templateParamList);
            }
        }
    };

    //Specialization for 0 value left in the list
    //-> actual kernel functor call
    template<class TList, class Func>
    struct KernelCaller<TList, 0, Func>
    {
        static void call(Func &functor)
        {
            //Call to the functor's kernel call method
            functor.call(TList()); //TList instantiated to get the method template parameter resolved
        }

        static void call(Func &functor, std::vector<int> &templateParams)
        {
            functor.call(TList());
        }
    };
}

#endif //_ncvruntimetemplates_hpp_
