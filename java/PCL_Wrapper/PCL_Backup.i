%module pcl

%include <stdint.i>


%{
#include <pcl/point_types.h>
%}

//%nspace pcl //will that compile without errors since it needs anthor parameter, to convert namespaces to packages
//SWIG_JAVABODY_PROXY(public, public, SWIGTYPE) //to deal with the problem: java classes in a package calling other classes in anthor package
//SWIG_JAVABODY_TYPEWRAPPER(public, public, public, SWIGTYPE) //to deal with the problem: java classes in a package calling other classes in anthor package

#pragma once

namespace pcl
{
	struct RGB
	  {
		RGB ();
		int rgb ;
		uint8_t r ;
		uint8_t g ;
		uint8_t b ;
	  };
}
