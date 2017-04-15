%include <java/java.swg> //for SWIG_JAVABODY_PROXY and SWIG_JAVABODY_TYPEWRAPPER macros
//various utility files for usage  in multiple classes
%include <java/boost_intrusive_ptr.i>
%include <java/boost_shared_ptr.i>
%include <java/std_vector.i>
%include <java/stl.i>
%include <java/std_common.i>
%include <inttypes.i>


/*
When using multiple modules or the nspace feature it is common to invoke SWIG with a different -package command line option for each module. However, by default the generated code may not compile if generated classes in one package use generated classes in another package. The visibility of the getCPtr() and pointer constructor generated from the javabody typemaps needs changing. The default visibility is protected but it needs to be public for access from a different package. Just changing 'protected' to 'public' in the typemap achieves this. Two macros are available in java.swg to make this easier and using them is the preferred approach over simply copying the typemaps and modifying as this is forward compatible with any changes in the javabody typemap in future versions of SWIG. The macros are for the proxy and typewrapper classes and can respectively be used to to make the method and constructor public: 
*/


SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)
SWIG_JAVABODY_TYPEWRAPPER(public, public, public, SWIGTYPE)
//%pragma(java) jniclasspackage="wrapper"
//Including modules main files

%include "swig/pcl_module.i"
%include "swig/search_module.i"
