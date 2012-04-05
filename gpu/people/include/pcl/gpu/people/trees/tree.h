/* *************************************************
 *
 * Copyright (2011) Willow Garage
 *
 * Author : Cedric Cagniart, Koen Buys
 * ************************************************* */

#ifndef PCL_GPU_PEOPLE_TREES_TREE_H_
#define PCL_GPU_PEOPLE_TREES_TREE_H_

#include<boost/cstdint.hpp>  //#include <cstdint>
#include <pcl/gpu/containers/device_array.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace trees
      {
        // this has nothing to do here...
        static const double focal = 1000.;

        // ###############################################
        // compile type values
        enum { NUMATTRIBS  = 2000 };
        enum { NUMLABELS = 32 };

        // ###############################################
        // base data types used in the structures
	      typedef boost::int16_t Attrib;
        typedef boost::uint8_t Label;
        typedef boost::uint32_t Label32;
        typedef boost::uint16_t Depth;

        struct AttribLocation {
          inline AttribLocation () {du1=dv1=du2=dv2=0;}
          inline AttribLocation (int u1, int v1, int u2, int v2):du1(u1),dv1(v1),du2(u2),dv2(v2){}
          boost::int16_t du1,dv1,du2,dv2;
        };
        static const Label NOLABEL = 31;

        // ###############################################
        // Tree basic Structure
        struct Node {
          inline Node () {}
          inline Node (const AttribLocation& l, const Attrib& t):loc(l),thresh(t){}
          AttribLocation loc;
          Attrib         thresh;
        };

        // GPU Typedefs
        typedef DeviceArray<Label>      D_Label_8;
        typedef DeviceArray<Label32>    D_Label_32;
        typedef DeviceArray<Depth>      D_Depth;

      } // end namespace Trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl
#endif  // PCL_GPU_PEOPLE_TREES_TREE_H_
