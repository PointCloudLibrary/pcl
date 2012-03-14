/* *************************************************
 *
 * Copyright (2011) Willow Garage
 *
 * Author : Cedric Cagniart, Koen Buys
 * ************************************************* */

#ifndef PCL_PEOPLE_TREES_TREE_H_DEFINED
#define PCL_PEOPLE_TREES_TREE_H_DEFINED

#include<boost/cstdint.hpp>  //#include <cstdint>

namespace pcl
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
      typedef int16_t Attrib;
      typedef uint8_t Label;

      struct AttribLocation {
        inline AttribLocation () {du1=dv1=du2=dv2=0;}
        inline AttribLocation (int u1, int v1, int u2, int v2):du1(u1),dv1(v1),du2(u2),dv2(v2){}
        int16_t du1,dv1,du2,dv2;
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
    } // end namespace Trees
  } // end namespace people
} // end namespace pcl
#endif  // PCL_PEOPLE_TREES_TREE_H_DEFINED
