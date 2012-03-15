/* *************************************************
 *
 * @copyright (2011) Willow Garage
 *
 * @author : Cedric Cagniart, Koen Buys
 * ************************************************* */

#ifndef PCL_PEOPLE_TREES_TEX_FETCH_H_
#define PCL_PEOPLE_TREES_TEX_FETCH_H_

#include <stdint.h> //#include <cstdint>
#include <cmath> // for round

namespace pcl
{
  namespace people
  {
    namespace trees
    {

      /**
       * \brief Simple helper structure to fetch the texture without bothering about limits
       * This will be done by CUDA directly for the run time part of the stuff
       */
      struct Tex2Dfetcher
      {
        inline Tex2Dfetcher( const uint16_t* dmap, int W, int H ):m_dmap(dmap),m_W(W), m_H(H){}
        inline uint16_t operator () ( float uf, float vf ) {
          int u = round(uf);
          int v = round(vf);
          if( u < 0 ) u = 0;
          if( v < 0 ) v = 0;
          if( u >= m_W ) u = m_W-1;
          if( v >= m_H ) v = m_H-1;
        return m_dmap[u+v*m_W]; // this is going to be SLOOOWWW
        }
        const uint16_t* m_dmap;
        const int       m_W;
        const int       m_H;
      };
    } // end namespace trees
  } // end namespace people
} // end namespace pcl

#endif
