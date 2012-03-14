/* *************************************************
 *
 * Copyright (2011) Willow Garage
 *
 * Author : Cedric Cagniart, Koen Buys 
 * ************************************************* */

#ifndef PCL_PEOPLE_TREES_TRUN_H_DEFINED
#define PCL_PEOPLE_TREES_TRUN_H_DEFINED

#include "tree.h"
#include "tree_io.h"
#include "tex_fetch.h"
#include <fstream>
#include <limits>
#include <iostream>

namespace pcl
{
  namespace people
  {
    namespace trees
    {
      static int loadTree( std::istream& is,
                           std::vector<Node>&  tree,
                           std::vector<Label>& leaves )
      {
        // load the depth
        int maxDepth;
        is >> maxDepth;
        int numNodes = pow(2,maxDepth)-1;
        int numLeaves = pow(2,maxDepth);

        // alloc
        tree.resize(numNodes);
        leaves.resize(numLeaves);

        // read
        for(int ni=0;ni<numNodes;++ni)  is >> tree[ni];
        // the int is necessary.. otherwise it will format it as unsigned char
        for(int li=0;li<numLeaves;++li) { int l; is >> l; leaves[li] =l ; }

        // Check loading of the tree in terminal
        std::cout<<"(I) : loadTree(): loaded with " << numNodes << " Nodes, " << numLeaves << " Leaves and Depth " << maxDepth << std::endl;

        if( is.fail() )
              throw std::runtime_error(std::string("(E) malformed *.tree stream") );
        return maxDepth;
      }

      static int loadTree( const std::string&  filename,
                           std::vector<Node>&  tree,
                           std::vector<Label>& leaves )
      {
        std::ifstream fin(filename.c_str() );
        if( !fin.is_open() ) throw std::runtime_error(std::string("(E) could not open") + filename );
        return loadTree(fin, tree, leaves);
      }

      static void runThroughTree( int                       maxDepth,
                                  const std::vector<Node>&  tree,
                                  const std::vector<Label>& leaves,
                                  int                       W, 
                                  int                       H,
                                  const uint16_t*           dmap,
                                  Label*                    lmap )
      {
        Tex2Dfetcher tfetch( dmap, W, H ); // the tex fetcher

        int numNodes = pow(2,maxDepth) - 1;
        for(int y=0;y<H;++y) {
          for(int x=0;x<W;++x) {
            uint16_t depth = tfetch(x,y);
            if(depth == std::numeric_limits<uint16_t>::max() ) {
              lmap[x+W*y] = NOLABEL;
              continue;
            }
            double scale = focal / depth;
            // and now for the tree processing 
            int nid = 0;
            for(int d=0;d<maxDepth;++d) {
              const Node& node = tree[nid];
              const AttribLocation& loc = node.loc;
              uint16_t d1       = tfetch(x+loc.du1*scale, y+loc.dv1*scale);
              uint16_t d2       = tfetch(x+loc.du2*scale, y+loc.dv2*scale);
              int32_t delta     = int32_t(d1) - int32_t(d2);
              bool test = delta > int32_t(node.thresh);
              if( test ) nid = nid*2+2;
              else       nid = nid*2+1;
            }
            lmap[x+W*y] = leaves[nid-numNodes];
          }
        }
      }
    } // end namespace Tree
  } // end namespace people
} // end namespace pcl
#endif
