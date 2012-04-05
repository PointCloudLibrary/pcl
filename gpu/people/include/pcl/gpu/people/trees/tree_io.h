/* *************************************************
 *
 * @copyright (2011) Willow Garage
 *
 * @author : Cedric Cagniart, Koen Buys
 * ************************************************* */

#ifndef PCL_GPU_PEOPLE_TREES_TREE_IO_H_
#define PCL_GPU_PEOPLE_TREES_TREE_IO_H_

#include "tree.h"
#include <fstream>
#include <cassert>
#include <string>
#include <vector>
#include <stdexcept>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace trees
      {
        // ##########################################
        // Reading and writing AttributeLocations
        inline std::ostream& operator << (std::ostream& os, const AttribLocation& aloc ) {
          os<<aloc.du1<<" "<<aloc.dv1<<" "<<aloc.du2<<" "<<aloc.dv2<<"\n";
          return os;
        }

        inline std::istream& operator >> (std::istream& is, AttribLocation& aloc ) {
          is >> aloc.du1 >> aloc.dv1 >> aloc.du2 >> aloc.dv2;
          return is;
        }

        static void writeAttribLocs( const std::string&                 filename,
                                     const std::vector<AttribLocation>& alocs )
        {
          std::ofstream fout(filename.c_str());				
          if(!fout.is_open()) 
	  		throw std::runtime_error(std::string("(E) could not open ") + filename);;

          // first we write the number of attribs we intend to write so that we ll avoid mismatches
          assert( alocs.size() == NUMATTRIBS );
          fout<<NUMATTRIBS<<"\n";
          for(int ai=0;ai<NUMATTRIBS;++ai) {
            fout<< alocs[ai];
          }
        }

        static void readAttribLocs( const std::string&               filename,
                                    std::vector<AttribLocation>&     alocs )
        {
          std::ifstream fin(filename.c_str() );
          if(!fin.is_open()) throw std::runtime_error(std::string("(E) could not open " + filename) );

          int numAttribs;
          fin >> numAttribs;
          if( numAttribs != NUMATTRIBS ) throw  std::runtime_error(std::string("(E) the attribloc file has a wrong number of attribs ") + filename );

          alocs.resize(NUMATTRIBS);
          for(int ai=0;ai<NUMATTRIBS;++ai) {
            fin >> alocs[ai];
          }
        }

        static void readThreshs( const std::string&            filename,
                                 std::vector<Attrib>&          threshs )
        {
          std::ifstream fin(filename.c_str() );
          if(!fin.is_open()) throw std::runtime_error(std::string("(E) could not open " + filename) );

          int numThreshs;
          fin >> numThreshs;
          threshs.resize(numThreshs);
          for(int ti =0;ti<numThreshs;++ti) {
            fin >> threshs[ti];
          }
          if( fin.fail() ) throw std::runtime_error(std::string("(E) malformed thresh file: " + filename) );
        }

        static void writeThreshs( const std::string&            filename,
                                  const std::vector<Attrib>&    threshs )
        {
          std::ofstream fout(filename.c_str() );
          if(!fout.is_open()) throw std::runtime_error(std::string("(E) could not open " + filename) );

          int numThreshs = threshs.size();

          fout << numThreshs<<"\n";
          for(int ti =0;ti<numThreshs;++ti) {
            fout << threshs[ti]<<"\n";
          }
        }

        // ##########################################
        // Reading and writing Trees

        static inline std::istream& operator >> ( std::istream& is, Node& n) {
          is >> n.loc;
          is >> n.thresh;
          return is;
        }

      } // end namespace trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl

#endif
