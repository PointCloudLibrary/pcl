/* *************************************************
 *
 * Copyright (2011) Willow Garage
 *
 * Author : Cedric Cagniart 
 * ************************************************* */

#pragma once

#include "tree.h"
#include <boost/array.hpp>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace trees
      {
        // ################################################
        // ################################################
        // histogram stuff
        class Histogram : public boost::array<std::uint32_t,NUMLABELS> {
          public :
          inline Histogram() { std::fill(begin(), end(), 0); }
        };

        struct HistogramPair {
          public :
            // accumulate on the histograms
            inline void accumTrue(const Label label ) {
              m_h_true[label]++;
            }
            inline void accumFalse(const Label label ) {
              m_h_false[label]++;
            }

            inline Histogram& h_false() { return m_h_false; }
            inline Histogram& h_true() { return m_h_true; }

            inline const Histogram h_false() const { return m_h_false; }
            inline const Histogram h_true() const { return m_h_true; }

          protected :
            Histogram m_h_false;
            Histogram m_h_true;
        };

        // ###############################################
        // ###############################################
        // SplitPoint
        struct SplitPoint{
          inline SplitPoint( int ai, Attrib t):attribId(ai), threshold(t){}
          int    attribId;
          Attrib threshold;
        };

        // ###############################################
        // ###############################################
        // Data Structures as stored in binary files
        struct LabeledAttrib {
          inline LabeledAttrib(){}
          inline LabeledAttrib( const Label& label, const Attrib& attrib): l(label), a(attrib){}
          Label l;
          Attrib a;
        };

        // this is only going to be a helper structure
        struct LabeledFeature {  // : boost::noncopyable {
          // constructors
          inline LabeledFeature(): l(NOLABEL){
          }
          inline LabeledFeature( const LabeledFeature& B){
            l = B.l;
            std::copy( B.attribs, B.attribs + NUMATTRIBS, attribs );
          }
          Label  l; // WARNING the compiler will pad here
          Attrib attribs[NUMATTRIBS];
        };


         // compute the number of elements
        static inline std::uint64_t numElements( const Histogram& h ) {
          std::uint64_t Ntotal = 0;
          for(int li=0;li<NUMLABELS;++li) Ntotal += std::uint64_t(h[li]);
          return Ntotal;
        }

        /**
         * This is cool
         */
        static inline double entropy( const Histogram& h ) {
          double Ntotal = numElements(h);
          double entropy = 0.;
          for(int li=0;li<NUMLABELS;++li) {
            if( h[li] != 0 ) {
              double p = double(h[li]) / Ntotal;
              entropy -= p*std::log(p);
            }
          }
          return entropy;
        }

        /**
         * This is a little weird.. it will just compute the entropy of the merged histograms
         */
        static inline double entropy_merged( const HistogramPair& hp ) {
          const Histogram& htrue  = hp.h_true();
          const Histogram& hfalse = hp.h_false();

          double Ntotal = numElements(htrue) + numElements(hfalse);
          double entropy = 0.;
          for(int li=0;li<NUMLABELS;++li) {
            std::uint64_t Ni = std::uint64_t(htrue[li]) + std::uint64_t(hfalse[li]);
            if( Ni != 0) {
              double p = double(Ni) / Ntotal;
              entropy -= p*std::log(p);
            }
          }
          return entropy;
        }

        /**
         * This will compute the gain in information resulting from the split
         */
        static inline double informationGain( const HistogramPair& hp) {
          double e0 = entropy_merged(hp);
          double etrue  = entropy(hp.h_true());
          double efalse = entropy(hp.h_false());

          double Ntrue  = numElements(hp.h_true());
          double Nfalse = numElements(hp.h_false());
          double Ntotal = Ntrue + Nfalse;

          // lets avoid division by 0 
          if( Ntotal == 0 ) return 0.;
          return e0 - (Ntrue/Ntotal)*etrue - (Nfalse/Ntotal)*efalse;
        }

            // #########################################
        // #########################################
        // Reading and writing histograms
        static inline std::ostream& operator << (std::ostream& os, const Histogram& h) {
          for(int li=0;li<NUMLABELS;++li) os<< h[li]<<" ";
          os<<"\n";
          return os;
        }

        static inline std::istream& operator >> (std::istream& is, Histogram& h) {
          for(int li=0;li<NUMLABELS;++li) is >> h[li];
          return is;
        }

        // #######################################
        // #######################################
        // reading and writing histogram Pairs
        static inline std::ostream& operator << ( std::ostream& os, const HistogramPair& hp) {
          os << hp.h_false();
          os << hp.h_true();
          return os;
        }

        static inline std::istream& operator >> ( std::istream& is, HistogramPair& hp) {
          is >> hp.h_false();
          is >> hp.h_true();
          return is;
        }

        // #########################################
        // #########################################
        // Reading and writing LabeledFeature Vectors ( label + collection of attrib )
        static void writeLabeledFeatureVec( std::ostream& os, const std::vector<LabeledFeature>& lfs ){
          os.write( (const char*)&lfs[0], sizeof(LabeledFeature)*lfs.size() );
        }

      //	static void readLabeledFeature( std::istream& is, LabeledFeature& lf)
      //	{
      //		is.read( (char*)&lf, sizeof(LabeledFeature) );
      //		if( is.fail() ) throw std::runtime_error();
      //	}

        // #######################################
        // #######################################
        // reading and writing split points
        inline std::ostream& operator << ( std::ostream& os, const SplitPoint& sp){
          os<<sp.attribId<<" "<<sp.threshold<<"\n";
          return os;
        }

        inline std::istream& operator >> ( std::istream& is, SplitPoint& sp){
          is >> sp.attribId >> sp.threshold;
          return is;
        }

        // #######################################
        // #######################################
        // reading and writing info files
        inline void writeInfoFile( const std::string&   filename,
                                   int                  attribId,
                                   Attrib               threshold,
                                   double               gain,
                                   const HistogramPair& HP){
          std::ofstream fout(filename.c_str() );
          if( !fout.is_open() ) throw std::runtime_error(std::string("(E) could not open ") + filename );

          fout<<int(attribId)<<" "<<int(threshold)<<"\n";
          fout<<gain<<"\n";
          fout<<HP;
        }

        inline void readInfoFile( const std::string& filename,
                                  int&               attribId,
                                  Attrib&            threshold,
                                  double&            gain,
                                  HistogramPair&     HP ) {
          std::ifstream fin(filename.c_str() );
          if( !fin.is_open() ) throw std::runtime_error(std::string("(E) could not open") + filename );

          fin>>attribId >>threshold>>gain>>HP;
          if( fin.fail() ) throw std::runtime_error(std::string("(E) malformed splitInfo file ") + filename );
        }


      } // end namespace trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl
