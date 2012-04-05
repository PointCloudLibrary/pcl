/* *************************************************
 *
 * Copyright (2011) Willow Garage
 *
 * Author : Cedric Cagniart, Koen Buys
 * ************************************************* */

#ifndef PCL_GPU_PEOPLE_TREES_TTRAIN_HISTOGRAM_H_
#define PCL_GPU_PEOPLE_TREES_TTRAIN_HISTOGRAM_H_

#include "TTrain.h"
#include <cmath>

#include <iostream>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace trees
      {
        // compute the number of elements
        static inline uint64_t numElements( const Histogram& h ) {
        	uint64_t Ntotal = 0;
        	for(int li=0;li<NUMLABELS;++li) Ntotal += uint64_t(h[li]);
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
        			entropy -= p*log(p);
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
        		uint64_t Ni = uint64_t(htrue[li]) + uint64_t(hfalse[li]);
        		if( Ni != 0) {
        			double p = double(Ni) / Ntotal;
        			entropy -= p*log(p);
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
      } // end namespace trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl
#endif
