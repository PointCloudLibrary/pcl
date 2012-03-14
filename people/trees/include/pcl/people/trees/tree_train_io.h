/* *************************************************
 *
 * Copyright (2011) Willow Garage
 *
 * Author : Cedric Cagniart 
 * ************************************************* */

#include "TTrain.h"
#include <fstream>
#include <stdexcept>
#include <iostream>

#include "Tree_io.h"

namespace Tree
{

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
		if( !fout.is_open() ) throw std::runtime_error(std::string("(E) could not open") + filename );

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




} // end namespace Tree
