/* *************************************************
 *
 * Copyright (2011) Willow Garage
 *
 * Author : Cedric Cagniart 
 * ************************************************* */

#ifndef TTRAIN_H_DEFINED
#define TTRAIN_H_DEFINED

#include "Tree.h"
#include <boost/array.hpp>

namespace Tree
{


	// ################################################
	// ################################################
	// histogram stuff
	class Histogram : public boost::array<uint32_t,NUMLABELS> {
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


} // end namespace Tree










#endif
