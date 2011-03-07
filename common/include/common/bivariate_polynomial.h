#ifndef BIVARIATE_POLYNOMIAL_H
#define BIVARIATE_POLYNOMIAL_H

#include <fstream>
#include <iostream>

namespace pcl {

/** \brief @b This represents a bivariate polynomial and provides some functionality for it
  * \author Bastian Steder */
template<typename real>
class BivariatePolynomialT {

  public:
    //-----CONSTRUCTOR&DESTRUCTOR-----
    /** Constructor */
    BivariatePolynomialT (int new_degree=0);
    /** Copy constructor */
    BivariatePolynomialT (const BivariatePolynomialT& other);
    /** Destructor */
    ~BivariatePolynomialT ();

    //-----OPERATORS-----
    /** = operator */
    BivariatePolynomialT&
      operator= (const BivariatePolynomialT& other) { deepCopy (other); return *this;}

    //-----METHODS-----
    /** Initialize members to default values */
    void
      setDegree (int new_degree);
    /** How many parametes has a bivariate polynomial with this degree */
    unsigned int
      getNoOfParameters () const { return getNoOfParametersFromDegree (degree);}
    /** Calculate the value of the polynomial at the given point */
    real
      getValue (real x, real y) const;  
    /** Calculate the gradient of this polynomial
     *  If forceRecalc is false, it will do nothing when the gradient already exists */
    void
      calculateGradient (bool forceRecalc=false);
    /** Calculate the value of the gradient at the given point */
    void
      getValueOfGradient (real x, real y, real& gradX, real& gradY);
    
    /** Returns critical points of the polynomial. type can be 0=maximum, 1=minimum, or 2=saddle point
     *  !!Currently only implemented for degree 2!! */
    void
      findCriticalPoints (std::vector<real>& x_values, std::vector<real>& y_values, std::vector<int>& types) const;
    
    /** write as binary to a stream */
    void
      writeBinary (std::ostream& os) const;
    /** write as binary into a file */
    void
      writeBinary (const char* filename) const;
    /** read binary from a stream */
    void
      readBinary (std::istream& os);
    /** read binary from a file */
    void
      readBinary (const char* filename);
    
    /** How many parametes has a bivariate polynomial of the given degree */
    static unsigned int
      getNoOfParametersFromDegree (int n) { return ((n+2)* (n+1))/2;}
    //-----VARIABLES-----
    int degree;
    real* parameters;
    BivariatePolynomialT<real>* gradient_x, * gradient_y;
    
  protected:
    //-----METHODS-----
    /** Delete all members */
    void
      memoryCleanUp ();
    /** Create a deep copy of the given polynomial */
    void
      deepCopy (const BivariatePolynomialT<real>& other);
  //-----VARIABLES-----
};

template<typename real>
std::ostream&
  operator<< (std::ostream& os, const BivariatePolynomialT<real>& p);

typedef BivariatePolynomialT<double> BivariatePolynomiald;
typedef BivariatePolynomialT<float>  BivariatePolynomial;

}  // end namespace

#include "pcl/common/bivariate_polynomial.hpp"

#endif
