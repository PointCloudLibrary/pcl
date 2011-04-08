template<typename real>
pcl::BivariatePolynomialT<real>::BivariatePolynomialT (int new_degree) :
  degree(0), parameters(NULL), gradient_x(NULL), gradient_y(NULL)
{
  setDegree(new_degree);
}

template<typename real>
pcl::BivariatePolynomialT<real>::BivariatePolynomialT (const BivariatePolynomialT& other) :
  degree(0), parameters(NULL), gradient_x(NULL), gradient_y(NULL)
{
  deepCopy (other);
}

template<typename real>
pcl::BivariatePolynomialT<real>::~BivariatePolynomialT ()
{
  memoryCleanUp ();
}

template<typename real>
void
  pcl::BivariatePolynomialT<real>::setDegree (int newDegree)
{
  if (newDegree <= 0)
  {
    degree = -1;
    memoryCleanUp();
    return;
  }
  int oldDegree = degree;
  degree = newDegree;
  if (oldDegree != degree)
  {
    delete[] parameters;
    parameters = new real[getNoOfParameters ()];
  }
  delete gradient_x; gradient_x = NULL;
  delete gradient_y; gradient_y = NULL;
}

template<typename real>
void
  pcl::BivariatePolynomialT<real>::memoryCleanUp ()
{
  delete[] parameters; parameters = NULL;
  delete gradient_x; gradient_x = NULL;
  delete gradient_y; gradient_y = NULL;
}

template<typename real>
void
  pcl::BivariatePolynomialT<real>::deepCopy (const pcl::BivariatePolynomialT<real>& other)
{
  if (this == &other) return;
  if (degree != other.degree) {
    memoryCleanUp ();
    degree = other.degree;
    parameters = new real[getNoOfParameters ()];
  }
  if (other.gradient_x == NULL) {
    delete gradient_x; gradient_x=NULL;
    delete gradient_y; gradient_y=NULL;
  }
  else if (gradient_x==NULL) {
    gradient_x = new pcl::BivariatePolynomialT<real> ();
    gradient_y = new pcl::BivariatePolynomialT<real> ();
  }
  real* tmpParameters1 = parameters;
  const real* tmpParameters2 = other.parameters;
  unsigned int noOfParameters = getNoOfParameters ();
  for (unsigned int i=0; i<noOfParameters; i++)
    *tmpParameters1++ = *tmpParameters2++;

  if (other.gradient_x != NULL) {
    gradient_x->deepCopy (*other.gradient_x);
    gradient_y->deepCopy (*other.gradient_y);
  }
}

template<typename real>
void
  pcl::BivariatePolynomialT<real>::calculateGradient (bool forceRecalc)
{
  if (gradient_x!=NULL && !forceRecalc) {
    //cout << "Gradient already exists\n";
    return;
  }
  
  if (gradient_x == NULL)
    gradient_x = new pcl::BivariatePolynomialT<real> (degree-1);
  if (gradient_y == NULL)
    gradient_y = new pcl::BivariatePolynomialT<real> (degree-1);
  
  unsigned int parameterPosDx=0, parameterPosDy=0;
  for (int xDegree=degree; xDegree>=0; xDegree--) {
    for (int yDegree=degree-xDegree; yDegree>=0; yDegree--) {
      if (xDegree > 0) {
        gradient_x->parameters[parameterPosDx] = xDegree * parameters[parameterPosDx];
        parameterPosDx++;
      }
      if (yDegree > 0) {
        gradient_y->parameters[parameterPosDy] = yDegree * parameters[ ( (degree+2-xDegree)* (degree+1-xDegree))/2 -
                                                                        yDegree - 1];
        parameterPosDy++;
      }
    }
  }
  //cout << "Original polynom: "<<*this<<"\n"<<"d/dx: "<<*gradient_x<<"\n"<<"d/dy: "<<*gradient_y<<"\n";
}

template<typename real>
real
  pcl::BivariatePolynomialT<real>::getValue (real x, real y) const
{
  unsigned int parametersSize = getNoOfParameters ();
  real* tmpParameter = &parameters[parametersSize-1];
  real tmpX=1.0, tmpY, ret=0;
  for (int xDegree=0; xDegree<=degree; xDegree++) {
    tmpY = 1.0;
    for (int yDegree=0; yDegree<=degree-xDegree; yDegree++)
    {
      ret += (*tmpParameter)*tmpX*tmpY;
      tmpY *= y;
      tmpParameter--;
    }
    tmpX *= x;
  }
  return ret;
}

template<typename real>
void
  pcl::BivariatePolynomialT<real>::getValueOfGradient (real x, real y, real& gradX, real& gradY)
{
  calculateGradient ();
  gradX = gradient_x->getValue (x, y);
  gradY = gradient_y->getValue (x, y);
  //cout << "Gradient at "<<x<<", "<<y<<" is "<<gradX<<", "<<gradY<<"\n";
}

//elliptischer Punkt hyperbolischer Punkt parabolischer Punkt
//Bei Funktionen in zwei Ver¨anderlichen kann der Typ anhand der Determinante der Hesse-
//Matrix klassifiziert werden. Ist det(Hf) > 0 (< 0), so handelt es sich um ein Extremum (einen
//Sattelpunkt). F¨ur ein Minimum bzw. ein Maximum ist Spur(Hf) > 0 bzw. < 0 . Verschwindet
//die Determinante und ist die Hesse-Matrix nicht Null, so ist der Punkt parabolisch.

template<typename real>
void
  pcl::BivariatePolynomialT<real>::findCriticalPoints (std::vector<real>& x_values, std::vector<real>& y_values,
                                                       std::vector<int>& types) const
{
  x_values.clear ();
  y_values.clear ();
  types.clear ();
  
  if (degree == 2)
  {
    real x = (real(2)*parameters[2]*parameters[3] - parameters[1]*parameters[4]) /
             (parameters[1]*parameters[1] - real(4)*parameters[0]*parameters[3]),
         y = (real(-2)*parameters[0]*x - parameters[2]) / parameters[1];
    
    if (!pcl_isfinite(x) || !pcl_isfinite(y))
      return;
    
    int type = 2;
    real det_H = real(4)*parameters[0]*parameters[3] - parameters[1]*parameters[1];
    //std::cout << "det(H) = "<<det_H<<"\n";
    if (det_H > real(0))  // Check Hessian determinant
    {
      if (parameters[0]+parameters[3] < real(0))  // Check Hessian trace
        type = 0;
      else
        type = 1;
    }
    x_values.push_back(x);
    y_values.push_back(y);
    types.push_back(type);
    
    //real gx, gy;
    //((BivariatePolynomialT<real>*)this)->getValueOfGradient (x, y, gx, gy);
    //std::cout << "Check: p'("<<x<<", "<<y<<") = ("<<gx<<", "<<gy<<")\n";
  }
  else
  {
    std::cerr << __PRETTY_FUNCTION__ << " is not implemented for polynomials of degree "<<degree<<". Sorry.\n";
  }
}

template<typename real>
std::ostream&
  pcl::operator<< (std::ostream& os, const pcl::BivariatePolynomialT<real>& p)
{
  real* tmpParameter = p.parameters;
  bool first = true;
  real currentParameter;
  for (int xDegree=p.degree; xDegree>=0; xDegree--) {
    for (int yDegree=p.degree-xDegree; yDegree>=0; yDegree--) {
      currentParameter = *tmpParameter;
      if (!first) {
        os << (currentParameter<0.0?" - ":" + ");
        currentParameter = fabs (currentParameter);
      }
      os << currentParameter;
      if (xDegree>0) {
        os << "x";
        if (xDegree>1)
          os<<"^"<<xDegree;
      }
      if (yDegree>0) {
        os << "y";
        if (yDegree>1)
          os<<"^"<<yDegree;
      }
      
      first = false;
      tmpParameter++;
    }
  }
  return os;
}

template<typename real>
void
  pcl::BivariatePolynomialT<real>::writeBinary (std::ostream& os) const
{
  os.write ( (char*)&degree, sizeof (int));
  unsigned int paramCnt = getNoOfParametersFromDegree (this->degree);
  os.write ( (char*) (this->parameters), paramCnt * sizeof (real));
}

template<typename real>
void
  pcl::BivariatePolynomialT<real>::writeBinary (const char* filename) const
{
  std::ofstream fout (filename);
  writeBinary (fout);
}

template<typename real>
void
  pcl::BivariatePolynomialT<real>::readBinary (std::istream& os)
{
  memoryCleanUp ();
  os.read ( (char*)& (this->degree), sizeof (int));
  unsigned int paramCnt = getNoOfParametersFromDegree (this->degree);
  parameters = new real[paramCnt];
  os.read ( (char*)& (*this->parameters), paramCnt * sizeof (real));
}

template<typename real>
void
  pcl::BivariatePolynomialT<real>::readBinary (const char* filename)
{
  std::ifstream fin (filename);
  readBinary (fin);
}



