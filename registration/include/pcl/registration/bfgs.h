#ifndef PCL_FOR_EIGEN_BFGS_H
#define PCL_FOR_EIGEN_BFGS_H

#if defined __GNUC__
#  pragma GCC system_header 
#endif

#include <pcl/registration/eigen.h>

namespace Eigen
{
  template< typename _Scalar >
  class PolynomialSolver<_Scalar,2> : public PolynomialSolverBase<_Scalar,2>
  {
    public:
      typedef PolynomialSolverBase<_Scalar,2>    PS_Base;
      EIGEN_POLYNOMIAL_SOLVER_BASE_INHERITED_TYPES( PS_Base )
        
    public:

      virtual ~PolynomialSolver () {}

      template< typename OtherPolynomial >
      inline PolynomialSolver( const OtherPolynomial& poly, bool& hasRealRoot )
      {
        compute( poly, hasRealRoot );
      }
      
      /** Computes the complex roots of a new polynomial. */
      template< typename OtherPolynomial >
      void compute( const OtherPolynomial& poly, bool& hasRealRoot)
      {
        const Scalar ZERO(0);
        Scalar a2(2 * poly[2]);
        assert( ZERO != poly[poly.size()-1] );
        Scalar discriminant ((poly[1] * poly[1]) - (4 * poly[0] * poly[2]));
        if (ZERO < discriminant)
        {
          Scalar discriminant_root (std::sqrt (discriminant));
          m_roots[0] = (-poly[1] - discriminant_root) / (a2) ;
          m_roots[1] = (-poly[1] + discriminant_root) / (a2) ;
          hasRealRoot = true;
        }
        else {
          if (ZERO == discriminant)
          {
            m_roots.resize (1);
            m_roots[0] = -poly[1] / a2;
            hasRealRoot = true;
          }
          else
          {
            Scalar discriminant_root (std::sqrt (-discriminant));
            m_roots[0] = RootType (-poly[1] / a2, -discriminant_root / a2);
            m_roots[1] = RootType (-poly[1] / a2,  discriminant_root / a2);
            hasRealRoot = false;
          }
        }
      }
      
      template< typename OtherPolynomial >
      void compute( const OtherPolynomial& poly)
      {
        bool hasRealRoot;
        compute(poly, hasRealRoot);
      }

    protected:
      using                   PS_Base::m_roots;
  };
}

template<typename _Scalar, int NX=Eigen::Dynamic>
struct BFGSDummyFunctor
{
  typedef _Scalar Scalar;
  enum { InputsAtCompileTime = NX };
  typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> VectorType;

  const int m_inputs;

  BFGSDummyFunctor() : m_inputs(InputsAtCompileTime) {}
  BFGSDummyFunctor(int inputs) : m_inputs(inputs) {}

  virtual ~BFGSDummyFunctor() {}
  int inputs() const { return m_inputs; }

  virtual double operator() (const VectorType &x) = 0;
  virtual void  df(const VectorType &x, VectorType &df) = 0;
  virtual void fdf(const VectorType &x, Scalar &f, VectorType &df) = 0;
};

namespace BFGSSpace {
  enum Status {
    NegativeGradientEpsilon = -3,
    NotStarted = -2,
    Running = -1,
    Success = 0,
    NoProgress = 1
  };
}

/**
 * BFGS stands for Broyden–Fletcher–Goldfarb–Shanno (BFGS) method for solving 
 * unconstrained nonlinear optimization problems. 
 * For further details please visit: http://en.wikipedia.org/wiki/BFGS_method
 * The method provided here is almost similar to the one provided by GSL.
 * It reproduces Fletcher's original algorithm in Practical Methods of Optimization
 * algorithms : 2.6.2 and 2.6.4 and uses the same politics in GSL with cubic 
 * interpolation whenever it is possible else falls to quadratic interpolation for 
 * alpha parameter.
 */
template<typename FunctorType>
class BFGS
{
public:
  typedef typename FunctorType::Scalar Scalar;
  typedef typename FunctorType::VectorType FVectorType;

  BFGS(FunctorType &_functor) 
      : pnorm(0), g0norm(0), iter(-1), functor(_functor) {  }

  typedef Eigen::DenseIndex Index;

  struct Parameters {
    Parameters()
    : max_iters(400)
      , bracket_iters(100)
      , section_iters(100)
      , rho(0.01)
      , sigma(0.01)
      , tau1(9)
      , tau2(0.05)
      , tau3(0.5)
      , step_size(1)
      , order(3) {}
    Index max_iters;   // maximum number of function evaluation
    Index bracket_iters;
    Index section_iters;
    Scalar rho;
    Scalar sigma;
    Scalar tau1;
    Scalar tau2;
    Scalar tau3;
    Scalar step_size;
    Index order;
  };

  BFGSSpace::Status minimize(FVectorType &x);
  BFGSSpace::Status minimizeInit(FVectorType &x);
  BFGSSpace::Status minimizeOneStep(FVectorType &x);
  BFGSSpace::Status testGradient(Scalar epsilon);
  void resetParameters(void) { parameters = Parameters(); }
  
  Parameters parameters;
  Scalar f;
  FVectorType gradient;
private:
  
  BFGS& operator=(const BFGS&);
  BFGSSpace::Status lineSearch (Scalar rho, Scalar sigma, 
                                Scalar tau1, Scalar tau2, Scalar tau3,
                                int order, Scalar alpha1, Scalar &alpha_new);
  Scalar interpolate (Scalar a, Scalar fa, Scalar fpa,
                      Scalar b, Scalar fb, Scalar fpb, Scalar xmin, Scalar xmax,
                      int order);  
  void checkExtremum (const Eigen::Matrix<Scalar, 4, 1>& coefficients, Scalar x, Scalar& xmin, Scalar& fmin);
  void moveTo (Scalar alpha);
  Scalar slope ();
  Scalar applyF (Scalar alpha);
  Scalar applyDF (Scalar alpha);
  void applyFDF (Scalar alpha, Scalar &f, Scalar &df);
  void updatePosition (Scalar alpha, FVectorType& x, Scalar& f, FVectorType& g);
  void changeDirection ();
  
  Scalar delta_f, fp0;
  FVectorType x0, dx0, dg0, g0, dx, p;
  Scalar pnorm, g0norm;

  Scalar f_alpha;
  Scalar df_alpha;
  FVectorType x_alpha;
  FVectorType g_alpha;
  
  // cache "keys"
  Scalar f_cache_key;
  Scalar df_cache_key;
  Scalar x_cache_key;
  Scalar g_cache_key;

  Index iter;
  FunctorType &functor;
};


template<typename FunctorType> void
BFGS<FunctorType>::checkExtremum(const Eigen::Matrix<Scalar, 4, 1>& coefficients, Scalar x, Scalar& xmin, Scalar& fmin)
{
  Scalar y = Eigen::poly_eval(coefficients, x);
  if(y < fmin) { xmin = x; fmin = y; }
}

template<typename FunctorType> void
BFGS<FunctorType>::moveTo(Scalar alpha)
{
  x_alpha = x0 + alpha * p;
  x_cache_key = alpha;
}

template<typename FunctorType> typename BFGS<FunctorType>::Scalar
BFGS<FunctorType>::slope()
{
  return (g_alpha.dot (p));
}

template<typename FunctorType> typename BFGS<FunctorType>::Scalar
BFGS<FunctorType>::applyF(Scalar alpha)
{
  if (alpha == f_cache_key) return f_alpha;
  moveTo (alpha);
  f_alpha = functor (x_alpha);
  f_cache_key = alpha;
  return (f_alpha);
}

template<typename FunctorType> typename BFGS<FunctorType>::Scalar
BFGS<FunctorType>::applyDF(Scalar alpha)
{
  if (alpha == df_cache_key) return df_alpha;
  moveTo (alpha);
  if(alpha != g_cache_key)
  {
    functor.df (x_alpha, g_alpha);
    g_cache_key = alpha;
  }
  df_alpha = slope ();
  df_cache_key = alpha;
  return (df_alpha);
}

template<typename FunctorType> void
BFGS<FunctorType>::applyFDF(Scalar alpha, Scalar& f, Scalar& df)
{
  if(alpha == f_cache_key && alpha == df_cache_key)
  {
    f = f_alpha;
    df = df_alpha;
    return;
  }

  if(alpha == f_cache_key || alpha == df_cache_key)
  {
    f = applyF (alpha);
    df = applyDF (alpha);
    return;
  }

  moveTo (alpha);
  functor.fdf (x_alpha, f_alpha, g_alpha);
  f_cache_key = alpha;
  g_cache_key = alpha;
  df_alpha = slope ();
  df_cache_key = alpha;
  f = f_alpha;
  df = df_alpha;
}

template<typename FunctorType> void
BFGS<FunctorType>::updatePosition (Scalar alpha, FVectorType &x, Scalar &f, FVectorType &g)
{
  { 
    Scalar f_alpha, df_alpha; 
    applyFDF (alpha, f_alpha, df_alpha); 
  } ;

  f = f_alpha;
  x = x_alpha;
  g = g_alpha;
}  

template<typename FunctorType> void
BFGS<FunctorType>::changeDirection ()
{
  x_alpha = x0;
  x_cache_key = 0.0;
  f_cache_key = 0.0;
  g_alpha = g0;
  g_cache_key = 0.0;
  df_alpha = slope ();
  df_cache_key = 0.0;
}

template<typename FunctorType> BFGSSpace::Status
BFGS<FunctorType>::minimize(FVectorType  &x)
{
  BFGSSpace::Status status = minimizeInit(x);
  do {
    status = minimizeOneStep(x);
    iter++;
  } while (status==BFGSSpace::Success && iter < parameters.max_iters);
  return status;
}

template<typename FunctorType> BFGSSpace::Status
BFGS<FunctorType>::minimizeInit(FVectorType  &x)
{
  iter = 0;
  delta_f = 0;
  dx.setZero ();
  functor.fdf(x, f, gradient);
  x0 = x;
  g0 = gradient;
  g0norm = g0.norm ();
  p = gradient * -1/g0norm;
  pnorm = p.norm ();
  fp0 = -g0norm;

  {
    x_alpha = x0; x_cache_key = 0;
    
    f_alpha = f; f_cache_key = 0;
    
    g_alpha = g0; g_cache_key = 0;
    
    df_alpha = slope (); df_cache_key = 0;
  }

  return BFGSSpace::NotStarted;
}

template<typename FunctorType> BFGSSpace::Status
BFGS<FunctorType>::minimizeOneStep(FVectorType  &x)
{
  Scalar alpha = 0.0, alpha1;
  Scalar f0 = f;
  if (pnorm == 0.0 || g0norm == 0.0 || fp0 == 0)
  {
    dx.setZero ();
    return BFGSSpace::NoProgress;
  }

  if (delta_f < 0)
  {
    Scalar del = std::max (-delta_f, 10 * std::numeric_limits<Scalar>::epsilon() * fabs(f0));
    alpha1 = std::min (1.0, 2.0 * del / (-fp0));
  }
  else
    alpha1 = fabs(parameters.step_size);

  BFGSSpace::Status status = lineSearch(parameters.rho, parameters.sigma, 
                                        parameters.tau1, parameters.tau2, parameters.tau3, 
                                        parameters.order, alpha1, alpha);

  if(status != BFGSSpace::Success)
    return status;

  updatePosition(alpha, x, f, gradient);

  delta_f = f - f0;

  /* Choose a new direction for the next step */
  {
    /* This is the BFGS update: */
    /* p' = g1 - A dx - B dg */
    /* A = - (1+ dg.dg/dx.dg) B + dg.g/dx.dg */
    /* B = dx.g/dx.dg */

    Scalar dxg, dgg, dxdg, dgnorm, A, B;

    /* dx0 = x - x0 */
    dx0 = x - x0;
    dx = dx0; /* keep a copy */

    /* dg0 = g - g0 */
    dg0 = gradient - g0;
    dxg = dx0.dot (gradient);
    dgg = dg0.dot (gradient);
    dxdg = dx0.dot (dg0);
    dgnorm = dg0.norm ();

    if (dxdg != 0)
    {
      B = dxg / dxdg;
      A = -(1.0 + dgnorm * dgnorm / dxdg) * B + dgg / dxdg;
    }
    else
    {
      B = 0;
      A = 0;
    }

    p = -A * dx0;
    p+= gradient;
    p+= -B * dg0 ;
  }

  g0 = gradient;
  x0 = x;
  g0norm = g0.norm ();
  pnorm = p.norm ();
  
  Scalar dir = ((p.dot (gradient)) > 0) ? -1.0 : 1.0;
  p*= dir / pnorm;
  pnorm = p.norm ();
  fp0 = p.dot (g0);

  changeDirection();
  return BFGSSpace::Success;
}

template<typename FunctorType> typename BFGSSpace::Status 
BFGS<FunctorType>::testGradient(Scalar epsilon)
{
  if(epsilon < 0)
    return BFGSSpace::NegativeGradientEpsilon;
  else
  {
    if(gradient.norm () < epsilon)
      return BFGSSpace::Success;
    else
      return BFGSSpace::Running;
  }
}

template<typename FunctorType> typename BFGS<FunctorType>::Scalar 
BFGS<FunctorType>::interpolate (Scalar a, Scalar fa, Scalar fpa,
                                Scalar b, Scalar fb, Scalar fpb, 
                                Scalar xmin, Scalar xmax,
                                int order)
{
  /* Map [a,b] to [0,1] */
  Scalar y, alpha, ymin, ymax, fmin;

  ymin = (xmin - a) / (b - a);
  ymax = (xmax - a) / (b - a);
  
  // Ensure ymin <= ymax
  if (ymin > ymax) { Scalar tmp = ymin; ymin = ymax; ymax = tmp; };

  if (order > 2 && !(fpb != fpa) && fpb != std::numeric_limits<Scalar>::infinity ())
  {
    fpa = fpa * (b - a);
    fpb = fpb * (b - a);

    Scalar eta = 3 * (fb - fa) - 2 * fpa - fpb;
    Scalar xi = fpa + fpb - 2 * (fb - fa);
    Scalar c0 = fa, c1 = fpa, c2 = eta, c3 = xi;
    Scalar y0, y1;
    Eigen::Matrix<Scalar, 4, 1> coefficients;
    coefficients << c0, c1, c2, c3;
    
    y = ymin; 
    // Evaluate the cubic polyinomial at ymin;
    fmin = Eigen::poly_eval (coefficients, ymin);
    checkExtremum (coefficients, ymax, y, fmin);
    {
      // Solve quadratic polynomial for the derivate
      Eigen::Matrix<Scalar, 3, 1> coefficients2;
      coefficients2 << c1, 2 * c2, 3 * c3;
      bool real_roots;
      Eigen::PolynomialSolver<Scalar, 2> solver (coefficients2, real_roots);
      if(real_roots)
      {
        if ((solver.roots ()).size () == 2)  /* found 2 roots */
        {
          y0 = std::real (solver.roots () [0]);
          y1 = std::real (solver.roots () [1]);
          if(y0 > y1) { Scalar tmp (y0); y0 = y1; y1 = tmp; }
          if (y0 > ymin && y0 < ymax) 
            checkExtremum (coefficients, y0, y, fmin);
          if (y1 > ymin && y1 < ymax) 
            checkExtremum (coefficients, y1, y, fmin);
        }
        else if ((solver.roots ()).size () == 1)  /* found 1 root */
        {
          y0 = std::real (solver.roots () [0]);
          if (y0 > ymin && y0 < ymax) 
            checkExtremum (coefficients, y0, y, fmin);
        }
      }
    }
  } 
  else 
  {
    fpa = fpa * (b - a);
    Scalar fl = fa + ymin*(fpa + ymin*(fb - fa -fpa));
    Scalar fh = fa + ymax*(fpa + ymax*(fb - fa -fpa));
    Scalar c = 2 * (fb - fa - fpa);       /* curvature */
    y = ymin; fmin = fl;
    
    if (fh < fmin) { y = ymax; fmin = fh; } 
    
    if (c > a)  /* positive curvature required for a minimum */
    {
      Scalar z = -fpa / c;      /* location of minimum */
      if (z > ymin && z < ymax) {
        Scalar f = fa + z*(fpa + z*(fb - fa -fpa));
        if (f < fmin) { y = z; fmin = f; };
      }
    }
  }
  
  alpha = a + y * (b - a);
  return alpha;
}

template<typename FunctorType> BFGSSpace::Status 
BFGS<FunctorType>::lineSearch(Scalar rho, Scalar sigma, 
                              Scalar tau1, Scalar tau2, Scalar tau3,
                              int order, Scalar alpha1, Scalar &alpha_new)
{
  Scalar f0, fp0, falpha, falpha_prev, fpalpha, fpalpha_prev, delta, alpha_next;
  Scalar alpha = alpha1, alpha_prev = 0.0;
  Scalar a, b, fa, fb, fpa, fpb;
  Index i = 0;

  applyFDF (0.0, f0, fp0);

  falpha_prev = f0;
  fpalpha_prev = fp0;

  /* Avoid uninitialized variables morning */
  a = 0.0; b = alpha;
  fa = f0; fb = 0.0;
  fpa = fp0; fpb = 0.0;

  /* Begin bracketing */  

  while (i++ < parameters.bracket_iters)
  {
    falpha = applyF (alpha);

    if (falpha > f0 + alpha * rho * fp0 || falpha >= falpha_prev)
    {
      a = alpha_prev; fa = falpha_prev; fpa = fpalpha_prev;
      b = alpha; fb = falpha; fpb = std::numeric_limits<Scalar>::quiet_NaN ();
      break;
    } 

    fpalpha = applyDF (alpha);

    /* Fletcher's sigma test */
    if (fabs (fpalpha) <= -sigma * fp0)
    {
      alpha_new = alpha;
      return BFGSSpace::Success;
    }

    if (fpalpha >= 0)
    {
      a = alpha; fa = falpha; fpa = fpalpha;
      b = alpha_prev; fb = falpha_prev; fpb = fpalpha_prev;
      break;                /* goto sectioning */
    }

    delta = alpha - alpha_prev;

    {
      Scalar lower = alpha + delta;
      Scalar upper = alpha + tau1 * delta;

      alpha_next = interpolate (alpha_prev, falpha_prev, fpalpha_prev,
                                alpha, falpha, fpalpha, lower, upper, order);

    }

    alpha_prev = alpha;
    falpha_prev = falpha;
    fpalpha_prev = fpalpha;
    alpha = alpha_next;
  }
  /*  Sectioning of bracket [a,b] */
  while (i++ < parameters.section_iters)
  {
    delta = b - a;
      
    {
      Scalar lower = a + tau2 * delta;
      Scalar upper = b - tau3 * delta;
        
      alpha = interpolate (a, fa, fpa, b, fb, fpb, lower, upper, order);
    }
    falpha = applyF (alpha);
    if ((a-alpha)*fpa <= std::numeric_limits<Scalar>::epsilon ()) {
      /* roundoff prevents progress */
      return BFGSSpace::NoProgress;
    };

    if (falpha > f0 + rho * alpha * fp0 || falpha >= fa)
    {
      /*  a_next = a; */
      b = alpha; fb = falpha; fpb = std::numeric_limits<Scalar>::quiet_NaN ();
    }
    else
    {
      fpalpha = applyDF (alpha);
          
      if (fabs(fpalpha) <= -sigma * fp0)
      {
        alpha_new = alpha;
        return BFGSSpace::Success;  /* terminate */
      }
          
      if ( ((b-a) >= 0 && fpalpha >= 0) || ((b-a) <=0 && fpalpha <= 0))
      {
        b = a; fb = fa; fpb = fpa;
        a = alpha; fa = falpha; fpa = fpalpha;
      }
      else
      {
        a = alpha; fa = falpha; fpa = fpalpha;
      }
    }
  }
  return BFGSSpace::Success;
}
#endif // PCL_FOR_EIGEN_BFGS_H

