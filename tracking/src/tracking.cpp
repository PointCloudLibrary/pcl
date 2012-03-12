#include <pcl/tracking/tracking.h>

double
pcl::tracking::sampleNormal (double mean, double sigma)
{
  using namespace boost;
  static mt19937 rng(static_cast<unsigned> (std::time (0)));
  
  normal_distribution<double> norm_dist (mean, sqrt (sigma));
  
  variate_generator<mt19937&, normal_distribution<double> >
    normal_sampler (rng, norm_dist);
  
  return normal_sampler ();
}
