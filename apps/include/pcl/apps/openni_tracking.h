// -*- mode: c++ -*-

#ifndef OPENNI_TRACKING_H_
#define OPENNI_TRACKING_H_

#include <pcl/common/common.h>
#include <pcl/common/transform.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <boost/function.hpp>
#include <boost/random.hpp>
#include <boost/math/distributions/normal.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

namespace pcl
{
  namespace apps
  {
    // pfilterlib.h
    namespace PF                // utilities related with particle filter
    {
      double
      SampleNormal (double mean, double sigma)
      {
        using namespace boost;
        static mt19937 rng(static_cast<unsigned> (std::time (0)));
        
        normal_distribution<double> norm_dist (mean, sqrt (sigma));
        
        variate_generator<mt19937&, normal_distribution<double> >
          normal_sampler (rng, norm_dist);
        
        return normal_sampler ();
      }
      
      class Particle
      {
      protected:
        double weight_;
        double orig_likelihood_;
        std::vector<double> state_;
        
      public:
        Particle () {}
        Particle (const int dim): weight_ (0.0), state_ (dim, 0.0) {}
        Particle (const Particle& orig): weight_ (orig.weight_),
                                         orig_likelihood_ (orig.orig_likelihood_)
          {
            state_ = std::vector<double>(orig.getState().size());
            for (size_t i = 0; i < orig.getState().size(); i++)
              state_[i] = orig.getStateValue(i);
          }
        virtual
        ~Particle () {}

        virtual void
        sample(const std::vector<double>& mean,
               const std::vector<double>& cov)
          {
            for ( size_t j = 0; j < state_.size(); j++ )
            {
              state_[j] += SampleNormal(mean[j], cov[j]);
            }
          }

        // accessors
        virtual inline const std::vector<double>&
        getState () const
          {
            return state_;
          }
        
        /* does not overwrite orig_likelihood_ */
        virtual inline void
        updateWeight (const double weight)
          {
            weight_ = weight;
          }
        
        // overwrite orig_likelihood_
        virtual inline void
        setWeight (const double weight)
          {
            weight_ = weight;
            orig_likelihood_ = weight;
          }
        
        virtual inline double
        getWeight () const { return weight_; };
        
        virtual inline double
        getStateValue (const size_t i) const
          {
            return state_[i];
          }
        
        virtual inline void
        setStateValue (const size_t i, const double val)
          {
            state_[i] = val;
          }
        
        virtual inline double
        getOriginalLikelihood () const
          {
            return orig_likelihood_;
          }

        virtual inline void
        zero () // zero set
          {
            for ( size_t j = 0; j < state_.size(); j++ )
              state_[j] = 0.0;
          }
        
      };

      // for debuging
      std::ostream&
      operator << (std::ostream& s, const Particle &p)
      {
        s << p.getState ()[0] << ", " << p.getState ()[1] << ", "
          << p.getState ()[2] << ", " << p.getState ()[3] << ", "
          << p.getState ()[4] << ", " << p.getState ()[5];
        return s;
      }
    
      typedef enum
      {
        UPDATE_MEAN, UPDATE_MAX
      } UpdateParticleFilterMethod;
      
      class ParticleFilter
      {
        // using gaussian to make noise
      protected:
        boost::function<double (std::vector<double>)> likelihood_function_;
        unsigned int dim_;
        unsigned int particle_num_;
        std::vector<double> initial_noise_covariance_;
        std::vector<double> initial_noise_mean_;
        std::vector<double> step_noise_covariance_;
        UpdateParticleFilterMethod update_method_;
        std::vector<Particle> particles_;
        Particle representative_state_;
        
        void
        normalizeWeight ()
          {
            // calc weight summation
            double sum = 0.0;
            for ( unsigned int i = 0; i < particle_num_; i++ )
              sum += particles_[i].getWeight ();
            // too small weight summation, re-initialize particles
            // if ( sum <= 1e-5 )
            //   initParticles ();
            //else
              for ( unsigned int i = 0; i < particle_num_; i++ )
                particles_[i].updateWeight (particles_[i].getWeight() / sum);
          }
        
      public:
        ParticleFilter (unsigned int dim, unsigned int particle_num,
                        std::vector<double> initial_noise_mean,
                        std::vector<double> initial_noise_covariance,
                        std::vector<double> step_noise_covariance,
                        boost::function<double (std::vector<double>)> likelihood_function,
                        UpdateParticleFilterMethod method = UPDATE_MEAN):
          likelihood_function_ (likelihood_function), dim_ (dim), particle_num_ (particle_num),
          initial_noise_covariance_ (initial_noise_covariance),
          initial_noise_mean_ (initial_noise_mean),
          step_noise_covariance_ (step_noise_covariance),
          update_method_ (method)
          {
            representative_state_ = Particle (dim);
            initParticles ();
          }
        
        virtual inline Particle
        getResult (){ return representative_state_; }
        virtual
        ~ParticleFilter() { }
        
        /* sampling with replacement based on Walker's alias method
                   
           @article{355749,
           author = {Walker, Alastair J.},
           title = {An Efficient Method for Generating Discrete
           Random Variables with General Distributions},
           journal = {ACM Trans. Math. Softw.},
           volume = {3},
           number = {3},
           year = {1977},
           issn = {0098-3500},
           pages = {253--256},
           doi = {http://doi.acm.org/10.1145/355744.355749},
           publisher = {ACM},
           address = {New York, NY, USA},
           }
        */
        virtual void
        genAliasTable (std::vector<int> &a, std::vector<double> &q)
          {
            /* generate an alias table, a and q */
            std::vector<int> HL (particle_num_);
            std::vector<int>::iterator H = HL.begin ();
            std::vector<int>::iterator L = HL.end () - 1;
            for ( unsigned int i = 0; i < particle_num_; i++ )
              q[i] = particles_[i].getWeight() * particle_num_;
            for ( unsigned int i = 0; i < particle_num_; i++ )
              a[i] = i;
            // setup H and L
            for ( unsigned int i = 0; i < particle_num_; i++ )
              if ( q[i] >= 1.0 )
                *H++ = i;
              else
                *L-- = i;
            
            while ( H != HL.begin() && L != HL.end() - 1 )
            {
              int j = *(L + 1);
              int k = *(H - 1);
              a[j] = k;
              q[k] += q[j] - 1;
              L++;
              if ( q[k] < 1.0 )
              {
                *L-- = k;
                --H;
              }
            }
          }

        virtual inline int
        sampleWithReplacement (const std::vector<int>& a,
                               const std::vector<double>& q)
          {
            using namespace boost;
            static mt19937 gen (static_cast<unsigned long>(time (0)));
            uniform_real<> dst (0.0, 1.0);
            variate_generator<mt19937&, uniform_real<> > rand (gen, dst);
            double rU = rand () * particle_num_;
            int k = (int)rU;
            rU -= k;    /* rU - [rU] */
            if ( rU < q[k] )
              return k;
            else
              return a[k];
          }
        
        void
        resample ()
          {
            std::vector<int> a (particle_num_);
            std::vector<double> q (particle_num_);
            genAliasTable (a, q);
            
            const static std::vector<double> zero_mean (dim_, 0.0);
            // memoize the original list of particles
            std::vector<Particle> origparticles = particles_;
            
            particles_.clear ();
            // the first particle, it is a just copy of the maximum result
            Particle p = representative_state_;
            particles_.push_back (p);
            for ( unsigned int i = 1; i < particle_num_; i++ )
            {
              int target_particle_index = sampleWithReplacement (a, q);
              Particle p = origparticles[target_particle_index];
              // add noise using gaussian
              p.sample (zero_mean, step_noise_covariance_);
              particles_.push_back (p);
            }
          }
        
        void
        weight ()
          {
            // call likelihood_function_ for each particles
            for ( unsigned int i = 0; i < particle_num_; i++ )
            {
              double likelihood = likelihood_function_ (particles_[i].getState ()); // call likelihood function
              particles_[i].setWeight (likelihood);
            }
            normalizeWeight();
          }
        
        void
        updateMean ()
          {
            //fill by zero
            representative_state_.zero ();
            // cannot define the weight of representative_state_.
            // so we define the weight of representative_state_ as
            // the miximum weight of the particles
            double maxweight_ = 0.0;
            for ( unsigned int i = 0; i < particle_num_; i++)
            {
              Particle p = particles_[i];
              if ( maxweight_ < p.getOriginalLikelihood () )
                maxweight_ = p.getOriginalLikelihood ();
              for ( unsigned int j = 0; j < dim_; j++ )
              {
                double a = p.getStateValue (j) * p.getWeight ();
                double b = representative_state_.getStateValue (j);
                representative_state_.setStateValue (j, a + b);
              }
            }
            representative_state_.setWeight (maxweight_);
          }
        
        void
        updateMax ()
          {
            int max_index = 0;
            double maxweight_ = 0.0;
            for ( unsigned int i = 0; i < particle_num_; i++)
            {
              Particle p = particles_[i];
              if ( p.getWeight() > maxweight_ )
              {
                maxweight_ = p.getWeight();
                max_index = i;
              }
            }
            representative_state_ = particles_[max_index];
          }
        
        inline void
        update ()
          {
            // update
            switch (update_method_)
            {
            case UPDATE_MEAN:
              updateMean ();
              break;
            case UPDATE_MAX:
              updateMax ();
              break;
            default:            // should raise an exception?
              break;
            }
          }
        
        void
        initParticles ()    // finish
          {
            std::cout << "initParticles" << std::endl;
            particles_ = std::vector<Particle> (particle_num_, dim_);
            representative_state_.zero ();
            representative_state_.setWeight (0.0);
            // sampling...
            for ( unsigned int i = 0; i < particle_num_; i++ )
            {
              Particle p (dim_);
              // w = 1 / N;
              p.setWeight (1.0 / particle_num_);
              p.zero ();
              p.sample (initial_noise_mean_, initial_noise_covariance_);
              particles_[i] = p; // update
            }
          }
        
        inline std::vector<Particle>
        getParticles ()
          {
            return particles_;
          }
      };
    }

    // pfilter_util.h
    class ParticleFilterParameter
    {
    public:
      ParticleFilterParameter (const double x_width,
                               const double y_width,
                               const double z_width,
                               const double roll_width,
                               const double pitch_width,
                               const double yaw_width,
                               const double x_offset,
                               const double y_offset,
                               const double z_offset,
                               const double roll_offset,
                               const double pitch_offset,
                               const double yaw_offset)
        : x_width_ (x_width), y_width_ (y_width), z_width_ (z_width),
          roll_width_ (roll_width), pitch_width_ (pitch_width),
          yaw_width_ (yaw_width),
          x_offset_ (x_offset), y_offset_ (y_offset), z_offset_ (z_offset),
          roll_offset_ (roll_offset), pitch_offset_ (pitch_offset),
          yaw_offset_ (yaw_offset),
          trans_ (Eigen::Affine3f::Identity ())
        {
          // do nothing
        }
      
      virtual ~ParticleFilterParameter () {}

      inline double getXWidth () const {return x_width_;}
      inline double getYWidth () const {return y_width_;}
      inline double getZWidth () const {return z_width_;}
      inline double getRollWidth () const {return roll_width_;}
      inline double getPitchWidth () const {return pitch_width_;}
      inline double getYawWidth () const {return yaw_width_;}
      inline double getXOffset () const {return x_offset_;}
      inline double getYOffset () const {return y_offset_;}
      inline double getZOffset () const {return z_offset_;}
      inline double getRollOffset () const {return roll_offset_;}
      inline double getPitchOffset () const {return pitch_offset_;}
      inline double getYawOffset () const {return yaw_offset_;}

      inline void setTrans (const Eigen::Affine3f& trans) { trans_ = trans; }
      
      Eigen::Vector3f
      getOffsetVector ()
        {
          Eigen::Vector3f offset;
          offset[0] = getXOffset ();
          offset[1] = getYOffset ();
          offset[2] = getZOffset ();
          return offset;
        }
      
      Eigen::Matrix4f
      getOffsetMatrix ()
        {
          Eigen::Vector3f offset = getOffsetVector ();
          Eigen::Matrix4f search_trans = Eigen::Matrix4f::Identity ();
          for (int i = 0; i < 3; i++)
            search_trans (i, 3) = offset[i];
          return search_trans;
        }
      
      double
      normalizeAngle (const double val)
        {
          if ( val > M_PI * 2.0 )
          {
            int offset = val / ( M_PI * 2.0 );
            return val - M_PI * 2.0 * offset;
          }
          else if ( val < 0.0 )
          {
            int offset = abs(val) / ( M_PI * 2.0 );
            return val + M_PI * 2.0 * offset;
          }
          else
          {
            return val;
          }
        }
      
      Eigen::Affine3f
      toEigenMatrix (std::vector<double> state)
        {
          double x = state[0];
          double y = state[1];
          double z = state[2];
          double roll = state[3];
          double pitch = state[4];
          double yaw = state[5];
          
          double xval = getXWidth () * ( x - 0.5 ) + getXOffset ();
          double yval = getYWidth () * ( y - 0.5 ) + getYOffset ();
          double zval = getZWidth () * ( z - 0.5 ) + getZOffset ();
          // i need to normalize euler angles?
          double rollval = normalizeAngle(roll_width_ * ( roll - 0.5 ) + roll_offset_);
          double pitchval = normalizeAngle(pitch_width_ * ( pitch - 0.5 ) + pitch_offset_);
          double yawval = normalizeAngle(yaw_width_ * ( yaw - 0.5 ) + yaw_offset_);
          //Eigen::Affine3f particle_trans = getTransformation(xval, yval, zval, rollval, pitchval, yawval);
          
          return trans_ * getTransformation(xval, yval, zval, rollval, pitchval, yawval);
        }
      
      
    protected:
      const double x_width_;
      const double y_width_;
      const double z_width_;
      const double roll_width_;
      const double pitch_width_;
      const double yaw_width_;
      const double x_offset_;
      const double y_offset_;
      const double z_offset_;
      const double roll_offset_;
      const double pitch_offset_;
      const double yaw_offset_;
      Eigen::Affine3f trans_;

    };
    
    // should be removed or reimplemented
    inline std::vector<double>
    RGB2HSV (const double r, const double g, const double b)
    {
      double ma = .0, mi = .0, h = .0, s = .0, v = .0;
      if ( r == g && g == b)
      {
        ma = r;
        mi = r;
        h = 0.0;
        v = r;
      }
      else if ( r > g && r > b )
      {
        ma = r;
        mi = std::min(g, b);
        h = 60.0 * (g - b) / (ma - mi) + 0.0;
      }
      else if ( g > b && g >= r )
      {
        ma = g;
        mi = std::min(b, r);
        h = 60.0 * (b - r ) / (ma - mi) + 120.0;
      }
      else if ( b >= r && b >= g )
      {
        ma = b;
        mi = std::min(r, g);
        h = 60.0 * (r - g) / (ma - mi) + 240.0;
      }
    
      if ( ma == 0.0 )
        return std::vector<double> (3, 0.0);
      s = ( ma - mi ) / ma;
      v = ma;
      std::vector<double> ret (3, 0);
      ret[0] = h / 180 * M_PI;    // convert to radian
      ret[1] = s;
      ret[2] = v;
      return ret;
    }

/*
  e = (alpha (h_a - h_b)^2 + beta (s_a - s_b)^2 + gamma (v_a - v_b)^2)^(1/2)
  
  In the original paper below, YCV is used, but i use HSV instead of it.
  @inproceedings{Johnson_1997_3141,
  author = "Andrew Johnson and Sing Bing Kang",
  title = "Registration and Integration of Textured 3-D Data",
  booktitle = "InternationalConference on Recent Advances in
  3-D Digital Imaging and Modeling (3DIM '97)",
  pages = "234 - 241",
  month = "May",
  year = "1997",
  }
*/
    inline double
    HSVColorDistance (const std::vector<double> &src,
                      const std::vector<double> &target,
                      const double h_scale,
                      const double s_scale,
                      const double v_scale)
    {
      const double _h_diff = abs (src[0] - target[0]);
      double h_diff;
      // hue value is in 0 ~ 2pi, but circulated.
      // so i calc here minimum distance between two hue value.
      // and normalize the value to 0 ~ 1
      if ( _h_diff > 180.0 / 180.0 * M_PI )
      {
        h_diff = h_scale * (_h_diff - M_PI) * (_h_diff - M_PI) / M_PI / M_PI;
      }
      else
      {
        h_diff = h_scale * _h_diff * _h_diff / M_PI / M_PI;
      }
    
      const double s_diff = s_scale * (src[1] - target[1]) * (src[1] - target[1]);
      const double v_diff = v_scale * (src[2] - target[2]) * (src[2] - target[2]);
      return sqrt(h_diff + s_diff + v_diff);
    }

    inline double
    RGBColorDistance (const float a_rgb, const float b_rgb,
                      const double h_scale, const double s_scale,
                      const double v_scale)
    {
      const float* rgb_src_ptr = &(a_rgb);
      const float* rgb_target_ptr = &(b_rgb);
      const unsigned int rgb_src = *(unsigned int*)(rgb_src_ptr);
      const unsigned int rgb_target = *(unsigned int*)(rgb_target_ptr);
      const unsigned char r_src = (rgb_src >> 16) & 0xFF;
      const unsigned char g_src = (rgb_src >> 8) & 0xFF;
      const unsigned char b_src = (rgb_src >> 0) & 0xFF;
      const unsigned char r_target = (rgb_target >> 16) & 0xFF;
      const unsigned char g_target = (rgb_target >> 8) & 0xFF;
      const unsigned char b_target = (rgb_target >> 0) & 0xFF;
      // rgb -> hsv
      std::vector<double> hsv_src = RGB2HSV (r_src / 256.0,
                                             g_src / 256.0,
                                             b_src / 256.0);
      std::vector<double> hsv_target = RGB2HSV (r_target / 256.0,
                                                g_target / 256.0,
                                                b_target / 256.0);

      return HSVColorDistance (hsv_src, hsv_target,
                               h_scale, s_scale, v_scale);
    }

/*
  1.0
  ----------------
  1 + beta * C_d
*/
    template <typename PointType>
    inline double ColorDistance (const PointType& src_point,
                                 const PointType& target_point,
                                 const double color_blending,
                                 const double h_scale, const double s_scale,
                                 const double v_scale)
    {
      const float a_rgb = src_point.rgb;
      const float b_rgb = target_point.rgb;
      const float* rgb_src_ptr = &(a_rgb);
      const float* rgb_target_ptr = &(b_rgb);
      const unsigned int rgb_src = *(unsigned int*)(rgb_src_ptr);
      const unsigned int rgb_target = *(unsigned int*)(rgb_target_ptr);
      const unsigned char r_src = (rgb_src >> 16) & 0xFF;
      const unsigned char g_src = (rgb_src >> 8) & 0xFF;
      const unsigned char b_src = (rgb_src >> 0) & 0xFF;
      const unsigned char r_target = (rgb_target >> 16) & 0xFF;
      const unsigned char g_target = (rgb_target >> 8) & 0xFF;
      const unsigned char b_target = (rgb_target >> 0) & 0xFF;
      // rgb -> hsv
      std::vector<double> hsv_src = RGB2HSV (r_src / 256.0,
                                             g_src / 256.0,
                                             b_src / 256.0);
      std::vector<double> hsv_target = RGB2HSV (r_target / 256.0,
                                                g_target / 256.0,
                                                b_target / 256.0);

      double dist =  HSVColorDistance (hsv_src, hsv_target,
                                       h_scale, s_scale, v_scale);
      return 1.0 / (1.0 + color_blending * dist * dist);
    }

/*
  1.0
  ---------------------
  theta  ^2
  1.0 + beta *  -----
  pi

*/
    template <typename PointType>
    inline double
    NormalDistance (const PointType& src_point,
                    const PointType& target_point,
                    const double normal_blending)
    {
      Eigen::Vector4f n (src_point.normal[0],
                         src_point.normal[1],
                         src_point.normal[2],
                         0.0f);
      Eigen::Vector4f n_dash (target_point.normal[0],
                              target_point.normal[1],
                              target_point.normal[2],
                              0.0f);
      //std::cout << "n: " << n.length() << ", n-dash: " << n_dash.length() << std::endl;
    
      if ( n.norm () <= 1e-5 || n_dash.norm () <= 1e-5 )
      {
        PCL_ERROR("might be ZERO!");
        return 0.0;
      }
      else
      {
        n.normalize ();
        n_dash.normalize ();
        
        double theta = pcl::getAngle3D (n, n_dash);
        
        if (!pcl_isnan (theta))
        {
          return 1.0 / (1.0 + normal_blending * theta * theta);
        }
        else
        {
          std::cout << "n:\n" << n << std::endl;
          std::cout << "n_dash:\n" << n_dash << std::endl;
          return 0.0;
        }
      }
    }

/*
  1
  -------------
  1 + alpha d^2
*/
    template <typename PointType>
    inline double
    PointDistance (const PointType& src_point,
                   const PointType& target_point,
                   const double alpha)
    {
      Eigen::Vector4f p (src_point.x, src_point.y, src_point.z, 0.0f);
      Eigen::Vector4f p_dash (target_point.x, target_point.y, target_point.z, 0.0f);
      double d = (p - p_dash).norm ();
      return 1.0 / (1.0 + d * d * alpha);
    }

/* distance function, current implementation
   -+--+-
   L =  |  |  l_n
   l_n = d_n * c_n * n_n
*/
    template <typename PointType, typename KdTreePtrType>
    double PointCloudDistance
    (const typename pcl::PointCloud<PointType> &reference_point_cloud,
     const KdTreePtrType tree, const double color_blending,
     const double h_scale, const double s_scale, const double v_scale,
     const double alpha, const double normal_blending)
    {
      double ret = 1.0;
      int num = 0;
      size_t cloud_size = reference_point_cloud.points.size();
      typename pcl::PointCloud<PointType>::ConstPtr measured_point_cloud_ptr
        = tree->getInputCloud();
    
      // static more fast??
      std::vector<int> k_indices(1);
      std::vector<float> k_distances(1);
    
      // reference -> data
      for ( size_t i = 0; i < cloud_size; i++ )
      {
        // reduction src pointcloud
        PointType input_point = reference_point_cloud.points[i];
        // btVector3 p = btVector3(input_point.x, input_point.y, input_point.z);
        Eigen::Vector4f p(input_point.x, input_point.y, input_point.z, 0.0f);
        // btVector3 n = btVector3(input_point.normal[0],
        //                         input_point.normal[1],
        //                         input_point.normal[2]);
        Eigen::Vector4f n(input_point.normal[0], input_point.normal[1], input_point.normal[2], 0.0f);
        
        p.normalize();
        n.normalize();
        
        // only compute when input_point is visible, not ocluded
        // by refering the angle between a positoin vector and its normal.
        // this method is sometimes good, and sometimes bad.
        // it may cause a local optimization.
        if ( pcl::getAngle3D(p, n) > (M_PI / 2.0 + 0.1) )
        {
          tree->nearestKSearch(reference_point_cloud.points[i],
                               1, k_indices, k_distances);
          PointType target_point =
            measured_point_cloud_ptr->points[k_indices[0]];
          if (!pcl_isnan(target_point.x)
              && !pcl_isnan(target_point.y)
              && !pcl_isnan(target_point.z)
              && !pcl_isnan(target_point.normal[0])
              && !pcl_isnan(target_point.normal[1])
              && !pcl_isnan(target_point.normal[2]))
          {
            double d_n = PointDistance<PointType>(input_point,
                                                  target_point,
                                                  alpha);
            double n_n = NormalDistance<PointType>(input_point,
                                                   target_point,
                                                   normal_blending);
            double c_n = ColorDistance<PointType>(input_point,
                                                  target_point,
                                                  color_blending,
                                                  h_scale, s_scale, v_scale);
            //PCL_INFO("d_n -> %f", d_n);
            //PCL_INFO("n_n -> %f", n_n);
            //PCL_INFO("c_n -> %f", c_n);
            //ret *= d_n * n_n * c_n;
            ret *= d_n * n_n * c_n;
            ++num;
          }
          else
          {
            PCL_INFO("nan points!");
          }
          //ret *= d_n * c_n;
        }
      }
      double likelihood;
      // TODO: parameterize
      if ( num >= reference_point_cloud.points.size() / 3.0 )
        likelihood = ret;
      //likelihood = exp((((ret * ret ) / num) / (- 2.0 * sigma2)));
      else
        likelihood = 0.0;
      return likelihood;
    }
        
    class PointCloudTracking
    {
    public:
      typedef pcl::PointXYZRGBNormal PointType;
      typedef pcl::PointCloud<PointType> Cloud;
      typedef pcl::KdTreeFLANN<PointType> KdTree;
      typedef KdTree::Ptr KdTreePtr;
      typedef boost::shared_ptr<ParticleFilterParameter> ParticleFilterParameterPtr;
      typedef boost::shared_ptr<PF::ParticleFilter> ParticleFilterPtr;
      
    private:
      boost::shared_ptr<PF::ParticleFilter> particle_filter_;
      ParticleFilterParameterPtr pfilter_parameter_;
      //ParticleFilterParameter* pfilter_parameter_;
      //typedef typename pcl::KdTree<PointNormal>::Ptr KdTreePtr;
      //pcl::PointCloud<PointType>::ConstPtr reference_cloud_;
      pcl::PointCloud<PointType>::Ptr reference_cloud_;
      KdTreePtr target_cloud_tree_;
      
      // parameters
      double x_width_, y_width_, z_width_, roll_width_, yaw_width_, pitch_width_;
      double x_offset_, y_offset_, z_offset_, roll_offset_, yaw_offset_, pitch_offset_;
      double resample_likelihood_thr_;
      std::vector<double> step_noise_covariance_;
      std::vector<double> initial_noise_covariance_;
      std::vector<double> initial_noise_mean_;
      int num_samples_;
      int num_iteration_;
      double distance_blending_;
      double normal_blending_;
      double color_blending_;
      double h_scale_, s_scale_, v_scale_;
      
    public:
      PointCloudTracking (PF::UpdateParticleFilterMethod update_method = PF::UPDATE_MEAN,
                          int num_samples = 200, int num_iteration = 1,
                          double distance_blending = 10.0, double normal_blending = 0.0, double color_blending = 0.0,
                          std::vector<double> default_step_covariance = std::vector<double> (6, pow(0.02,2)),
                          std::vector<double> initial_noise_covariance = std::vector<double> (6, pow(0.3,2)),
                          std::vector<double> default_initial_mean = std::vector<double> (6, 0.5))
        : x_width_ (1.0)
        , y_width_ (1.0)
        , z_width_ (1.0)
        , roll_width_ (2.0 * M_PI)
        , yaw_width_ (2.0 * M_PI)
        , pitch_width_ (2.0 * M_PI)
        , x_offset_ (0.0)
        , y_offset_ (1.0)
        , z_offset_ (0.5)
        , roll_offset_ (M_PI)
        , yaw_offset_ (M_PI)
        , pitch_offset_ (M_PI)
        , resample_likelihood_thr_ (0.0)
        , step_noise_covariance_ (default_step_covariance)
        , initial_noise_covariance_ (initial_noise_covariance)
        , initial_noise_mean_ (default_initial_mean)
        , num_samples_ (num_samples)
        , num_iteration_ (num_iteration)
        , distance_blending_ (distance_blending)
        , normal_blending_ (normal_blending)
        , color_blending_ (color_blending)
        , h_scale_ (1.0)
        , s_scale_ (1.0)
        , v_scale_ (0.0)
        {
          
          pfilter_parameter_ = boost::shared_ptr<ParticleFilterParameter>
            (new ParticleFilterParameter (x_width_, y_width_, z_width_,
                                          roll_width_, pitch_width_, yaw_width_,
                                          x_offset_, y_offset_, z_offset_,
                                          roll_offset_, pitch_offset_, yaw_offset_));
          particle_filter_ = boost::shared_ptr<PF::ParticleFilter>
            (new PF::ParticleFilter (6, num_samples_, initial_noise_mean_,
                                     initial_noise_covariance_,
                                     step_noise_covariance_,
                                     boost::bind(&PointCloudTracking::likelihood, this, _1),
                                     update_method));
        }

      virtual ~PointCloudTracking() {}

      inline ParticleFilterParameterPtr
      getParticleFilterParameter ()
        {
          return pfilter_parameter_;
        }

      inline ParticleFilterPtr
      getParticleFilter ()
        {
          return particle_filter_;
        }
      
      void
      setReferencePointCloud (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud)
        {
          reference_cloud_ = cloud;
        }
      
      double
      likelihood (std::vector<double> state)
        {
          // trans = the origin -> a hypothesis
          Eigen::Affine3f trans = pfilter_parameter_->toEigenMatrix (state);
          // move reference_cloud_ by trans
          Cloud transed_reference;
          pcl::transformPointCloudWithNormals<PointType> (*reference_cloud_, transed_reference, trans);
          double d = PointCloudDistance<PointType, KdTreePtr>
            (transed_reference, target_cloud_tree_, color_blending_,
             h_scale_, s_scale_, v_scale_, distance_blending_, normal_blending_);
          return d;
        }

      void
      proc (pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &cloud)
        {
          // first of all, create kdtree
          target_cloud_tree_ = boost::shared_ptr<KdTreeFLANN<pcl::PointXYZRGBNormal> >
            (new KdTreeFLANN<pcl::PointXYZRGBNormal> ());
          target_cloud_tree_->setInputCloud (cloud);
          
          for (int i = 0; i < num_iteration_; i++)
          {
            particle_filter_->resample ();
            particle_filter_->weight (); // likelihood will be called in it
            particle_filter_->update ();
          }
          
          if ( particle_filter_->getResult ().getOriginalLikelihood () < resample_likelihood_thr_ )
          {
            PCL_WARN ("too small likelihood, re-initializing...\n");
            particle_filter_->initParticles ();
            return;
          }
        }
      
    };
    
  } // namespace apps
} // namespace pcl

#endif  // OPENNI_TRACKING_H_

