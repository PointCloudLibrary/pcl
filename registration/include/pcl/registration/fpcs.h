#ifndef PCL_FPCS_H_
#define PCL_FPCS_H_

#include <limits>
#include <vector>
#include <Eigen/Core>

//PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>

#include <sensor_msgs/PointCloud2.h>

#define DELTA 0.001
#define MIN_DELTA 1 - DELTA/1.5
#define MAX_DELTA 1 + DELTA/1.5

//template <typename PointSource, typename PointTarget, typename Scalar>
struct set4
{
	int p1_index_;
	int p2_index_;
	int p3_index_;
	int p4_index_;

	set4(int p1, int p2, int p3, int p4)
	{
		p1_index_ = p1;
		p2_index_ = p2;
		p3_index_ = p3;
		p4_index_ = p4;
	}

	const int& operator[] (int index)
	{
		if (index == 0) return p1_index_;
		if (index == 1) return p2_index_;
		if (index == 2) return p3_index_;
		if (index == 3) return p4_index_;
	}
};

namespace pcl
{
	template <typename PointSource, typename PointTarget, typename Scalar = float>
	class FPCS : public Registration<PointSource, PointTarget, Scalar>
	{
	public:
		typedef typename Registration<PointSource, PointTarget, Scalar>::PointCloudSource PointCloudSource;
		typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
		typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

		typedef typename Registration<PointSource, PointTarget, Scalar>::PointCloudTarget PointCloudTarget;
		typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
		typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

		//typedef boost::shared_ptr<FPCS<PointSource, PointTarget, Scalar> > Ptr;
		//typedef boost::shared_ptr<const FPCS<PointSource, PointTarget, Scalar> > ConstPtr;

		using Registration<PointSource, PointTarget, Scalar>::reg_name_;
		using Registration<PointSource, PointTarget, Scalar>::getClassName;
		using Registration<PointSource, PointTarget, Scalar>::setInputSource;
		using Registration<PointSource, PointTarget, Scalar>::input_;
		using Registration<PointSource, PointTarget, Scalar>::setInputTarget;
		using Registration<PointSource, PointTarget, Scalar>::target_;
		using Registration<PointSource, PointTarget, Scalar>::ransac_iterations_;
		using Registration<PointSource, PointTarget, Scalar>::final_transformation_;
		using Registration<PointSource, PointTarget, Scalar>::transformation_;
		
		typedef typename Registration<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;

		FPCS () : l_ (10)
				, delta_ (0.1)
				, overlap_ (0.8)
				, f_ (1)
		{
			reg_name_ = "FPCS";
			source_markers_ = PointCloudSourcePtr (new PointCloudSource);
			target_markers_ = PointCloudTargetPtr (new PointCloudTarget);
			basis_ = PointCloudSourcePtr (new PointCloudSource);
			u_ = pcl::PointCloud<set4>::Ptr (new pcl::PointCloud<set4>);
		}

		~FPCS ()
		{
			source_markers_->~PointCloud();
			target_markers_->~PointCloud();
			copy_source_->~PointCloud();
			copy_target_->~PointCloud();
			basis_->~PointCloud();
			u_->~PointCloud();
		}

		void
		setNumOfIteration (const int l)
		{
			Registration<PointSource, PointTarget, Scalar>::setRANSACIterations (l);
		}

		void 
		setDelta (const double delta)
		{
			delta_ = delta;
		}

		double
		getDelta ()
		{
			return (delta_);
		}

		void
		setOverlap (const double overlap)
		{
			overlap_ = overlap;
		}

		double
		getOverlap ()
		{
			return (overlap_);
		}

		virtual void 
		setInputSource (const PointCloudSourceConstPtr &cloud)
		{
			Registration<PointSource, PointTarget, Scalar>::setInputSource (cloud);
		}

		virtual void 
		setInputTarget (const PointCloudTargetConstPtr &cloud)
		{
			Registration<PointSource, PointTarget, Scalar>::setInputTarget (cloud);
		}

		void 
		setMarkers (pcl::PolygonMesh::Ptr source_markers, pcl::PolygonMesh::Ptr target_markers)
		{
			pcl::fromPCLPointCloud2(source_markers->cloud, *source_markers_);
			pcl::fromPCLPointCloud2(target_markers->cloud, *target_markers_);
		}

		void 
		setMarkers (PointCloudSourcePtr source_markers, PointCloudTargetPtr target_markers)
		{
			*source_markers_ = *source_markers;
			*target_markers_ = *target_markers;
		}

		virtual void
		computeTransformation (PointCloudSource &cloud, const Matrix4 &guess);

		Matrix4 
		getTransform ()
		{
			return (getMat4());
		}
		
	protected:
		int l_;
		double delta_;
		double overlap_;
		double f_;//fraction = 1, 0.5, 0.25...
		PointCloudSourcePtr copy_source_;
		PointCloudTargetPtr copy_target_;
		PointCloudSourcePtr basis_;
		pcl::PointCloud<set4>::Ptr u_;
		PointCloudSourcePtr source_markers_;
		PointCloudTargetPtr target_markers_;

		Matrix4 matrix_;
		Eigen::Matrix3f rotation_;
		Eigen::Vector3f offset_;

		void
		startWithMarkers ();

		void
		start ();

		int
		findBasis (const double dist);

		int
		findCongruents ();

		int
		correspondencesRegistration (const double dist);

		Eigen::Quaternion<float>
		estimateTransform (const int corr_index);

		int
		getIndex (const int pIndex, const std::vector<std::pair<int, int> > set);

		Matrix4
		getMat4 ()
		{
			matrix_ = matrix_.Identity();
			for (int ind1 = 0; ind1 < 3; ++ind1)
			{
				for (int ind2 = 0; ind2 < 3; ++ind2)
				{
					matrix_ (ind1, ind2) = rotation_ (ind1, ind2);
				}
			}
			matrix_ (0,3) = offset_.x(); matrix_ (1,3) = offset_.y(); matrix_ (2,3) = offset_.z(); matrix_ (3,3) = 1;
			matrix_ (3,0) = 0; matrix_ (3,1) = 0; matrix_ (3,2) = 0;
			return matrix_;
		}

		Eigen::Quaternion<float>
		estimateRotationMatrix (const PointCloudSourceConstPtr left, const PointCloudTargetConstPtr right);

		Eigen::Vector3f
		centeringSource (PointCloudSourcePtr cl);

		Eigen::Vector3f
		centeringTarget (PointCloudTargetPtr cl);
	};
}

#include <pcl/registration/impl/fpcs.hpp>

#endif //PCL_FPCS_H_
