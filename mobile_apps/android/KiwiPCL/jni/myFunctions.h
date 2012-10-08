#include <boost/make_shared.hpp>
#include <pcl/point_types.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/features/normal_3d.h>

#include <pcl/features/integral_image_normal.h>



//convenient typedefs

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


    timespec diff(timespec, timespec);
    class MyReg
    {
    public:
        int width_orig;
        int height_orig;
        int width_ds;
        int height_ds;
        timespec time1, time2;

        MyReg();
        ~MyReg() {};

        void OldDownSampling (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr src, PointCloud::Ptr tgt );
        void DownSampling(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr src, PointCloud::Ptr tgt );
        void Filter(PointCloud::Ptr src, PointCloud::Ptr tgt);
        void ZNormalization(pcl::PointCloud<PointNormalT>::Ptr  src, pcl::PointCloud<PointNormalT>::Ptr  tgt);
        void Normals(pcl::PointCloud<PointNormalT>::Ptr points_with_normals_src, pcl::PointCloud<PointNormalT>::Ptr points_with_normals_tgt,PointCloud::Ptr src, PointCloud::Ptr tgt);
        void OldNormals(pcl::PointCloud<PointNormalT>::Ptr points_with_normals_src, pcl::PointCloud<PointNormalT>::Ptr points_with_normals_tgt,PointCloud::Ptr src, PointCloud::Ptr tgt);
        void MatrixEstimation(pcl::PointCloud<PointNormalT>::Ptr points_with_normals_src, pcl::PointCloud<PointNormalT>::Ptr points_with_normals_tgt, Eigen::Matrix4f &GlobalTransf);

    };


