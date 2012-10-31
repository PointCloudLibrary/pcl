#include "myFunctions.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
#ifdef __ANDROID__
  #include <android/log.h>
  #define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "PointCloud", __VA_ARGS__))
#else
  #include <string.h>
  #include <fstream>
  using namespace std;
  #define LOGI(...) (printf(__VA_ARGS__))
  #include <pcl/visualization/pcl_visualizer.h>
  #include <pcl/visualization/cloud_viewer.h>
#endif



using namespace Eigen;

MyReg::MyReg()
{
    width_orig=640;
    height_orig=480;
    width_ds=64;
    height_ds=48;

}

void  MyReg::DownSampling(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr src, PointCloud::Ptr tgt)
{
//    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

        for(int  i=0;i<height_ds;i++)
        {
            for(int j=0;j<width_ds;j++)
            {

                int iold=i*height_orig/height_ds;
                int jold=j*width_orig/width_ds;

                (*src).points[i*width_ds+j]=(*cloud_src).points[iold*width_orig+jold];
                (*tgt).points[i*width_ds+j]=(*cloud_tgt).points[iold*width_orig+jold];

            /*    if(isnan((*cloud_src).points[iold*width_orig+jold].x) || isnan((*cloud_src).points[iold*width_orig+jold].y) || isnan((*cloud_src).points[iold*width_orig+jold].z) )
                {
                    (*src).points[i*width_ds+j].x=0;
                    (*src).points[i*width_ds+j].y=0;
                    (*src).points[i*width_ds+j].z=0;
                }

                if(isnan((*cloud_tgt).points[iold*width_orig+jold].x) || isnan((*cloud_tgt).points[iold*width_orig+jold].y) || isnan((*cloud_tgt).points[iold*width_orig+jold].z) )
                {
                    (*tgt).points[i*width_ds+j].x=0;
                    (*tgt).points[i*width_ds+j].y=0;
                    (*tgt).points[i*width_ds+j].z=0;
                }
            */
            }
        }

  /*  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
    std::cout<<"Time of downsampling: "<<diff(time1,time2).tv_sec<<":"<<diff(time1,time2).tv_nsec<<std::endl;
LOGI("Time  of downsampling: :%d:%d",diff(time1,time2).tv_sec,diff(time1,time2).tv_nsec);
*/
}
void  MyReg::Filter(PointCloud::Ptr src, PointCloud::Ptr tgt)
{
    Eigen::Matrix4f myTi = Eigen::Matrix4f::Identity ();
    PointCloud::Ptr tmp (new PointCloud(width_ds,height_ds));
    pcl::transformPointCloud (*src, *tmp, myTi);

    for(int  i=0;i<height_ds;i++)
    {
        for(int j=0;j<width_ds;j++)
        {
           float sq = ( (*tgt).points[i*width_ds+j].x-(*tmp)[i*width_ds+j].x) * ((*tgt).points[i*width_ds+j].x-(*tmp)[i*width_ds+j].x)+
                   ( (*tgt).points[i*width_ds+j].y-(*tmp)[i*width_ds+j].y) * ((*tgt).points[i*width_ds+j].y-(*tmp)[i*width_ds+j].y)+
                   ( (*tgt).points[i*width_ds+j].z-(*tmp)[i*width_ds+j].z) * ((*tgt).points[i*width_ds+j].z-(*tmp)[i*width_ds+j].z);
           std::cout << sq << "\t";

      /*     if(sq<0.01) {
               (*src).points[i*width_ds+j].x=0;
               (*src).points[i*width_ds+j].y=0;
               (*src).points[i*width_ds+j].z=0;
                       }
       */
        }
        std::cout << std::endl;
    }


}
void  MyReg::OldDownSampling (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr src, PointCloud::Ptr tgt )
    {
  /*  pcl::VoxelGrid<PointT> grid;
               clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

    grid.setLeafSize (0.05, 0.05, 0.05);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
               cout<<"Time of setLeafSize: "<<diff(time1,time2).tv_sec<<":"<<diff(time1,time2).tv_nsec<<endl;


                  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);

              clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
              cout<<"Time of downsampling: "<<diff(time1,time2).tv_sec<<":"<<diff(time1,time2).tv_nsec<<endl;
*/
    }
void  MyReg::ZNormalization(pcl::PointCloud<PointNormalT>::Ptr  src, pcl::PointCloud<PointNormalT>::Ptr tgt)
{

    for(int  i=0;i<height_ds;i++)
    {
        for(int j=0;j<width_ds;j++)
        {

            std::cout << (*src).points[i*width_ds+j];

            ////PlaneProjective

            double fx=52.5;
            double fy=52.5;
            double cx=64/2;
            double cy=48/2;


            int jj=fx*(*src).points[i*width_ds+j].x/(*src).points[i*width_ds+j].z+cx;
            int ii=fy*(*src).points[i*width_ds+j].y/(*src).points[i*width_ds+j].z+cy;

            std::cout << i <<"-"<<ii<<"\t";
            std::cout << j <<"-"<<jj<<std::endl;



            (*src).points[i*width_ds+j].x=(*src).points[i*width_ds+j].x/(*src).points[i*width_ds+j].z;
            (*src).points[i*width_ds+j].y=(*src).points[i*width_ds+j].y/(*src).points[i*width_ds+j].z;
            (*src).points[i*width_ds+j].z=1;

            (*tgt).points[i*width_ds+j].x=(*tgt).points[i*width_ds+j].x/(*tgt).points[i*width_ds+j].z;
            (*tgt).points[i*width_ds+j].y=(*tgt).points[i*width_ds+j].y/(*tgt).points[i*width_ds+j].z;
            (*tgt).points[i*width_ds+j].z=2;

            std::cout<<"--" << (*src).points[i*width_ds+j]<<std::endl;

/*
            if(isnan((*cloud_src).points[iold*width_orig+jold].x) || isnan((*cloud_src).points[iold*width_orig+jold].y) || isnan((*cloud_src).points[iold*width_orig+jold].z) ) {
                (*src).points[i*width_ds+j].x=0;
                (*src).points[i*width_ds+j].y=0;
                (*src).points[i*width_ds+j].z=0;
        }

            if(isnan((*cloud_tgt).points[iold*width_orig+jold].x) || isnan((*cloud_tgt).points[iold*width_orig+jold].y) || isnan((*cloud_tgt).points[iold*width_orig+jold].z) ) {
                (*tgt).points[i*width_ds+j].x=0;
                (*tgt).points[i*width_ds+j].y=0;
                (*tgt).points[i*width_ds+j].z=0;
        }
*/


        }

    }
}

void MyReg:: Normals(pcl::PointCloud<PointNormalT>::Ptr points_with_normals_src, pcl::PointCloud<PointNormalT>::Ptr points_with_normals_tgt,PointCloud::Ptr src, PointCloud::Ptr tgt)
{

      pcl::IntegralImageNormalEstimation<pcl::PointXYZ, PointNormalT> ne;
    //  pcl::NormalEstimation<PointT, PointNormalT> ne;
    //  ne.setKSearch(30);

      //pcl::search::OrganizedNeighbor<PointT>::Ptr organized (new pcl::search::OrganizedNeighbor<PointT> ());
      //ne.setSearchMethod (organized);

      ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);

      // ne.setMaxDepthChangeFactor(0.02f);
      // ne.setNormalSmoothingSize(100.0f);


      // ne.setRadiusSearch(0.002);
      // ne.setNormalSmoothingSize(0.001f);
      // ne.setKSearch(0);
      // ne.setMaxDepthChangeFactor(0.02f);
      // ne.setNormalSmoothingSize(0.01f);


        //             clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

      ne.setInputCloud(src);
      ne.compute(*points_with_normals_src);
      pcl::copyPointCloud (*src, *points_with_normals_src);

      ne.setInputCloud(tgt);
      ne.compute(*points_with_normals_tgt);
      pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

           /*       clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
                  cout<<"Time of normal_estimation: "<<diff(time1,time2).tv_sec<<":"<<diff(time1,time2).tv_nsec<<endl;
LOGI("Time of normal_estimation :%d:%d",diff(time1,time2).tv_sec,diff(time1,time2).tv_nsec);
*/

  /*
            for(int  i=0;i<height_ds;i++)
            {
                for(int j=0;j<width_ds;j++)
                {
             //   (*points_with_normals_src)[i*width_ds+j].curvature=0.01f*i;
             //   (*points_with_normals_tgt)[i*width_ds+j].curvature=0.01f*j;
                    std::cout << (*src)[i*width_ds+j];
                    std::cout << (*points_with_normals_src)[i*width_ds+j]<<"\t";
                }
                std::cout << endl;

              }

  */

  /*  pcl::visualization::PCLVisualizer viewer("PCL Viewer1");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);

    viewer.addPointCloudNormals<pcl::PointXYZ,PointNormalT>(src, points_with_normals_src,1);

    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }

    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  p->spin();

    //////////////////////////////
  */
}
void MyReg::OldNormals(pcl::PointCloud<PointNormalT>::Ptr points_with_normals_src, pcl::PointCloud<PointNormalT>::Ptr points_with_normals_tgt,PointCloud::Ptr src, PointCloud::Ptr tgt)
{


//      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);



      pcl::NormalEstimation<PointT, PointNormalT> norm_est;

     // pcl::search::OrganizedNeighbor<PointT>::Ptr organized (new pcl::search::OrganizedNeighbor<PointT> ());
    //  norm_est.setSearchMethod (organized);
    //  norm_est.setRadiusSearch(0.3);

      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
      norm_est.setSearchMethod (tree);
      norm_est.setKSearch (30);


      norm_est.setInputCloud (src);
      norm_est.compute (*points_with_normals_src);
      pcl::copyPointCloud (*src, *points_with_normals_src);

      norm_est.setInputCloud (tgt);
      norm_est.compute (*points_with_normals_tgt);
      pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

/*                          clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
                          cout<<"Time of compute surface normals and curvature: "<<diff(time1,time2).tv_sec<<":"<<diff(time1,time2).tv_nsec<<endl;
LOGI("Time of compute surface normals and curvature: :%d:%d",diff(time1,time2).tv_sec,diff(time1,time2).tv_nsec);
*/

                          for(int  i=0;i<height_ds;i++)
                          {
                              for(int j=0;j<width_ds;j++)
                              {
                                  std::cout << (*src)[i*width_ds+j];
                                  std::cout << (*points_with_normals_src)[i*width_ds+j]<<"\t";
                              }
                              std::cout << std::endl;

                            }


}

void MyReg::MatrixEstimation(pcl::PointCloud<PointNormalT>::Ptr src, pcl::PointCloud<PointNormalT>::Ptr tgt, Eigen::Matrix4f & GlobalTransf)
{
//    LOGI("MatrixEstimation start");

 //pcl::visualization::PCLVisualizer viewer("PCL Viewer1");

    Eigen::Matrix<double, 64*48, 6, Eigen::RowMajor> A;
    Eigen::Matrix<double, 64*48, 1, Eigen::ColMajor> b;


    GlobalTransf << 1,0,0,0,
                    0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;


 //   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

    for(int s=0;s<10;s++)
    {
        int q=0;
float av_sq=0;
int tot=0;
int tot1=0;
int tot2=0;
int tot3=0;
int tot4=0;
int tot5=0;

//	LOGI("A, b initialization,iteration=%d",s);
        for(int  i=0;i<height_ds;i++)
        {
            for(int j=0;j<width_ds;j++)
            {
		//LOGI("** %d %d",i,j);
		 tot1++;                
		if(     
 			 fabs((*src).points[i*width_ds+j].x)<0.000001f ||
                          fabs((*src).points[i*width_ds+j].y)<0.000001f ||
                          fabs((*src).points[i*width_ds+j].z)<0.000001f ||

			isnan((*src).points[i*width_ds+j].x) ||
                        isnan((*src).points[i*width_ds+j].y) ||
                        isnan((*src).points[i*width_ds+j].z)

                   )
                {
                continue;
                }


                double six=(*src).points[i*width_ds+j].x;
                double siy=(*src).points[i*width_ds+j].y;
                double siz=(*src).points[i*width_ds+j].z;

		 tot2++;

//LOGI("** init six=%f siy=%f siz=%f",six,siy,siz);

                float fx=52.5;
                float fy=52.5;
                float cx=64/2;
                float cy=48/2;


                if(fabs((*src).points[i*width_ds+j].z)<0.000001f)
                {continue;
                }

                int jj=fx*(*src).points[i*width_ds+j].x/(*src).points[i*width_ds+j].z+cx;
                int ii=fy*(*src).points[i*width_ds+j].y/(*src).points[i*width_ds+j].z+cy;

if(jj>=width_ds || ii>=height_ds || jj<0 || ii<0) {
//LOGI("********************* init ii=%d jj=%d x=%f y=%f z=%f",ii,jj,(*src).points[i*width_ds+j].x,(*src).points[i*width_ds+j].y,(*src).points[i*width_ds+j].z);
continue;}
tot3++;

                 if(       isnan((*tgt).points[ii*width_ds+jj].x) ||
                           isnan((*tgt).points[ii*width_ds+jj].y) ||
                           isnan((*tgt).points[ii*width_ds+jj].z) ||
  			
                           fabs((*tgt).points[ii*width_ds+jj].x)<0.000001f ||
                           fabs((*tgt).points[ii*width_ds+jj].y)<0.000001f ||
                           fabs((*tgt).points[ii*width_ds+jj].z)<0.000001f ||

                           isnan((*tgt).points[ii*width_ds+jj].normal_x) ||
                           isnan((*tgt).points[ii*width_ds+jj].normal_y) ||
                           isnan((*tgt).points[ii*width_ds+jj].normal_z)
                           )
                 {continue;}

tot4++;
                 double dix=(*tgt).points[ii*width_ds+jj].x;
                 double diy=(*tgt).points[ii*width_ds+jj].y;
                 double diz=(*tgt).points[ii*width_ds+jj].z;

//LOGI("** init dix=%f diy=%f diz=%f",dix,diy,diz);

                 double sq = ( dix-six) * (dix-six)+
                          (diy-siy) * (diy-siy)+
                          (diz-siz) * (diz-siz);

//LOGI("sq=%f %f %f %f %f %f %f q=%d ii=%d jj=%d i=%d j=%d",sq,six,siy,siz,dix,diy,diz, q, ii, jj, i, j);

av_sq+=sq;
tot++;

                 if (sq>0.01) {continue;}  //0.01
tot5++;

                 double nix=(*tgt).points[ii*width_ds+jj].normal_x;
                 double niy=(*tgt).points[ii*width_ds+jj].normal_y;
                 double niz=(*tgt).points[ii*width_ds+jj].normal_z;


                 double ai1=niz*siy-niy*siz;
                 double ai2=nix*siz-niz*six;
                 double ai3=niy*six-nix*siy;


//if(s==3) { LOGI("%f %f %f %f %f %f", ai1,ai2,ai3,nix,niy,niz);}

                 A(q,0)=ai1;
                 A(q,1)=ai2;
                 A(q,2)=ai3;
                 A(q,3)=nix;
                 A(q,4)=niy;
                 A(q,5)=niz;

                 b(q,0)=nix*dix+niy*diy+niz*diz-nix*six-niy*siy-niz*siz;

                 q++;
            }
        }
LOGI("q=%d av_sq=%f tot=%d; tot1=%d tot2=%d tot3=%d tot4=%d tot5=%d  ",q,av_sq/(float)tot, tot,tot1,tot2,tot3,tot4,tot5);
//LOGI("Start block");

        Eigen::MatrixXd mA(q,6);
        Eigen::MatrixXd mb(q,1);
        mA=A.block(0,0,q,6);
        mb=b.block(0,0,q,1);

       // std::cout<<"rows:" <<mA.rows();



//LOGI("Start transpose");
        MatrixXd AA=mA.transpose() * mA;
        VectorXd bb=mA.transpose() * mb;
        VectorXd result;

      /*  std::cout <<"AT*A"<<std::endl;
        std::cout << AA;

        std::cout <<"AT*b"<<std::endl;
        std::cout << bb;
*/
        //AA.svd().solve(bb, &result);
        // result = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
//LOGI("Start jacobi");       
  result = AA.jacobiSvd(ComputeThinU | ComputeThinV).solve(bb);

       //  result = AA.llt().solve(bb);
  //       std::cout<<"\n\n result \n"<< result;


//LOGI("matrix generating"); 
     Eigen::Matrix<float,4,4> M;
     M  <<  1.,-result(2),result(1),result(3),
            result(2),1.,-result(0),result(4),
            -result(1),-result(0),1.,result(5),
            0,0,0,1.;


  //   std::cout<< M<< std::endl;




  //   std::cout<< Rinc<< std::endl;

     Eigen::Matrix4f RT;

     RT << 1,0,0,0,
             0,1,0,0,
             0,0,1,0,
             0,0,0,1;

     float alpha = result (0);
     float beta  = result (1);
     float gamma = result (2);

LOGI("alpha:%f beta:%f gamma:%f",alpha,beta,gamma);

     Eigen::Matrix3f Rinc = (Eigen::Matrix3f)AngleAxisf (gamma, Vector3f::UnitZ ()) * AngleAxisf (beta, Vector3f::UnitY ()) * AngleAxisf (alpha, Vector3f::UnitX ());
     RT.topLeftCorner(3,3)=Rinc;
     RT(0,3)=result(3);
     RT(1,3)=result(4);
     RT(2,3)=result(5);




		string outputstr;
		ostringstream out_message;
		out_message << RT;
		outputstr=out_message.str();
		LOGI("%s", outputstr.c_str());





if(fabs(alpha)>0.3 || fabs(beta)>0.3 || fabs(gamma)>0.3 ) {continue;}

//LOGI("GlobalTransf=RT*GlobalTransf"); 
   



 //    std::cout<<"!!!!!!!!!!!!"<< RT<<std::endl;






  GlobalTransf=RT*GlobalTransf;
    PointCloudWithNormals::Ptr transf (new PointCloudWithNormals);

LOGI("Start Transf");
      pcl::transformPointCloud (*src, *transf,  RT);
LOGI("End Transf");

    //  std::cout<< "afterTransformSource"<<std::endl;
      src=transf;

//LOGI("transf");




  /*   Vector3f tinc = result.tail<3> ();
     //compose
     tcurr = Rinc * tcurr + tinc;
     Rcurr = Rinc * Rcurr;
*/








    }

/*    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
  cout<<"\n Time of AT*A: "<<diff(time1,time2).tv_sec<<":"<<diff(time1,time2).tv_nsec<<endl;
 LOGI("Time of AT*A:%d:%d",diff(time1,time2).tv_sec,diff(time1,time2).tv_nsec);
*/
/*
    std::cout<<"\n GlobalTransform\n "<< GlobalTransf <<std::endl;


    viewer.removePointCloud ("vp1");
    viewer.removePointCloud ("vp2");
    pcl::visualization::PointCloudColorHandlerCustom<PointNormalT> src_h (src, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointNormalT> tgt_h (tgt, 255, 0, 0);
    viewer.addPointCloud (tgt, tgt_h, "vp1");
    viewer.addPointCloud (src,src_h, "vp2");
    viewer.spin();
*/

    for(int  i=0;i<height_ds;i++)
    {
        for(int j=0;j<width_ds;j++)
        {
            (*src).points[i*width_ds+j].curvature=1;
        }
    }








    //////////////////registration////////////////////////

    //points_with_normals_src
    //points_with_normals_tgt

    //Eigen::Matrix4f TM = Eigen::Matrix4f::Identity ();

    /*
     Eigen::Matrix<double, 6, 6, Eigen::RowMajor> A;
     Eigen::Matrix<double, 6, 1> b;

     A.transpose;
     Eigen::Matrix<float, 6, 1> result = A.llt ().solve (b).cast<float>();
            float alpha = result (0);
            float beta  = result (1);
            float gamma = result (2);


    */

/*
        Eigen::Matrix<double,4,4> twist;
        twist << 0., -ksi_ptr[2], ksi_ptr[1], ksi_ptr[3],
                 ksi_ptr[2], 0., -ksi_ptr[0], ksi_ptr[4],
                 -ksi_ptr[1], ksi_ptr[0], 0, ksi_ptr[5],
                 0., 0., 0., 0.;
        g = twist.exp();

        eigen2cv(g, Rt);
    #else
        // TODO: check computeProjectiveMatrix when there is not eigen library,
        // because it gives less accurate pose of the camera
        Rt = Mat::eye(4, 4, CV_64FC1);

        Mat R = Rt(Rect(0,0,3,3));
        Mat rvec = ksi.rowRange(0,3);

        Rodrigues(rvec, R);

        Rt.at<double>(0,3) = ksi.at<double>(3);
        Rt.at<double>(1,3) = ksi.at<double>(4);
        Rt.at<double>(2,3) = ksi.at<double>(5);
    #endif
    }
*/

    //////////////////end registration////////////////////////

}

