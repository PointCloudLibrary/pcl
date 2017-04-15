import wrapper.pcl.NormalEstimation_PointXYZ_Normal;
import wrapper.pcl.PointCloud_Normal;
import wrapper.pcl.PointCloud_PointXYZ;
import wrapper.pcl.search.KdTree_PointXYZ;
import wrapper.vector_int;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author teammember
 */
public class How_Features_work_tutorial_test {
    static void program1_test()
    {
        wrapper.pcl.PointCloud_PointXYZ cloud =new PointCloud_PointXYZ();
        //passing points
        cloud.push_back(new wrapper.pcl.PointXYZ((float)0.025, (float)0.060, (float)0.070));
        cloud.push_back(new wrapper.pcl.PointXYZ((float)0.026, (float)0.060, (float)0.070));
        cloud.push_back(new wrapper.pcl.PointXYZ((float)0.027, (float)0.060, (float)0.070));
        cloud.push_back(new wrapper.pcl.PointXYZ((float)0.028, (float)0.060, (float)0.070));
        // Create the normal estimation class, and pass the input dataset to it
        wrapper.pcl.NormalEstimation_PointXYZ_Normal ne=new NormalEstimation_PointXYZ_Normal();
        ne.setInputCloud(cloud);
        wrapper.pcl.search.KdTree_PointXYZ tree=new KdTree_PointXYZ(0, true);
        ne.setSearchMethod(tree);
        wrapper.pcl.PointCloud_Normal cloud_normals=new PointCloud_Normal();
        ne.setRadiusSearch(0.03);
        ne.compute(cloud_normals);
        System.out.println("Normal Cloud Size:"+cloud_normals.size());
        for (int i = 0; i < cloud_normals.size(); i++) {
            wrapper.pcl.Normal N=cloud_normals.at(i);
            System.out.print("Point ");
            System.out.println(i);
            System.out.println(N.getNormal_x());
            System.out.println(N.getNormal_y());
            System.out.println(N.getNormal_z());
        }
    }
    
    static void program2_test()
    {
        wrapper.pcl.PointCloud_PointXYZ cloud =new PointCloud_PointXYZ();
        //passing points
        cloud.push_back(new wrapper.pcl.PointXYZ((float)0.025, (float)0.060, (float)0.070));
        cloud.push_back(new wrapper.pcl.PointXYZ((float)0.026, (float)0.060, (float)0.070));
        cloud.push_back(new wrapper.pcl.PointXYZ((float)0.027, (float)0.060, (float)0.070));
        cloud.push_back(new wrapper.pcl.PointXYZ((float)0.028, (float)0.060, (float)0.070));
        // Create the normal estimation class, and pass the input dataset to it
        wrapper.pcl.NormalEstimation_PointXYZ_Normal ne=new NormalEstimation_PointXYZ_Normal();
        ne.setInputCloud(cloud);
        wrapper.pcl.search.KdTree_PointXYZ tree=new KdTree_PointXYZ(0, true);
        ne.setSearchMethod(tree);
        wrapper.pcl.PointCloud_Normal cloud_normals=new PointCloud_Normal();
        ne.setRadiusSearch(0.03);
        wrapper.vector_int v=new vector_int();
        v.add(1);
        v.add(2);
        ne.setIndices(v);
        ne.compute(cloud_normals);
        System.out.println("Normal Cloud Size:"+cloud_normals.size());
        for (int i = 0; i < cloud_normals.size(); i++) {
            wrapper.pcl.Normal N=cloud_normals.at(i);
            System.out.print("Point ");
            System.out.println(i);
            System.out.println(N.getNormal_x());
            System.out.println(N.getNormal_y());
            System.out.println(N.getNormal_z());
        }
    }
    
    static void program3_test()
    {
        wrapper.pcl.PointCloud_PointXYZ cloud =new PointCloud_PointXYZ();
        //passing points
        cloud.push_back(new wrapper.pcl.PointXYZ((float)0.025, (float)0.060, (float)0.070));
        cloud.push_back(new wrapper.pcl.PointXYZ((float)0.026, (float)0.060, (float)0.070));
        cloud.push_back(new wrapper.pcl.PointXYZ((float)0.027, (float)0.060, (float)0.070));
        cloud.push_back(new wrapper.pcl.PointXYZ((float)0.028, (float)0.060, (float)0.070));
        // Create the normal estimation class, and pass the input dataset to it
        wrapper.pcl.NormalEstimation_PointXYZ_Normal ne=new NormalEstimation_PointXYZ_Normal();
        ne.setInputCloud(cloud);
        wrapper.pcl.search.KdTree_PointXYZ tree=new KdTree_PointXYZ(0, true);
        ne.setSearchMethod(tree);
        wrapper.pcl.PointCloud_Normal cloud_normals=new PointCloud_Normal();
        ne.setRadiusSearch(0.03);
        wrapper.pcl.PointCloud_PointXYZ cloud_downsampled=new PointCloud_PointXYZ();
        cloud_downsampled.push_back(new wrapper.pcl.PointXYZ((float)0.026, (float)0.060, (float)0.070));
        cloud_downsampled.push_back(new wrapper.pcl.PointXYZ((float)0.027, (float)0.060, (float)0.070));
        ne.setSearchSurface(cloud_downsampled);
        ne.compute(cloud_normals);
        System.out.println("Normal Cloud Size:"+cloud_normals.size());
        for (int i = 0; i < cloud_normals.size(); i++) {
            wrapper.pcl.Normal N=cloud_normals.at(i);
            System.out.print("Point ");
            System.out.println(i);
            System.out.println(N.getNormal_x());
            System.out.println(N.getNormal_y());
            System.out.println(N.getNormal_z());
        }
    }
}
