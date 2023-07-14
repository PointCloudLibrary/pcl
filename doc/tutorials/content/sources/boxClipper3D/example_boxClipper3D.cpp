/*!
* \Author: Patrick Charron-Morneau
* \Interdisciplinary Centre for the Development of Ocean Mapping (CIDCO)
*/

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>


class pointExtractor{
	
	public:
		pointExtractor(){
			this->cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
		}
		
		~pointExtractor(){}
		
		
		void extractPoints(pcl::Indices & windowPoints){
			
			this->inliers = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
			
			Eigen::Vector3f rodrigues(0, 0, -0.7853982); // Eigen::Vector3f rodrigues(x,y,z) in radians 
			Eigen::Vector3f translation(-1 ,-1 ,-1);
			Eigen::Vector3f box_size (1/1, 1/1, 1/1);
			
			pcl::BoxClipper3D<pcl::PointXYZRGB> boxClipper(rodrigues, translation, box_size);
			boxClipper.clipPointCloud3D (*this->cloud, windowPoints);
			std::cerr<<"nb inliers: " << windowPoints.size()<<"\n";
			
			pcl::IndicesPtr idxPtr = std::make_shared<pcl::Indices>(windowPoints);
			
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud (this->cloud);
			extract.setIndices (idxPtr);
			extract.setNegative (false);
			extract.filter (*this->inliers);
			
			//showPointCloud(this->inliers, "box clipper");
		}
		
		void setInlier(){
			pcl::PointXYZRGB pt;
			uint8_t r = 0, g = 255, b = 0;
			uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
			pt.x = 1.1;
			pt.y = 1.1;
			pt.z = 1.1;
			pt.rgba = rgb;
			this->cloud->push_back(pt);
			
			pcl::PointXYZRGB pt1;
			pt1.x = 1.4;
			pt1.y = 1.4;
			pt1.z = 1.4;
			pt1.rgba = rgb;
			this->cloud->push_back(pt1);
			
			pcl::PointXYZRGB pt8;
			pt8.x = 0.1;
			pt8.y = 0.1;
			pt8.z = 0.1;
			pt8.rgba = rgb;
			this->cloud->push_back(pt8);
		}
		
		void setOutlier(){
			pcl::PointXYZRGB pt;
			uint8_t r = 255, g = 0, b = 0;
			uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
			pt.x = 2.1;
			pt.y = 2.1;
			pt.z = 2.1;
			pt.rgba = rgb;
			this->cloud->push_back(pt);
			
			pcl::PointXYZRGB pt9;
			pt9.x = 2.5;
			pt9.y = 2.5;
			pt9.z = 2.5;
			pt9.rgba = rgb;
			this->cloud->push_back(pt9);
			
			pcl::PointXYZRGB pt11;
			pt11.x = -2.7;
			pt11.y = -2.7;
			pt11.z = -2.7;
			pt11.rgba = rgb;
			this->cloud->push_back(pt11);
		}
		
		void rotateCloud(){
			Eigen::Affine3f transform = Eigen::Affine3f::Identity();
			
			// 45 deg
			transform.rotate (Eigen::AngleAxisf (0.7853982, Eigen::Vector3f::UnitZ()));
			
			// Executing the transformation
			this->transformed_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::transformPointCloud (*this->cloud, *transformed_cloud, transform);
			
			//showPointCloud(this->cloud, "original cloud");
			this->cloud->swap(*transformed_cloud);
			//showPointCloud(this->cloud, "rotated cloud");
		}
		
		
		void showPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, std::string windowName){
			pcl::visualization::CloudViewer viewer (windowName);
		
			viewer.showCloud (pointCloud);
			
			while (!viewer.wasStopped ()){
				
			}
		}
		
	private:
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud;
		
};


int main(int argc,char** argv){
	
	pointExtractor pe;
	
	pe.setInlier();
	
	pe.setOutlier();
	
	pe.rotateCloud();
	
	pcl::Indices indexes;
	
	pe.extractPoints(indexes);
}

