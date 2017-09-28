#include "ProjectionAlgorithm.h"

ProjectionAlgorithm::ProjectionAlgorithm()
:m_bx(0), m_by(0), m_bz(0)
{
	imgRows = 240;
	imgClos = 320;
}


ProjectionAlgorithm::~ProjectionAlgorithm()

{
    m_feapicked.clear();
}
//0-x
void ProjectionAlgorithm::fun0(const string &LaserPoints6D)
{
	typedef struct tagPOINT_6D
	{
		double x;  //mm world coordinate x
		double y;  //mm world coordinate y
		double z;  //mm world coordinate z
		double r;
		double g;
		double b;
	}POINT_WORLD;
	/////load data txt
	int number_Txt;
	FILE *fp_txt;
	tagPOINT_6D TxtPoint;
	vector<tagPOINT_6D> m_vTxtPoints;
	fp_txt = fopen(LaserPoints6D.c_str(), "r");


	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf %lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z, &TxtPoint.r, &TxtPoint.g, &TxtPoint.b) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
	}
	else
		cout << "load data txt failure£¡" << endl;
	number_Txt = m_vTxtPoints.size();
	pcl::PointCloud<pcl::PointXYZRGB> cloudtmp;


	// Fill in the cloud data
	cloudtmp.width = number_Txt;
	cloudtmp.height = 1;
	cloudtmp.is_dense = false;
	cloudtmp.points.resize(cloudtmp.width * cloudtmp.height);
	for (size_t i = 0; i < cloudtmp.points.size(); ++i)
	{
		cloudtmp.points[i].x = m_vTxtPoints[i].y;
		cloudtmp.points[i].y = m_vTxtPoints[i].x;
		cloudtmp.points[i].z = m_vTxtPoints[i].z;
		uint8_t r(m_vTxtPoints[i].r), g(m_vTxtPoints[i].g), b(m_vTxtPoints[i].b);
		uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
			static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		cloudtmp.points[i].rgb = *reinterpret_cast<float*>(&rgb);
	}
	m_vTxtPoints.clear();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);


	*cloud = cloudtmp;


	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	for (size_t i = 0; i < cloud_filtered->width; i++)
	{
		vector<float> tmp;
		tmp.push_back(cloud_filtered->points[i].x);
		tmp.push_back(cloud_filtered->points[i].y);
		tmp.push_back(cloud_filtered->points[i].z);
		tmp.push_back(cloud_filtered->points[i].r);
		tmp.push_back(cloud_filtered->points[i].g);
		tmp.push_back(cloud_filtered->points[i].b);
		m_feapicked.push_back(tmp);
		tmp.clear();

	}
	cloud_filtered->clear();
	cloud->clear();
}
//1-y
void ProjectionAlgorithm::fun1(const string &LaserPoints6D)
{
	typedef struct tagPOINT_6D
	{
		double x;  //mm world coordinate x
		double y;  //mm world coordinate y
		double z;  //mm world coordinate z
		double r;
		double g;
		double b;
	}POINT_WORLD;
	/////load data txt
	int number_Txt;
	FILE *fp_txt;
	tagPOINT_6D TxtPoint;
	vector<tagPOINT_6D> m_vTxtPoints;
	fp_txt = fopen(LaserPoints6D.c_str(), "r");


	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf %lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z, &TxtPoint.r, &TxtPoint.g, &TxtPoint.b) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
	}
	else
		cout << "load data txt failure£¡" << endl;
	number_Txt = m_vTxtPoints.size();
	pcl::PointCloud<pcl::PointXYZRGB> cloudtmp;


	// Fill in the cloud data
	cloudtmp.width = number_Txt;
	cloudtmp.height = 1;
	cloudtmp.is_dense = false;
	cloudtmp.points.resize(cloudtmp.width * cloudtmp.height);
	for (size_t i = 0; i < cloudtmp.points.size(); ++i)
	{
		cloudtmp.points[i].x = m_vTxtPoints[i].x;
		cloudtmp.points[i].y = m_vTxtPoints[i].y;
		cloudtmp.points[i].z = m_vTxtPoints[i].z;
		uint8_t r(m_vTxtPoints[i].r), g(m_vTxtPoints[i].g), b(m_vTxtPoints[i].b);
		uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
			static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		cloudtmp.points[i].rgb = *reinterpret_cast<float*>(&rgb);
	}
	m_vTxtPoints.clear();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);


	*cloud = cloudtmp;


	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	for (size_t i = 0; i < cloud_filtered->width; i++)
	{
		vector<float> tmp;
		tmp.push_back(cloud_filtered->points[i].x);
		tmp.push_back(cloud_filtered->points[i].y);
		tmp.push_back(cloud_filtered->points[i].z);
		tmp.push_back(cloud_filtered->points[i].r);
		tmp.push_back(cloud_filtered->points[i].g);
		tmp.push_back(cloud_filtered->points[i].b);
		m_feapicked.push_back(tmp);
		tmp.clear();

	}
	cloud_filtered->clear();
	cloud->clear();
}
//2-z
void ProjectionAlgorithm::fun2(const string &LaserPoints6D)
{
	typedef struct tagPOINT_6D
	{
		double x;  //mm world coordinate x
		double y;  //mm world coordinate y
		double z;  //mm world coordinate z
		double r;
		double g;
		double b;
	}POINT_WORLD;
	/////load data txt
	int number_Txt;
	FILE *fp_txt;
	tagPOINT_6D TxtPoint;
	vector<tagPOINT_6D> m_vTxtPoints;
	fp_txt = fopen(LaserPoints6D.c_str(), "r");


	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf %lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z, &TxtPoint.r, &TxtPoint.g, &TxtPoint.b) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
	}
	else
		cout << "load data txt failure£¡" << endl;
	number_Txt = m_vTxtPoints.size();
	pcl::PointCloud<pcl::PointXYZRGB> cloudtmp;


	// Fill in the cloud data
	cloudtmp.width = number_Txt;
	cloudtmp.height = 1;
	cloudtmp.is_dense = false;
	cloudtmp.points.resize(cloudtmp.width * cloudtmp.height);
	for (size_t i = 0; i < cloudtmp.points.size(); ++i)
	{
		cloudtmp.points[i].x = m_vTxtPoints[i].x;
		cloudtmp.points[i].y = m_vTxtPoints[i].z;
		cloudtmp.points[i].z = m_vTxtPoints[i].y;
		uint8_t r(m_vTxtPoints[i].r), g(m_vTxtPoints[i].g), b(m_vTxtPoints[i].b);
		uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
			static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		cloudtmp.points[i].rgb = *reinterpret_cast<float*>(&rgb);
	}
	m_vTxtPoints.clear();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);


	*cloud = cloudtmp;


	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	for (size_t i = 0; i < cloud_filtered->width; i++)
	{
		vector<float> tmp;
		tmp.push_back(cloud_filtered->points[i].x);
		tmp.push_back(cloud_filtered->points[i].y);
		tmp.push_back(cloud_filtered->points[i].z);
		tmp.push_back(cloud_filtered->points[i].r);
		tmp.push_back(cloud_filtered->points[i].g);
		tmp.push_back(cloud_filtered->points[i].b);
		m_feapicked.push_back(tmp);
		tmp.clear();

	}
	cloud_filtered->clear();
	cloud->clear();
}
//x upward -0,x upward -1,x upward -2
void ProjectionAlgorithm::PointsFilter6D(const string &LaserPoints6D,int flag)
{
	if (flag == 0)
		fun0(LaserPoints6D);
	if (flag == 1)
		fun1(LaserPoints6D);
	if (flag == 2)
		fun2(LaserPoints6D);
	if (flag != 0 && flag != 1 && flag != 2)
		ExitProcess(0);
	
}
void ProjectionAlgorithm::ProjectionMirror6D()
{

		 float x0 = 1, y0 = 2, z0 = 3;
		 int eh = 1000;
         CvRNG rng0 = cvRNG(0xffffffff);
		 CvMat *thrMat = cvCreateMat(1, eh, CV_32FC1);
		 CvRNG rng1 = cvRNG(0xffffffff);
		 CvMat *faMat = cvCreateMat(1, eh, CV_32FC1);
		 cvRandArr(&rng0, faMat, CV_RAND_UNI, cvScalar(CV_PI / 3), cvScalar(CV_PI / 3*2));
		 cvRandArr(&rng1, thrMat, CV_RAND_UNI, cvScalar(0), cvScalar(CV_PI));
 
		 vector<float> disVec;
		 disVec.resize(eh);
#pragma omp parallel for
		 for (int cnt = 0; cnt < eh; cnt++)
		 {
			 vector<float> xp;
			 vector<float> yp;
			 vector<float> zp;
			 float fa = cvmGet(faMat, 0, cnt);
			 float thr = cvmGet(thrMat, 0, cnt);
#pragma omp parallel for
			 for (int i = 0; i < m_feapicked.size(); i++)
			 {
				 float t = sin(fa)*cos(thr)*(x0 - m_feapicked[i][0]) + cos(fa)*(y0 - m_feapicked[i][1]) +
					 sin(fa)*sin(thr)*(z0 - m_feapicked[i][2]);
				 float tmpx, tmpy, tmpz;
				 tmpx = m_feapicked[i][0] + sin(fa)*cos(thr)*t;
				 tmpy = m_feapicked[i][1] + cos(fa)*t;
				 tmpz = m_feapicked[i][2] + sin(fa)*sin(thr)*t;
				 xp.push_back(tmpx), yp.push_back(tmpy), zp.push_back(tmpz);
			 }
			 float xpm, ypm, zpm;
			 xpm = std::accumulate(xp.begin(), xp.end(),0.0) / m_feapicked.size();
			 ypm = std::accumulate(yp.begin(), yp.end(), 0.0) / m_feapicked.size();
			 zpm = std::accumulate(zp.begin(), zp.end(), 0.0) / m_feapicked.size();
			 float distmp = 0.0;
#pragma omp parallel for reduction(+:distmp)
			 for (int j = 0; j < m_feapicked.size(); j++)
			 {
				 distmp += sqrt((m_feapicked[j][0] - xpm)*(m_feapicked[j][0] - xpm) +
					 (m_feapicked[j][1] - ypm)*(m_feapicked[j][1] - ypm) +
					 (m_feapicked[j][2] - zpm)*(m_feapicked[j][2] - zpm));
			 }
			 disVec[cnt] = distmp / m_feapicked.size();
			 xp.clear(), yp.clear(), zp.clear();
		 }
		 	std::vector<float>::iterator biggest = std::max_element(std::begin(disVec), std::end(disVec));
		 	int pos = std::distance(std::begin(disVec), biggest);



			vector<vector<float>> nXYZRGB;
			nXYZRGB.resize(m_feapicked.size());
			for (int i = 0; i < m_feapicked.size(); i++)
			{
				vector<float> tmp;
				float fa = cvmGet(faMat, 0, pos);
				float thr = cvmGet(thrMat, 0, pos);
				float t = sin(fa)*cos(thr)*(x0 - m_feapicked[i][0]) + cos(fa)*(y0 - m_feapicked[i][1]) +
					sin(fa)*sin(thr)*(z0 - m_feapicked[i][2]);
				float tmpx, tmpy, tmpz,tmpr,tmpg,tmpb;
				tmpx = m_feapicked[i][0] + sin(fa)*cos(thr)*t;
				tmpy = m_feapicked[i][1] + cos(fa)*t;
				tmpz = m_feapicked[i][2] + sin(fa)*sin(thr)*t;
				tmpr = m_feapicked[i][3];
				tmpg = m_feapicked[i][4];
				tmpb = m_feapicked[i][5];
				tmp.push_back(tmpx), tmp.push_back(tmpy), tmp.push_back(tmpz);
				tmp.push_back(tmpr), tmp.push_back(tmpg), tmp.push_back(tmpb);
				nXYZRGB[i]=tmp;
				tmp.clear();
			}

		 	ofstream outfile("MirrorPoint6DNew.txt");
		 	for (size_t i = 0; i < nXYZRGB.size(); i++)
		 	{
		 		for (size_t j = 0; j < nXYZRGB[i].size(); j++)
		 		{
		 			outfile << nXYZRGB[i][j] << " ";
		 		}
		 		outfile << "\n";
		 	}
		 	outfile.close();
		 
			cvReleaseMat(&thrMat), cvReleaseMat(&faMat);
			disVec.clear();

		 	vector<float> nxVectmp, nyVectmp;
		 	for (int i = 0; i < nXYZRGB.size(); i++)nxVectmp.push_back(nXYZRGB[i][0]), nyVectmp.push_back(nXYZRGB[i][1]);
		 	std::vector<float>::iterator smallestx = std::min_element(std::begin(nxVectmp), std::end(nxVectmp));
		 	std::vector<float>::iterator biggestx = std::max_element(std::begin(nxVectmp), std::end(nxVectmp));

		 	float qdx = *smallestx;
		 	float zdx = *biggestx;
		 	std::vector<float>::iterator smallesty = std::min_element(std::begin(nyVectmp), std::end(nyVectmp));
		 	std::vector<float>::iterator biggesty = std::max_element(std::begin(nyVectmp), std::end(nyVectmp));

		 	float qdy = *smallesty;
		 	float zdy = *biggesty;

		 	CvSize sz;
		 	sz.height = imgRows, sz.width = imgClos;
		 	IplImage* resImage = cvCreateImage(sz, IPL_DEPTH_8U, 3);
		 	cvZero(resImage);
		 	for (size_t i = 0; i < nXYZRGB.size(); i++)
		 	{
		 		if (nXYZRGB[i][1]<qdy || nXYZRGB[i][1]>zdy || nXYZRGB[i][0]<qdx || nXYZRGB[i][0]>zdx)continue;
		 		int xIm = static_cast<int>(standardization(qdx, zdx, 0.0, ((float)imgClos - 1.0), nXYZRGB[i][0]));
		 		int yIm = static_cast<int>(standardization(qdy, zdy, 0.0, ((float)imgRows - 1.0), nXYZRGB[i][1]));
				if (imgRows - yIm >= 0 && imgRows - yIm <= imgRows - 1)
				{
					CV_IMAGE_ELEM(resImage, uchar, imgRows - yIm, xIm * 3 + 0) = static_cast<uchar>(nXYZRGB[i][5]);
					CV_IMAGE_ELEM(resImage, uchar, imgRows - yIm, xIm * 3 + 1) = static_cast<uchar>(nXYZRGB[i][4]);
					CV_IMAGE_ELEM(resImage, uchar, imgRows - yIm, xIm * 3 + 2) = static_cast<uchar>(nXYZRGB[i][3]);

				}
		 		
		 	}

		 	cvSaveImage("resImageANew.png", resImage);
		 	cvFlip(resImage, resImage, 1);
		 	cvSaveImage("resImageBNew.png", resImage);
		 	nxVectmp.clear(); nyVectmp.clear();
		 	nXYZRGB.clear();
		 	cvReleaseImage(&resImage);

}
float ProjectionAlgorithm::standardization(float a, float b, float c, float d, float x)
{

	if (x <= a)return c;
	if (x >= b)return d;
	float k = abs((b - x) / (x - a));
	return (d + c*k) / (k + 1);
}
