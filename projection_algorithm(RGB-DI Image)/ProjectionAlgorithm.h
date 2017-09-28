#ifndef __ProjectionAlgorithm
#define __ProjectionAlgorithm
#include"header.h"

class ProjectionAlgorithm
{
public:
	ProjectionAlgorithm();
	~ProjectionAlgorithm();
	void fun0(const string &LaserPoints6D);
	void fun1(const string &LaserPoints6D);
	void fun2(const string &LaserPoints6D);
	//x upward -0,x upward -1,x upward -2
	void PointsFilter6D(const string &LaserPoints6D,int flag);
	float standardization(float a, float b, float c, float d, float x);
	void ProjectionMirror6D();
    
private:
	vector<vector<float>> m_feapicked;
	int imgRows;
	int imgClos;
	float m_bx; float m_by; float m_bz;
};

#endif
