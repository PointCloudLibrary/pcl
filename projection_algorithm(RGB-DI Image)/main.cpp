
#include"ProjectionAlgorithm.h"
int main()
{   


	boost::progress_timer ptx;
	string data6dtxt("TestData6D_RGB.txt");
	ProjectionAlgorithm *pt = new ProjectionAlgorithm;
	//x upward -0,y upward -1,z upward -2
	pt->PointsFilter6D(data6dtxt,1);
	pt->ProjectionMirror6D();
	delete pt;

	return 0;
}


