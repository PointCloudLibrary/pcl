%nspace pcl::PointXYZ;

%include <stdint.i>

%{
#include <pcl/point_types.h>
%}


namespace pcl
{
	struct PointXYZ
	{
		PointXYZ ();
		PointXYZ (float _x, float _y, float _z);
		float x, y, z;
		float data[4];		
	};
}
