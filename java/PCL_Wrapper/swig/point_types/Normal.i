%nspace pcl::Normal;

%{
#include <pcl/point_types.h>
%}

namespace pcl
{
	struct Normal
	{
		Normal ();
		Normal (float n_x, float n_y, float n_z);
		float normal_x, normal_y, normal_z, curvature, data_n[3];	
	};
}
