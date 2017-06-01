#ifndef PCL_FPCS_IMPL_HPP_
#define PCL_FPCS_IMPL_HPP_

#include <pcl/registration/fpcs.h>

template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::FPCS<PointSource, PointTarget, Scalar>::computeTransformation (PointCloudSource &output, const Matrix4 &guess)
{
	PointCloudSourcePtr input_transformed (new PointCloudSource);

	startWithMarkers ();

	transformation_ = getMat4 ();

	final_transformation_ = transformation_;

	output = *input_;

	pcl::transformPointCloud (*input_, output, final_transformation_);
}

template <typename PointSource, typename PointTarget, typename Scalar> void 
pcl::FPCS<PointSource, PointTarget, Scalar>::startWithMarkers ()
{
	if (source_markers_->size() < 1 || target_markers_->size() < 1)
	{
		std::cout << "Source or target markesrs is empty. Process interrupted." << std::endl;
		return;
	}
	double max_dist = 0;
	for (int i = 0; i < input_->size()-6; i += 5)
	{
		for (int j = i+1; j < input_->size(); ++j)
		{
			double d = pcl::euclideanDistance(input_->points[i], input_->points[j]);
			if (d > max_dist)
			{
				max_dist = d;
			}
		}
	}
	int h = 0;
	double err = 0;

	std::vector<int> inds_source, inds_target;
	PointCloudSourcePtr source_corr (new PointCloudSource);
	PointCloudTargetPtr target_corr (new PointCloudTarget);
	pcl::KdTreeFLANN<PointSource> tree_source;
	tree_source.setInputCloud (input_);
	for (int i = 0; i < source_markers_->size(); ++i)
	{
		std::vector<int> inds;
		std::vector<float> dists;
		tree_source.radiusSearch (source_markers_->points[i], 0.1, inds, dists);
		for (int j = 0; j < inds.size(); ++j)
		{
			inds_source.push_back (inds[j]);
			source_corr->push_back (input_->points[inds[j]]);
		}
	}
	pcl::KdTreeFLANN<PointTarget> tree_target;
	tree_target.setInputCloud (target_);
	for (int i = 0; i < target_markers_->size(); ++i)
	{
		std::vector<int> inds;
		std::vector<float> dists;
		tree_target.radiusSearch (target_markers_->points[i], 0.1, inds, dists);
		for (int j = 0; j < inds.size(); ++j)
		{
			inds_target.push_back (inds[j]);
			target_corr->push_back (target_->points[inds[j]]);
		}
	}
	
	for (int i = 0; i < l_; ++i)
	{
		u_ = pcl::PointCloud<set4>::Ptr (new pcl::PointCloud<set4>);
		f_ = 1;
		bool next = false;

		while (!correspondencesRegistration (max_dist * overlap_ * f_))
		{
			//std::cout << "Registration stopped. Iteration #" << i << std::endl;
			if (f_ < 0.25)
			{
				f_ = 1;
				next = true;
				break;
			}
			f_ /= 2;
		}
		if (next) continue;

		Eigen::Vector3f oP (0,0,0), oQ (0,0,0);

		std::cout << "U.size: " << u_->size() << std::endl;
		for (int curr = 0; curr < u_->size(); ++curr)
		{
			copy_source_ = PointCloudSourcePtr (new PointCloudSource);
			*copy_source_ = *input_;
			copy_target_ = PointCloudTargetPtr (new PointCloudTarget);
			*copy_target_ = *target_;
			
			oP = centeringSource (copy_source_);
			oQ = centeringTarget (copy_target_);
			
			int hi = 0;
			double errori = 0;
			PointCloudSourcePtr sij (new PointCloudSource);
			Eigen::Quaternion<float> q = estimateTransform (curr);
			Eigen::Vector3f o (0,0,0);
			
			pcl::transformPointCloud (*copy_source_, *copy_source_, o, q);

			for (int j = 0; j < inds_source.size(); ++j)
			{
				for (int j1 = 0; j1 < inds_target.size(); ++j1)
				{
					double dd = pcl::euclideanDistance (copy_source_->points[inds_source[j]], copy_target_->points[inds_target[j1]]);
					if (dd <= delta_)
					{
						errori += dd;
						hi++;
						sij->push_back (copy_source_->points[inds_source[j]]);
						break;
					}
				}
			}
			if (hi > h)
			{
				h = hi;
				err = errori / hi;
				rotation_ = q.matrix();
				offset_ = -oP + oQ;
				
				PointCloudTargetPtr Uij (new PointCloudTarget);
				Uij->push_back (copy_target_->points[u_->points[curr][0]]);
				Uij->push_back (copy_target_->points[u_->points[curr][1]]);
				Uij->push_back (copy_target_->points[u_->points[curr][2]]);
				Uij->push_back (copy_target_->points[u_->points[curr][3]]);
				
				std::cout << "Best: " << h << std::endl << "err:" << err << std::endl;
				
				Uij->~PointCloud();
			}
		} // end of loop for u_
	} // end RANSAC loop
}

template <typename PointSource, typename PointTarget, typename Scalar> void 
pcl::FPCS<PointSource, PointTarget, Scalar>::start ()
{
	if (source_markers_->size() < 1 || target_markers_->size() < 1)
	{
		std::cout << "Source or target markesrs is empty. Process interrupted." << std::endl;
		return;
	}
	double max_dist = 0;
	for (int i = 0; i < input_->size()-6; i += 5)
	{
		for (int j = i+1; j < input_->size(); ++j)
		{
			double d = pcl::euclideanDistance(input_->points[i], input_->points[j]);
			if (d > max_dist)
			{
				max_dist = d;
			}
		}
	}
	int h = 0;
	double err = 0;

	for (int i = 0; i < l_; ++i)
	{
		u_ = pcl::PointCloud<set4>::Ptr (new pcl::PointCloud<set4>);
		f_ = 1;
		bool next = false;

		while (!correspondencesRegistration (max_dist * overlap_ * f_))
		{
			//std::cout << "Registration stopped. Iteration #" << i << std::endl;
			if (f_ < 0.25)
			{
				f_ = 1;
				next = true;
				break;
			}
			f_ /= 2;
		}
		if (next) continue;

		Eigen::Vector3f oP (0,0,0), oQ (0,0,0);

		//std::cout << "U.size: " << U->size() << std::endl;
		for (int curr = 0; curr < u_->size(); ++curr)
		{
			copy_source_ = PointCloudSourcePtr (new PointCloudSource);
			*copy_source_ = *input_;
			copy_target_ = PointCloudTargetPtr (new PointCloudTarget);
			*copy_target_ = *target_;
			
			oP = centeringSource (copy_source_);
			oQ = centeringTarget (copy_target_);
			
			int hi = 0;
			double errori = 0;
			PointCloudSourcePtr sij (new PointCloudSource);
			Eigen::Quaternion<float> q = estimateTransform (curr);
			Eigen::Vector3f o (0,0,0);
			
			pcl::transformPointCloud (*copy_source_, *copy_source_, o, q);

			for (int j = 0; j < copy_source_.size(); ++j)
			{
				for (int j1 = 0; j1 < copy_target_.size(); ++j1)
				{
					double dd = pcl::euclideanDistance (copy_source_->points[j], copy_target_->points[j1]);
					if (dd <= delta_)
					{
						errori += dd;
						hi++;
						sij->push_back (copy_source_->points[j]);
						break;
					}
				}
			}
			if (hi > h)
			{
				h = hi;
				err = errori / hi;
				rotation_ = q.matrix();
				offset_ = -oP + oQ;
				
				PointCloudTargetPtr Uij (new PointCloudTarget);
				Uij->push_back (copy_target_->points[u_->points[curr][0]]);
				Uij->push_back (copy_target_->points[u_->points[curr][1]]);
				Uij->push_back (copy_target_->points[u_->points[curr][2]]);
				Uij->push_back (copy_target_->points[u_->points[curr][3]]);
				//save Uij
				//save basis_
				//std::cout << "Best: " << h << std::endl;
				
				Uij->~PointCloud();
			}
		} // end of loop for u_
	} // end RANSAC loop
}

template <typename PointSource, typename PointTarget, typename Scalar> int 
pcl::FPCS<PointSource, PointTarget, Scalar>::correspondencesRegistration (const double dist)
{
	if (!findBasis(dist))
	{
		std::cout << "basis not found. ";
		return 0;
	}

	try
	{
	if (!findCongruents())
	{
		std::cout << "Congruent sets not found. ";
		return 0;
	}
	}
	catch (std::exception e)
	{
		cout << "Error! " << e.what() << endl;
		return 0;
	}

	return 1;
}

template <typename PointSource, typename PointTarget, typename Scalar> int 
pcl::FPCS<PointSource, PointTarget, Scalar>::findCongruents ()
{
	//std::cout << "Congruents..." << std::endl;
	
	float e_x = ( (basis_->points[2].y-basis_->points[0].y)
				* (basis_->points[1].x-basis_->points[0].x)
				* (basis_->points[3].x-basis_->points[2].x) + basis_->points[0].x
				* (basis_->points[3].x-basis_->points[2].x)
				* (basis_->points[1].y-basis_->points[0].y) - basis_->points[2].x
				* (basis_->points[1].x-basis_->points[0].x)
				* (basis_->points[3].y-basis_->points[2].y)) / 
				( (basis_->points[3].x-basis_->points[2].x) * (basis_->points[1].y-basis_->points[0].y)
				- (basis_->points[1].x-basis_->points[0].x) * (basis_->points[3].y-basis_->points[2].y)),
		e_y = (e_x - basis_->points[0].x) * (basis_->points[1].y-basis_->points[0].y) / (basis_->points[1].x-basis_->points[0].x) + basis_->points[0].y,
		e_z = (e_x - basis_->points[0].x) * (basis_->points[1].z-basis_->points[0].z) / (basis_->points[1].x-basis_->points[0].x) + basis_->points[0].z;
	
	PointTarget e (e_x, e_y, e_z);

	float d1 = pcl::euclideanDistance (basis_->points[0], basis_->points[1]),
		d2 = pcl::euclideanDistance (basis_->points[2], basis_->points[3]);
	if (d1 == 0 || d2 == 0) return 0;
	float r1 = pcl::euclideanDistance (basis_->points[0], e) / d1,
		  r2 = pcl::euclideanDistance (basis_->points[2], e) / d2;
	if (r1 > 1 || r2 > 1) return 0;

	std::vector<std::pair<int,int> > pairs1, pairs2;

	for (int i1 = 0; i1 < target_->size()-1; ++i1)
	{
		for (int i2 = i1+1; i2 < target_->size(); ++i2)
		{
			std::pair<int,int> p;
			float dist = pcl::euclideanDistance (target_->points[i1], target_->points[i2]);
			p.first = i1;
			p.second = i2;
			if(dist >= d1 * MIN_DELTA && dist <= d1 * MAX_DELTA)
			{
				pairs1.push_back (p);
			}
			if(dist >= d2 * MIN_DELTA && dist <= d2 * MAX_DELTA)
			{
				pairs2.push_back (p);
			}
			if (pairs1.size() > target_->size() || pairs2.size() > target_->size())
			{
				i1 = target_->size() - 2;
				break;
			}
		}
	}

	if (pairs1.size() < 1 || pairs2.size() < 1) return 0;

	PointCloudTargetPtr e1_set (new PointCloudTarget);
	std::vector<std::pair<int, int> > e1_inds, e2_inds;
	
	for (int i = 0; i < pairs1.size(); ++i)
	{
		PointTarget ee1(0,0,0), ee2(0,0,0), ee3(0,0,0), ee4(0,0,0);
		ee1.x = target_->points[pairs1[i].first].x + r1*(target_->points[pairs1[i].second].x - target_->points[pairs1[i].first].x);
		ee1.y = target_->points[pairs1[i].first].y + r1*(target_->points[pairs1[i].second].y - target_->points[pairs1[i].first].y);
		ee1.z = target_->points[pairs1[i].first].z + r1*(target_->points[pairs1[i].second].z - target_->points[pairs1[i].first].z);
		e1_set->push_back(ee1);
		int p_index = e1_set->size()-1;
		std::pair<int, int> p1(p_index, i);
		e1_inds.push_back(p1);

		ee2.x = target_->points[pairs1[i].second].x + r1*(target_->points[pairs1[i].first].x - target_->points[pairs1[i].second].x);
		ee2.y = target_->points[pairs1[i].second].y + r1*(target_->points[pairs1[i].first].y - target_->points[pairs1[i].second].y);
		ee2.z = target_->points[pairs1[i].second].z + r1*(target_->points[pairs1[i].first].z - target_->points[pairs1[i].second].z);
		e1_set->push_back(ee2);
		p_index = e1_set->size()-1;
		std::pair<int, int> p2(p_index, i);
		e1_inds.push_back(p2);

		ee3.x = target_->points[pairs1[i].first].x + r2*(target_->points[pairs1[i].second].x - target_->points[pairs1[i].first].x);
		ee3.y = target_->points[pairs1[i].first].y + r2*(target_->points[pairs1[i].second].y - target_->points[pairs1[i].first].y);
		ee3.z = target_->points[pairs1[i].first].z + r2*(target_->points[pairs1[i].second].z - target_->points[pairs1[i].first].z);
		e1_set->push_back(ee3);
		p_index = e1_set->size()-1;
		std::pair<int, int> p3(p_index, i);
		e1_inds.push_back(p3);

		ee4.x = target_->points[pairs1[i].second].x + r2*(target_->points[pairs1[i].first].x - target_->points[pairs1[i].second].x);
		ee4.y = target_->points[pairs1[i].second].y + r2*(target_->points[pairs1[i].first].y - target_->points[pairs1[i].second].y);
		ee4.z = target_->points[pairs1[i].second].z + r2*(target_->points[pairs1[i].first].z - target_->points[pairs1[i].second].z);
		e1_set->push_back(ee4);
		p_index = e1_set->size()-1;
		std::pair<int, int> p4(p_index, i);
		e1_inds.push_back(p4);
	}

	pcl::KdTreeFLANN<PointTarget> kd_tree;
	kd_tree.setInputCloud (e1_set);

	PointCloudTargetPtr e2_set(new PointCloudTarget);

	for (int i = 0; i < pairs2.size(); ++i)
	{
		PointTarget ee1(0,0,0), ee2(0,0,0), ee3(0,0,0), ee4(0,0,0);
		ee1.x = target_->points[pairs2[i].first].x + r1*(target_->points[pairs2[i].second].x - target_->points[pairs2[i].first].x);
		ee1.y = target_->points[pairs2[i].first].y + r1*(target_->points[pairs2[i].second].y - target_->points[pairs2[i].first].y);
		ee1.z = target_->points[pairs2[i].first].z + r1*(target_->points[pairs2[i].second].z - target_->points[pairs2[i].first].z);
		e2_set->push_back(ee1);
		int p_index = e2_set->size() - 1;
		std::pair<int, int> p1(p_index, i);
		e2_inds.push_back(p1);

		ee2.x = target_->points[pairs2[i].second].x + r1*(target_->points[pairs2[i].first].x - target_->points[pairs2[i].second].x);
		ee2.y = target_->points[pairs2[i].second].y + r1*(target_->points[pairs2[i].first].y - target_->points[pairs2[i].second].y);
		ee2.z = target_->points[pairs2[i].second].z + r1*(target_->points[pairs2[i].first].z - target_->points[pairs2[i].second].z);
		e2_set->push_back(ee2);
		p_index = e2_set->size() - 1;
		std::pair<int, int> p2(p_index, i);
		e2_inds.push_back(p2);

		ee3.x = target_->points[pairs2[i].first].x + r2*(target_->points[pairs2[i].second].x - target_->points[pairs2[i].first].x);
		ee3.y = target_->points[pairs2[i].first].y + r2*(target_->points[pairs2[i].second].y - target_->points[pairs2[i].first].y);
		ee3.z = target_->points[pairs2[i].first].z + r2*(target_->points[pairs2[i].second].z - target_->points[pairs2[i].first].z);
		e2_set->push_back(ee3);
		p_index = e2_set->size() - 1;
		std::pair<int, int> p3(p_index, i);
		e2_inds.push_back(p3);

		ee4.x = target_->points[pairs2[i].second].x + r2*(target_->points[pairs2[i].first].x - target_->points[pairs2[i].second].x);
		ee4.y = target_->points[pairs2[i].second].y + r2*(target_->points[pairs2[i].first].y - target_->points[pairs2[i].second].y);
		ee4.z = target_->points[pairs2[i].second].z + r2*(target_->points[pairs2[i].first].z - target_->points[pairs2[i].second].z);
		e2_set->push_back(ee4);
		p_index = e2_set->size() - 1;
		std::pair<int, int> p4(p_index, i);
		e2_inds.push_back(p4);
	}

	for (int i = 0; i < e2_set->size(); ++i)
	{
		std::vector<int> inds;
		std::vector<float> dists;
		PointTarget e2 = e2_set->points[i];
		kd_tree.radiusSearch (*e2_set, i, DELTA/2, inds, dists);
		for (int j = 0; j < inds.size(); ++j)
		{
			set4 Ui (-27,-27,-27,-27);
			int indexx = getIndex (inds[j], e1_inds);
			if (indexx != -1)
			{
				Ui.p1_index_ = pairs1[indexx].first;
				Ui.p2_index_ = pairs1[indexx].second;
				indexx = getIndex (i, e2_inds);
				if (indexx != -1)
				{
					Ui.p3_index_ = pairs2[indexx].first;
					Ui.p4_index_ = pairs2[indexx].second;
					u_->push_back(Ui);
				}
			}
		}
	}

	e1_set->~PointCloud();
	e2_set->~PointCloud();
	e1_inds.~vector();
	e2_inds.~vector();

	if (u_->size() < 1) return 0;

	return 1;
}

template <typename PointSource, typename PointTarget, typename Scalar> int 
pcl::FPCS<PointSource, PointTarget, Scalar>::getIndex (const int p_index, const std::vector<std::pair<int, int> > set)
{
	for (int i = 0; i < set.size(); ++i)
	{
		if (set[i].first == p_index)
		{
			return set[i].second;
		}
	}
	return -1;
}

template <typename PointSource, typename PointTarget, typename Scalar> Eigen::Quaternion<float>
pcl::FPCS<PointSource, PointTarget, Scalar>::estimateTransform (const int corr_index)
{
	PointCloudTargetPtr Uij(new PointCloudTarget);
	Uij->push_back(copy_target_->points[u_->points[corr_index][0]]);
	Uij->push_back(copy_target_->points[u_->points[corr_index][1]]);
	Uij->push_back(copy_target_->points[u_->points[corr_index][2]]);
	Uij->push_back(copy_target_->points[u_->points[corr_index][3]]);
	Eigen::Quaternion<float> q = estimateRotationMatrix (basis_, Uij);
	return q;
}

template <typename PointSource, typename PointTarget, typename Scalar> int 
pcl::FPCS<PointSource, PointTarget, Scalar>::findBasis (const double dist)
{
	//std::cout << "Basis..." << std::endl;
	basis_ = PointCloudSourcePtr (new PointCloudSource);
	bool next = false;

	int index = (rand() / (double)RAND_MAX) * (input_->size()-1);
	basis_->push_back (input_->points[index]);
	pcl::KdTreeFLANN<PointSource> tree;
	tree.setInputCloud (input_);
	
	std::vector<int> is;
	std::vector<float> ds;

	tree.radiusSearch (basis_->points[0], dist * MAX_DELTA, is, ds);
	for (index = 0; index < is.size(); ++index)
	{
		double dd = pcl::euclideanDistance (basis_->points[0], input_->points[is[index]]);
		if (dd > dist * MIN_DELTA / 2)
		{
			basis_->push_back (input_->points[is[index]]);
			break;
		}
	}
	if (index == is.size()) 
	{
		basis_->~PointCloud();
		//std::cout << "Point 2 in ";
		return 0;
	}
	for (index = 0; index < is.size(); ++index)
	{
		double dd2 = pcl::euclideanDistance (basis_->points[1], input_->points[is[index]]);
		if (dd2 < dist * MAX_DELTA)
		{
			basis_->push_back (input_->points[index]);
			break;
		}
	}
	if (index == is.size()) 
	{
		basis_->~PointCloud();
		//std::cout << "Point 3 in ";
		return 0;
	}
	next = false;
	//tree.radiusSearch(basis_->points[2], dist*MAX_DELTA, is, ds);
	for (int i = 0; i < input_->size(); ++i)
	{
		if (pcl::euclideanDistance (basis_->points[0], input_->points[i]) == 0					|| 
			pcl::euclideanDistance (basis_->points[0], input_->points[i]) < dist * MIN_DELTA	||
			pcl::euclideanDistance (basis_->points[1], input_->points[i]) == 0					|| 
			pcl::euclideanDistance (basis_->points[1], input_->points[i]) < dist * MIN_DELTA	||
			pcl::euclideanDistance (basis_->points[2], input_->points[i]) == 0					|| 
			pcl::euclideanDistance (basis_->points[2], input_->points[i]) < dist * MIN_DELTA)
		{
			continue;
		}
		float x = input_->points[i].x
			, y = input_->points[i].y
			, z = input_->points[i].z
			, x1 = basis_->points[0].x
			, y1 = basis_->points[0].y
			, z1 = basis_->points[0].z
			, x2 = basis_->points[1].x
			, y2 = basis_->points[1].y
			, z2 = basis_->points[1].z
			, x3 = basis_->points[2].x
			, y3 = basis_->points[2].y
			, z3 = basis_->points[2].z;
		double det = ((x-x1)*(y2-y1)*(z3-z1)
					+ (y-y1)*(z2-z1)*(x3-x1)
					+ (z-z1)*(x2-x1)*(y3-y1)) 
					- ((z-z1)*(y2-y1)*(x3-x1) 
					+ (x-x1)*(z2-z1)*(y3-y1) 
					+ (z3-z1)*(y-y1)*(x2-x1));
		
		if (fabs (det) < 0.01)
		{
			basis_->push_back (input_->points[i]);
			next = true;
			break;
		}
	}
	if (next) 
	{
		return 1;
	}
	basis_->~PointCloud();
	//std::cout << "Point 4 in ";
	return 0;
}

template <typename PointSource, typename PointTarget, typename Scalar> Eigen::Quaternion<float>
pcl::FPCS<PointSource, PointTarget, Scalar>::estimateRotationMatrix (const PointCloudSourceConstPtr left, const PointCloudTargetConstPtr right)
{
	PointCloudSourcePtr left1 (new PointCloudSource);
	PointCloudTargetPtr right1 (new PointCloudTarget);
	*left1 = *left;
	*right1 = *right;
	
	//1.planes rotation

	//left surface normal
	float x1 = left1->points[0].x, y1 = left1->points[0].y, z1 = left1->points[0].z,
		  x2 = left1->points[1].x, y2 = left1->points[1].y, z2 = left1->points[1].z,
		  x3 = left1->points[2].x, y3 = left1->points[2].y, z3 = left1->points[2].z;
	Eigen::Vector3f n1(	(y2-y1)*(z3-z1) - (y3-y1)*(z2-z1), 
						(x2-x1)*(z3-z1) - (x3-x1)*(z2-z1), 
						(x2-x1)*(y3-y1) - (x3-x1)*(y2-y1));
	
	Eigen::Vector3f l1(x1, y1, z1), l2(x2, y2, z2), l3(x3,y3,z3), nl = l2.cross(l1);

	//right surface normal
	x1 = right1->points[0].x; y1 = right1->points[0].y; z1 = right1->points[0].z;
	x2 = right1->points[1].x; y2 = right1->points[1].y; z2 = right1->points[1].z;
	x3 = right1->points[2].x; y3 = right1->points[2].y; z3 = right1->points[2].z;
	
	Eigen::Vector3f n2(	(y2-y1)*(z3-z1) - (y3-y1)*(z2-z1), 
						(x2-x1)*(z3-z1) - (x3-x1)*(z2-z1), 
						(x2-x1)*(y3-y1) - (x3-x1)*(y2-y1));
	
	Eigen::Vector3f r1(x1, y1, z1), r2(x2, y2, z2), r3(x3, y3, z3), nr = r2.cross(r1);
	
	n1 = nl;
	n2 = nr;
	if (n1.isZero() || n2.isZero())
	{
		left1->~PointCloud();
		right1->~PointCloud();
		Eigen::Quaternion<float> q(0,0,0,0);
		//cout << "bad normals..." << endl;
		return q;
	}

	
	//crossing line for 2 planes (a) and sin & cos for rotation
	Eigen::Vector3f a = n1.cross(n2);
	a /= a.norm();
	
	n1 /= n1.norm();
	n2 /= n2.norm();

	float sin, cos = n1.dot(n2);
	sin = n1.cross(n2).norm();

	///need cos alpha/2 and sin alpha/2 if cos alpha = n1 dot n2 and sin alpha = n1 cross n2
	cos = sqrt((cos + 1)/2);
	sin = cos == 0 ? 1 : sin/(2*cos);
	
	Eigen::Quaternion<float> qa(cos, sin*a.x(), sin*a.y(), sin*a.z());
	
	//applying plane rotation
	std::vector<Eigen::Vector3f> ll;
	for (int i = 0; i < left1->size(); ++i)
	{
		float x1 = left1->points[i].x, y1 = left1->points[i].y, z1 = left1->points[i].z;
		Eigen::Vector3f l1(x1, y1, z1);

		ll.push_back(qa.matrix() * l1);
		left1->points[i].x = ll[i].x();
		left1->points[i].y = ll[i].y();
		left1->points[i].z = ll[i].z();
	}

	//2.rotation in plane|points rotation
	float C = r1.dot(ll[0]) + r2.dot(ll[1]) + r3.dot(ll[2]), 
		S = (r1.cross(ll[0]) + r2.cross(ll[1]) + r3.cross(ll[2])).dot(n2), 
		osin = S/sqrt(S*S + C*C), 
		ocos = C/sqrt(S*S + C*C);
	osin = ocos == -1 ? 1 : osin/sqrt(2*(1 + ocos));
	ocos = sqrt((ocos + 1)/2);
	
	Eigen::Quaternion<float> qp(ocos, osin*n2.x(), osin*n2.y(), osin*n2.z());
	
	//result
	Eigen::Quaternion<float> qe = qp*qa;

	ll.~vector();
	right1->~PointCloud();
	left1->~PointCloud();

	return qe;
}

template <typename PointSource, typename PointTarget, typename Scalar> Eigen::Vector3f
pcl::FPCS<PointSource, PointTarget, Scalar>::centeringSource (PointCloudSourcePtr cl)
{
	PointSource p(0,0,0);
	int number_of_points = cl->size();
	for (int i = 0; i < number_of_points; ++i)
	{
		p.x += cl->points[i].x;
		p.y += cl->points[i].y;
		p.z += cl->points[i].z;
	}
	p.x /= number_of_points;
	p.y /= number_of_points;
	p.z /= number_of_points;
	
	for (int i = 0; i < number_of_points; ++i)
	{
		cl->points[i].x -= p.x;
		cl->points[i].y -= p.y;
		cl->points[i].z -= p.z;
	}
	Eigen::Vector3f res(p.x, p.y, p.z);

	return res;
}

template <typename PointSource, typename PointTarget, typename Scalar> Eigen::Vector3f
pcl::FPCS<PointSource, PointTarget, Scalar>::centeringTarget (PointCloudTargetPtr cl)
{
	PointTarget p(0,0,0);
	int number_of_points = cl->size();
	for (int i = 0; i < number_of_points; ++i)
	{
		p.x += cl->points[i].x;
		p.y += cl->points[i].y;
		p.z += cl->points[i].z;
	}
	p.x /= number_of_points;
	p.y /= number_of_points;
	p.z /= number_of_points;
	
	for (int i = 0; i < number_of_points; ++i)
	{
		cl->points[i].x -= p.x;
		cl->points[i].y -= p.y;
		cl->points[i].z -= p.z;
	}
	Eigen::Vector3f res(p.x, p.y, p.z);

	return res;
}

#endif //PCL_FPCS_IMPL_HPP
