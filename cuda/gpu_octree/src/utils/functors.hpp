#ifndef IMPL_FUNCTIONAL_HPP_
#define IMPL_FUNCTIONAL_HPP_

//#include<thrust/tuple.h>
//#include<thrust/functional.h>

struct SelectMinPoint
{
	__host__ __device__ float3 operator()(const float3& e1, const float3& e2) const
	{
		return make_float3(fmin(e1.x, e2.x), fmin(e1.y, e2.y), fmin(e1.z, e2.z));	
	}	
};

struct SelectMaxPoint
{
	__host__ __device__ float3 operator()(const float3& e1, const float3& e2) const 
	{		
		return make_float3(fmax(e1.x, e2.x), fmax(e1.y, e2.y), fmax(e1.z, e2.z));	
	}	
};

//struct Shift3D
//{	
//	float x, y, z;
//	Shift3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_)  {}
//	
//	__host__ __device__ 
//	thrust::tuple<float, float, float> operator()(const thrust::tuple<float, float, float>& point) const
//	{
//		thrust::tuple<float, float, float> result = point;
//		result.get<0>() += x;
//		result.get<1>() += y;
//		result.get<2>() += z;
//		return result;
//	}
//
//	__host__ __device__ 
//	float3 operator()(const float3& point) const
//	{ 
//		return make_float3(point.x + x, point.y + y, point.z + z);
//	}
//};
//
//struct NotMinus1 : public thrust::unary_function<int, int> 
//{ 
//    __host__ __device__ int operator()(int e) const {  return e == -1 ? 0 : 1; } 
//};
//
//struct float3_to_3floats
//{
//	__host__ __device__ thrust::tuple<float, float, float> operator()(const float3 in) const 
//	{
//		return thrust::tuple<float, float, float>(in.x, in.y, in.z);		
//	}
//};


#endif /* IMPL_FUNCTIONAL_HPP_ */