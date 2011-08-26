


template<typename T>
struct plus
{
    __device__ __host__ __forceinline__ T operator()(const T& t1, const T& t2) const 
    {
        return t1 + t2;
    }
};

