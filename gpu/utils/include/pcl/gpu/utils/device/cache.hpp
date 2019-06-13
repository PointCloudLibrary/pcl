


//will contain a lot of load/store utils

namespace pcl
{
    namespace device
    {
        template<class T>
        struct NonCachedLoad
        {
            __device__ static T Invoke(const T* ptr)
            {
                //asm code insertion 
                asm(...);
            }
        };

    }

}