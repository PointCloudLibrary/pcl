#include "internal.hpp"
#include <algorithm>
#include "pcl/gpu/utils/safe_call.hpp"
#include "pcl/pcl_exports.h"

namespace pcl
{
    namespace device
    {
        struct Info
        {
            enum { SIZE = 4 };
            int data[SIZE];
        };

        template<int n>
        struct Point
        {
            int data[n];
        };

        template<int in, int out, typename Info>
        __global__ void deviceCopyFields4B(const Info info, const int size, const void* input, void* output)
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;

            if (idx < size)
            {
                Point<in>  point_in  = reinterpret_cast<const  Point<in>* >( input)[idx];
                Point<out> point_out = reinterpret_cast<      Point<out>* >(output)[idx];

                for(int i = 0; i < Info::SIZE; ++i)
                {
                    int byte;
                    int code = info.data[i];

                    byte = ((code >> 0) & 0xFF);

                    if (byte == 0xFF)
                        break;
                    else
                        point_out.data[byte >> 4] = point_in.data[byte & 0xF];

                    byte = ((code >> 8) & 0xFF);

                    if (byte == 0xFF)
                        break;
                    else
                        point_out.data[byte >> 4] = point_in.data[byte & 0xF];

                    byte = ((code >> 16) & 0xFF);

                    if (byte == 0xFF)
                        break;
                    else
                        point_out.data[byte >> 4] = point_in.data[byte & 0xF];

                    byte = ((code >> 24) & 0xFF);

                    if (byte == 0xFF)
                        break;
                    else
                        point_out.data[byte >> 4] = point_in.data[byte & 0xF];
                }

                reinterpret_cast< Point<out>* >(output)[idx] = point_out;
            }
        };

        template<int in_size, int out_size>
        void cf(int info[4], int size, const void* input, void* output)
        {
            Info i;
            std::copy(info, info + 4, i.data);

            dim3 block(256);
            dim3 grid(divUp(size, block.x));

            deviceCopyFields4B<in_size, out_size><<<grid, block>>>(i, size, input, output);
            cudaSafeCall ( cudaGetLastError () );
            cudaSafeCall (cudaDeviceSynchronize ());
        }

        typedef void (*copy_fields_t)(int info[4], int size, const void* input, void* output);
    }
}

namespace pcl
{
    namespace gpu
    {
        using namespace pcl::device;

        PCL_EXPORTS void copyFieldsImpl(int in_size, int out_size, int rules[4], int size, const void* input, void* output)
        {
            pcl::device::copy_fields_t funcs[16][16] =
            {
                { /**/ cf<1,1>,  cf<1, 2>, cf<1, 3>, cf<1, 4>, /**/ cf<1, 5>, cf<1, 6>, cf<1, 7>, cf<1, 8>, /**/ cf<1, 9>, cf<1,10>, cf<1, 11>, cf<1, 12>, /**/ cf<1, 13>, cf<1, 14>, cf<1, 15>,  cf<1,16> },
                { /**/ cf<2,1>,  cf<2, 2>, cf<2, 3>, cf<2, 4>, /**/ cf<2, 5>, cf<2, 6>, cf<2, 7>, cf<2, 8>, /**/ cf<2, 9>, cf<1,10>, cf<2, 11>, cf<2, 12>, /**/ cf<2, 13>, cf<2, 14>, cf<2, 15>,  cf<2,16> },
                { /**/ cf<3,1>,  cf<3, 2>, cf<3, 3>, cf<3, 4>, /**/ cf<3, 5>, cf<3, 6>, cf<3, 7>, cf<3, 8>, /**/ cf<3, 9>, cf<1,10>, cf<3, 11>, cf<3, 12>, /**/ cf<3, 13>, cf<3, 14>, cf<3, 15>,  cf<3,16> },
                { /**/ cf<4,1>,  cf<4, 2>, cf<4, 3>, cf<4, 4>, /**/ cf<4, 5>, cf<4, 6>, cf<4, 7>, cf<4, 8>, /**/ cf<4, 9>, cf<1,10>, cf<4, 11>, cf<4, 12>, /**/ cf<4, 13>, cf<4, 14>, cf<4, 15>,  cf<4,16> },
                { /**/ cf<5,1>,  cf<5, 2>, cf<5, 3>, cf<5, 4>, /**/ cf<5, 5>, cf<5, 6>, cf<5, 7>, cf<5, 8>, /**/ cf<5, 9>, cf<1,10>, cf<5, 11>, cf<5, 12>, /**/ cf<5, 13>, cf<5, 14>, cf<5, 15>,  cf<5,16> },
                { /**/ cf<6,1>,  cf<6, 2>, cf<6, 3>, cf<6, 4>, /**/ cf<6, 5>, cf<6, 6>, cf<6, 7>, cf<6, 8>, /**/ cf<6, 9>, cf<1,10>, cf<6, 11>, cf<6, 12>, /**/ cf<6, 13>, cf<6, 14>, cf<6, 15>,  cf<6,16> },
                { /**/ cf<7,1>,  cf<7, 2>, cf<7, 3>, cf<7, 4>, /**/ cf<7, 5>, cf<7, 6>, cf<7, 7>, cf<7, 8>, /**/ cf<7, 9>, cf<1,10>, cf<7, 11>, cf<7, 12>, /**/ cf<7, 13>, cf<7, 14>, cf<7, 15>,  cf<7,16> },
                { /**/ cf<8,1>,  cf<8, 2>, cf<8, 3>, cf<8, 4>, /**/ cf<8, 5>, cf<8, 6>, cf<8, 7>, cf<8, 8>, /**/ cf<8, 9>, cf<1,10>, cf<8, 11>, cf<8, 12>, /**/ cf<8, 13>, cf<8, 14>, cf<8, 15>,  cf<8,16> },
                { /**/ cf<9,1>,  cf<9, 2>, cf<9, 3>, cf<9, 4>, /**/ cf<9, 5>, cf<9, 6>, cf<9, 7>, cf<9, 8>, /**/ cf<9, 9>, cf<1,10>, cf<9, 11>, cf<9, 12>, /**/ cf<9, 13>, cf<9, 14>, cf<9, 15>,  cf<9,16> },
                { /**/ cf<10,1>, cf<10,2>, cf<10,3>, cf<10,4>, /**/ cf<10,5>, cf<10,6>, cf<10,7>, cf<10,8>, /**/ cf<10,9>, cf<1,10>, cf<10,11>, cf<10,12>, /**/ cf<10,13>, cf<10,14>, cf<10,15>, cf<10,16> },
                { /**/ cf<11,1>, cf<11,2>, cf<11,3>, cf<11,4>, /**/ cf<11,5>, cf<11,6>, cf<11,7>, cf<11,8>, /**/ cf<11,9>, cf<1,10>, cf<11,11>, cf<11,12>, /**/ cf<11,13>, cf<11,14>, cf<11,15>, cf<11,16> },
                { /**/ cf<12,1>, cf<12,2>, cf<12,3>, cf<12,4>, /**/ cf<12,5>, cf<12,6>, cf<12,7>, cf<12,8>, /**/ cf<12,9>, cf<1,10>, cf<12,11>, cf<12,12>, /**/ cf<12,13>, cf<12,14>, cf<12,15>, cf<12,16> },
                { /**/ cf<13,1>, cf<13,2>, cf<13,3>, cf<13,4>, /**/ cf<13,5>, cf<13,6>, cf<13,7>, cf<13,8>, /**/ cf<13,9>, cf<1,10>, cf<13,11>, cf<13,12>, /**/ cf<13,13>, cf<13,14>, cf<13,15>, cf<13,16> },
                { /**/ cf<14,1>, cf<14,2>, cf<14,3>, cf<14,4>, /**/ cf<14,5>, cf<14,6>, cf<14,7>, cf<14,8>, /**/ cf<14,9>, cf<1,10>, cf<14,11>, cf<14,12>, /**/ cf<14,13>, cf<14,14>, cf<14,15>, cf<14,16> },
                { /**/ cf<15,1>, cf<15,2>, cf<15,3>, cf<15,4>, /**/ cf<15,5>, cf<15,6>, cf<15,7>, cf<15,8>, /**/ cf<15,9>, cf<1,10>, cf<15,11>, cf<15,12>, /**/ cf<15,13>, cf<15,14>, cf<15,15>, cf<15,16> },
                { /**/ cf<16,1>, cf<16,2>, cf<16,3>, cf<16,4>, /**/ cf<16,5>, cf<16,6>, cf<16,7>, cf<16,8>, /**/ cf<16,9>, cf<1,10>, cf<16,11>, cf<16,12>, /**/ cf<16,13>, cf<16,14>, cf<16,15>, cf<16,16> }
            };

            funcs[in_size-1][out_size-1](rules, size, input, output);
        }
    }
}

