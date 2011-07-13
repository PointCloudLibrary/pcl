#pragma once

template<typename Iterator, typename T, typename BinaryPredicate>
__host__ __device__ Iterator lower_bound(Iterator first, Iterator last, const T &val, BinaryPredicate comp)
{  
    int len = last - first;

    while(len > 0)
    {
        int half = len >> 1;
        Iterator middle = first;

        middle += half;

        if(comp(*middle, val))
        {
            first = middle;
            ++first;
            len = len - half - 1;
        }
        else
        {
            len = half;
        }
    }

    return first;
}