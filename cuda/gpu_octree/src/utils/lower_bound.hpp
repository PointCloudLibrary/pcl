#pragma once

template<typename RandomAccessIterator, typename T, typename BinaryPredicate>
__host__ __device__ RandomAccessIterator lower_bound(RandomAccessIterator first, RandomAccessIterator last, const T &val, BinaryPredicate comp)
{  
    int len = last - first;

    while(len > 0)
    {
        int half = len >> 1;
        RandomAccessIterator middle = first;

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