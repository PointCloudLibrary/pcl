#pragma once

__device__ __host__ __forceinline__
static inline bool checkIfNodeInsideSphere(const float3& minp, const float3& maxp, const float3& c, float r)
{
    r *= r;

    float d2_xmin = (minp.x - c.x) * (minp.x - c.x);
    float d2_ymin = (minp.y - c.y) * (minp.y - c.y);
    float d2_zmin = (minp.z - c.z) * (minp.z - c.z);

    if (d2_xmin + d2_ymin + d2_zmin > r)
        return false;

    float d2_zmax = (maxp.z - c.z) * (maxp.z - c.z);

    if (d2_xmin + d2_ymin + d2_zmax > r)
        return false;

    float d2_ymax = (maxp.y - c.y) * (maxp.y - c.y);

    if (d2_xmin + d2_ymax + d2_zmin > r)
        return false;

    if (d2_xmin + d2_ymax + d2_zmax > r)
        return false;

    float d2_xmax = (maxp.x - c.x) * (maxp.x - c.x);

    if (d2_xmax + d2_ymin + d2_zmin > r)
        return false;

    if (d2_xmax + d2_ymin + d2_zmax > r)
        return false;

    if (d2_xmax + d2_ymax + d2_zmin > r)
        return false;

    if (d2_xmax + d2_ymax + d2_zmax > r)
        return false;

    return true;
}

__device__ __host__ __forceinline__ 
static inline bool checkIfNodeOutsideSphere(const float3& minp, const float3& maxp, const float3& c, float r)
{
    if (maxp.x < (c.x - r) ||  maxp.y < (c.y - r) || maxp.z < (c.z - r))
        return true;

    if ((c.x + r) < minp.x || (c.y + r) < minp.y || (c.z + r) < minp.z)
        return true;

    return false;
}

__device__ __host__ __forceinline__
static inline void calcBoundingBox(int level, int code, float3& res_minp, float3& res_maxp)
{        
    int cell_x, cell_y, cell_z;
    Morton::decomposeCode(code, cell_x, cell_y, cell_z);   

    float cell_size_x = (res_maxp.x - res_minp.x) / (1 << level);
    float cell_size_y = (res_maxp.y - res_minp.y) / (1 << level);
    float cell_size_z = (res_maxp.z - res_minp.z) / (1 << level);

    res_minp.x = cell_x * cell_size_x;
    res_minp.y = cell_y * cell_size_y;
    res_minp.z = cell_z * cell_size_z;

    res_maxp.x = res_minp.x + cell_size_x;
    res_maxp.y = res_minp.y + cell_size_y;
    res_maxp.z = res_minp.z + cell_size_z;       
}