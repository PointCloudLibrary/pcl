#include <pcl/octree/octree_key.h>

constexpr unsigned char const pcl::octree::OctreeKey::maxDepth{
    static_cast<unsigned char>(sizeof(uindex_t) * 8)};
