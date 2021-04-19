#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl/PolygonMesh.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    pcl::PCLPointCloud2 cloud_blob;
    pcl::PLYReader reader;
    char filename[256];
    sprintf(filename, "/tmp/libfuzzer.%d", getpid());

    FILE *fp = fopen(filename, "wb");
    if (!fp)
        return 0;
    fwrite(data, size, 1, fp);
    fclose(fp);

    reader.read (filename, cloud_blob); 
    unlink(filename);
    return 0;
}
