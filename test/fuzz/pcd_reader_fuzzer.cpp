// Fuzz target for the PCD (Point Cloud Data) native-format reader.
//
// Threat model:
// - Untrusted input: a .pcd file opened by a PCL application (the standard
//   way PCL persists point clouds; pcl::io::load() dispatches .pcd here).
//   The whole file is attacker-controlled: header keywords, field layout
//   (FIELDS/SIZE/TYPE/COUNT), WIDTH/HEIGHT/POINTS counts, and the body in
//   ascii, binary or binary_compressed (LZF) encoding.
// - Entry point: pcl::PCDReader::read(), which runs readHeader() and the
//   ascii/binary/binary_compressed body readers, including pcl::lzfDecompress.
// - Expected bug classes: out-of-bounds reads/writes from count/step
//   arithmetic, heap overflow in the binary body decode, OOB in the LZF
//   decompressor, integer overflow in point_step*width products, OOM from
//   inflated counts, UBSan on malformed float/ASCII data.
//
// The write-to-temp-file + read-by-path pattern matches the existing
// ply_reader_fuzzer harness convention in this directory (PCDReader, like
// PLYReader, exposes file-path based reads).

#include <cstdio>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    pcl::PCLPointCloud2 cloud_blob;
    pcl::PCDReader reader;
    char filename[256];
    sprintf(filename, "/tmp/libfuzzer-pcd.%d", getpid());

    FILE *fp = fopen(filename, "wb");
    if (!fp)
        return 0;
    fwrite(data, size, 1, fp);
    fclose(fp);

    reader.read(filename, cloud_blob);
    unlink(filename);
    return 0;
}
