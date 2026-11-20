// Fuzz target for the PLY mesh-read direction and the PLY write direction.
//
// Threat model:
// - Untrusted input: a .ply file. The existing ply_reader_fuzzer only
//   exercises the point-cloud read path (PCLPointCloud2). This harness
//   targets the rest of the PLY stack: PolygonMesh parsing (face elements,
//   list properties such as vertex_indices, cameras) and PLYWriter output
//   in both ascii and binary_little_endian storage (the encode direction),
//   followed by round-trip re-reads of what was written.
// - Entry point: pcl::PLYReader::read(file, pcl::PolygonMesh&) and
//   pcl::PLYWriter::write(...).
// - Expected bug classes: vertex-index out-of-bounds from face lists,
//   parser state-machine bugs on element/property declarations, integer and
//   type-handling UB in the binary writer, round-trip inconsistencies.
//
// Usage pattern note: one reader instance per read call, mirroring
// pcl::io::loadPLYFile (ply_io.h constructs a local PLYReader per call).
// PLYReader carries per-read state (range_grid_), and the canonical pattern
// is a fresh instance per file rather than reuse across reads.
//
// The write-to-temp-file + read-by-path pattern matches the existing
// ply_reader_fuzzer harness convention in this directory.

#include <cstdio>
#include <Eigen/Geometry>
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/PCLPointCloud2.h>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    char filename[256], out_bin[256], out_ascii[256];
    sprintf(filename, "/tmp/libfuzzer-plymesh.%d", getpid());
    sprintf(out_bin, "/tmp/libfuzzer-plymesh-bin.%d", getpid());
    sprintf(out_ascii, "/tmp/libfuzzer-plymesh-ascii.%d", getpid());

    FILE *fp = fopen(filename, "wb");
    if (!fp)
        return 0;
    fwrite(data, size, 1, fp);
    fclose(fp);

    pcl::PolygonMesh mesh;
    {
        pcl::PLYReader reader;
        if (reader.read(filename, mesh) < 0) {
            unlink(filename);
            return 0;
        }
    }

    // Writer direction in both storage formats, each followed by a
    // round-trip re-read of the produced file (fresh reader per call).
    const Eigen::Vector4f origin = Eigen::Vector4f::Zero();
    const Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    pcl::PLYWriter writer;
    if (writer.write(out_bin, mesh.cloud, origin, orientation, true, true) >= 0) {
        pcl::PCLPointCloud2 back;
        pcl::PLYReader reader;
        reader.read(out_bin, back);
    }
    if (writer.write(out_ascii, mesh.cloud, origin, orientation, false, false) >= 0) {
        pcl::PCLPointCloud2 back;
        pcl::PLYReader reader;
        reader.read(out_ascii, back);
    }

    unlink(filename);
    unlink(out_bin);
    unlink(out_ascii);
    return 0;
}
