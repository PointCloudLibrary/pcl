// Fuzz target for the OBJ mesh text parser.
//
// Threat model:
// - Untrusted input: a .obj mesh file (widely exchanged mesh format,
//   dispatched by pcl::io::load). The whole file is attacker-controlled:
//   v/vn/vt vertex data, f face lines indexing the vertex arrays, g/o/s
//   grouping state, usemtl/mtllib material references.
// - Entry point: pcl::OBJReader::read() into all three destination shapes
//   (PCLPointCloud2, PolygonMesh, TextureMesh). The three overloads share
//   the tokenizer but build different structures, so each is called.
// - Expected bug classes: vertex-index out-of-bounds from face lines,
//   tokenizer/parser crashes on malformed lines, leaks on error paths,
//   integer UB in index handling.
//
// The write-to-temp-file + read-by-path pattern matches the existing
// ply_reader_fuzzer harness convention in this directory.

#include <cstdio>
#include <pcl/io/obj_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    char filename[256];
    sprintf(filename, "/tmp/libfuzzer-obj.%d", getpid());

    FILE *fp = fopen(filename, "wb");
    if (!fp)
        return 0;
    fwrite(data, size, 1, fp);
    fclose(fp);

    pcl::OBJReader reader;

    pcl::PCLPointCloud2 blob;
    reader.read(filename, blob);

    pcl::PolygonMesh mesh;
    reader.read(filename, mesh);

    pcl::TextureMesh tex_mesh;
    reader.read(filename, tex_mesh);

    unlink(filename);
    return 0;
}
