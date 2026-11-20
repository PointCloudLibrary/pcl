// Fuzz target for the IFS binary mesh reader.
//
// Threat model:
// - Untrusted input: an .ifs mesh file (dispatched by pcl::io::load). The
//   whole file is attacker-controlled: length-prefixed magic/version/name
//   strings, the VERTICES block with its u32 point count, and the
//   TRIANGLES block with its u32 facet count and index triples.
// - Entry point: pcl::IFSReader::read() into PCLPointCloud2 and into
//   PolygonMesh. The mesh path additionally parses the TRIANGLES block and
//   memcpy's both blocks out of a boost::iostreams mapped file whose length
//   is derived from the attacker-controlled header counts.
// - Expected bug classes: out-of-bounds reads when declared counts exceed
//   the actual file (truncated/lying headers drive memcpy/mmap reads past
//   EOF), huge-allocation OOM from inflated counts, facet index UB.
//
// The write-to-temp-file + read-by-path pattern matches the existing
// ply_reader_fuzzer harness convention in this directory.

#include <cstdio>
#include <pcl/io/ifs_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    char filename[256];
    sprintf(filename, "/tmp/libfuzzer-ifs.%d", getpid());

    FILE *fp = fopen(filename, "wb");
    if (!fp)
        return 0;
    fwrite(data, size, 1, fp);
    fclose(fp);

    pcl::IFSReader reader;

    int ifs_version = 0;
    pcl::PCLPointCloud2 blob;
    reader.read(filename, blob, ifs_version);

    int ifs_version_mesh = 0;
    pcl::PolygonMesh mesh;
    reader.read(filename, mesh, ifs_version_mesh);

    unlink(filename);
    return 0;
}
