#include <android/log.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "grabber.h"

namespace fs = boost::filesystem;

class FileGrabber : public Grabber
{
  std::vector<fs::path> file_list;
  std::size_t current_file;

public:
  FileGrabber(const std::string & directory);

  virtual bool isConnected() const { return !file_list.empty(); }

  virtual void getFrame(Cloud & frame);

  virtual void start() {}
  virtual void stop() {}
};

FileGrabber::FileGrabber(const std::string & directory)
  : current_file(0)
{
  __android_log_write(ANDROID_LOG_DEBUG, "FileGrabber", "Iteratin'");
  for (fs::directory_iterator iter = fs::directory_iterator(directory);
       iter != fs::directory_iterator(); ++iter)
  {
    if (iter->status().type() != fs::regular_file)
      continue;
    file_list.push_back(iter->path());
  }
}

void FileGrabber::getFrame(Cloud & frame)
{
  fs::ifstream is(file_list[current_file], std::ios::binary);

  boost::uint32_t width, height;
  is.read(reinterpret_cast<char *>(&width), sizeof width);
  is.read(reinterpret_cast<char *>(&height), sizeof height);

  frame.resize(width, height);

  ChannelRef<RGB> rgb = frame.get<TagColor>();
  ChannelRef<Depth> depth = frame.get<TagDepth>();

  is.read(reinterpret_cast<char *>(rgb.data), width * height * sizeof *rgb.data);
  is.read(reinterpret_cast<char *>(depth.data), width * height * sizeof *depth.data);

  ++current_file;
  if (current_file >= file_list.size())
    current_file = 0;
}

Grabber * Grabber::createFileGrabber(const std::string & directory)
{
  return new FileGrabber(directory);
}
