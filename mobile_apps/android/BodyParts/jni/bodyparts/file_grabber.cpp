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

  virtual void getFrame(RGBDImage * frame);

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

void FileGrabber::getFrame(RGBDImage * frame)
{
  __android_log_print(ANDROID_LOG_DEBUG, "FileGrabber", "Readin' %s", file_list[current_file].c_str());
  fs::ifstream is(file_list[current_file], std::ios::binary);

  is.seekg(0, std::ios::end);
  std::ios::pos_type size = is.tellg();

  is.seekg(0, std::ios::beg);
  std::vector<char> contents(size);
  is.read(&contents.front(), size);

  __android_log_print(ANDROID_LOG_DEBUG, "FileGrabber", "Read %d bytes", int(size));

  frame->parse(&contents.front());

  ++current_file;
  if (current_file >= file_list.size())
    current_file = 0;
}

Grabber * Grabber::createFileGrabber(const std::string & directory)
{
  return new FileGrabber(directory);
}
