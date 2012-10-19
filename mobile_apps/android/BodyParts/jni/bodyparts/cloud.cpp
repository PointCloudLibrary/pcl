#include <new>

#include <boost/container/vector.hpp>

#include "cloud.h"

namespace bc = boost::container;

template <typename Format>
ChannelRef<Format>::ChannelRef(int width, int height, Format * data)
  : width(width), height(height), size(width * height), data(data)
{ }

typedef boost::container::flat_map<const std::type_info *, void *, TypeInfoComparator>::iterator storage_iterator;

void
Cloud::resize(int width, int height)
{
  if (width == width_ && height == height_) return;

  for (storage_iterator it = storage_.begin(); it != storage_.end(); ++it)
    operator delete[](it->second);

  storage_.clear();
  width_ = width;
  height_ = height;
}

template <typename Format> ChannelRef<Format>
Cloud::get(const std::type_info * key)
{
  return ChannelRef<Format>(width_, height_, getRaw<Format>(key));
}

template <typename Format> Format *
Cloud::getRaw(const std::type_info * key)
{
  storage_iterator pos = storage_.lower_bound(key);
  if (pos != storage_.end() && pos->first == key) return static_cast<Format *>(pos->second);

  Format * new_data = static_cast<Format *>(operator new[](width_ * height_ * sizeof(Format)));
  storage_.insert(pos, std::make_pair(key, new_data));

  return new_data;
}

Cloud::~Cloud()
{
  resize(0, 0);
}

#define DEFINE_FORMAT(Format) \
  template ChannelRef<Format> Cloud::get(const std::type_info * key); \
  template Format * Cloud::getRaw(const std::type_info * key);

DEFINE_FORMAT(unsigned char)
DEFINE_FORMAT(signed char)
DEFINE_FORMAT(unsigned short)
DEFINE_FORMAT(signed short)
DEFINE_FORMAT(unsigned int)
DEFINE_FORMAT(signed int)

DEFINE_FORMAT(RGB)
