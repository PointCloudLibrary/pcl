#ifndef CLOUD_H_
#define CLOUD_H_

#include <typeinfo>

#include <boost/noncopyable.hpp>
#include <boost/config.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/cstdint.hpp>

template <typename Format>
struct ChannelRef
{
  const int width;
  const int height;
  const int size;

  Format * const data;

  const Format & at(int row, int column) const { return data[row * width + column]; }
  Format & at(int row, int column) { return data[row * width + column]; }

  Format atDef(int row, int column, Format def) const {
    if (row < 0 || row >= height || column < 0 || column >= width) return def;
    return at(row, column);
  }

  ChannelRef(int width, int height, Format * data);
};

struct TypeInfoComparator
{
  bool operator () (const std::type_info * t1, const std::type_info * t2) const { return (*t1).before(*t2); }
};

struct Cloud : boost::noncopyable
{
  template <typename Tag> ChannelRef<typename Tag::format_type>
  get()
  {
    return get<typename Tag::format_type>(&typeid(Tag));
  }

  void
  resize(int width, int height);

  int
  getWidth() const { return width_; }

  int
  getHeight() const { return height_; }

  ~Cloud();

private:
  int width_, height_;
  boost::container::flat_map<const std::type_info *, void *, TypeInfoComparator> storage_;

  template <typename Format> ChannelRef<Format>
  get(const std::type_info * key);

  template <typename Format> Format *
  getRaw(const std::type_info * key);
};

#define DECLARE_CLOUD_TAG(tag, format) struct tag { typedef format format_type; };

struct RGB
{
  boost::uint8_t r, g, b, dummy;
};

typedef boost::int16_t Depth;

DECLARE_CLOUD_TAG(TagColor, RGB)
DECLARE_CLOUD_TAG(TagDepth, Depth)

#endif
