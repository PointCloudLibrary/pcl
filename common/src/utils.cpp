#include <pcl/common/utils.h>
#include <pcl/exceptions.h>

int
pcl::utils::interpolate (int p, int len, int type)
{
  if (static_cast<unsigned> (p) >= static_cast<unsigned> (len))
  {
    if (type == BORDER_REPLICATE)
      p = p < 0 ? 0 : len - 1;
    else if (type == BORDER_REFLECT || type == BORDER_REFLECT_101)
    {
      int delta = type == BORDER_REFLECT_101;
      if (len == 1)
        return 0;
      do
      {
        if (p < 0)
          p = -p - 1 + delta;
        else
          p = len - 1 - (p - len) - delta;
      }
      while (static_cast<unsigned> (p) >= static_cast<unsigned> (len));
    }
    else if (type == BORDER_WRAP)
    {
      if (p < 0)
        p -= ((p-len+1)/len)*len;
      if (p >= len)
        p %= len;
    }
    else if (type == BORDER_CONSTANT)
      p = -1;
    else
    {
      PCL_THROW_EXCEPTION (BadArgumentsException,
                           "[pcl::interpolate] error: Unhandled interpolation type " 
                           << type << " !");
    }
  }
  
  return (p);
}
