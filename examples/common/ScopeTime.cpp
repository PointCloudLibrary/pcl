#include <iostream>

#include <pcl/common/time.h>

int main (int argc, char** argv)
{
  pcl::ScopeTime scopeTime ("Test loop");
  {
    float total = 0.0f;
    for (size_t i = 0; i < 1e4; ++i)
      {
      total += i;
      }
  }
  std::cout << "Done." << std::endl;

  return (0);
}
