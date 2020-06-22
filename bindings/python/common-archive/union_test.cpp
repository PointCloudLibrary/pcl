#include <pybind11/pybind11.h>
namespace py = pybind11;

union RGB {
  int rgb;
  struct {
    int r;
    int g;
    int b;
  };
};

PYBIND11_MODULE(pcl, m)
{
  py::class_<RGB>(m, "RGB")
      .def(py::init<>())
      .def_property(
          "rgb",
          [](RGB& self) -> int { return self.rgb; },
          [](RGB& self, int value) { self.rgb = value; })
      .def_property(
          "r",
          [](RGB& self) -> int { return self.r; },
          [](RGB& self, int value) { self.r = value; })
      .def_property(
          "g",
          [](RGB& self) -> int { return self.g; },
          [](RGB& self, int value) { self.g = value; })
      .def_property(
          "b",
          [](RGB& self) -> int { return self.b; },
          [](RGB& self, int value) { self.b = value; });
}
