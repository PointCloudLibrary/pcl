#include <pcl/point_types.h>

#include <pybind11/pybind11.h>
namespace py = pybind11;
namespace pcl {
PYBIND11_MODULE(pcl, m)
{
  py::class_<_PointXYZ>(m, "_PointXYZ");
  py::class_<PointXYZ, _PointXYZ>(m, "PointXYZ")
      .def(py::init<const _PointXYZ&>())
      .def(py::init<>())
      .def(py::init<float, float, float>());
  py::class_<_RGB>(m, "_RGB");
  py::class_<RGB, _RGB>(m, "RGB")
      .def(py::init<const _RGB>())
      .def(py::init<>())
      .def(py::init<std::uint8_t, std::uint8_t, std::uint8_t>());
}
} // namespace pcl
