#include <Python.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include "pygen.i"

PyObject *makePointCloud(PyObject *self, PyObject *args, PyObject *kwds)
{
  PyObject *o = PyObject_CallObject((PyObject *)&PointCloud_PointXYZ_Type, NULL);
  return o;
#if 0
  PointCloud_PointXYZC *o = PyObject_New(PointCloud_PointXYZC, PointCloud_PointXYZ_Type);
  if (PyArg_ParseTuple(args, "O", &point_type)) {
    new(&object->i) pcl::PointCloud<pcl::PointXYZ>();
    return 0;
  } else {
    return -1;
  }
#endif
}

static PyMethodDef methods[] = {
  {"PointCloud", (PyCFunction)makePointCloud, METH_KEYWORDS},
  {NULL},
};

extern "C" void initpcl()
{
  PyObject *m, *d, *item;

  m = Py_InitModule("pcl", methods);
  d = PyModule_GetDict(m);

  PointXYZ_Type.tp_new = PyType_GenericNew;
  if (PyType_Ready(&PointXYZ_Type) != 0)
      return;
  PyModule_AddObject(m, "PointXYZ", (PyObject *)&PointXYZ_Type);

  PointCloud_PointXYZ_Type.tp_new = PyType_GenericNew;
  PointCloud_PointXYZ_Type.tp_init = makePointCloud_PointXYZ;
  if (PyType_Ready(&PointCloud_PointXYZ_Type) != 0)
      return;

  PointCloudPoints_PointXYZ_Type.tp_new = PyType_GenericNew;
  if (PyType_Ready(&PointCloudPoints_PointXYZ_Type) != 0)
      return;

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  }
  PyDict_SetItemString(d, "__version__", item= PyString_FromString("0.0.0")); Py_DECREF(item);
}
