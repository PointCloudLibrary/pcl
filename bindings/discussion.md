## Python bindings

### Shared pointer usage
#### [05-06-2020]
- > When we want `cloud = pcl.PointCloud[pcl.PointXYZ]()` to be a shared_ptr by default, we set the holder class as shared_ptr. This is needed in some cases because the interface otherwise would be non-idomatic:
  > ```py
  > import pcl
  > 
  > filter = pcl.filters.PassThrough[pcl.PointXYZ]()
  > filter.setInput(cloud)
  > ```
  > 
  > Here, cloud needs to be a shared_ptr. That can be done in 2 ways
  > 1. `cloud = pcl.PointCloud[pcl.PointXYZ]()` and the holder class as shared_ptr
  > 2. `cloud = pcl.make_shared[pcl.PointXYZ]()` and holder class as void
  > 
  > The issue is the ease-of-use and expected semantics
  > ```py
  > cloud2 = cloud1 # Python user will assume this is a shallow copy
  > ```
  > 
  > This will only be true for a variable held in a shared_ptr. This is the semantics in Python.
  > 
  > However, wrapping everything in shared_ptr has downsides for C++ wrapper:
  > ```py
  > import pcl
  > 
  > point = pcl.PointXYZ()
  > cloud[100] = point
  > ```
  > 
  > If PointXYZ is held in a shared_ptr... things go south. If not, things go south

### Handling unions
#### [04-06-2020]
- > given `assert(&(union.r) == &(union.rgb));` does the following hold:
  > `assert id(union_wrapper.r) == id(union_wrapper.rgb) ?`
  Yes. Tested.
- Working example:
  ```cpp
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
  ```

### General
#### [05-06-2020]
- MetaCPP relies on Clang's LibTooling to generate all the metadata: https://github.com/mlomb/MetaCPP

#### [04-06-2020]
- > Was reading this yesterday: https://peerj.com/articles/cs-149.pdf
  > 
  > Summary: 
  > - They too, automatically generate Python bindings for C++ code using Boost::Python.
  > - The whole process is python based.
  > - Same design as to what we have in mind i.e., parse, process, and generate.
  > - Their data structure of choice for the whole process is Abstract Semantic Graph: https://github.com/StatisKit/AutoWIG/blob/master/src/py/autowig/asg.py
  > - The architecture is plugin-based, and somebody added a pybind plugin some months back: https://github.com/StatisKit/AutoWIG/blob/master/src/py/autowig/pybind11_generator.py
  > - They use libclang for frontend parsing(python API). The project was done some time back so they wrote their own py-libclang code: https://github.com/StatisKit/AutoWIG/blob/master/src/py/autowig/libclang_parser.py
  > - Repo: https://github.com/StatisKit/AutoWIG
  > 
  > I think it can act as a good reference for the project. Have a look at the pdf and the source if you wish.
  > The libclang python part can be explored from their repo as of now (as python-libclang has no documentation whatsoever and the 1-2 example articles are outdated)
- Problems:
  > Templates:
  > * suffixing doesn't work well. Unless you're a fan of the pseudo-Hungarian notation espoused by older devs (and by MS). It's ok for 1 (or maybe 2) arguments.
  > * Templates in Python represent an instantiation of C++ template. It should be easy to add/remove an instantiation without affecting needless code. It should also be visually easy to switch the template type without having to lookup the notation or count underscores
  > * Python has a strong syntax for this: index lookup via __getitem__ and __setitem__
  > * Using strings as keys is bad because the editor can't help if spelling is wrong. Pandas MultiKey failed here.
- Use of a templating engine for pybind11 code gen (jinja2>mako)

#### [03-06-2020]
- Ambiguity in the phrase: "full control over clang's AST"

#### [02-06-2020]
- Use of python bindings of libclang, for faster prototyping: https://github.com/llvm/llvm-project/blob/master/clang/bindings/python/clang/cindex.py 
- > protyping and exploring python bindings in which everything is runtime and can be done interactively would usually be my first approach

#### [28-05-2020]
- > While reading lsst's documentation, came to find out they use a __str__ method:
  > ```py
  > cls.def("__str__", [](Class const& self) {
  >     std::ostringstream os;
  >     os << self;
  >     return os.str();
  > });
  > ```
- > the << operator with ostreams is the most common way in C++ of extracting a string representation of a given object (I have no idea why there's no practice of implementing the cast to string method),
That being said I believe you can use << with std::stringstreams, effectively allowing you to fetch a string representation of PCL objects which have operator<< (std::ostream, ....) implemented.

#### [15-05-2020]
- > You can create docstring from \brief part and copy the function signature via libtooling.

#### [09-05-2020]
- Start with binding PointTypes.
- AST parsing helps in cases of convoluted code.
- > We can keep 2 approaches in parallel:
  > 1. header parser on a limited number of files
  > 2. libtooling to replace it
  > 1st will allow the pipeline to be developed
  > 2nd will replace that
- > We can make a prototype which works on manually provided API points
- > From my understanding:
  > 1. Code -> AST -> JSON: use some tool for it first, then replace with libtooling
  > 2. JSON -> cpp: Python tool, language dependent
  > 3. CMake + compile_database.json: rest of toolchain
  > 4. organize properly so usage in LANG is nice

#### [05-05-2020]
- > I'd put PyPi integration immediately after we get 1 module working. That'd allow us to keep shipping improved bindings after GSoC (in case the timeline gets delayed)
The order in which modules are tackled should be the dependency order (because we don't have the popularity data from our users)

***

## Javascript Bindings
#### [05-05-2020]
- Webassembly as an option: https://en.wikipedia.org/wiki/WebAssembly
- Emscripten as an option: https://emscripten.org/
- > *  Getting clang to compile to WebAsm will be the best "performance".
  > * Using Emscripten on the other hand is a well-travelled road, but the performance will be similar to hand written JS (or worse).
  > Both approaches need clang so that's a milestone we need to conquer first.
    
***
    
## Jupyter Visualisation
#### [05-05-2020]
- > WebGL view straddles JS bindings and Python bindings. It should come before JS bindings in terms of priority keeping in mind the popularity of Python as a second language for the kind of C++ users PCL attracts (academia)
- > https://jupyter.org/widgets has pythreejs which is embedding js in python. .... That's 1 step above webgl, involves using JS in Py, but is faster to get up and running.... We can tackle this based on time when we reach some stage
