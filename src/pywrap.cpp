#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "driver.cpp"

namespace py = pybind11;
constexpr auto byref = py::return_value_policy::reference_internal;

PYBIND11_MODULE(lns, m) {
    m.doc() = "Large Neighborhood Search Multi-Agent Task Allocation Function";
    m.def("LNS", &pymain, "A function");
}
