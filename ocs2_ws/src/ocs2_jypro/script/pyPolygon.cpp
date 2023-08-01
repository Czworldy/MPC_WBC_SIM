#include <ocs2_python_interface/PybindMacros.h>

#include "ocs2_jypro/constraint/Polygon.hpp"
namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::vector<ocs2::Polygon::Position>)
PYBIND11_MODULE(Polygon, m) {
    VECTOR_TYPE_BINDING(std::vector<ocs2::Polygon::Position>, "Vertices")   
    py::class_<ocs2::Polygon>(m, "Polygon")
            .def(py::init<const std::vector<ocs2::Polygon::Position>&>(), py::arg("vertices"))
            .def_readwrite("constraintA", &ocs2::Polygon::constraintA_)
            .def_readwrite("constraintb", &ocs2::Polygon::constraintb_)
            .def("addVertex", &ocs2::Polygon::addVertex, py::arg("vertex"))
            .def("getVertex", &ocs2::Polygon::getVertex, py::arg("index"))
            .def("nVertices", &ocs2::Polygon::nVertices)
            .def("getArea", &ocs2::Polygon::getArea)
            .def("convert2InequalityConstraints", &ocs2::Polygon::convert2InequalityConstraints);

}