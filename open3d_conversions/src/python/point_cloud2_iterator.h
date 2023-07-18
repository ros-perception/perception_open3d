#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace py = pybind11;

void bind_point_cloud2_modifier(py::module &m) {
    py::class_<sensor_msgs::PointCloud2Modifier>(m, "PointCloud2Modifier")
        .def(py::init<sensor_msgs::PointCloud2 &>())
        .def("size", &sensor_msgs::PointCloud2Modifier::size)
        .def("reserve", &sensor_msgs::PointCloud2Modifier::reserve)
        .def("resize", &sensor_msgs::PointCloud2Modifier::resize)
        .def("clear", &sensor_msgs::PointCloud2Modifier::clear)
        .def("setPointCloud2Fields",
            [](sensor_msgs::PointCloud2Modifier &self, int n_fields, py::args args) {
                std::vector<std::string> field_args;
                for (const auto &arg : args) {
                    field_args.push_back(arg.cast<std::string>());
                }
                self.setPointCloud2Fields(n_fields, field_args.data());
            })
        .def("setPointCloud2FieldsByString",
            [](sensor_msgs::PointCloud2Modifier &self, int n_fields, py::args args) {
                std::vector<std::string> field_args;
                for (const auto &arg : args) {
                    field_args.push_back(arg.cast<std::string>());
                }
                self.setPointCloud2FieldsByString(n_fields, field_args.data());
            });
}

template <typename T>
void bind_point_cloud2_iterator(py::module &m, const std::string &name) {
    using PointCloud2IteratorType = sensor_msgs::PointCloud2Iterator<T>;
    py::class_<PointCloud2IteratorType>(m, name.c_str())
        .def(py::init<sensor_msgs::PointCloud2 &, const std::string &>())
        .def("__iter__", [](const PointCloud2IteratorType &iter) -> const PointCloud2IteratorType & { return iter; })
        .def("__next__", [](PointCloud2IteratorType &iter) -> T & { return *++iter; })
        .def("size", &PointCloud2IteratorType::size)
        .def("reserve", &PointCloud2IteratorType::reserve)
        .def("resize", &PointCloud2IteratorType::resize)
        .def("__getitem__", [](const PointCloud2IteratorType &iter, size_t i) -> T & { return iter[i]; });
}
