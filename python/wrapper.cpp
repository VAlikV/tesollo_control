#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "../dg_control/control.hpp"

using namespace handcontrol;

namespace py = pybind11;

PYBIND11_MODULE(dg5f_python, m)
{
    py::class_<DGControl>(m, "DGApi")
        .def_static(
            "instance",
            &DGControl::getInstance,
            py::return_value_policy::reference
        )
        .def("start", &DGControl::start)
        .def("stop", &DGControl::stop)
        .def("set_target_position",
            [](handcontrol::DGControl& self,
            py::array_t<float, py::array::c_style | py::array::forcecast> arr)
            {
                auto buf = arr.request();

                if (buf.ndim != 1)
                    throw std::runtime_error("Expected 1D array");

                if (buf.size != MAX_JOINT_COUNT)
                    throw std::runtime_error(
                        "Expected array of size " + std::to_string(MAX_JOINT_COUNT));

                const float* ptr = static_cast<const float*>(buf.ptr);

                return self.setTragetPosition(ptr);
            }
        )
        .def("get_current_position",
            [](handcontrol::DGControl& self)
            {
                py::array_t<float> arr({MAX_JOINT_COUNT});

                auto buf = arr.request();
                float* ptr = static_cast<float*>(buf.ptr);

                bool ok = self.getCurrentPosition(ptr);

                return py::make_tuple(ok, arr);
            }
        )                                                                                                                                                                                                                                                                                                                
        .def("get_current_current",
            [](handcontrol::DGControl& self)
            {
                py::array_t<float> arr({MAX_JOINT_COUNT});

                auto buf = arr.request();
                float* ptr = static_cast<float*>(buf.ptr);

                bool ok = self.getCurrentCurrent(ptr);

                return py::make_tuple(ok, arr);
            }
        ) 
        .def("get_current_velocity",
            [](handcontrol::DGControl& self)
            {
                py::array_t<float> arr({MAX_JOINT_COUNT});

                auto buf = arr.request();
                float* ptr = static_cast<float*>(buf.ptr);

                bool ok = self.getCurrentVelocity(ptr);

                return py::make_tuple(ok, arr);
            }
        ) 
        .def("get_current_temp",
            [](handcontrol::DGControl& self)
            {
                py::array_t<float> arr({MAX_JOINT_COUNT});

                auto buf = arr.request();
                float* ptr = static_cast<float*>(buf.ptr);

                bool ok = self.getCurrentTemperature(ptr);

                return py::make_tuple(ok, arr);
            }
        );

}