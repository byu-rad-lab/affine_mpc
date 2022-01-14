#ifndef AFFINE_MPC_MODULE_HPP
#define AFFINE_MPC_MODULE_HPP

#include <pybind11/pybind11.h>

namespace py = pybind11;

void moduleAddOsqpSettings(py::module& m);
void moduleAddImplicitMPC(py::module& m);

#endif // AFFINE_MPC_MODULE_HPP
