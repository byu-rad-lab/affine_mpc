#include "affine_mpc_py_module.hpp"
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(affine_mpc_py, m)
{
  m.doc() = "Affine MPC module";

  moduleAddOsqpSettings(m);
  moduleAddImplicitMPC(m);
}
