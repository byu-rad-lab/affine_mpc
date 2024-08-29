#include "affine_mpc_py_module.hpp"

#include <pybind11/pybind11.h>

namespace affine_mpc {
namespace py = pybind11;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Affine MPC module";

  moduleAddOsqpSettings(m);
  moduleAddMPCBase(m);
  moduleAddImplicitMPC(m);
  moduleAddMpcLogger(m);
}

} // namespace affine_mpc
