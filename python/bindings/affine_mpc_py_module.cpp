#include "affine_mpc_py_module.hpp"

#include <pybind11/pybind11.h>

namespace affine_mpc_py {

// module name must match CMake target "_bindings"
PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Affine MPC module";

  moduleAddOptions(m);
  moduleAddParameterization(m);
  moduleAddDiscretizer(m);
  moduleAddOsqpSettings(m);
  moduleAddDaqpSettings(m);
  moduleAddSolveStatus(m);
  moduleAddQuadraticProgram(m);
  moduleAddMPC(m);
  moduleAddMpcLogger(m);
}

} // namespace affine_mpc_py
