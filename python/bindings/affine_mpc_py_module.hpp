#ifndef AFFINE_MPC_MODULE_HPP
#define AFFINE_MPC_MODULE_HPP

#include <pybind11/pybind11.h>

namespace affine_mpc_py {

void moduleAddOptions(pybind11::module& m);
void moduleAddParameterization(pybind11::module& m);
void moduleAddOsqpSettings(pybind11::module& m);
void moduleAddSolveStatus(pybind11::module& m);
void moduleAddMPCBase(pybind11::module& m);
void moduleAddCondensedMPC(pybind11::module& m);
void moduleAddSparseMPC(pybind11::module& m);
void moduleAddMpcLogger(pybind11::module& m);

} // namespace affine_mpc_py

#endif // AFFINE_MPC_MODULE_HPP
