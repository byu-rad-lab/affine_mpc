#include "affine_mpc_py_module.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "affine_mpc/mpc_logger.hpp"

namespace affine_mpc {
namespace py = pybind11;

void moduleAddMpcLogger(py::module& m)
{
  py::class_<MPCLogger> log(m, "MPCLogger", "MPC logger class");
  log.def(py::init<const MPCBase* const, const std::string&>(), "Constructor",
           py::arg("mpc"), py::arg("save_location")="/tmp/mpc_data");
  log.def("logPreviousSolve", &MPCLogger::logPreviousSolve,
    "Log data from previous MPC solve",
    py::arg("t0"), py::arg("ts"), py::arg("x0"), py::arg("solve_time")=-1, py::arg("write_every")=1);

  log.def("writeParamFile", &MPCLogger::writeParamFile,
    "Write current MPC params to a YAML file", py::arg("filename")="params.yaml");
}

} // namespace affine_mpc
