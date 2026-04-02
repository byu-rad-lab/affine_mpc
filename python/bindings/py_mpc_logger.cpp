#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl/filesystem.h>

#include "affine_mpc/mpc_logger.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;


void moduleAddMpcLogger(py::module& m)
{
  py::class_<ampc::MPCLogger> log(m, "MPCLogger",
                                  R"doc(
High-performance binary logger for MPC data and metadata.

Uses a "write-raw, pack-later" strategy to support high logging frequencies.
Per-step data is temporarily stored in binary files and then packed into a
single .npz file during finalization. Metadata is stored in a .yaml file.
Everything in the YAML file is also contained in the NPZ file, but the YAML
provides a quick and easy way to see parameters from a simulation.

The logger is designed to be used within a simulation or control loop. It
provides a convenience method to automatically extract and stride
trajectories from an MPC object.
                                  )doc");

  log.def(py::init<const ampc::MPCBase* const, const std::filesystem::path&,
                   double, int, bool, const std::string&>(),
          R"doc(
Construct an MPCLogger for a given MPC instance; the logger is linked to this
single MPC instance.

IMPORTANT: The logger does not own the MPC object; the MPC instance must outlive
the logger.

The logger automatically captures a snapshot of the MPC object's current
parameters (dimensions, weights, limits, etc.) at construction. If these
parameters change later, captureMPCSnapshot() should be called manually.

The destructor calls finalize() if it has not been manually called.

Args:
    mpc: The MPC object from which to log parameters.
    save_dir: Directory to save log files. Created if it doesn't exist.
    ts: Model propagation time step used to align predicted trajectories in time.
    prediction_stride: Factor to downsample predicted trajectories.
        - 0: Log only the current step (minimal mode).
        - 1: Log every step of the horizon.
        - K: Log every K-th step of the horizon.
        Note: The terminal state (T) is always included if prediction_stride > 0.
    log_control_points: If true, logs control points of the parameterized input
        trajectory instead of the evaluated dense input trajectory.
    save_name: Base name for the .npz output file (default: "log").
          )doc",
          py::arg("mpc"), py::arg("save_dir"), py::arg("ts"),
          py::arg("prediction_stride") = 1,
          py::arg("log_control_points") = false, py::arg("save_name") = "log",
          py::keep_alive<1, 2>());

  log.def("logStep", &ampc::MPCLogger::logStep,
          R"doc(
Log a single step of data, automatically fetching trajectories and references
from the provided MPC object with applied striding logic.

Args:
    t: Current simulation time (time of solve).
    x0: Current state at time t (same as provided to solve() since MPCBase does
        not store this).
    solve_time: Optional user-calculated solve time (likely to include setup
        time). The solve time reported by OSQP is also logged separately.
          )doc",
          py::arg("t"), py::arg("x0"), py::arg("solve_time") = -1.0);

  log.def(
      "addMetadata",
      [](ampc::MPCLogger& self, const std::string& key, py::object value,
         int precision) {
        if (py::isinstance<py::bool_>(value)
            || py::isinstance<py::int_>(value)) {
          self.addMetadata(key, value.cast<int>(), precision);
        } else if (py::isinstance<py::float_>(value)) {
          self.addMetadata(key, value.cast<double>(), precision);
        } else if (py::isinstance<py::str>(value)) {
          self.addMetadata(key, value.cast<std::string>(), precision);
        } else if (!py::isinstance<py::str>(value)
                   && !py::isinstance<py::bytes>(value)
                   && py::isinstance<py::sequence>(value)) {
          py::array_t<double, py::array::c_style | py::array::forcecast> arr{
              value};
          py::array squeezed = arr.squeeze();
          if (squeezed.ndim() != 1)
            throw py::type_error("metadata arrays/sequences must be 1D");
          self.addMetadata(key, py::cast<Eigen::VectorXd>(squeezed), precision);
        } else {
          throw py::type_error("value must be an int, float, string, or 1D "
                               "numeric sequence/array");
        }
      },
      R"doc(
Add or overwrite custom metadata to be saved in both NPZ and YAML.

User-added metadata is preserved in the order it was added and appears after the
automatic MPC snapshot in the output files.

Args:
    key: Unique identifier for the metadata entry.
    value: The value to store (int, float, string, 1D NDArray, list[float]).
    precision: Optional decimal precision for floating point output in YAML.
        -1 uses default precision.
      )doc",
      py::arg("key"), py::arg("value"), py::arg("precision") = -1);

  log.def("captureMPCSnapshot", &ampc::MPCLogger::captureMPCSnapshot,
          R"doc(
Manually capture a snapshot of an MPC object's current parameters.

This is called automatically in the constructor, but can be re-called if
weights or limits are updated during the simulation.
          )doc");

  log.def("finalize", &ampc::MPCLogger::finalize,
          R"doc(
Pack all temporary binary data into the final .npz file and write the parameter
YAML file.

This operation involves file I/O and should be called after the simulation
loop ends. Temporary files are deleted upon successful completion.
          )doc");

  log.def("writeParamFile", &ampc::MPCLogger::writeParamFile,
          R"doc(
Write the internal metadata map to a YAML file.

Args:
    filename: Output filename (should end with .yaml or .yml).
          )doc",
          py::arg("filename") = "params.yaml");
}

} // namespace affine_mpc_py
