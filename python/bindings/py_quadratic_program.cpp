#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

#include "affine_mpc/quadratic_program.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

void moduleAddQuadraticProgram(py::module& m)
{
  py::class_<ampc::QuadraticProgram>(m, "QuadraticProgram",
                                     R"doc(
The QP an MPC solves,
minimize 1/2 z^T P z + q^T z subject to l <= A z <= u,
for analysis or for handing to another solver. Obtain with MPC.getQP() or
MPC.assembleQP().

P and A are scipy.sparse.csc_matrix objects holding the nonzero entries of the
QP, in the same form for every formulation; P is full symmetric (both
triangles). Reading P or A needs SciPy (pip install "affine-mpc[sparse]").
The cost omits the constant terms of the MPC cost, so its value at the solution
is not the full MPC cost. Equality rows have l = u, and unbounded sides are
+-inf. The order of the variables z and of the rows of A is given by
MPC.getQPVariableBlocks() and MPC.getQPConstraintBlocks().
)doc")
      .def(py::init<>(), "Empty QP, to pass to MPC.assembleQP().")
      .def_readwrite("P", &ampc::QuadraticProgram::P,
                     "Cost matrix (full symmetric, scipy.sparse.csc_matrix).")
      .def_readwrite("A", &ampc::QuadraticProgram::A,
                     "Constraint matrix (scipy.sparse.csc_matrix).")
      .def_readwrite("q", &ampc::QuadraticProgram::q, "Cost vector.")
      .def_readwrite("l", &ampc::QuadraticProgram::l,
                     "Constraint lower bounds.")
      .def_readwrite("u", &ampc::QuadraticProgram::u,
                     "Constraint upper bounds.");

  py::class_<ampc::QPBlock>(m, "QPBlock",
                            R"doc(
A named block of QP variables or constraint rows: indices offset, ...,
offset + size - 1. Returned by MPC.getQPVariableBlocks() and
MPC.getQPConstraintBlocks().
)doc")
      .def_readonly("name", &ampc::QPBlock::name,
                    "Block name, e.g. 'control_points' or 'input_bounds'.")
      .def_readonly("offset", &ampc::QPBlock::offset,
                    "Index of the first variable or row.")
      .def_readonly("size", &ampc::QPBlock::size,
                    "Number of variables or rows.")
      .def("__repr__", [](const ampc::QPBlock& block) {
        std::ostringstream os;
        os << "QPBlock(name='" << block.name << "', offset=" << block.offset
           << ", size=" << block.size << ")";
        return os.str();
      });
}

} // namespace affine_mpc_py
