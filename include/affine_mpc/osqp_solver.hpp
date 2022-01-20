#ifndef OSQP_BASE_HPP
#define OSQP_BASE_HPP

#include <Eigen/Core>
#include <osqp.h>

using namespace Eigen;

class OSQPSolver
{
public:
  typedef Matrix<c_float,Dynamic,Dynamic> MatrixXF;
  typedef Matrix<c_float,Dynamic,1> VectorXF;
  typedef Matrix<c_int,Dynamic,1> VectorXI;

  OSQPSolver(const int num_variables, const int num_constraints);
  virtual ~OSQPSolver();
  const c_float* getSolutionPtr() const;
  bool solve(Ref<VectorXF> solution);
  bool solve();
  bool initialize(const Ref<const MatrixXF>& P, const Ref<const MatrixXF>& A,
                  Ref<VectorXF> q, Ref<VectorXF> l, Ref<VectorXF> u,
                  const OSQPSettings* settings = nullptr);
  bool updateCostMatrix(const Ref<const MatrixXF>& P);
  bool updateConstraintMatrix(const Ref<const MatrixXF>& A);
  bool updateCostVector(Ref<VectorXF> q);
  bool updateBounds(Ref<VectorXF> l, Ref<VectorXF> u);

private:
  int countUpperTriangle(const Ref<const MatrixXF>& mat);
  void initializeCostMatrix(const Ref<const MatrixXF>& P);
  void initializeConstraintMatrix(const Ref<const MatrixXF>& A);
  void setCustomSettings(const OSQPSettings* settings);

  OSQPWorkspace *work_;
  OSQPSettings *settings_;
  OSQPData *data_;
  bool workspace_initialized_;
  bool P_is_set_;
  bool A_is_set_;

protected: // Not private for unit tests
  c_int n_;
  c_int m_;
  c_int P_nnz_;
  c_int A_nnz_;
  VectorXF P_x_;
  VectorXF A_x_;
  VectorXI P_i_;
  VectorXI A_i_;
  VectorXI P_p_;
  VectorXI A_p_;
};

#endif // OSQP_BASE_HPP
