#ifndef OSQP_BASE_HPP
#define OSQP_BASE_HPP

#include <Eigen/Core>
#include <osqp.h>

namespace affine_mpc {

class OSQPSolver
{
public:
  typedef Eigen::Matrix<c_float,Eigen::Dynamic,Eigen::Dynamic> MatrixXF;
  typedef Eigen::Matrix<c_float,Eigen::Dynamic,1> VectorXF;
  typedef Eigen::Matrix<c_int,Eigen::Dynamic,1> VectorXI;

  OSQPSolver(const int num_variables, const int num_constraints);
  virtual ~OSQPSolver();
  const c_float* getSolutionPtr() const;
  c_float getSolveTime() const;
  bool solve(Eigen::Ref<VectorXF> solution);
  bool solve();
  bool initialize(const Eigen::Ref<const MatrixXF>& P,
                  const Eigen::Ref<const MatrixXF>& A,
                  Eigen::Ref<VectorXF> q, Eigen::Ref<VectorXF> l, Eigen::Ref<VectorXF> u,
                  const OSQPSettings* settings = nullptr);
  bool updateCostMatrix(const Eigen::Ref<const MatrixXF>& P);
  bool updateConstraintMatrix(const Eigen::Ref<const MatrixXF>& A);
  bool updateCostVector(Eigen::Ref<VectorXF> q);
  bool updateBounds(Eigen::Ref<VectorXF> l, Eigen::Ref<VectorXF> u);

private:
  int countUpperTriangle(const Eigen::Ref<const MatrixXF>& mat);
  void initializeCostMatrix(const Eigen::Ref<const MatrixXF>& P);
  void initializeConstraintMatrix(const Eigen::Ref<const MatrixXF>& A);
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

} // namespace affine_mpc

#endif // OSQP_BASE_HPP
