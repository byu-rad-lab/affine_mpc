#ifndef OSQP_BASE_HPP
#define OSQP_BASE_HPP

#include <Eigen/Core>
#include <osqp.h>

namespace affine_mpc {

class OSQPSolver
{
public:
  typedef Eigen::Matrix<OSQPFloat, Eigen::Dynamic, Eigen::Dynamic> MatrixXF;
  typedef Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1> VectorXF;
  typedef Eigen::Matrix<OSQPInt, Eigen::Dynamic, 1> VectorXI;

  OSQPSolver(const int num_variables, const int num_constraints);
  virtual ~OSQPSolver() = default;
  
  // Delete copy operations (C API handles make this unsafe)
  OSQPSolver(const OSQPSolver&) = delete;
  OSQPSolver& operator=(const OSQPSolver&) = delete;
  
  // Default move operations (smart pointers handle this correctly)
  OSQPSolver(OSQPSolver&&) = default;
  OSQPSolver& operator=(OSQPSolver&&) = default;

  static OSQPSettings getDefaultSettings() noexcept;
  static OSQPSettings getRecommendedSettings(const bool polish_near_boundaries) noexcept;
  
  const OSQPFloat* getSolutionPtr() const noexcept;
  OSQPFloat getSolveTime() const noexcept;
  bool solve(Eigen::Ref<VectorXF> solution);
  bool solve();
  bool initialize(const Eigen::Ref<const MatrixXF>& P,
                  const Eigen::Ref<const MatrixXF>& A,
                  Eigen::Ref<VectorXF> q,
                  Eigen::Ref<VectorXF> l,
                  Eigen::Ref<VectorXF> u,
                  const OSQPSettings& settings);
  bool updateCostMatrix(const Eigen::Ref<const MatrixXF>& P);
  bool updateConstraintMatrix(const Eigen::Ref<const MatrixXF>& A);
  bool updateCostVector(Eigen::Ref<VectorXF> q);
  bool updateBounds(Eigen::Ref<VectorXF> l, Eigen::Ref<VectorXF> u);

private:
  int countUpperTriangle(const Eigen::Ref<const MatrixXF>& mat);
  void initializeCostMatrix(const Eigen::Ref<const MatrixXF>& P);
  void initializeConstraintMatrix(const Eigen::Ref<const MatrixXF>& A);
  void setCustomSettings(const OSQPSettings& settings);

  // Choosing to modernize even though OSQP documentation uses raw pointers
  std::unique_ptr<::OSQPSolver, decltype(&osqp_cleanup)> solver_;
  std::unique_ptr<OSQPCscMatrix, decltype(&OSQPCscMatrix_free)> P_;
  std::unique_ptr<OSQPCscMatrix, decltype(&OSQPCscMatrix_free)> A_;
  bool workspace_initialized_;
  bool P_is_set_;
  bool A_is_set_;

protected: // Not private for unit tests
  const OSQPInt n_;
  const OSQPInt m_;
  OSQPInt P_nnz_;
  OSQPInt A_nnz_;
  VectorXF P_x_;
  VectorXF A_x_;
  VectorXI P_i_;
  VectorXI A_i_;
  VectorXI P_p_;
  VectorXI A_p_;
};

} // namespace affine_mpc

#endif // OSQP_BASE_HPP
