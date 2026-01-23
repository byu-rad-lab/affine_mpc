#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Core>
#include <iostream>

#define PRINT_MAT(A) (std::cout << #A << ":\n" << A << std::endl)

template <typename Derived1, typename Derived2>
bool expectEigenNear(const Eigen::MatrixBase<Derived1>& mat1,
                     const Eigen::MatrixBase<Derived2>& mat2,
                     double delta)
{
  if (mat1.rows() != mat2.rows() && mat1.cols() != mat2.cols())
    return false;
  Derived1 diff{(mat1 - mat2).cwiseAbs()};
  return diff.maxCoeff() < delta;
}

#endif // UTILS_HPP
