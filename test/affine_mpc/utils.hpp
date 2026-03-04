#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <iostream>
#include <stdexcept>

#define PRINT_MAT(A) (std::cout << #A << ":\n" << A << std::endl)

template <typename Derived1, typename Derived2>
bool expectEigenNear(const Eigen::MatrixBase<Derived1>& mat1,
                     const Eigen::MatrixBase<Derived2>& mat2,
                     double delta)
{
  if (mat1.rows() != mat2.rows() || mat1.cols() != mat2.cols())
    return false;
  Derived1 diff{(mat1 - mat2).cwiseAbs()};
  return diff.maxCoeff() < delta;
}

void expectInvalidArgumentWithMessage(const std::function<void()>& fn,
                                      const std::string& expected_substring)
{
  try {
    fn();
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_TRUE(std::string(e.what()).find(expected_substring)
                != std::string::npos)
        << "Exception message was: " << e.what();
  } catch (...) {
    FAIL() << "Expected std::invalid_argument";
  }
}

void expectLogicErrorWithMessage(const std::function<void()>& fn,
                                 const std::string& expected_substring)
{
  try {
    fn();
    FAIL() << "Expected std::logic_error";
  } catch (const std::logic_error& e) {
    EXPECT_TRUE(std::string(e.what()).find(expected_substring)
                != std::string::npos)
        << "Exception message was: " << e.what();
  } catch (...) {
    FAIL() << "Expected std::logic_error";
  }
}

#endif // UTILS_HPP
