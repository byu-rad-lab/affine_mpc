#ifndef AFFINE_MPC_QP_ENTRIES_HPP
#define AFFINE_MPC_QP_ENTRIES_HPP

// Internal (not installed): a set of entries of P or A that one writer
// declares at construction and then writes by position.

#include <Eigen/Core>
#include <cassert>
#include <cstddef>
#include <vector>

namespace affine_mpc::detail {

/**
 * Entries (row, col) of a QP matrix that one writer may write, declared at
 * construction in a fixed order. Once the QP is allocated, resolve() turns
 * each entry into a position in the matrix's contiguous value array, and the
 * writer then writes values by entry index in the declared order
 * (values[pos] = v), so it never depends on how the matrix is stored.
 *
 * The same entry may be declared by several writers (e.g. a diagonal entry of
 * P shared by two costs); each set then holds its own position for it.
 */
class QPEntries
{
public:
  QPEntries() = default;
  // writers register a pointer to their set (see QPLayout)
  QPEntries(const QPEntries&) = delete;
  QPEntries& operator=(const QPEntries&) = delete;

  // ---- declaration (construction only) -------------------------------------
  void reserve(std::size_t count)
  {
    rows_.reserve(count);
    cols_.reserve(count);
  }
  void declare(int row, int col)
  {
    rows_.push_back(row);
    cols_.push_back(col);
  }
  /// A dense rows x cols block from (row, col), declared column by column.
  void declareBlock(int row, int col, int rows, int cols)
  {
    for (int j{0}; j < cols; ++j)
      for (int i{0}; i < rows; ++i)
        declare(row + i, col + j);
  }

  Eigen::Index size() const noexcept
  {
    return static_cast<Eigen::Index>(rows_.size());
  }
  int row(Eigen::Index i) const { return rows_[index(i)]; }
  int col(Eigen::Index i) const { return cols_[index(i)]; }

  /// Resolve the entries to positions in M's values (dense, column-major).
  void resolve(Eigen::MatrixXd& M)
  {
    positions_.resize(rows_.size());
    for (std::size_t i{0}; i < rows_.size(); ++i) {
      assert(rows_[i] >= 0 && rows_[i] < M.rows());
      assert(cols_[i] >= 0 && cols_[i] < M.cols());
      positions_[i] = static_cast<Eigen::Index>(cols_[i]) * M.rows() + rows_[i];
    }
    values_ = M.data();
  }

  // ---- writes (after resolve()) --------------------------------------------
  void set(Eigen::Index i, double value) { at(i) = value; }
  void add(Eigen::Index i, double value) { at(i) += value; }

  void setZero() { setConstant(0.0); }
  void setOnes() { setConstant(1.0); }
  void setConstant(double value) { setConstant(0, size(), value); }
  void setZero(Eigen::Index first, Eigen::Index count)
  {
    setConstant(first, count, 0.0);
  }
  void setConstant(Eigen::Index first, Eigen::Index count, double value)
  {
    for (Eigen::Index i{first}; i < first + count; ++i)
      at(i) = value;
  }

  /// Entries first, first + 1, ... get v(0), v(1), ...
  void assign(const Eigen::Ref<const Eigen::VectorXd>& v,
              Eigen::Index first = 0)
  {
    for (Eigen::Index i{0}; i < v.size(); ++i)
      at(first + i) = v(i);
  }
  void add(const Eigen::Ref<const Eigen::VectorXd>& v, Eigen::Index first = 0)
  {
    for (Eigen::Index i{0}; i < v.size(); ++i)
      at(first + i) += v(i);
  }

  /// For a block declared with declareBlock() at entry `first`: assign M.
  void assignBlock(Eigen::Index first,
                   const Eigen::Ref<const Eigen::MatrixXd>& M)
  {
    for (Eigen::Index j{0}; j < M.cols(); ++j)
      for (Eigen::Index i{0}; i < M.rows(); ++i)
        at(first++) = M(i, j);
  }
  /// For a block declared with declareBlock() at entry `first`: assign s M.
  void assignBlock(Eigen::Index first,
                   double s,
                   const Eigen::Ref<const Eigen::MatrixXd>& M)
  {
    for (Eigen::Index j{0}; j < M.cols(); ++j)
      for (Eigen::Index i{0}; i < M.rows(); ++i)
        at(first++) = s * M(i, j);
  }
  /// For a block declared with declareBlock() at entry `first`: add s M.
  void addBlock(Eigen::Index first,
                double s,
                const Eigen::Ref<const Eigen::MatrixXd>& M)
  {
    for (Eigen::Index j{0}; j < M.cols(); ++j)
      for (Eigen::Index i{0}; i < M.rows(); ++i)
        at(first++) += s * M(i, j);
  }

private:
  static std::size_t index(Eigen::Index i)
  {
    return static_cast<std::size_t>(i);
  }
  double& at(Eigen::Index i)
  {
    assert(values_ != nullptr && i >= 0 && i < size());
    return values_[positions_[index(i)]];
  }

  std::vector<int> rows_, cols_;        // the declared entries
  std::vector<Eigen::Index> positions_; // their places in values_
  double* values_{nullptr};             // the matrix's values (resolve())
};

} // namespace affine_mpc::detail

#endif // AFFINE_MPC_QP_ENTRIES_HPP
