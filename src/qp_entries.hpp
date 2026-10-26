#ifndef AFFINE_MPC_QP_ENTRIES_HPP
#define AFFINE_MPC_QP_ENTRIES_HPP

// Internal (not installed): a set of entries of P or A that one writer
// declares at construction and then writes by position.

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <vector>

#include "affine_mpc/detail/qp.hpp"

namespace affine_mpc::detail {

/**
 * Entries (row, col) of a QP matrix that one writer may write, declared at
 * construction in a fixed order. Once the QP is allocated, resolve() turns
 * each entry into its place in the matrix's contiguous value array (dense:
 * col * rows + row; CSC: the entry's index in the pattern), and the writer
 * then writes values by entry index in the declared order, so it never
 * depends on how the matrix is stored.
 *
 * The same entry may be declared by several writers (e.g. a diagonal entry of
 * P shared by two costs); each set then holds its own place for it.
 *
 * A CSC cost matrix stores only its upper triangle (as OSQP takes it), so
 * entries below the diagonal (declared by symmetric control point costs for
 * the dense storage) resolve to a slot of the set that nothing reads.
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
    blocks_.push_back({size(), rows, cols});
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
  /// Whether blocks were declared with declareBlock(), and (after resolve())
  /// whether block writes copy whole columns (see findContiguousColumns()).
  bool hasBlocks() const noexcept { return !blocks_.empty(); }
  bool blockColumnsContiguous() const noexcept { return contiguous_columns_; }

  /// Resolve the entries to places in M's values (dense, column-major).
  void resolve(Eigen::MatrixXd& M)
  {
    targets_.resize(rows_.size());
    for (std::size_t i{0}; i < rows_.size(); ++i) {
      assert(rows_[i] >= 0 && rows_[i] < M.rows());
      assert(cols_[i] >= 0 && cols_[i] < M.cols());
      targets_[i] =
          M.data() + static_cast<Eigen::Index>(cols_[i]) * M.rows() + rows_[i];
    }
    findContiguousColumns();
  }
  /// Resolve the entries to places in M's values (CSC, compressed), which
  /// must hold every entry; with `upper`, M is the upper triangle of a
  /// symmetric matrix and entries below the diagonal are dropped.
  void resolve(QP::SparseMatrix& M, bool upper)
  {
    assert(M.isCompressed());
    targets_.resize(rows_.size());
    const auto* outer{M.outerIndexPtr()};
    const auto* inner{M.innerIndexPtr()};
    for (std::size_t i{0}; i < rows_.size(); ++i) {
      const int row{rows_[i]}, col{cols_[i]};
      assert(row >= 0 && row < M.rows() && col >= 0 && col < M.cols());
      if (upper && row > col) {
        targets_[i] = &dropped_;
        continue;
      }
      const auto* first{inner + outer[col]};
      const auto* last{inner + outer[col + 1]};
      const auto* found{std::lower_bound(first, last, row)};
      assert(found != last && *found == row);
      targets_[i] = M.valuePtr() + (found - inner);
    }
    findContiguousColumns();
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

  // The block writes below take `first` at the start of a declared block,
  // and copy whole columns when every block's columns are contiguous.

  /// For a block declared with declareBlock() at entry `first`: assign M.
  void assignBlock(Eigen::Index first,
                   const Eigen::Ref<const Eigen::MatrixXd>& M)
  {
    if (contiguous_columns_) {
      for (Eigen::Index j{0}; j < M.cols(); ++j, first += M.rows()) {
        double* dst{column(first, M.rows())};
        for (Eigen::Index i{0}; i < M.rows(); ++i)
          dst[i] = M(i, j);
      }
      return;
    }
    for (Eigen::Index j{0}; j < M.cols(); ++j)
      for (Eigen::Index i{0}; i < M.rows(); ++i)
        at(first++) = M(i, j);
  }
  /// For a block declared with declareBlock() at entry `first`: assign s M.
  void assignBlock(Eigen::Index first,
                   double s,
                   const Eigen::Ref<const Eigen::MatrixXd>& M)
  {
    if (contiguous_columns_) {
      for (Eigen::Index j{0}; j < M.cols(); ++j, first += M.rows()) {
        double* dst{column(first, M.rows())};
        for (Eigen::Index i{0}; i < M.rows(); ++i)
          dst[i] = s * M(i, j);
      }
      return;
    }
    for (Eigen::Index j{0}; j < M.cols(); ++j)
      for (Eigen::Index i{0}; i < M.rows(); ++i)
        at(first++) = s * M(i, j);
  }
  /// For a block declared with declareBlock() at entry `first`: add s M.
  void addBlock(Eigen::Index first,
                double s,
                const Eigen::Ref<const Eigen::MatrixXd>& M)
  {
    if (contiguous_columns_) {
      for (Eigen::Index j{0}; j < M.cols(); ++j, first += M.rows()) {
        double* dst{column(first, M.rows())};
        for (Eigen::Index i{0}; i < M.rows(); ++i)
          dst[i] += s * M(i, j);
      }
      return;
    }
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
    assert(i >= 0 && i < static_cast<Eigen::Index>(targets_.size()));
    return *targets_[index(i)];
  }

  // The values of a block's column from entry `first`, as one contiguous
  // array (contiguous columns only). A plain loop over it vectorizes and,
  // unlike an Eigen map, costs nothing extra for short columns.
  double* column(Eigen::Index first, Eigen::Index rows)
  {
    assert(first >= 0 && first + rows <= size());
    (void)rows;
    return targets_[index(first)];
  }
  // Whether each column of every declared block is a contiguous run in the
  // matrix's values (always for dense storage; for CSC when no other stored
  // entry lies between the block's rows), so block writes copy columns
  void findContiguousColumns()
  {
    contiguous_columns_ = !blocks_.empty();
    for (const Block& b : blocks_)
      for (Eigen::Index j{0}; j < b.cols; ++j) {
        const Eigen::Index first{b.first + j * b.rows};
        for (Eigen::Index i{1}; i < b.rows; ++i)
          if (targets_[index(first + i)] != targets_[index(first)] + i)
            contiguous_columns_ = false;
      }
  }

  struct Block // a block declared with declareBlock()
  {
    Eigen::Index first; // its first entry
    int rows, cols;
  };

  std::vector<int> rows_, cols_; // the declared entries
  std::vector<Block> blocks_;
  // their places in the matrix's values (resolve()), or dropped_
  std::vector<double*> targets_;
  double dropped_{0.0}; // written for dropped entries, never read
  bool contiguous_columns_{false};
};

} // namespace affine_mpc::detail

#endif // AFFINE_MPC_QP_ENTRIES_HPP
