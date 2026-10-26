#include <Eigen/Core>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/sparse_mpc.hpp"
#include "qp_builder.hpp" // internal (src/): the declared entries
#include "qp_entries.hpp"

namespace ampc = affine_mpc;
using namespace Eigen;

namespace {

using Mask = Matrix<bool, Dynamic, Dynamic>;

// Exposes the assembled QP and the entries declared at construction
class PatternInspector : public ampc::SparseMPC
{
public:
  using SparseMPC::SparseMPC;
  void assemble(const Ref<const VectorXd>& x0) { (void)this->qpUpdateX0(x0); }

  // Nonzero entries of the QP outside every declared entry of P (A)
  std::vector<std::string> undeclaredNonzeros() const
  {
    std::vector<std::string> found;
    check("P", this->qp().P, this->qpBuilder().declaredP(), found);
    check("A", this->qp().A, this->qpBuilder().declaredA(), found);
    return found;
  }

private:
  static void check(const char* name,
                    const MatrixXd& M,
                    const std::vector<ampc::detail::QPEntries*>& sets,
                    std::vector<std::string>& found)
  {
    Mask declared{Mask::Constant(M.rows(), M.cols(), false)};
    for (const ampc::detail::QPEntries* entries : sets)
      for (Index i{0}; i < entries->size(); ++i)
        declared(entries->row(i), entries->col(i)) = true;
    for (Index j{0}; j < M.cols(); ++j)
      for (Index i{0}; i < M.rows(); ++i)
        if (M(i, j) != 0.0 && !declared(i, j))
          found.push_back(std::string{name} + "(" + std::to_string(i) + ", "
                          + std::to_string(j) + ")");
  }
};

// Random values with about a third of the entries zero, so entries switch
// between zero and nonzero from one draw to the next
class Draw
{
public:
  explicit Draw(unsigned seed) : gen_{seed} {}
  double value() { return std::uniform_real_distribution<>{-1.0, 1.0}(gen_); }
  bool coin(double p = 0.5) { return std::bernoulli_distribution{p}(gen_); }
  int integer(int lo, int hi)
  {
    return std::uniform_int_distribution<>{lo, hi}(gen_);
  }
  MatrixXd sparse(Index rows, Index cols)
  {
    MatrixXd M{rows, cols};
    for (Index j{0}; j < cols; ++j)
      for (Index i{0}; i < rows; ++i)
        M(i, j) = coin(1.0 / 3.0) ? 0.0 : value();
    return M;
  }
  // non-negative weights, some zero
  VectorXd weights(Index size) { return sparse(size, 1).cwiseAbs(); }

private:
  std::mt19937 gen_;
};

struct Case
{
  int n, m;
  ampc::Parameterization param;
  ampc::Options opts;
  int num_bounded;
};

Case drawCase(Draw& draw)
{
  using P = ampc::Parameterization;
  const std::vector<P> bases{P::moveBlocking(8, 8),    P::moveBlocking(12, 4),
                             P::linearInterp(10, 4),   P::bspline(10, 2, 5),
                             P::bspline(15, 3, 6),     P::moveBlocking(1, 1),
                             P::bspline(20, 2, 7, 1.3)};
  Case c{draw.integer(1, 4),
         draw.integer(1, 3),
         bases[static_cast<std::size_t>(
             draw.integer(0, static_cast<int>(bases.size()) - 1))],
         {},
         0};
  ampc::Options& o{c.opts};
  o.use_input_cost = draw.coin();
  o.use_input_rate_cost = draw.coin();
  o.saturate_input_trajectory = draw.coin();
  o.slew_initial_input = draw.coin();
  o.slew_control_points = draw.coin() && c.param.num_control_points > 1;
  o.saturate_states = draw.coin();
  o.soft_state_bounds = o.saturate_states && draw.coin();
  if (o.saturate_states && draw.coin())
    for (int i{0}; i < c.n; ++i)
      if (draw.coin())
        o.bounded_states.push_back(i);
  o.num_general_constraints = draw.coin() ? draw.integer(1, 3) : 0;
  o.num_terminal_constraints = draw.coin() ? draw.integer(1, 3) : 0;
  c.num_bounded = o.bounded_states.empty()
                      ? c.n
                      : static_cast<int>(o.bounded_states.size());
  return c;
}

// Sets every value the options use, drawn anew
void setAll(ampc::MPCBase& mpc, const Case& c, Draw& draw)
{
  const int n{c.n}, m{c.m}, T{c.param.horizon_steps};
  const int nc{m * c.param.num_control_points};
  const ampc::Options& o{c.opts};
  mpc.setModelDiscrete(MatrixXd::Identity(n, n) + 0.1 * draw.sparse(n, n),
                       draw.sparse(n, m), draw.sparse(n, 1));
  mpc.setStateWeights(draw.weights(n), draw.weights(n));
  if (o.use_input_cost) {
    mpc.setInputWeights(draw.weights(m));
    mpc.setReferenceInputControlPoints(draw.sparse(nc, 1));
  }
  if (o.use_input_rate_cost)
    mpc.setInputRateWeights(draw.weights(m));
  mpc.setReferenceStateTrajectory(draw.sparse(n * T, 1));
  mpc.setInputLimits(VectorXd::Constant(m, -2.0), VectorXd::Constant(m, 2.0));
  const int k{c.num_bounded};
  if (o.saturate_states)
    mpc.setStateLimits(VectorXd::Constant(k, -5.0), VectorXd::Constant(k, 5.0));
  if (o.soft_state_bounds)
    mpc.setStateBoundPenalties(draw.weights(k), draw.weights(k));
  if (o.num_general_constraints > 0) {
    const int p{o.num_general_constraints};
    mpc.setGeneralConstraints(draw.sparse(p, n), draw.sparse(p, m),
                              VectorXd::Constant(p, -3.0),
                              VectorXd::Constant(p, 3.0));
  }
  if (o.num_terminal_constraints > 0) {
    const int p{o.num_terminal_constraints};
    mpc.setTerminalConstraints(draw.sparse(p, n), VectorXd::Constant(p, -3.0),
                               VectorXd::Constant(p, 3.0));
  }
  if (o.slew_control_points)
    mpc.setSlewRate(VectorXd::Constant(m, 0.7));
  if (o.slew_initial_input)
    mpc.setSlewRateInitial(VectorXd::Constant(m, 0.3));
  if (o.slew_initial_input || o.use_input_rate_cost)
    mpc.setPreviousInput(draw.sparse(m, 1));
}

std::string describe(const Case& c)
{
  const ampc::Options& o{c.opts};
  std::string s{"n" + std::to_string(c.n) + " m" + std::to_string(c.m) + " T"
                + std::to_string(c.param.horizon_steps) + " nc"
                + std::to_string(c.param.num_control_points) + " deg"
                + std::to_string(c.param.degree) + " opts "};
  for (const bool b :
       {o.use_input_cost, o.use_input_rate_cost, o.saturate_input_trajectory,
        o.slew_initial_input, o.slew_control_points, o.saturate_states,
        o.soft_state_bounds})
    s += b ? '1' : '0';
  s += " bounded " + std::to_string(o.bounded_states.size()) + " p"
       + std::to_string(o.num_general_constraints) + " pf"
       + std::to_string(o.num_terminal_constraints);
  return s;
}

} // namespace

// The sparse QP's nonzeros stay within the entries declared at construction,
// over many option sets and value changes (the pattern the sparse storage is
// built from)
TEST(QPPatternTester, givenSparseMPC_NonzerosAreWithinDeclaredEntries)
{
  for (unsigned seed{0}; seed < 200; ++seed) {
    Draw draw{seed};
    const Case c{drawCase(draw)};
    SCOPED_TRACE("seed " + std::to_string(seed) + ": " + describe(c));
    PatternInspector mpc{c.n, c.m, c.param, c.opts};
    setAll(mpc, c, draw);
    const VectorXd x0{draw.sparse(c.n, 1)};
    mpc.assemble(x0);
    EXPECT_EQ(mpc.undeclaredNonzeros(), std::vector<std::string>{})
        << "assembled";
    (void)mpc.initializeSolver();
    EXPECT_EQ(mpc.undeclaredNonzeros(), std::vector<std::string>{})
        << "placeholders";
    for (int change{0}; change < 4; ++change) {
      setAll(mpc, c, draw);
      mpc.assemble(draw.sparse(c.n, 1));
      EXPECT_EQ(mpc.undeclaredNonzeros(), std::vector<std::string>{})
          << "change " << change;
    }
    (void)mpc.solve(x0); // whatever the status, the QP was assembled
    EXPECT_EQ(mpc.undeclaredNonzeros(), std::vector<std::string>{}) << "solve";
  }
}
