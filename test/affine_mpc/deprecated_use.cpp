// Uses one deprecated name, chosen by AMPC_DEPRECATED_NAME, and is compiled
// by the deprecated_*_warns tests with deprecation warnings as errors: the
// build must fail with a deprecation message (see CMakeLists.txt).
#include "affine_mpc/affine_mpc.hpp"

namespace ampc = affine_mpc;

int main()
{
  ampc::AMPC_DEPRECATED_NAME* mpc{nullptr};
  return mpc == nullptr ? 0 : 1;
}
