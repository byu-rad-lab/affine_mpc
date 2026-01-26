#ifndef EIGEN_COMPAT_HPP
#define EIGEN_COMPAT_HPP

#include <Eigen/Core>

// Eigen 5.0+ moved indexing placeholders to Eigen::placeholders namespace
// Eigen 3.x has them directly in Eigen namespace
// This compatibility layer provides a consistent 'ph' namespace for both versions

#if EIGEN_VERSION_AT_LEAST(5, 0, 0)
  // Eigen 5.x: placeholders are in Eigen::placeholders
  namespace ph = Eigen::placeholders;
#else
  // Eigen 3.x: placeholders are directly in Eigen namespace
  namespace ph {
    using Eigen::all;
    using Eigen::last;
    using Eigen::lastN;
  }
#endif

#endif // EIGEN_COMPAT_HPP
