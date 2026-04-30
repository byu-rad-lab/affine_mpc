#ifndef AFFINE_MPC_NPY_WRITER_HPP
#define AFFINE_MPC_NPY_WRITER_HPP

#include <filesystem>
#include <vector>

namespace affine_mpc {

/**
 * @brief Internal write-only NPY utility used by MPCLogger fallback output.
 */
class NpyWriter
{
public:
  static void writeDoubleArrayFromFile(const std::filesystem::path& npy_path,
                                       const std::filesystem::path& raw_path,
                                       const std::vector<size_t>& shape);
};

} // namespace affine_mpc

#endif // AFFINE_MPC_NPY_WRITER_HPP
