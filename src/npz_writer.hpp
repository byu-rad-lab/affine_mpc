#ifndef AFFINE_MPC_NPZ_WRITER_HPP
#define AFFINE_MPC_NPZ_WRITER_HPP

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace affine_mpc {

class NpzWriter
{
public:
  explicit NpzWriter(const std::filesystem::path& path);
  ~NpzWriter();

  NpzWriter(const NpzWriter&) = delete;
  NpzWriter& operator=(const NpzWriter&) = delete;

  void addArray(const std::string& name,
                const float* data,
                const std::vector<size_t>& shape);
  void addArray(const std::string& name,
                const double* data,
                const std::vector<size_t>& shape);
  void addArray(const std::string& name,
                const std::int32_t* data,
                const std::vector<size_t>& shape);
  void addArray(const std::string& name,
                const std::int64_t* data,
                const std::vector<size_t>& shape);

  void addScalar(const std::string& name, float value);
  void addScalar(const std::string& name, double value);
  void addScalar(const std::string& name, std::int32_t value);
  void addScalar(const std::string& name, std::int64_t value);

  void finalize();

private:
  struct Impl;
  Impl* impl_;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_NPZ_WRITER_HPP
