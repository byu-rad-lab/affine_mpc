#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <vector>

#include "npz_writer.hpp"

namespace ampc = affine_mpc;
namespace fs = std::filesystem;

TEST(NpzWriterTest, WritesArchiveWithoutThrowing)
{
  const fs::path test_dir = fs::temp_directory_path() / "npz_writer_test";
  fs::create_directories(test_dir);
  const fs::path npz_path = test_dir / "writer_test.npz";

  EXPECT_NO_THROW(([&npz_path]() {
    ampc::NpzWriter writer(npz_path);
    const std::vector<double> data{0.1, 0.2, 0.3, 0.4};
    writer.addArray("double_data", data.data(), {2, 2});
    writer.addScalar("scalar_double", 4.5);
    writer.finalize();
  })());

  EXPECT_TRUE(fs::exists(npz_path));
  fs::remove_all(test_dir);
}

TEST(NpzWriterTest, WritesFileBackedArchiveWithoutThrowing)
{
  const fs::path test_dir = fs::temp_directory_path() / "npz_writer_file_test";
  fs::create_directories(test_dir);
  const fs::path raw_path = test_dir / "payload.raw";
  const fs::path npz_path = test_dir / "writer_file_test.npz";
  const std::vector<double> data{0.1, 0.2, 0.3, 0.4};

  {
    std::ofstream fout(raw_path, std::ios::binary | std::ios::out);
    fout.write(reinterpret_cast<const char*>(data.data()),
               static_cast<std::streamsize>(data.size() * sizeof(double)));
  }

  EXPECT_NO_THROW(([&npz_path, &raw_path]() {
    ampc::NpzWriter writer(npz_path);
    writer.addDoubleArrayFromFile("double_data", raw_path, {2, 2});
    writer.finalize();
  })());

  EXPECT_TRUE(fs::exists(npz_path));
  fs::remove_all(test_dir);
}
