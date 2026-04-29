#include <cnpy.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

#include "npz_writer.hpp"

namespace ampc = affine_mpc;
namespace fs = std::filesystem;

TEST(NpzWriterTest, WritesArraysWithExpectedDtypesAndValues)
{
  const fs::path test_dir = fs::temp_directory_path() / "npz_writer_test";
  fs::create_directories(test_dir);
  const fs::path npz_path = test_dir / "writer_test.npz";

  {
    ampc::NpzWriter writer(npz_path);

    const std::vector<float> float_data{1.5f, -2.0f, 3.25f};
    const std::vector<double> double_data{0.1, 0.2, 0.3, 0.4};
    const std::vector<std::int32_t> int32_data{1, 2, 3, 4};
    const std::vector<std::int64_t> int64_data{9, 8, 7};

    writer.addArray("float_data", float_data.data(), {3});
    writer.addArray("double_data", double_data.data(), {2, 2});
    writer.addArray("int32_data", int32_data.data(), {4});
    writer.addArray("int64_data", int64_data.data(), {3});
    writer.addScalar("scalar_double", 4.5);
    writer.addScalar("scalar_int", static_cast<std::int32_t>(6));
    writer.finalize();
  }

  const cnpy::npz_t npz = cnpy::npz_load(npz_path.string());

  ASSERT_EQ(npz.at("float_data").shape.size(), 1);
  EXPECT_EQ(npz.at("float_data").shape[0], 3);
  EXPECT_FLOAT_EQ(npz.at("float_data").data<float>()[2], 3.25f);

  ASSERT_EQ(npz.at("double_data").shape.size(), 2);
  EXPECT_EQ(npz.at("double_data").shape[0], 2);
  EXPECT_EQ(npz.at("double_data").shape[1], 2);
  EXPECT_DOUBLE_EQ(npz.at("double_data").data<double>()[3], 0.4);

  ASSERT_EQ(npz.at("int32_data").shape.size(), 1);
  EXPECT_EQ(npz.at("int32_data").data<std::int32_t>()[1], 2);

  ASSERT_EQ(npz.at("int64_data").shape.size(), 1);
  EXPECT_EQ(npz.at("int64_data").data<std::int64_t>()[0], 9);

  EXPECT_DOUBLE_EQ(npz.at("scalar_double").data<double>()[0], 4.5);
  EXPECT_EQ(npz.at("scalar_int").data<std::int32_t>()[0], 6);

  fs::remove_all(test_dir);
}

TEST(NpzWriterTest, WritesCompressedArchiveWhenAvailable)
{
  const fs::path test_dir = fs::temp_directory_path() / "npz_writer_zip_test";
  fs::create_directories(test_dir);
  const fs::path npz_path = test_dir / "compression_test.npz";

  {
    ampc::NpzWriter writer(npz_path);
    std::vector<double> data(1024, 1.2345);
    writer.addArray("payload", data.data(), {data.size()});
    writer.finalize();
  }

  std::ifstream fin(npz_path, std::ios::binary);
  ASSERT_TRUE(fin.is_open());
  std::vector<unsigned char> header(10);
  fin.read(reinterpret_cast<char*>(header.data()), header.size());
  ASSERT_EQ(static_cast<std::uint32_t>(header[0])
                | (static_cast<std::uint32_t>(header[1]) << 8)
                | (static_cast<std::uint32_t>(header[2]) << 16)
                | (static_cast<std::uint32_t>(header[3]) << 24),
            0x04034b50u);
  const std::uint16_t compression_method =
      static_cast<std::uint16_t>(header[8])
      | (static_cast<std::uint16_t>(header[9]) << 8);
  EXPECT_TRUE(compression_method == 0 || compression_method == 8);

  const cnpy::npz_t npz = cnpy::npz_load(npz_path.string());
  EXPECT_EQ(npz.at("payload").shape[0], 1024);
  EXPECT_DOUBLE_EQ(npz.at("payload").data<double>()[512], 1.2345);

  fs::remove_all(test_dir);
}
