#include <cstdint>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "npz_writer.hpp"

namespace fs = std::filesystem;
namespace ampc = affine_mpc;

int main(int argc, char** argv)
{
  if (argc != 3) {
    std::cerr << "Usage: npz_writer_fixture <case_name> <output_path>\n";
    return 1;
  }

  const std::string case_name{argv[1]};
  const fs::path output_path{argv[2]};

  try {
    ampc::NpzWriter writer(output_path);

    if (case_name == "basic") {
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
    } else if (case_name == "compression") {
      const std::vector<double> payload(1024, 1.2345);
      writer.addArray("payload", payload.data(), {payload.size()});
    } else {
      throw std::invalid_argument("Unknown fixture case: " + case_name);
    }

    writer.finalize();
  } catch (const std::exception& exc) {
    std::cerr << exc.what() << "\n";
    return 1;
  }

  return 0;
}
