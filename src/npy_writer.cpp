#include "npy_writer.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace affine_mpc {

namespace {

std::string makeShapeString(const std::vector<size_t>& shape)
{
  std::ostringstream oss;
  oss << "(";
  for (size_t i{0}; i < shape.size(); ++i) {
    if (i > 0)
      oss << ", ";
    oss << shape[i];
  }
  if (shape.size() == 1)
    oss << ",";
  oss << ")";
  return oss.str();
}

void writeNpyHeaderV2(std::ofstream& fout, const std::vector<size_t>& shape)
{
  std::ostringstream header;
  header << "{'descr': '<f8', 'fortran_order': False, 'shape': "
         << makeShapeString(shape) << ", }";
  std::string header_str = header.str();

  constexpr size_t preamble_size{12};
  const size_t padding =
      (16 - ((preamble_size + header_str.size() + 1) % 16)) % 16;
  header_str.append(padding, ' ');
  header_str.push_back('\n');

  if (header_str.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::length_error(
        "[NpyWriter] NPY header too large for v2.0 format.");
  }

  fout.write("\x93NUMPY", 6);
  const std::uint8_t version[2]{2, 0};
  fout.write(reinterpret_cast<const char*>(version), 2);

  const std::uint32_t header_len =
      static_cast<std::uint32_t>(header_str.size());
  const std::uint8_t header_len_le[4]{
      static_cast<std::uint8_t>(header_len & 0xffu),
      static_cast<std::uint8_t>((header_len >> 8) & 0xffu),
      static_cast<std::uint8_t>((header_len >> 16) & 0xffu),
      static_cast<std::uint8_t>((header_len >> 24) & 0xffu)};
  fout.write(reinterpret_cast<const char*>(header_len_le), 4);
  fout.write(header_str.data(),
             static_cast<std::streamsize>(header_str.size()));
}

} // namespace

void NpyWriter::writeDoubleArrayFromFile(const std::filesystem::path& npy_path,
                                         const std::filesystem::path& raw_path,
                                         const std::vector<size_t>& shape)
{
  std::ifstream fin(raw_path, std::ios::binary | std::ios::ate);
  if (!fin.is_open()) {
    throw std::runtime_error("[NpyWriter] Failed to open raw payload file: "
                             + raw_path.string());
  }

  const auto size = fin.tellg();
  if (size < std::streamoff{0}) {
    throw std::runtime_error("[NpyWriter] Failed to query raw payload size: "
                             + raw_path.string());
  }

  size_t expected_numel{1};
  for (const size_t dim : shape) {
    if (dim > 0 && expected_numel > std::numeric_limits<size_t>::max() / dim) {
      throw std::overflow_error("[NpyWriter] NPY shape overflow.");
    }
    expected_numel *= dim;
  }

  const auto expected_size =
      static_cast<std::uintmax_t>(expected_numel) * sizeof(double);
  if (static_cast<std::uintmax_t>(size) != expected_size) {
    throw std::runtime_error("[NpyWriter] Raw payload size does not match the "
                             "expected NPY array shape for: "
                             + raw_path.string());
  }

  fin.seekg(0, std::ios::beg);
  std::ofstream fout(npy_path, std::ios::binary | std::ios::out);
  if (!fout.is_open()) {
    throw std::runtime_error("[NpyWriter] Failed to open output NPY file: "
                             + npy_path.string());
  }

  writeNpyHeaderV2(fout, shape);

  std::array<char, 1 << 16> buffer{};
  while (fin) {
    fin.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = fin.gcount();
    if (count > 0)
      fout.write(buffer.data(), count);
  }

  if (!fout) {
    throw std::runtime_error(
        "[NpyWriter] Failed while writing output NPY file: "
        + npy_path.string());
  }
}

} // namespace affine_mpc
