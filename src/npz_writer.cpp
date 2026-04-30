#include "npz_writer.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#if AFFINE_MPC_HAS_ZLIB
#include <zlib.h>
#endif

namespace affine_mpc {

namespace {

template <typename T> struct DTypeTraits;

template <> struct DTypeTraits<float>
{
  static constexpr const char* descr = "<f4";
};

template <> struct DTypeTraits<double>
{
  static constexpr const char* descr = "<f8";
};

template <> struct DTypeTraits<std::int32_t>
{
  static constexpr const char* descr = "<i4";
};

template <> struct DTypeTraits<std::int64_t>
{
  static constexpr const char* descr = "<i8";
};

void appendLittleEndian16(std::vector<std::uint8_t>& out, std::uint16_t value)
{
  out.push_back(static_cast<std::uint8_t>(value & 0xffu));
  out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xffu));
}

void appendLittleEndian32(std::vector<std::uint8_t>& out, std::uint32_t value)
{
  out.push_back(static_cast<std::uint8_t>(value & 0xffu));
  out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xffu));
  out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xffu));
  out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xffu));
}

void appendBytes(std::vector<std::uint8_t>& out,
                 const void* data,
                 const size_t num_bytes)
{
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  out.insert(out.end(), bytes, bytes + num_bytes);
}

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

template <typename T>
std::vector<std::uint8_t> makeNpyPayload(const T* data,
                                         const std::vector<size_t>& shape)
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>
                    || std::is_same_v<T, std::int32_t>
                    || std::is_same_v<T, std::int64_t>,
                "Unsupported npy dtype");

  size_t num_elements{1};
  for (const size_t dim : shape) {
    if (dim > 0 && num_elements > std::numeric_limits<size_t>::max() / dim)
      throw std::overflow_error("[NpzWriter] NPY payload size overflow.");
    num_elements *= dim;
  }

  std::ostringstream header;
  header << "{'descr': '" << DTypeTraits<T>::descr
         << "', 'fortran_order': False, 'shape': " << makeShapeString(shape)
         << ", }";
  std::string header_str = header.str();

  constexpr size_t preamble_size{10};
  size_t header_len{header_str.size() + 1};
  const size_t padding = (16 - ((preamble_size + header_len) % 16)) % 16;
  header_str.append(padding, ' ');
  header_str.push_back('\n');

  if (header_str.size() > std::numeric_limits<std::uint16_t>::max())
    throw std::length_error(
        "[NpzWriter] NPY header too large for v1.0 format.");

  std::vector<std::uint8_t> payload;
  payload.reserve(preamble_size + header_str.size() + num_elements * sizeof(T));
  appendBytes(payload, "\x93NUMPY", 6);
  payload.push_back(1);
  payload.push_back(0);
  appendLittleEndian16(payload, static_cast<std::uint16_t>(header_str.size()));
  appendBytes(payload, header_str.data(), header_str.size());
  appendBytes(payload, data, num_elements * sizeof(T));
  return payload;
}

std::uint32_t crc32Fallback(const std::uint8_t* data, const size_t size)
{
  static std::array<std::uint32_t, 256> table{};
  static bool initialized{false};
  if (!initialized) {
    for (std::uint32_t i{0}; i < table.size(); ++i) {
      std::uint32_t crc{i};
      for (int bit{0}; bit < 8; ++bit)
        crc = (crc & 1u) ? (0xedb88320u ^ (crc >> 1)) : (crc >> 1);
      table[i] = crc;
    }
    initialized = true;
  }

  std::uint32_t crc{0xffffffffu};
  for (size_t i{0}; i < size; ++i)
    crc = table[(crc ^ data[i]) & 0xffu] ^ (crc >> 8);
  return crc ^ 0xffffffffu;
}

std::uint32_t computeCrc32(const std::vector<std::uint8_t>& bytes)
{
#if AFFINE_MPC_HAS_ZLIB
  return static_cast<std::uint32_t>(
      ::crc32(0u, bytes.data(), static_cast<uInt>(bytes.size())));
#else
  return crc32Fallback(bytes.data(), bytes.size());
#endif
}

std::vector<std::uint8_t> maybeCompress(const std::vector<std::uint8_t>& bytes,
                                        std::uint16_t& compression_method)
{
#if AFFINE_MPC_HAS_ZLIB
  z_stream stream{};
  if (deflateInit2(&stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED, -MAX_WBITS, 8,
                   Z_DEFAULT_STRATEGY)
      != Z_OK)
    throw std::runtime_error("[NpzWriter] Failed to initialize zlib deflater.");

  std::vector<std::uint8_t> compressed;
  compressed.resize(deflateBound(&stream, static_cast<uLong>(bytes.size())));
  // zlib's streaming API is not const-correct here; the input buffer is not
  // modified.
  stream.next_in =
      const_cast<Bytef*>(reinterpret_cast<const Bytef*>(bytes.data()));
  stream.avail_in = static_cast<uInt>(bytes.size());
  stream.next_out = reinterpret_cast<Bytef*>(compressed.data());
  stream.avail_out = static_cast<uInt>(compressed.size());

  const int status = deflate(&stream, Z_FINISH);
  if (status != Z_STREAM_END) {
    deflateEnd(&stream);
    throw std::runtime_error("[NpzWriter] Failed to compress npz entry.");
  }
  compressed.resize(stream.total_out);
  deflateEnd(&stream);
  compression_method = 8;
  return compressed;
#else
  compression_method = 0;
  return bytes;
#endif
}

} // namespace

struct NpzWriter::Impl
{
  struct Entry
  {
    std::string filename;
    std::uint16_t compression_method;
    std::uint32_t crc32;
    std::uint32_t compressed_size;
    std::uint32_t uncompressed_size;
    std::uint32_t local_header_offset;
  };

  explicit Impl(const std::filesystem::path& path) : out(path, std::ios::binary)
  {
    if (!out.is_open())
      throw std::runtime_error("[NpzWriter] Failed to open npz output file.");
  }

  std::ofstream out;
  std::vector<Entry> entries;
  bool finalized{false};

  void writeBytes(const std::vector<std::uint8_t>& bytes)
  {
    out.write(reinterpret_cast<const char*>(bytes.data()),
              static_cast<std::streamsize>(bytes.size()));
    if (!out)
      throw std::runtime_error("[NpzWriter] Failed to write npz data.");
  }

  void writeLocalHeader(const Entry& entry)
  {
    std::vector<std::uint8_t> header;
    header.reserve(30 + entry.filename.size());
    appendLittleEndian32(header, 0x04034b50u);
    appendLittleEndian16(header, 20);
    appendLittleEndian16(header, 0);
    appendLittleEndian16(header, entry.compression_method);
    appendLittleEndian16(header, 0);
    appendLittleEndian16(header, 0);
    appendLittleEndian32(header, entry.crc32);
    appendLittleEndian32(header, entry.compressed_size);
    appendLittleEndian32(header, entry.uncompressed_size);
    appendLittleEndian16(header,
                         static_cast<std::uint16_t>(entry.filename.size()));
    appendLittleEndian16(header, 0);
    appendBytes(header, entry.filename.data(), entry.filename.size());
    writeBytes(header);
  }

  void writeCentralDirectory()
  {
    const std::uint32_t central_dir_offset =
        static_cast<std::uint32_t>(out.tellp());

    for (const Entry& entry : entries) {
      std::vector<std::uint8_t> header;
      header.reserve(46 + entry.filename.size());
      appendLittleEndian32(header, 0x02014b50u);
      appendLittleEndian16(header, 20);
      appendLittleEndian16(header, 20);
      appendLittleEndian16(header, 0);
      appendLittleEndian16(header, entry.compression_method);
      appendLittleEndian16(header, 0);
      appendLittleEndian16(header, 0);
      appendLittleEndian32(header, entry.crc32);
      appendLittleEndian32(header, entry.compressed_size);
      appendLittleEndian32(header, entry.uncompressed_size);
      appendLittleEndian16(header,
                           static_cast<std::uint16_t>(entry.filename.size()));
      appendLittleEndian16(header, 0);
      appendLittleEndian16(header, 0);
      appendLittleEndian16(header, 0);
      appendLittleEndian16(header, 0);
      appendLittleEndian32(header, 0);
      appendLittleEndian32(header, entry.local_header_offset);
      appendBytes(header, entry.filename.data(), entry.filename.size());
      writeBytes(header);
    }

    const std::uint32_t central_dir_end =
        static_cast<std::uint32_t>(out.tellp());
    const std::uint32_t central_dir_size = central_dir_end - central_dir_offset;

    std::vector<std::uint8_t> end_record;
    end_record.reserve(22);
    appendLittleEndian32(end_record, 0x06054b50u);
    appendLittleEndian16(end_record, 0);
    appendLittleEndian16(end_record, 0);
    appendLittleEndian16(end_record,
                         static_cast<std::uint16_t>(entries.size()));
    appendLittleEndian16(end_record,
                         static_cast<std::uint16_t>(entries.size()));
    appendLittleEndian32(end_record, central_dir_size);
    appendLittleEndian32(end_record, central_dir_offset);
    appendLittleEndian16(end_record, 0);
    writeBytes(end_record);
  }
};

NpzWriter::NpzWriter(const std::filesystem::path& path) :
    impl_(std::make_unique<Impl>(path))
{}

NpzWriter::~NpzWriter() noexcept
{
  if (impl_ != nullptr) {
    if (!impl_->finalized) {
      try {
        finalize();
      } catch (...) {
        // Destructors must not throw; finalization is best-effort only.
      }
    }
  }
}

void NpzWriter::addArray(const std::string& name,
                         const float* data,
                         const std::vector<size_t>& shape)
{
  if (impl_->finalized)
    throw std::logic_error("[NpzWriter] Cannot add entries after finalize().");
  const std::vector<std::uint8_t> npy_bytes = makeNpyPayload(data, shape);
  Impl::Entry entry{};
  entry.filename = name + ".npy";
  entry.uncompressed_size = static_cast<std::uint32_t>(npy_bytes.size());
  entry.local_header_offset = static_cast<std::uint32_t>(impl_->out.tellp());
  entry.crc32 = computeCrc32(npy_bytes);
  std::vector<std::uint8_t> payload =
      maybeCompress(npy_bytes, entry.compression_method);
  entry.compressed_size = static_cast<std::uint32_t>(payload.size());
  impl_->writeLocalHeader(entry);
  impl_->writeBytes(payload);
  impl_->entries.push_back(std::move(entry));
}

void NpzWriter::addArray(const std::string& name,
                         const double* data,
                         const std::vector<size_t>& shape)
{
  if (impl_->finalized)
    throw std::logic_error("[NpzWriter] Cannot add entries after finalize().");
  const std::vector<std::uint8_t> npy_bytes = makeNpyPayload(data, shape);
  Impl::Entry entry{};
  entry.filename = name + ".npy";
  entry.uncompressed_size = static_cast<std::uint32_t>(npy_bytes.size());
  entry.local_header_offset = static_cast<std::uint32_t>(impl_->out.tellp());
  entry.crc32 = computeCrc32(npy_bytes);
  std::vector<std::uint8_t> payload =
      maybeCompress(npy_bytes, entry.compression_method);
  entry.compressed_size = static_cast<std::uint32_t>(payload.size());
  impl_->writeLocalHeader(entry);
  impl_->writeBytes(payload);
  impl_->entries.push_back(std::move(entry));
}

void NpzWriter::addArray(const std::string& name,
                         const std::int32_t* data,
                         const std::vector<size_t>& shape)
{
  if (impl_->finalized)
    throw std::logic_error("[NpzWriter] Cannot add entries after finalize().");
  const std::vector<std::uint8_t> npy_bytes = makeNpyPayload(data, shape);
  Impl::Entry entry{};
  entry.filename = name + ".npy";
  entry.uncompressed_size = static_cast<std::uint32_t>(npy_bytes.size());
  entry.local_header_offset = static_cast<std::uint32_t>(impl_->out.tellp());
  entry.crc32 = computeCrc32(npy_bytes);
  std::vector<std::uint8_t> payload =
      maybeCompress(npy_bytes, entry.compression_method);
  entry.compressed_size = static_cast<std::uint32_t>(payload.size());
  impl_->writeLocalHeader(entry);
  impl_->writeBytes(payload);
  impl_->entries.push_back(std::move(entry));
}

void NpzWriter::addArray(const std::string& name,
                         const std::int64_t* data,
                         const std::vector<size_t>& shape)
{
  if (impl_->finalized)
    throw std::logic_error("[NpzWriter] Cannot add entries after finalize().");
  const std::vector<std::uint8_t> npy_bytes = makeNpyPayload(data, shape);
  Impl::Entry entry{};
  entry.filename = name + ".npy";
  entry.uncompressed_size = static_cast<std::uint32_t>(npy_bytes.size());
  entry.local_header_offset = static_cast<std::uint32_t>(impl_->out.tellp());
  entry.crc32 = computeCrc32(npy_bytes);
  std::vector<std::uint8_t> payload =
      maybeCompress(npy_bytes, entry.compression_method);
  entry.compressed_size = static_cast<std::uint32_t>(payload.size());
  impl_->writeLocalHeader(entry);
  impl_->writeBytes(payload);
  impl_->entries.push_back(std::move(entry));
}

void NpzWriter::addScalar(const std::string& name, const float value)
{
  addArray(name, &value, {1});
}

void NpzWriter::addScalar(const std::string& name, const double value)
{
  addArray(name, &value, {1});
}

void NpzWriter::addScalar(const std::string& name, const std::int32_t value)
{
  addArray(name, &value, {1});
}

void NpzWriter::addScalar(const std::string& name, const std::int64_t value)
{
  addArray(name, &value, {1});
}

void NpzWriter::finalize()
{
  if (impl_->finalized)
    return;
  impl_->writeCentralDirectory();
  impl_->out.close();
  impl_->finalized = true;
}

} // namespace affine_mpc
