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

constexpr std::uint16_t kZipMethodStored = 0;
constexpr std::uint16_t kZipMethodDeflated = 8;

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

template <typename T>
constexpr bool kSupportedNpyType =
    std::is_same_v<T, float> || std::is_same_v<T, double>
    || std::is_same_v<T, std::int32_t> || std::is_same_v<T, std::int64_t>;

std::uint32_t toUint32Size(std::uint64_t value, const char* const context);

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
                 size_t num_bytes)
{
  if (num_bytes == 0)
    return;
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
std::vector<std::uint8_t> makeNpyHeader(const std::vector<size_t>& shape)
{
  static_assert(kSupportedNpyType<T>, "Unsupported npy dtype");

  std::ostringstream header;
  header << "{'descr': '" << DTypeTraits<T>::descr
         << "', 'fortran_order': False, 'shape': " << makeShapeString(shape)
         << ", }";
  std::string header_str{header.str()};

  constexpr size_t preamble_size{10};
  size_t header_len{header_str.size() + 1};
  const size_t padding = (16 - ((preamble_size + header_len) % 16)) % 16;
  header_str.append(padding, ' ');
  header_str.push_back('\n');

  if (header_str.size() > std::numeric_limits<std::uint16_t>::max())
    throw std::length_error(
        "[NpzWriter] NPY header too large for v1.0 format.");

  std::vector<std::uint8_t> payload;
  payload.reserve(preamble_size + header_str.size());
  appendBytes(payload, "\x93NUMPY", 6);
  payload.push_back(1);
  payload.push_back(0);
  appendLittleEndian16(payload, static_cast<std::uint16_t>(header_str.size()));
  appendBytes(payload, header_str.data(), header_str.size());
  return payload;
}

size_t countShapeElements(const std::vector<size_t>& shape)
{
  size_t num_elements{1};
  for (const size_t dim : shape) {
    if (dim > 0 && num_elements > std::numeric_limits<size_t>::max() / dim)
      throw std::overflow_error("[NpzWriter] NPY payload size overflow.");
    num_elements *= dim;
  }
  return num_elements;
}

template <typename T>
std::uint32_t computePayloadSize(const std::vector<size_t>& shape)
{
  static_assert(kSupportedNpyType<T>, "Unsupported npy dtype");
  const size_t num_elements = countShapeElements(shape);
  const size_t num_bytes = num_elements * sizeof(T);
  return toUint32Size(num_bytes, "NPY payload size");
}

template <typename T>
std::vector<std::uint8_t> makeNpyPayload(const T* data,
                                         const std::vector<size_t>& shape)
{
  static_assert(kSupportedNpyType<T>, "Unsupported npy dtype");

  const size_t num_elements = countShapeElements(shape);
  std::vector<std::uint8_t> payload = makeNpyHeader<T>(shape);
  payload.reserve(payload.size() + num_elements * sizeof(T));
  appendBytes(payload, data, num_elements * sizeof(T));
  return payload;
}

std::uint32_t crc32Fallback(const std::uint8_t* data, size_t size)
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

std::uint32_t
crc32FallbackUpdate(std::uint32_t crc, const std::uint8_t* data, size_t size)
{
  static std::array<std::uint32_t, 256> table{};
  static bool initialized{false};
  if (!initialized) {
    for (std::uint32_t i{0}; i < table.size(); ++i) {
      std::uint32_t value{i};
      for (int bit{0}; bit < 8; ++bit)
        value = (value & 1u) ? (0xedb88320u ^ (value >> 1)) : (value >> 1);
      table[i] = value;
    }
    initialized = true;
  }

  crc ^= 0xffffffffu;
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

std::uint32_t
updateCrc32(std::uint32_t crc, const std::uint8_t* data, size_t size)
{
#if AFFINE_MPC_HAS_ZLIB
  return static_cast<std::uint32_t>(
      ::crc32(crc, data, static_cast<uInt>(size)));
#else
  return crc32FallbackUpdate(crc, data, size);
#endif
}

std::uint32_t toUint32Size(std::uint64_t value, const char* const context)
{
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::overflow_error(std::string{"[NpzWriter] "} + context
                              + " exceeds ZIP32 size limits.");
  }
  return static_cast<std::uint32_t>(value);
}

std::uint32_t toUint32Offset(const std::streampos position,
                             const char* const context)
{
  if (position < std::streampos{0}) {
    throw std::runtime_error(std::string{"[NpzWriter] Failed to query "}
                             + context + ".");
  }

  const auto offset{static_cast<std::uint64_t>(position)};
  if (offset > std::numeric_limits<std::uint32_t>::max()) {
    throw std::overflow_error(std::string{"[NpzWriter] "} + context
                              + " exceeds ZIP32 offset limits.");
  }
  return static_cast<std::uint32_t>(offset);
}

std::vector<std::uint8_t> maybeCompress(const std::vector<std::uint8_t>& bytes,
                                        std::uint16_t& compression_method,
                                        NpzWriter::CompressionMode mode)
{
  if (mode == NpzWriter::CompressionMode::Stored) {
    compression_method = kZipMethodStored;
    return bytes;
  }

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
  compression_method = kZipMethodDeflated;
  return compressed;
#else
  compression_method = kZipMethodStored;
  return bytes;
#endif
}

struct FilePayloadInfo
{
  std::uint32_t crc32;
  std::uint32_t compressed_size;
  std::uint32_t uncompressed_size;
  std::uint16_t compression_method;
};

std::ifstream openRawFile(const std::filesystem::path& raw_path,
                          std::ios::openmode mode = std::ios::binary)
{
  std::ifstream fin{raw_path, mode};
  if (!fin.is_open()) {
    throw std::runtime_error("[NpzWriter] Failed to open raw payload file: "
                             + raw_path.string());
  }
  return fin;
}

std::uintmax_t getRawFileSize(const std::filesystem::path& raw_path)
{
  std::ifstream fin = openRawFile(raw_path, std::ios::binary | std::ios::ate);
  const auto size = fin.tellg();
  if (size < std::streamoff{0}) {
    throw std::runtime_error("[NpzWriter] Failed to query raw payload size: "
                             + raw_path.string());
  }
  return static_cast<std::uintmax_t>(size);
}

#if AFFINE_MPC_HAS_ZLIB
template <typename Emit>
std::uint64_t deflateBuffer(z_stream& stream,
                            const std::uint8_t* data,
                            size_t size,
                            int flush,
                            Emit&& emit)
{
  std::array<std::uint8_t, 1 << 16> out_buffer{};
  std::uint64_t total_out{0};
  stream.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(data));
  stream.avail_in = static_cast<uInt>(size);

  do {
    stream.next_out = reinterpret_cast<Bytef*>(out_buffer.data());
    stream.avail_out = static_cast<uInt>(out_buffer.size());

    const int status = deflate(&stream, flush);
    if (status != Z_OK && status != Z_STREAM_END) {
      throw std::runtime_error("[NpzWriter] Failed to compress npz entry.");
    }

    const size_t produced = out_buffer.size() - stream.avail_out;
    if (produced > 0) {
      emit(out_buffer.data(), produced);
      total_out += produced;
    }

    if (flush == Z_FINISH && status == Z_STREAM_END)
      break;
  } while (stream.avail_in > 0 || stream.avail_out == 0 || flush == Z_FINISH);

  return total_out;
}

template <typename Emit>
std::uint64_t
streamCompressedFilePayload(const std::vector<std::uint8_t>& header,
                            const std::filesystem::path& raw_path,
                            Emit&& emit)
{
  z_stream stream{};
  if (deflateInit2(&stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED, -MAX_WBITS, 8,
                   Z_DEFAULT_STRATEGY)
      != Z_OK) {
    throw std::runtime_error("[NpzWriter] Failed to initialize zlib deflater.");
  }

  std::uint64_t total_out{0};
  try {
    total_out +=
        deflateBuffer(stream, header.data(), header.size(), Z_NO_FLUSH, emit);

    std::ifstream fin = openRawFile(raw_path);
    std::array<char, 1 << 16> in_buffer{};
    while (fin) {
      fin.read(in_buffer.data(),
               static_cast<std::streamsize>(in_buffer.size()));
      const auto count = fin.gcount();
      if (count > 0) {
        total_out += deflateBuffer(
            stream, reinterpret_cast<const std::uint8_t*>(in_buffer.data()),
            static_cast<size_t>(count), Z_NO_FLUSH, emit);
      }
    }

    total_out += deflateBuffer(stream, nullptr, 0, Z_FINISH, emit);
  } catch (...) {
    deflateEnd(&stream);
    throw;
  }

  deflateEnd(&stream);
  return total_out;
}
#endif

FilePayloadInfo analyzeDoubleFilePayload(const std::filesystem::path& raw_path,
                                         const std::vector<size_t>& shape,
                                         NpzWriter::CompressionMode mode)
{
  const std::vector<std::uint8_t> header = makeNpyHeader<double>(shape);
  const std::uint32_t body_size = computePayloadSize<double>(shape);
  const std::uint64_t expected_size = body_size;
  const std::uintmax_t raw_size = getRawFileSize(raw_path);
  if (raw_size != expected_size) {
    throw std::runtime_error("[NpzWriter] Raw payload size does not match the "
                             "expected NPZ array shape for: "
                             + raw_path.string());
  }

  std::uint32_t crc{0};
  crc = updateCrc32(crc, header.data(), header.size());

  std::ifstream fin = openRawFile(raw_path);
  std::array<char, 1 << 16> buffer{};
  while (fin) {
    fin.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = fin.gcount();
    if (count > 0) {
      crc =
          updateCrc32(crc, reinterpret_cast<const std::uint8_t*>(buffer.data()),
                      static_cast<size_t>(count));
    }
  }

  FilePayloadInfo info{};
  info.crc32 = crc;
  info.uncompressed_size =
      toUint32Size(static_cast<std::uint64_t>(header.size()) + raw_size,
                   "NPZ entry uncompressed size");
#if AFFINE_MPC_HAS_ZLIB
  if (mode == NpzWriter::CompressionMode::Deflated) {
    info.compression_method = kZipMethodDeflated;
    info.compressed_size =
        toUint32Size(streamCompressedFilePayload(
                         header, raw_path, [](const std::uint8_t*, size_t) {}),
                     "NPZ entry compressed size");
  } else {
    info.compression_method = kZipMethodStored;
    info.compressed_size = info.uncompressed_size;
  }
#else
  info.compression_method = kZipMethodStored;
  info.compressed_size = info.uncompressed_size;
#endif
  return info;
}

void streamStoredFilePayload(std::ofstream& out,
                             const std::vector<std::uint8_t>& header,
                             const std::filesystem::path& raw_path)
{
  out.write(reinterpret_cast<const char*>(header.data()),
            static_cast<std::streamsize>(header.size()));
  if (!out)
    throw std::runtime_error("[NpzWriter] Failed to write npz data.");

  std::ifstream fin = openRawFile(raw_path);
  std::array<char, 1 << 16> buffer{};
  while (fin) {
    fin.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = fin.gcount();
    if (count > 0) {
      out.write(buffer.data(), count);
      if (!out)
        throw std::runtime_error("[NpzWriter] Failed to write npz data.");
    }
  }
}

void streamFilePayloadToArchive(std::ofstream& out,
                                const std::vector<std::uint8_t>& header,
                                const std::filesystem::path& raw_path,
                                std::uint16_t compression_method)
{
#if AFFINE_MPC_HAS_ZLIB
  if (compression_method == kZipMethodDeflated) {
    streamCompressedFilePayload(
        header, raw_path, [&](const std::uint8_t* data, size_t size) {
          out.write(reinterpret_cast<const char*>(data),
                    static_cast<std::streamsize>(size));
          if (!out)
            throw std::runtime_error("[NpzWriter] Failed to write npz data.");
        });
    return;
  }
#endif

  streamStoredFilePayload(out, header, raw_path);
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

  explicit Impl(const std::filesystem::path& path,
                NpzWriter::CompressionMode compression_mode) :
      out{path, std::ios::binary}, compression_mode{compression_mode}
  {
    if (!out.is_open())
      throw std::runtime_error("[NpzWriter] Failed to open npz output file.");
  }

  std::ofstream out;
  std::vector<Entry> entries;
  bool finalized{false};
  NpzWriter::CompressionMode compression_mode;

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
        toUint32Offset(out.tellp(), "central directory offset");

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
        toUint32Offset(out.tellp(), "central directory end offset");
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

template <typename T>
void NpzWriter::addArrayImpl(const std::string& name,
                             const T* data,
                             const std::vector<size_t>& shape)
{
  ensureNotFinalized();

  const std::vector<std::uint8_t> npy_bytes = makeNpyPayload(data, shape);
  Impl::Entry entry{};
  entry.filename = name + ".npy";
  entry.uncompressed_size =
      toUint32Size(npy_bytes.size(), "NPZ entry uncompressed size");
  entry.local_header_offset =
      toUint32Offset(impl_->out.tellp(), "NPZ entry local header offset");
  entry.crc32 = computeCrc32(npy_bytes);
  std::vector<std::uint8_t> payload{maybeCompress(
      npy_bytes, entry.compression_method, impl_->compression_mode)};
  entry.compressed_size =
      toUint32Size(payload.size(), "NPZ entry compressed size");
  impl_->writeLocalHeader(entry);
  impl_->writeBytes(payload);
  impl_->entries.push_back(std::move(entry));
}

void NpzWriter::addDoubleArrayFromFile(const std::string& name,
                                       const std::filesystem::path& raw_path,
                                       const std::vector<size_t>& shape)
{
  ensureNotFinalized();

  const std::vector<std::uint8_t> header = makeNpyHeader<double>(shape);
  const FilePayloadInfo info =
      analyzeDoubleFilePayload(raw_path, shape, impl_->compression_mode);

  Impl::Entry entry{};
  entry.filename = name + ".npy";
  entry.compression_method = info.compression_method;
  entry.crc32 = info.crc32;
  entry.compressed_size = info.compressed_size;
  entry.uncompressed_size = info.uncompressed_size;
  entry.local_header_offset =
      toUint32Offset(impl_->out.tellp(), "NPZ entry local header offset");

  impl_->writeLocalHeader(entry);
  streamFilePayloadToArchive(impl_->out, header, raw_path,
                             entry.compression_method);
  impl_->entries.push_back(std::move(entry));
}

NpzWriter::NpzWriter(const std::filesystem::path& path,
                     CompressionMode compression_mode) :
    impl_(std::make_unique<Impl>(path, compression_mode))
{}

void NpzWriter::ensureNotFinalized() const
{
  if (impl_->finalized)
    throw std::logic_error("[NpzWriter] Cannot add entries after finalize().");
}

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
  addArrayImpl(name, data, shape);
}

void NpzWriter::addArray(const std::string& name,
                         const double* data,
                         const std::vector<size_t>& shape)
{
  addArrayImpl(name, data, shape);
}

void NpzWriter::addArray(const std::string& name,
                         const std::int32_t* data,
                         const std::vector<size_t>& shape)
{
  addArrayImpl(name, data, shape);
}

void NpzWriter::addArray(const std::string& name,
                         const std::int64_t* data,
                         const std::vector<size_t>& shape)
{
  addArrayImpl(name, data, shape);
}

void NpzWriter::addScalar(const std::string& name, float value)
{
  addArray(name, &value, {1});
}

void NpzWriter::addScalar(const std::string& name, double value)
{
  addArray(name, &value, {1});
}

void NpzWriter::addScalar(const std::string& name, std::int32_t value)
{
  addArray(name, &value, {1});
}

void NpzWriter::addScalar(const std::string& name, std::int64_t value)
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
