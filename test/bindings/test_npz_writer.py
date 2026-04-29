import os
import subprocess
import tempfile
import zipfile
from pathlib import Path

import numpy as np


def _fixture_binary() -> str:
    path = os.environ.get("AFFINE_MPC_NPZ_WRITER_FIXTURE")
    assert path, "AFFINE_MPC_NPZ_WRITER_FIXTURE is not set"
    return path


def _has_zlib() -> bool:
    return os.environ.get("AFFINE_MPC_HAS_ZLIB") == "1"


def _run_fixture(case_name: str, output_path: Path) -> None:
    subprocess.run([_fixture_binary(), case_name, str(output_path)], check=True)


def test_npz_writer_basic_roundtrip():
    with tempfile.TemporaryDirectory() as tmpdir:
        npz_path = Path(tmpdir) / "writer_test.npz"
        _run_fixture("basic", npz_path)

        with np.load(npz_path, allow_pickle=False) as data:
            assert set(data.files) == {
                "double_data",
                "float_data",
                "int32_data",
                "int64_data",
                "scalar_double",
                "scalar_int",
            }

            np.testing.assert_equal(data["float_data"].shape, (3,))
            np.testing.assert_equal(data["float_data"].dtype, np.dtype(np.float32))
            np.testing.assert_allclose(data["float_data"], np.array([1.5, -2.0, 3.25], dtype=np.float32))

            np.testing.assert_equal(data["double_data"].shape, (2, 2))
            np.testing.assert_equal(data["double_data"].dtype, np.dtype(np.float64))
            np.testing.assert_allclose(
                data["double_data"],
                np.array([[0.1, 0.2], [0.3, 0.4]], dtype=np.float64),
            )

            np.testing.assert_equal(data["int32_data"].dtype, np.dtype(np.int32))
            np.testing.assert_array_equal(data["int32_data"], np.array([1, 2, 3, 4], dtype=np.int32))

            np.testing.assert_equal(data["int64_data"].dtype, np.dtype(np.int64))
            np.testing.assert_array_equal(data["int64_data"], np.array([9, 8, 7], dtype=np.int64))

            np.testing.assert_equal(data["scalar_double"].shape, (1,))
            np.testing.assert_equal(data["scalar_double"].dtype, np.dtype(np.float64))
            np.testing.assert_allclose(data["scalar_double"], np.array([4.5], dtype=np.float64))

            np.testing.assert_equal(data["scalar_int"].shape, (1,))
            np.testing.assert_equal(data["scalar_int"].dtype, np.dtype(np.int32))
            np.testing.assert_array_equal(data["scalar_int"], np.array([6], dtype=np.int32))


def test_npz_writer_compression_mode():
    with tempfile.TemporaryDirectory() as tmpdir:
        npz_path = Path(tmpdir) / "compression_test.npz"
        _run_fixture("compression", npz_path)

        with zipfile.ZipFile(npz_path) as archive:
            info = archive.getinfo("payload.npy")
            expected = zipfile.ZIP_DEFLATED if _has_zlib() else zipfile.ZIP_STORED
            assert info.compress_type == expected

        with np.load(npz_path, allow_pickle=False) as data:
            np.testing.assert_equal(data["payload"].shape, (1024,))
            np.testing.assert_equal(data["payload"].dtype, np.dtype(np.float64))
            np.testing.assert_allclose(data["payload"][512], 1.2345)
