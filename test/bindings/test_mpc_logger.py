import os
import tempfile
import zipfile
from pathlib import Path

import numpy as np

import affine_mpc as ampc


def _has_zlib() -> bool:
    return os.environ.get("AFFINE_MPC_HAS_ZLIB") == "1"


def _make_mpc(use_input_cost: bool = True) -> ampc.CondensedMPC:
    opts = ampc.Options(use_input_cost=use_input_cost)
    mpc = ampc.CondensedMPC(2, 1, ampc.Parameterization.linearInterp(5, 2), opts)

    A = np.array([[1.0, 0.1], [0.0, 1.0]])
    B = np.array([0.0, 0.1])
    w = np.zeros(2)
    ts = 0.1
    mpc.setModelContinuous2Discrete(A, B, w, ts)
    mpc.setInputLimits(np.array([-1.0]), np.array([1.0]))
    if use_input_cost:
        mpc.setWeights(np.ones(2), np.ones(1))
    else:
        mpc.setStateWeights(np.ones(2))
    mpc.setReferenceState(np.zeros(2))
    assert mpc.initializeSolver()
    return mpc


def _log_single_step(
    save_dir: Path,
    save_name: str,
    *,
    prediction_stride: int,
    log_control_points: bool,
    use_input_cost: bool = True,
):
    mpc = _make_mpc(use_input_cost=use_input_cost)
    logger = ampc.MPCLogger(
        mpc=mpc,
        save_dir=save_dir,
        ts=0.1,
        prediction_stride=prediction_stride,
        log_control_points=log_control_points,
        save_name=save_name,
    )

    x0 = np.array([1.0, 0.5])
    assert mpc.solve(x0) == ampc.SolveStatus.Success
    logger.logStep(t=0.0, x0=x0, solve_time=0.001)
    logger.finalize()

    return save_dir / f"{save_name}.npz", save_dir / "params.yaml"


def test_mpc_logger_interface():
    with tempfile.TemporaryDirectory() as tmpdir:
        folder = Path(tmpdir)
        npz_path, param_path = _log_single_step(
            folder,
            "log",
            prediction_stride=1,
            log_control_points=False,
            use_input_cost=True,
        )

        assert param_path.exists()
        assert npz_path.exists()


def test_mpc_logger_npz_contents_default_stride():
    with tempfile.TemporaryDirectory() as tmpdir:
        npz_path, param_path = _log_single_step(
            Path(tmpdir),
            "conv_log",
            prediction_stride=1,
            log_control_points=False,
        )

        assert param_path.exists()
        with np.load(npz_path, allow_pickle=False) as data:
            np.testing.assert_equal(data["states"].shape, (1, 6, 2))
            np.testing.assert_equal(data["inputs"].shape, (1, 6, 1))
            np.testing.assert_equal(data["meta_t_pred"].shape, (6,))
            np.testing.assert_allclose(data["meta_t_pred"][-1], 0.5)


def test_mpc_logger_stride_behavior():
    with tempfile.TemporaryDirectory() as tmpdir:
        npz_path, _ = _log_single_step(
            Path(tmpdir),
            "stride_log",
            prediction_stride=2,
            log_control_points=False,
        )

        with np.load(npz_path, allow_pickle=False) as data:
            np.testing.assert_equal(data["states"].shape[1], 4)
            np.testing.assert_equal(data["inputs"].shape[1], 4)
            np.testing.assert_allclose(data["meta_t_pred"], np.array([0.0, 0.2, 0.4, 0.5]))


def test_mpc_logger_control_points_mode():
    with tempfile.TemporaryDirectory() as tmpdir:
        npz_path, _ = _log_single_step(
            Path(tmpdir),
            "ctrl_log",
            prediction_stride=1,
            log_control_points=True,
        )

        with np.load(npz_path, allow_pickle=False) as data:
            np.testing.assert_equal(data["states"].shape[1], 6)
            np.testing.assert_equal(data["inputs"].shape[1], int(data["meta_num_control_points"][0]))


def test_mpc_logger_zero_stride_squeezes_arrays():
    with tempfile.TemporaryDirectory() as tmpdir:
        npz_path, _ = _log_single_step(
            Path(tmpdir),
            "squeeze_log",
            prediction_stride=0,
            log_control_points=False,
        )

        with np.load(npz_path, allow_pickle=False) as data:
            np.testing.assert_equal(data["states"].shape, (1, 2))
            np.testing.assert_equal(data["inputs"].shape, (1, 1))


def test_mpc_logger_without_input_cost():
    with tempfile.TemporaryDirectory() as tmpdir:
        npz_path, _ = _log_single_step(
            Path(tmpdir),
            "no_input_cost",
            prediction_stride=1,
            log_control_points=False,
            use_input_cost=False,
        )

        with np.load(npz_path, allow_pickle=False) as data:
            assert "ref_inputs" not in data.files
            assert "meta_R_diag" not in data.files


def test_mpc_logger_npz_compression_mode():
    with tempfile.TemporaryDirectory() as tmpdir:
        npz_path, _ = _log_single_step(
            Path(tmpdir),
            "compression_log",
            prediction_stride=1,
            log_control_points=False,
        )

        with zipfile.ZipFile(npz_path) as archive:
            info = archive.getinfo("states.npy")
            expected = zipfile.ZIP_DEFLATED if _has_zlib() else zipfile.ZIP_STORED
            assert info.compress_type == expected
