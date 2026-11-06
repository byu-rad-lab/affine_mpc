import dataclasses
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np
import pytest

import affine_mpc as ampc
from affine_mpc.log import BUILTIN_KEYS

MODES = [
    ampc.MPCLogger.Mode.NpzCompressed,
    ampc.MPCLogger.Mode.NpzUncompressed,
    ampc.MPCLogger.Mode.Npy,
    ampc.MPCLogger.Mode.RawRecoverable,
]
ARTIFACTS = {
    ampc.MPCLogger.Mode.NpzCompressed: "log.npz",
    ampc.MPCLogger.Mode.NpzUncompressed: "log.npz",
    ampc.MPCLogger.Mode.Npy: "log_npy",
    ampc.MPCLogger.Mode.RawRecoverable: "log_raw",
}
DATA_FIELDS = (
    "time",
    "loop_time",
    "mpc_solve_time",
    "qp_solve_time",
    "prepare_time",
    "states",
    "ref_states",
    "inputs",
    "ref_inputs",
)
# custom metadata, including values a text format would round (0.1 + 0.2),
# turn into ints (2.0), or read as numbers or vectors (the strings)
CUSTOM_META = {
    "run_id": 7,
    "gain": 0.1 + 0.2,
    "whole": 2.0,
    "label": "mass spring: damper",
    "numeric_label": "1.5",
    "vector_label": "[1, 2]",
    "quoted": 'say "hi"\n',
    "unicode": "\u00b5 \u2713",
    "empty": "",
    "offsets": np.array([0.1 + 0.2, -np.inf, 1e-300]),
}


def _make_mpc(input_cost=ampc.InputCostMode.ControlPoints):
    mpc = ampc.MPC(
        2,
        1,
        ampc.Parameterization.bspline(10, 2, 4),
        ampc.Options(input_cost=input_cost),
    )
    mpc.setModelContinuous2Discrete(
        np.array([[0.0, 1.0], [-1.0, -0.1]]), np.array([0.0, 1.0]), np.zeros(2), 0.1
    )
    mpc.setInputLimits(np.array([-1.0]), np.array([1.0]))
    if input_cost != ampc.InputCostMode.Off:
        mpc.setWeights(np.array([1.0, 0.1]), np.array([1e-3]))
    else:
        mpc.setStateWeights(np.array([1.0, 0.1]))
    mpc.setReferenceState(np.array([0.5, 0.0]))
    assert mpc.initializeSolver()
    return mpc


def _write_log(
    save_dir,
    mode=ampc.MPCLogger.Mode.NpzCompressed,
    *,
    prediction_stride=2,
    log_control_points=False,
    input_cost=ampc.InputCostMode.ControlPoints,
    steps=4,
):
    mpc = _make_mpc(input_cost)
    logger = ampc.MPCLogger(
        mpc,
        save_dir,
        0.1,
        prediction_stride=prediction_stride,
        log_control_points=log_control_points,
        mode=mode,
    )
    for key, value in CUSTOM_META.items():
        logger.addMetadata(key, value)
    x = np.array([1.0, 0.0])
    for k in range(steps):
        assert mpc.solve(x) == ampc.SolveStatus.Success
        logger.logStep(0.1 * k, x, loop_time=0.01 * k)
        x = x + 0.1 * np.array([x[1], -x[0]])
    logger.finalize()


def _assert_meta_equal(meta, expected):
    assert meta.keys() == expected.keys()
    for key, value in expected.items():
        if isinstance(value, np.ndarray):
            np.testing.assert_array_equal(meta[key], value, err_msg=key)
        else:
            assert meta[key] == value and type(meta[key]) is type(value), key


@pytest.mark.parametrize("mode", MODES)
def test_load_log_matches_npz_in_every_mode(tmp_path, mode):
    _write_log(tmp_path / "ref")
    _write_log(tmp_path / "run", mode)
    with np.load(tmp_path / "ref" / "log.npz", allow_pickle=False) as data:
        expected = {key: data[key] for key in data.files if not key.startswith("meta_")}

    log = ampc.load_log(tmp_path / "run")

    assert log.path == tmp_path / "run" / ARTIFACTS[mode]
    assert log.finalized
    assert set(expected) == {
        "time",
        "loop_time",
        "mpc_solve_time",
        "qp_solve_time",
        "prepare_time",
        "states",
        "ref_states",
        "inputs",
        "ref_inputs",
    }
    for key in ("time", "loop_time", "states", "ref_states", "inputs", "ref_inputs"):
        np.testing.assert_array_equal(getattr(log, key), expected[key], err_msg=key)
    for key in ("mpc_solve_time", "qp_solve_time", "prepare_time"):
        assert getattr(log, key).shape == expected[key].shape

    # meta holds the user metadata only; the built-in values are attributes
    _assert_meta_equal(log.meta, CUSTOM_META)
    assert log.formulation == ampc.Formulation.Condensed
    assert log.options.input_cost == ampc.InputCostMode.ControlPoints
    assert log.parameterization_type == ampc.Parameterization.Type.BSpline
    assert log.spline_degree == 2 and log.basis.shape == (10, 4)
    assert log.state_dim == 2 and type(log.state_dim) is int
    assert log.ts == 0.1 and type(log.ts) is float
    assert log.logging_mode == mode
    assert log.log_format_version == 1
    np.testing.assert_array_equal(log.u_max, [1.0])
    np.testing.assert_array_equal(log.R_diag, [1e-3])
    assert log.x_min is None and log.Hx is None


@pytest.mark.parametrize("mode", MODES[1:])
def test_load_log_attributes_are_the_same_in_every_mode(tmp_path, mode):
    _write_log(tmp_path / "npz")
    _write_log(tmp_path / "other", mode)
    other = ampc.load_log(tmp_path / "other")
    npz = ampc.load_log(tmp_path / "npz")
    for field in dataclasses.fields(ampc.MPCLog):
        if field.name in DATA_FIELDS or field.name in (
            "logging_mode",
            "path",
            "format",
        ):
            continue
        a, b = getattr(other, field.name), getattr(npz, field.name)
        if field.name == "options":
            assert repr(a) == repr(b)
        elif field.name == "meta":
            _assert_meta_equal(a, b)
        elif isinstance(b, np.ndarray):
            np.testing.assert_array_equal(a, b, err_msg=field.name)
        else:
            assert a == b, field.name


def test_load_log_with_every_option(tmp_path):
    options = ampc.Options(
        input_cost=ampc.InputCostMode.Trajectory,
        use_input_rate_cost=True,
        input_bounds=ampc.InputBoundMode.Refined,
        refined_bound_interval=3,
        slew_initial_input=True,
        slew_bounds=ampc.SlewBoundMode.ControlPoints,
        state_bounds=ampc.StateBoundMode.Soft,
        bounded_states=[1],
        num_general_constraints=2,
        num_terminal_constraints=1,
    )
    mpc = ampc.MPC(
        2,
        1,
        ampc.Parameterization.bspline(10, 2, 4),
        options,
        formulation=ampc.Formulation.Sparse,
    )
    ampc.MPCLogger(mpc, tmp_path, 0.1).finalize()

    log = ampc.load_log(tmp_path)
    # every built-in key became an attribute or part of options
    assert log.meta == {}
    assert repr(log.options) == repr(options)
    assert log.formulation == ampc.Formulation.Sparse
    assert log.spline_degree == 2
    np.testing.assert_array_equal(log.knots, [0.0, 0.0, 0.0, 4.5, 9.0, 9.0, 9.0])
    assert log.Hx.shape == (2, 2) and log.Hu.shape == (2, 1)
    assert log.Hf.shape == (1, 2)
    assert log.h_min.shape == (2,) and log.hf_max.shape == (1,)
    assert log.x_min.shape == (1,) and log.x_soft_l2.shape == (1,)
    for name in ("R_diag", "R_rate_diag", "u0_slew", "slew_rate"):
        assert getattr(log, name).shape == (1,), name
    assert log.num_steps == 0


def test_load_log_with_a_custom_basis(tmp_path):
    T, nc = 6, 3
    basis = np.random.default_rng(0).standard_normal((T, nc))
    mpc = ampc.MPC(2, 1, ampc.Parameterization.fromBasis(T, basis))
    ampc.MPCLogger(mpc, tmp_path, 0.1).finalize()

    log = ampc.load_log(tmp_path)
    assert log.meta == {}
    assert log.parameterization_type == ampc.Parameterization.Type.Custom
    assert log.spline_degree is None and log.knots is None
    np.testing.assert_array_equal(log.basis, basis)


@pytest.mark.parametrize("mode", list(ampc.SlewBoundMode))
def test_load_log_with_each_slew_bound_mode(tmp_path, mode):
    options = ampc.Options(slew_bounds=mode)
    mpc = ampc.MPC(2, 1, ampc.Parameterization.bspline(10, 2, 4), options)
    if mode != ampc.SlewBoundMode.Off:
        mpc.setSlewRate(slew_rate=np.array([0.25]))
    ampc.MPCLogger(mpc, tmp_path, 0.1).finalize()

    log = ampc.load_log(tmp_path)
    assert log.meta == {}
    assert log.options.slew_bounds == mode
    if mode == ampc.SlewBoundMode.Off:
        assert log.slew_rate is None
    else:
        np.testing.assert_array_equal(log.slew_rate, [0.25])


def test_builtin_keys_are_reserved(tmp_path):
    # the loader's built-in keys are all ones the logger rejects from users
    logger = ampc.MPCLogger(_make_mpc(), tmp_path, 0.1)
    for key in sorted(BUILTIN_KEYS):
        with pytest.raises(ValueError, match="logged by MPCLogger itself"):
            logger.addMetadata(key, 1.0)
    logger.finalize()


def test_npz_metadata_layout(tmp_path):
    # read directly with np.load: scalars and strings are 0-d, vectors 1-d
    _write_log(tmp_path)
    with np.load(tmp_path / "log.npz", allow_pickle=False) as data:
        assert data["meta_ts"].shape == () and data["meta_ts"].item() == 0.1
        assert data["meta_state_dim"].shape == ()
        assert data["meta_formulation"].dtype.kind == "U"
        assert data["meta_formulation"].item() == "condensed"
        assert data["meta_u_max"].shape == (1,)
    assert (tmp_path / "log_params.yaml").exists()
    assert not (tmp_path / "params.yaml").exists()


def test_load_log_rejects_other_format_versions(tmp_path):
    _write_log(tmp_path / "log")
    with np.load(tmp_path / "log" / "log.npz", allow_pickle=False) as data:
        arrays = {key: data[key] for key in data.files}
    arrays["meta_log_format_version"] = np.array(2, dtype=np.int32)
    np.savez(tmp_path / "newer.npz", **arrays)
    with pytest.raises(ValueError, match="log_format_version 2"):
        ampc.load_log(tmp_path / "newer.npz")
    del arrays["meta_log_format_version"]
    np.savez(tmp_path / "unversioned.npz", **arrays)
    with pytest.raises(ValueError, match="no log_format_version"):
        ampc.load_log(tmp_path / "unversioned.npz")


def test_load_log_shapes_and_views(tmp_path):
    _write_log(tmp_path)
    log = ampc.load_log(tmp_path)
    N, K = 4, 6  # stride 2 over T = 10: k = 0, 2, ..., 10
    assert log.num_steps == N
    assert log.states.shape == (N, K, 2)
    assert log.x_pred.shape == (N, K, 2)
    np.testing.assert_array_equal(log.x, log.states[:, 0])
    np.testing.assert_array_equal(log.x_ref, log.ref_states[:, 0])
    assert log.t_pred.shape == (N, K)
    np.testing.assert_allclose(
        log.t_pred[1], 0.1 + np.array([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
    )
    assert log.u_pred.shape == (N, K, 1) and log.u_ref_pred.shape == (N, K, 1)
    np.testing.assert_array_equal(log.u, log.inputs[:, 0])
    np.testing.assert_array_equal(log.u_ref, log.ref_inputs[:, 0])
    # the properties are views, not copies
    assert np.shares_memory(log.x, log.states)
    assert np.shares_memory(log.u_pred, log.inputs)
    with np.load(tmp_path / "log.npz", allow_pickle=False) as data:
        np.testing.assert_allclose(
            data["meta_t_pred_offsets"], log.t_pred[0] - log.time[0]
        )
    assert log.prediction_stride == 2
    assert not log.log_control_points


def test_load_log_minimal_stride(tmp_path):
    _write_log(tmp_path, prediction_stride=0)
    log = ampc.load_log(tmp_path)
    assert log.states.shape == (4, 2)
    assert log.x_pred.shape == (4, 1, 2)
    assert log.x_ref_pred.shape == (4, 1, 2)
    np.testing.assert_array_equal(log.x, log.states)
    assert log.t_pred.shape == (4, 1)
    assert log.u_pred.shape == (4, 1, 1)
    np.testing.assert_array_equal(log.u, log.inputs)


def test_load_log_control_points_and_no_input_cost(tmp_path):
    _write_log(tmp_path / "cp", log_control_points=True)
    log = ampc.load_log(tmp_path / "cp")
    assert log.log_control_points
    assert log.inputs.shape == (4, 4, 1)
    assert log.ref_inputs.shape == (4, 4, 1)

    _write_log(tmp_path / "off", input_cost=ampc.InputCostMode.Off)
    log = ampc.load_log(tmp_path / "off")
    assert log.ref_inputs is None and log.u_ref_pred is None and log.u_ref is None


def _assert_sampled_inputs_match(dense, cp):
    # the same runs logged both ways give the same sampled inputs
    np.testing.assert_array_equal(dense.x_pred, cp.x_pred)
    np.testing.assert_allclose(cp.u_pred, dense.u_pred, rtol=0, atol=1e-12)
    np.testing.assert_allclose(cp.u, dense.u, rtol=0, atol=1e-12)
    if dense.u_ref_pred is None:
        assert cp.u_ref_pred is None
    else:
        np.testing.assert_allclose(cp.u_ref_pred, dense.u_ref_pred, rtol=0, atol=1e-12)


@pytest.mark.parametrize("stride", [0, 1, 3])
@pytest.mark.parametrize(
    "input_cost",
    [
        ampc.InputCostMode.ControlPoints,
        ampc.InputCostMode.Trajectory,
        ampc.InputCostMode.Off,
    ],
)
def test_control_points_give_the_same_sampled_inputs(tmp_path, stride, input_cost):
    # stride 3 over T = 10 ends with the terminal step, which repeats u_9
    for name, cp in (("dense", False), ("cp", True)):
        _write_log(
            tmp_path / name,
            prediction_stride=stride,
            log_control_points=cp,
            input_cost=input_cost,
        )
    dense, cp = ampc.load_log(tmp_path / "dense"), ampc.load_log(tmp_path / "cp")
    assert cp.inputs.shape == (4, 4, 1)
    _assert_sampled_inputs_match(dense, cp)


def test_control_points_give_the_same_sampled_inputs_with_a_custom_basis(tmp_path):
    T, nc = 6, 3
    basis = np.random.default_rng(0).standard_normal((T, nc))
    for name, cp in (("dense", False), ("cp", True)):
        mpc = ampc.MPC(2, 1, ampc.Parameterization.fromBasis(T, basis))
        mpc.setModelContinuous2Discrete(
            np.array([[0.0, 1.0], [-1.0, -0.1]]), np.array([0.0, 1.0]), np.zeros(2), 0.1
        )
        mpc.setInputLimits(np.array([-1.0]), np.array([1.0]))
        mpc.setStateWeights(np.array([1.0, 0.1]))
        mpc.setReferenceState(np.array([0.5, 0.0]))
        assert mpc.initializeSolver()
        logger = ampc.MPCLogger(
            mpc, tmp_path / name, 0.1, prediction_stride=4, log_control_points=cp
        )
        x = np.array([1.0, 0.0])
        for k in range(3):
            assert mpc.solve(x) == ampc.SolveStatus.Success
            logger.logStep(0.1 * k, x)
        logger.finalize()
    dense, cp = ampc.load_log(tmp_path / "dense"), ampc.load_log(tmp_path / "cp")
    np.testing.assert_array_equal(cp.basis, basis)
    _assert_sampled_inputs_match(dense, cp)


def test_bspline_logs_its_sampled_basis(tmp_path):
    _write_log(tmp_path)
    log = ampc.load_log(tmp_path)
    param = ampc.Parameterization.bspline(10, 2, 4)
    expected = np.zeros((10, 4))
    for k in range(10):
        start = param.band_start[k]
        expected[k, start : start + param.band_width] = param.band_weights[:, k]
    np.testing.assert_array_equal(log.basis, expected)


@pytest.mark.parametrize(
    "mode", [ampc.MPCLogger.Mode.Npy, ampc.MPCLogger.Mode.RawRecoverable]
)
def test_load_log_mmap(tmp_path, mode):
    _write_log(tmp_path, mode)
    log = ampc.load_log(tmp_path, mmap=True)
    assert isinstance(log.states, np.memmap)
    np.testing.assert_array_equal(log.states, ampc.load_log(tmp_path).states)


def test_load_log_finds_artifacts(tmp_path):
    with pytest.raises(FileNotFoundError):
        ampc.load_log(tmp_path)

    _write_log(tmp_path)
    assert ampc.load_log(tmp_path / "log.npz").format == "npz"
    with pytest.raises(FileNotFoundError):
        ampc.load_log(tmp_path, name="other")

    _write_log(tmp_path, ampc.MPCLogger.Mode.Npy)
    with pytest.raises(ValueError, match="several logs"):
        ampc.load_log(tmp_path)
    assert ampc.load_log(tmp_path / "log_npy").format == "npy"


def test_load_log_recovers_a_log_that_was_never_finalized(tmp_path):
    # the process ends without finalize(), as in a crash: whatever the payload
    # streams flushed is on disk, with a different number of steps per file
    steps = 5000
    script = textwrap.dedent(f"""
        import os, sys
        import numpy as np
        sys.path.insert(0, {str(Path(__file__).parent)!r})
        import affine_mpc as ampc
        from test_log import _make_mpc

        mpc = _make_mpc()
        logger = ampc.MPCLogger(mpc, {str(tmp_path)!r}, 0.1, prediction_stride=5)
        x = np.array([1.0, 0.0])
        assert mpc.solve(x) == ampc.SolveStatus.Success
        for k in range({steps}):
            logger.logStep(0.1 * k, x)
        os._exit(0)
        """)
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)}
    subprocess.run([sys.executable, "-c", script], check=True, env=env)

    raw_dir = tmp_path / "log_raw"
    assert (raw_dir / "staging_info.yaml").exists()
    assert not (raw_dir / "data_info.yaml").exists()

    log = ampc.load_log(tmp_path)
    assert log.format == "raw" and not log.finalized
    N = log.num_steps
    assert 0 < N < steps
    np.testing.assert_allclose(log.time, 0.1 * np.arange(N))
    np.testing.assert_array_equal(log.x, np.tile([1.0, 0.0], (N, 1)))
    assert log.states.shape == (N, 3, 2)
    assert log.options.input_cost == ampc.InputCostMode.ControlPoints
    assert log.meta == {}

    # a partly written last step is dropped
    states_bin = raw_dir / "states.bin"
    row_bytes = 3 * 2 * 8
    with open(states_bin, "r+b") as f:
        f.truncate(N * row_bytes - 5)
    assert ampc.load_log(tmp_path).num_steps == N - 1


def test_raw_recoverable_finalize_replaces_staging_info(tmp_path):
    _write_log(tmp_path, ampc.MPCLogger.Mode.RawRecoverable)
    raw_dir = tmp_path / "log_raw"
    assert (raw_dir / "data_info.yaml").exists()
    assert not (raw_dir / "staging_info.yaml").exists()
