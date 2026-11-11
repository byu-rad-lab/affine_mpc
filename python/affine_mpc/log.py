"""
Load logs written by MPCLogger.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np
from numpy.typing import NDArray

from ._bindings import (
    Formulation,
    InputBoundMode,
    InputCostMode,
    MPCLogger,
    Options,
    Parameterization,
    SlewBoundMode,
    StateBoundMode,
)

__all__ = ["MPCLog", "load_log"]

_ROW_BYTES = np.dtype("<f8").itemsize
# the log_format_version this reader understands
_FORMAT_VERSION = 2

ArrayF = NDArray[np.float64]


@dataclass(frozen=True, kw_only=True)
class MPCLog:
    """One log written by MPCLogger.

    The per-step arrays are stored as MPCLogger writes them, with `N` logged
    steps, `K` logged prediction samples, `n` states, `m` inputs, and `nc`
    control points; the properties give the same data in fixed shapes. Every
    value the logger records about itself and the MPC is an attribute, and
    `meta` holds only the metadata added with `MPCLogger.addMetadata()`.
    Attributes for values that depend on an option are None when the option
    leaves them out.
    """

    # per-step data
    time: ArrayF
    """(N,) time of each logged solve"""
    loop_time: ArrayF
    """(N,) user-measured time passed to logStep(), -1 where not provided"""
    mpc_solve_time: ArrayF
    """(N,) duration of each solve() call"""
    qp_solve_time: ArrayF
    """(N,) time the QP solver spent solving"""
    prepare_time: ArrayF
    """(N,) time of the prepare() calls before each solve"""
    states: ArrayF
    """(N, K, n), or (N, n) with prediction_stride 0; starts at x0"""
    ref_states: ArrayF
    """same shape as states"""
    inputs: ArrayF
    """(N, nc, m) with log_control_points, else (N, K, m) or (N, m); u_pred
    gives the sampled inputs in either case"""
    ref_inputs: ArrayF | None = None
    """logged with an input cost (see the logging docs for its shape); u_ref_pred
    gives the sampled reference in every case"""

    # logger settings
    ts: float
    """time step used to align the predicted samples"""
    prediction_stride: int
    log_control_points: bool
    """whether inputs holds control points rather than sampled inputs"""
    logging_mode: MPCLogger.Mode

    # MPC
    state_dim: int
    input_dim: int
    horizon_steps: int
    num_control_points: int
    formulation: Formulation
    options: Options
    parameterization_type: Parameterization.Type
    spline_degree: int | None = None
    """B-spline parameterizations only"""
    knots: ArrayF | None = None
    """B-spline parameterizations only"""
    basis: ArrayF
    """(horizon_steps, num_control_points) sampled basis: the input at step k is
    basis[k] @ control_points"""
    u_min: ArrayF
    u_max: ArrayF
    Q_diag: ArrayF
    Qf_diag: ArrayF
    R_diag: ArrayF | None = None
    """with an input cost"""
    R_rate_diag: ArrayF | None = None
    """with options.use_input_rate_cost"""
    u0_slew: ArrayF | None = None
    """with options.slew_initial_input"""
    slew_rate: ArrayF | None = None
    """per sample step, with options.slew_bounds other than Off"""
    x_min: ArrayF | None = None
    """with state bounds, one entry per state in options.bounded_states"""
    x_max: ArrayF | None = None
    """with state bounds, one entry per state in options.bounded_states"""
    x_soft_l1: ArrayF | None = None
    """with soft state bounds"""
    x_soft_l2: ArrayF | None = None
    """with soft state bounds"""
    Hx: ArrayF | None = None
    """(num_general_constraints, n), with general constraints"""
    Hu: ArrayF | None = None
    """(num_general_constraints, m), with general constraints"""
    h_min: ArrayF | None = None
    h_max: ArrayF | None = None
    Hf: ArrayF | None = None
    """(num_terminal_constraints, n), with terminal constraints"""
    hf_min: ArrayF | None = None
    hf_max: ArrayF | None = None

    # user metadata and source
    meta: dict[str, Any]
    """metadata added with addMetadata(): int, float, str, or float64 array"""
    log_format_version: int
    path: Path
    """the artifact that was read: the .npz file, or the _npy or _raw directory"""
    format: str
    """the format that was read: npz, npy, or raw"""
    finalized: bool
    """False for raw payloads of a logger that was never finalized; those
    keep only the metadata known at construction, and a partly written last
    step is dropped"""

    @property
    def num_steps(self) -> int:
        """Number of logged steps N."""
        return len(self.time)

    def _pred_steps(self) -> NDArray[np.int64]:
        # the prediction step k of each logged sample, as the logger strides them
        if self.prediction_stride <= 0:
            return np.zeros(1, dtype=np.int64)
        steps = np.arange(0, self.horizon_steps + 1, self.prediction_stride)
        if steps[-1] != self.horizon_steps:
            steps = np.append(steps, self.horizon_steps)
        return steps

    @property
    def t_pred(self) -> ArrayF:
        """(N, K) time of each logged prediction sample, aligned with x_pred and u_pred."""
        return self.time[:, None] + self._pred_steps() * self.ts

    @property
    def x(self) -> ArrayF:
        """(N, n) state passed to logStep() at each step (a view of states)."""
        return self.x_pred[:, 0]

    @property
    def x_ref(self) -> ArrayF:
        """(N, n) reference state at each step (a view of ref_states)."""
        return self.x_ref_pred[:, 0]

    @property
    def x_pred(self) -> ArrayF:
        """(N, K, n) logged states, starting at x0 (a view of states)."""
        return _with_k_axis(self.states)

    @property
    def x_ref_pred(self) -> ArrayF:
        """(N, K, n) reference states at the logged prediction steps (a view of ref_states)."""
        return _with_k_axis(self.ref_states)

    @property
    def u(self) -> ArrayF:
        """(N, m) input applied at each step: the first sample of u_pred."""
        return self.u_pred[:, 0]

    @property
    def u_ref(self) -> ArrayF | None:
        """(N, m) reference input at each step, or None without an input cost."""
        u_ref_pred = self.u_ref_pred
        return None if u_ref_pred is None else u_ref_pred[:, 0]

    @property
    def u_pred(self) -> ArrayF:
        """(N, K, m) sampled inputs at the logged prediction steps, evaluated from
        the control points with log_control_points (the terminal step repeats the
        last input, since the horizon has one input fewer than states)."""
        if self.log_control_points:
            return self._evaluate(self.inputs)
        return _with_k_axis(self.inputs)

    @property
    def u_ref_pred(self) -> ArrayF | None:
        """(N, K, m) reference inputs at the logged prediction steps, or None
        without an input cost."""
        if self.ref_inputs is None:
            return None
        if (
            self.log_control_points
            and self.options.input_cost == InputCostMode.ControlPoints
        ):
            return self._evaluate(self.ref_inputs)
        return _with_k_axis(self.ref_inputs)

    def _evaluate(self, control_points: ArrayF) -> ArrayF:
        rows = self.basis[np.minimum(self._pred_steps(), self.horizon_steps - 1)]
        return np.einsum("kc,ncm->nkm", rows, control_points)


def _with_k_axis(values: ArrayF) -> ArrayF:
    return values if values.ndim == 3 else values[:, None]


def load_log(
    path: str | os.PathLike[str], name: str = "log", *, mmap: bool = False
) -> MPCLog:
    """Load a log written by MPCLogger in any mode.

    Args:
        path: The logger's save_dir, or one of its artifacts:
            `<save_dir>/<name>.npz` (NpzCompressed, NpzUncompressed),
            `<save_dir>/<name>_npy/` (Npy, and the fallback when an NPZ
            exceeds the ZIP32 limits), or `<save_dir>/<name>_raw/`
            (RawRecoverable, or the payloads staged by a logger that was never
            finalized). Given save_dir, exactly one of these must exist.
        name: The logger's save_name, used when path is save_dir.
        mmap: Memory-map .npy and raw payloads instead of reading them into
            memory (ignored for .npz).

    Raises:
        FileNotFoundError: if no artifact is found.
        ValueError: if save_dir holds more than one artifact for name, the
            files are not a valid log, or the log's log_format_version is not
            the one this affine_mpc reads.
    """
    path = Path(path)
    if path.is_dir() and not path.name.endswith(("_npy", "_raw")):
        found = [
            p
            for p in (path / f"{name}.npz", path / f"{name}_npy", path / f"{name}_raw")
            if p.exists()
        ]
        if not found:
            raise FileNotFoundError(f"No log named '{name}' in {path}")
        if len(found) > 1:
            listed = ", ".join(p.name for p in found)
            raise ValueError(
                f"Found several logs named '{name}' in {path} ({listed}); pass one of them as path"
            )
        path = found[0]

    if path.suffix == ".npz" and path.is_file():
        arrays, meta = _read_npz(path)
        return _make_log(arrays, meta, path, "npz", True)
    if path.name.endswith("_npy") and path.is_dir():
        mmap_mode = "r" if mmap else None
        arrays = {
            p.stem: np.load(p, mmap_mode=mmap_mode, allow_pickle=False)
            for p in path.glob("*.npy")
        }
        return _make_log(arrays, _read_meta_npz(path), path, "npy", True)
    if path.name.endswith("_raw") and path.is_dir():
        arrays, finalized = _read_raw(path, mmap)
        return _make_log(arrays, _read_meta_npz(path), path, "raw", finalized)
    raise FileNotFoundError(f"Not an MPCLogger log: {path}")


_DATA_FIELDS = (
    "time",
    "loop_time",
    "mpc_solve_time",
    "qp_solve_time",
    "prepare_time",
    "states",
    "ref_states",
    "inputs",
)


def _pascal(name: str) -> str:
    return "".join(word.capitalize() for word in name.split("_"))


def _vector(value: np.ndarray) -> ArrayF:
    return np.asarray(value, dtype=np.float64)


# converters for the built-in metadata that becomes an attribute of the same
# name; the opt_* keys and bounded_states become MPCLog.options, and the
# _UNUSED keys are for readers without this module
_BUILTIN: dict[str, Callable[[np.ndarray], Any]] = {
    "log_format_version": int,
    "ts": float,
    "prediction_stride": int,
    "log_control_points": bool,
    "logging_mode": lambda v: MPCLogger.Mode.__members__[str(v)],
    "state_dim": int,
    "input_dim": int,
    "horizon_steps": int,
    "num_control_points": int,
    "formulation": lambda v: Formulation.__members__[_pascal(str(v))],
    "parameterization_type": lambda v: {
        "bspline": Parameterization.Type.BSpline,
        "custom": Parameterization.Type.Custom,
    }[str(v)],
    "spline_degree": int,
    "knots": _vector,
    "basis": _vector,
    **{
        key: _vector
        for key in (
            "u_min",
            "u_max",
            "Q_diag",
            "Qf_diag",
            "R_diag",
            "R_rate_diag",
            "u0_slew",
            "slew_rate",
            "x_min",
            "x_max",
            "x_soft_l1",
            "x_soft_l2",
            "Hx",
            "Hu",
            "h_min",
            "h_max",
            "Hf",
            "hf_min",
            "hf_max",
        )
    },
}
_OPTIONS: dict[str, tuple[str, Callable[[np.ndarray], Any]]] = {
    "opt_input_cost": (
        "input_cost",
        lambda v: InputCostMode.__members__[_pascal(str(v))],
    ),
    "opt_use_input_rate_cost": ("use_input_rate_cost", bool),
    "opt_input_bounds": (
        "input_bounds",
        lambda v: InputBoundMode.__members__[_pascal(str(v))],
    ),
    "opt_min_refined_intervals": ("min_refined_intervals", int),
    "opt_slew_initial_input": ("slew_initial_input", bool),
    "opt_slew_bounds": (
        "slew_bounds",
        lambda v: SlewBoundMode.__members__[_pascal(str(v))],
    ),
    "opt_state_bounds": (
        "state_bounds",
        lambda v: StateBoundMode.__members__[_pascal(str(v))],
    ),
    "bounded_states": ("bounded_states", lambda v: [int(i) for i in v]),
    "opt_num_general_constraints": ("num_general_constraints", int),
    "opt_num_terminal_constraints": ("num_terminal_constraints", int),
}
_UNUSED = frozenset({"t_pred_offsets"})
BUILTIN_KEYS = frozenset(_BUILTIN) | frozenset(_OPTIONS) | _UNUSED
"""Metadata keys MPCLogger writes itself (addMetadata() rejects them)."""


def _make_log(
    arrays: dict[str, np.ndarray],
    raw_meta: dict[str, np.ndarray],
    path: Path,
    fmt: str,
    finalized: bool,
) -> MPCLog:
    missing = [f for f in _DATA_FIELDS if f not in arrays]
    if missing:
        raise ValueError(f"{path} is missing {', '.join(missing)}")
    if "log_format_version" not in raw_meta:
        raise ValueError(
            f"{path} has no log_format_version: it was written by an earlier "
            "affine_mpc, so read it with numpy.load()"
        )
    version = int(raw_meta["log_format_version"])
    if version != _FORMAT_VERSION:
        raise ValueError(
            f"{path} has log_format_version {version}, but this affine_mpc "
            f"reads version {_FORMAT_VERSION}"
        )

    fields: dict[str, Any] = {f: arrays[f] for f in _DATA_FIELDS}
    fields["ref_inputs"] = arrays.get("ref_inputs")
    options: dict[str, Any] = {}
    meta: dict[str, Any] = {}
    for key, value in raw_meta.items():
        if key in _BUILTIN:
            fields[key] = _BUILTIN[key](value)
        elif key in _OPTIONS:
            option, convert = _OPTIONS[key]
            options[option] = convert(value)
        elif key in _UNUSED:
            pass
        else:
            meta[key] = value.item() if value.ndim == 0 else value

    # matrices are logged row by row
    n, m = fields["state_dim"], fields["input_dim"]
    for key, cols in (("Hx", n), ("Hu", m), ("Hf", n)):
        if key in fields:
            fields[key] = fields[key].reshape(-1, cols)
    fields["basis"] = fields["basis"].reshape(-1, fields["num_control_points"])

    return MPCLog(
        **fields,
        options=Options(**options),
        meta=meta,
        path=path,
        format=fmt,
        finalized=finalized,
    )


def _read_npz(path: Path) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    arrays, meta = {}, {}
    with np.load(path, allow_pickle=False) as data:
        for key in data.files:
            if key.startswith("meta_"):
                meta[key[len("meta_") :]] = data[key]
            else:
                arrays[key] = data[key]
    return arrays, meta


def _read_meta_npz(log_dir: Path) -> dict[str, np.ndarray]:
    meta_path = log_dir / "meta.npz"
    if not meta_path.exists():
        raise ValueError(
            f"{log_dir} has no meta.npz: it was written by an earlier "
            "affine_mpc, so read it with numpy.load()"
        )
    return _read_npz(meta_path)[1]


def _read_raw(raw_dir: Path, mmap: bool) -> tuple[dict[str, np.ndarray], bool]:
    """Payloads of a finalized raw log (data_info.yaml written) have NPY headers;
    those of a logger that was never finalized have their row shapes in
    staging_info.yaml, and the step count follows from the file sizes."""
    finalized = (raw_dir / "data_info.yaml").exists()
    shapes = {}
    if finalized:
        for header in raw_dir.glob("*.npyh"):
            with open(header, "rb") as f:
                version = np.lib.format.read_magic(f)
                if version == (1, 0):
                    shape, fortran_order, dtype = np.lib.format.read_array_header_1_0(f)
                else:
                    shape, fortran_order, dtype = np.lib.format.read_array_header_2_0(f)
            if fortran_order or dtype != np.dtype("<f8"):
                raise ValueError(
                    f"{header} must describe little-endian float64 in C order"
                )
            shapes[header.stem] = tuple(shape)
    else:
        staging_info = raw_dir / "staging_info.yaml"
        if not staging_info.exists():
            raise ValueError(
                f"{raw_dir} has neither data_info.yaml nor staging_info.yaml"
            )
        rows = _read_row_shapes(staging_info)
        sizes = {key: (raw_dir / f"{key}.bin").stat().st_size for key in rows}
        num_steps = min(
            sizes[key] // (_ROW_BYTES * int(np.prod(row))) for key, row in rows.items()
        )
        shapes = {key: (num_steps, *row) for key, row in rows.items()}

    arrays = {}
    for key, shape in shapes.items():
        data_file = raw_dir / f"{key}.bin"
        count = int(np.prod(shape))
        if data_file.stat().st_size < count * _ROW_BYTES:
            raise ValueError(f"{data_file} is shorter than its shape {shape}")
        if mmap and count > 0:
            arrays[key] = np.memmap(data_file, dtype="<f8", mode="r", shape=shape)
        else:
            arrays[key] = np.fromfile(data_file, dtype="<f8", count=count).reshape(
                shape
            )
    return arrays, finalized


def _read_row_shapes(path: Path) -> dict[str, tuple[int, ...]]:
    rows, in_rows = {}, False
    for line in path.read_text().splitlines():
        if line.startswith("row_shapes:"):
            in_rows = True
        elif in_rows and line.startswith("  "):
            key, value = line.strip().split(": ", 1)
            rows[key] = tuple(int(v) for v in value.strip("[]").split(",") if v.strip())
        else:
            in_rows = False
    return rows
