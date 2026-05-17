import os
import sys
import tempfile
import zipfile
from pathlib import Path

import numpy as np


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _candidate_python_roots(root: Path) -> list[Path]:
    candidates = []

    override = os.environ.get("AFFINE_MPC_TEST_PY_ROOT")
    if override:
        candidates.append(Path(override))

    candidates.extend(
        [
            root / "install",
            root / "build" / "python",
            root / "out" / "Debug" / "python",
        ]
    )
    return candidates


def _has_importable_package(root: Path) -> bool:
    pkg_dir = root / "affine_mpc"
    if not (pkg_dir / "__init__.py").exists():
        return False

    bindings = list(pkg_dir.glob("_bindings*.so")) + list(pkg_dir.glob("_bindings*.pyd"))
    return len(bindings) > 0


def _find_repo_local_package_root(root: Path) -> Path:
    for candidate in _candidate_python_roots(root):
        if _has_importable_package(candidate):
            return candidate

    raise RuntimeError(
        "Could not locate a repo-local affine_mpc Python package. Expected one of:\n"
        "  install/affine_mpc\n"
        "  build/python/affine_mpc\n"
        "  out/Debug/python/affine_mpc\n"
        "Build bindings or install the python_bindings component first."
    )


def _can_import_installed_package() -> bool:
    try:
        import affine_mpc  # noqa: F401
    except ImportError:
        return False

    return True


def _infer_has_zlib(root: Path) -> str:
    override = os.environ.get("AFFINE_MPC_HAS_ZLIB")
    if override in {"0", "1"}:
        return override

    for build_dir in [root / "build", root / "out" / "Debug"]:
        cache = build_dir / "CMakeCache.txt"
        if not cache.exists():
            continue
        text = cache.read_text()
        if "ZLIB_LIBRARY_RELEASE:FILEPATH=" in text and "ZLIB_LIBRARY_RELEASE-NOTFOUND" not in text:
            return "1"
        if "ZLIB_FOUND:INTERNAL=TRUE" in text or "ZLIB_FOUND:BOOL=TRUE" in text:
            return "1"
        if "ZLIB_FOUND:INTERNAL=FALSE" in text or "ZLIB_FOUND:BOOL=FALSE" in text:
            return "0"

    return "0"


def _probe_installed_has_zlib() -> str:
    import affine_mpc as ampc

    opts = ampc.Options(use_input_cost=False)
    mpc = ampc.CondensedMPC(2, 1, ampc.Parameterization.linearInterp(5, 2), opts)

    A = np.array([[1.0, 0.1], [0.0, 1.0]])
    B = np.array([0.0, 0.1])
    w = np.zeros(2)
    ts = 0.1
    mpc.setModelContinuous2Discrete(A, B, w, ts)
    mpc.setInputLimits(np.array([-1.0]), np.array([1.0]))
    mpc.setStateWeights(np.ones(2))
    mpc.setReferenceState(np.zeros(2))
    assert mpc.initializeSolver()

    with tempfile.TemporaryDirectory() as tmpdir:
        logger = ampc.MPCLogger(
            mpc=mpc,
            save_dir=Path(tmpdir),
            ts=0.1,
            prediction_stride=1,
            log_control_points=False,
            save_name="zlib_probe",
            mode=ampc.MPCLogger.Mode.NpzCompressed,
        )
        x0 = np.array([1.0, 0.5])
        assert mpc.solve(x0) == ampc.SolveStatus.Success
        logger.logStep(t=0.0, x0=x0, solve_time=0.001)
        logger.finalize()

        with zipfile.ZipFile(Path(tmpdir) / "zlib_probe.npz") as archive:
            info = archive.getinfo("states.npy")
            return "1" if info.compress_type == zipfile.ZIP_DEFLATED else "0"


root = _repo_root()

if _can_import_installed_package():
    os.environ.setdefault("AFFINE_MPC_HAS_ZLIB", _probe_installed_has_zlib())
else:
    pkg_root = _find_repo_local_package_root(root)

    if str(pkg_root) not in sys.path:
        sys.path.insert(0, str(pkg_root))

    os.environ.setdefault("AFFINE_MPC_HAS_ZLIB", _infer_has_zlib(root))
