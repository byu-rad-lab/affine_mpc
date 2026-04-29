import os
from pathlib import Path


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _candidate_build_dirs(root: Path) -> list[Path]:
    return [
        root / "build",
        root / "out" / "Debug",
    ]


def _fixture_names() -> list[str]:
    if os.name == "nt":
        return ["npz_writer_fixture.exe"]
    return ["npz_writer_fixture"]


def _find_fixture(root: Path) -> Path:
    override = os.environ.get("AFFINE_MPC_NPZ_WRITER_FIXTURE")
    if override:
        path = Path(override)
        if path.exists():
            return path
        raise RuntimeError(
            f"AFFINE_MPC_NPZ_WRITER_FIXTURE is set but does not exist: {path}"
        )

    for build_dir in _candidate_build_dirs(root):
        for name in _fixture_names():
            candidate = build_dir / name
            if candidate.exists():
                return candidate

    raise RuntimeError(
        "Could not locate npz_writer_fixture. Build tests first so it exists under build/ or out/Debug/."
    )


def _infer_has_zlib(root: Path) -> str:
    override = os.environ.get("AFFINE_MPC_HAS_ZLIB")
    if override in {"0", "1"}:
        return override

    for build_dir in _candidate_build_dirs(root):
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


root = _repo_root()
os.environ.setdefault("AFFINE_MPC_NPZ_WRITER_FIXTURE", str(_find_fixture(root)))
os.environ.setdefault("AFFINE_MPC_HAS_ZLIB", _infer_has_zlib(root))
