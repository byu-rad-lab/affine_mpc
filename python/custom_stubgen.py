import argparse
from pathlib import Path
import pybind11_stubgen
import re
import sys
import shutil
import tempfile

sys.path = sys.path[1:]

debug = False

if debug:
    print("exe:", sys.executable)
    print("path:", sys.path)
    print()

parser = argparse.ArgumentParser(description="")
parser.add_argument(
    "--pkg-dir",
    type=str,
    help="Directory in which to load affine_mpc",
)
args = parser.parse_args()

if args.pkg_dir is not None:
    pkg_dir = Path(args.pkg_dir).resolve()
    if not pkg_dir.exists():
        print("Could not find ", pkg_dir)
    else:
        print(pkg_dir)
        sys.path.insert(0, str(pkg_dir))

try:
    import affine_mpc as ampc
except ImportError:
    print(
        "Could not find `affine_mpc`.",
        "Make sure it is installed and that --pkg-dir is correct.\n",
    )
    print(
        "Before trying to generate the stub file either:",
        '    pip install ".[dev]"',
        "    python python/custom_stubgen.py",
        "  OR:",
        "    pip install -r requirements/build.txt",
        "    cmake -S . -B build -AFFINE_MPC_BUILD_BINDINGS=ON",
        "    cmake --build build --config Release --parallel",
        "    cmake --install build --prefix install --component python_bindings",
        "    python python/custom_stubgen.py --pkg-dir=install",
        sep="\n",
    )
    exit(1)

pkg = "affine_mpc"
out_dir = Path(tempfile.gettempdir()).resolve()
stub_dir = out_dir / "stubs"


pybind11_stubgen.main(
    [
        "-o",
        stub_dir.as_posix(),
        pkg,
        "--numpy-array-use-type-var",
        # "--numpy-array-wrap-with-annotated",
    ]
)

stub_file = stub_dir / pkg / "_bindings.pyi"
repo_stub_file = Path(__file__).parent / pkg / "_bindings.pyi"

if debug:
    print("exists:", stub_dir.exists())
    print("out_dir:", stub_dir.exists())
    for entry in stub_dir.iterdir():
        print(entry.name)
    print()

    print(pkg)
    for entry in stub_file.parent.iterdir():
        print(entry.name)
    print()

    print("stub_file:", stub_file)
    print("exists:", stub_file.exists())

with open(stub_file, "r", encoding="utf-8") as f:
    content = f.read()
content = re.sub(r"from __future__ import annotations", "", content)
# content = re.sub(
#     r"import numpy",
#     "import numpy as np\nfrom numpy.typing import NDArray",
#     content,
#     count=1,
# )
content = re.sub(
    r"import typing",
    "import typing\nimport os",
    content,
    count=1,
)
content = re.sub("PathLike ", "PathLike[str] ", content)
# content = re.sub("typing.SupportsInt . typing.SupportsIndex", "int", content)
content = re.sub(" . typing.SupportsIndex", "", content)
content = re.sub("typing.SupportsInt", "int", content)
content = re.sub("typing.SupportsFloat", "float", content)
content = re.sub(r"M = .*\nN = .*", "", content)
content = re.sub(
    r"tuple\[M,[^,]*, numpy\.dtype\[numpy\.float64\]", "numpy.float64", content
)
# content = re.sub(r"numpy\.ndarray", "NDArray", content)
# content = re.sub(r" = \.\.\.", " = numpy.empty(0)", content)
content = re.sub(r"Options = \.\.\.", "Options = Options()", content)
content = re.sub(r"OSQPSettings = \.\.\.", "OSQPSettings = OSQPSettings()", content)

try:
    import black

    formatted = black.format_file_contents(
        content, fast=True, mode=black.FileMode(is_pyi=True)
    )
except ImportError:
    print("black not found!")
    formatted = content

## put stub file in build folder and install it
# with open(stub_file, "w", encoding="utf-8") as f:
#     f.write(formatted)

## put stub file in repo and commit it
with open(repo_stub_file, "w", encoding="utf-8") as f:
    f.write(formatted)
print("Stub file written to:", repo_stub_file)

shutil.rmtree(stub_dir)
