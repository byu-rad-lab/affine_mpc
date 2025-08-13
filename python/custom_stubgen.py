import argparse
import pybind11_stubgen
import re
import black
from pathlib import Path


script_dir = Path(__file__).parent.resolve()
parser = argparse.ArgumentParser(description="")
parser.add_argument(
    "dir",
    nargs="?",
    type=str,
    default=dir,
    help="Directory in which to create generated 'stubs' dir",
)
args = parser.parse_args()
stub_dir = f"{args.dir}/stubs"
print("Stub Dir:", stub_dir)

# pybind11_stubgen.main(["-o", "stubs", "affine_mpc_py"])
#
# stub_file = "./stubs/affine_mpc_py/_bindings.pyi"
# pattern = re.compile(r"numpy\.ndarray\[numpy\.float64[^\]]\]\]")
# replacement = "NDArray[np.float64]"
#
# with open(stub_file, "r", encoding="utf-8") as f:
#     content = f.read()
# content = re.sub(
#     r"import numpy",
#     "import numpy as np\nfrom numpy.typing import NDArray",
#     content,
#     count=1,
# )
# content = re.sub(r",\snumpy\.ndarray\.flags\.writeable", "", content)
# content = re.sub(r"numpy\.ndarray", "NDArray", content)
# content = re.sub(r"numpy\.float64\[[^\]]*\]", "np.float64", content)
# with open(stub_file, "w", encoding="utf-8") as f:
#     f.write(content)

########

pybind11_stubgen.main(["-o", stub_dir, "affine_mpc_py", "--numpy-array-use-type-var"])

stub_file = f"{stub_dir}/affine_mpc_py/_bindings.pyi"

with open(stub_file, "r", encoding="utf-8") as f:
    content = f.read()
content = re.sub(r"from __future__ import annotations", "", content)
content = re.sub(
    r"import numpy",
    "import numpy as np\nfrom numpy.typing import NDArray",
    content,
    count=1,
)
content = re.sub(r"M = .*\nN = .*", "", content)
content = re.sub(
    r"tuple\[M,[^,]*, numpy\.dtype\[numpy\.float64\]", "np.float64", content
)
content = re.sub(r"numpy\.ndarray", "NDArray", content)
content = re.sub(r"OSQPSettings = None", "OSQPSettings|None = None", content)

try:
    formatted = black.format_file_contents(
        content, fast=True, mode=black.FileMode(is_pyi=True)
    )
except:
    formatted = content

with open(stub_file, "w", encoding="utf-8") as f:
    f.write(formatted)
