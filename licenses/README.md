# Third-party licenses

affine_mpc itself is licensed under the BSD 3-Clause License (`../LICENSE`).
The Python wheels also contain compiled code from the projects below, so their license texts and notices are included here and in each wheel's `.dist-info/licenses/` directory.
The source distribution contains only affine_mpc's own source; these projects are downloaded when it is built.

| Component | Version | License | Files | Source |
| --- | --- | --- | --- | --- |
| OSQP | 1.0.0 | Apache-2.0 | `OSQP-LICENSE`, `OSQP-NOTICE` | <https://github.com/osqp/osqp> |
| QDLDL (part of OSQP) | 0.1.8 | Apache-2.0 | `QDLDL-LICENSE` | <https://github.com/osqp/qdldl> |
| AMD (SuiteSparse, part of OSQP) | as bundled with OSQP 1.0.0 | BSD-3-Clause | `AMD-LICENSE` | <https://github.com/DrTimothyAldenDavis/SuiteSparse> |
| DAQP | 0.10.3 | MIT | `DAQP-LICENSE` | <https://github.com/darnstrom/daqp> |
| Eigen (header-only, compiled into the wheel) | 3.4 or 5.0 | MPL-2.0 (some files BSD-3-Clause or Apache-2.0) | `EIGEN-COPYING.*` | <https://gitlab.com/libeigen/eigen> |
| pybind11 (headers compiled into the wheel) | 3.x | BSD-3-Clause | `PYBIND11-LICENSE` | <https://github.com/pybind/pybind11> |
| zlib (statically linked into the Windows wheels only) | from vcpkg | Zlib | `ZLIB-LICENSE` | <https://zlib.net> |
| GCC runtime libraries `libstdc++` and `libgcc_s` (copied into `affine_mpc.libs/` of the musllinux wheels only) | from the build image | GPL-3.0-or-later WITH GCC-exception-3.1 | `GCC-RUNTIME-COPYING3`, `GCC-RUNTIME-COPYING.RUNTIME` | <https://gcc.gnu.org> |
| Microsoft Visual C++ runtime `msvcp140.dll` (copied into `affine_mpc.libs/` of the Windows wheels only) | from the build image | Microsoft Visual Studio distributable code terms | none | <https://visualstudio.microsoft.com/license-terms/> |

The Eigen code in the wheels is unmodified; its source code is available at the address above, as the Mozilla Public License 2.0 requires for distribution in executable form.

When updating a component's version in `cmake/`, update its row and replace its license file if the text changed.
