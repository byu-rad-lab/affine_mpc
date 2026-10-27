"""
Affine MPC module.
"""

try:
    from ._version import __version__
except ModuleNotFoundError:
    print(
        "_version not found. Must install with pip to get _version.",
        "Setting __version__ to 0.0.0",
    )
    __version__ = "0.0.0"

from ._bindings import (
    Options,
    ModelSparsity,
    Parameterization,
    QPBlock,
    QuadraticProgram,
    Discretizer,
    OSQPSettings,
    SolveStatus,
    SolveInfo,
    Formulation,
    MPC,
    MPCLogger,
)
from ._compat import CondensedMPC, SparseMPC

MPCBase = MPC

__all__ = [
    "Options",
    "ModelSparsity",
    "Parameterization",
    "QPBlock",
    "QuadraticProgram",
    "Discretizer",
    "OSQPSettings",
    "SolveStatus",
    "SolveInfo",
    "Formulation",
    "MPC",
    "MPCLogger",
    "MPCBase",
    "CondensedMPC",
    "SparseMPC",
]
