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
    Parameterization,
    OSQPSettings,
    SolveStatus,
    MPCBase,
    CondensedMPC,
    SparseMPC,
    MPCLogger,
)

__all__ = [
    "Options",
    "Parameterization",
    "OSQPSettings",
    "SolveStatus",
    "MPCBase",
    "CondensedMPC",
    "SparseMPC",
    "MPCLogger",
]
