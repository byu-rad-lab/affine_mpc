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
    StateBoundMode,
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

# Deprecated names, removed in 0.5.0: CondensedMPC and SparseMPC warn when
# constructed, MPCBase (see __getattr__) when accessed
from ._compat import CondensedMPC, SparseMPC

__all__ = [
    "Options",
    "StateBoundMode",
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
    "CondensedMPC",
    "SparseMPC",
]


def __getattr__(name):
    if name == "MPCBase":
        import warnings

        warnings.warn(
            "MPCBase is deprecated and will be removed in 0.5.0; use MPC",
            DeprecationWarning,
            stacklevel=2,
        )
        return MPC
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
