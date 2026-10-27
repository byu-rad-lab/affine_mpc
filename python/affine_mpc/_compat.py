"""
Deprecated class names, removed in 0.5.0: use MPC with a Formulation.
"""

import warnings

from ._bindings import Formulation, MPC


class CondensedMPC(MPC):
    """MPC with Formulation.Condensed.

    Deprecated: use MPC, whose formulation defaults to Formulation.Condensed.
    CondensedMPC will be removed in 0.5.0.
    """

    def __init__(self, *args, **kwargs):
        warnings.warn(
            "CondensedMPC is deprecated and will be removed in 0.5.0; use MPC "
            "(formulation=Formulation.Condensed is the default)",
            DeprecationWarning,
            stacklevel=2,
        )
        super().__init__(*args, formulation=Formulation.Condensed, **kwargs)


class SparseMPC(MPC):
    """MPC with Formulation.Sparse.

    Deprecated: use MPC with formulation=Formulation.Sparse. SparseMPC will be
    removed in 0.5.0.
    """

    def __init__(self, *args, **kwargs):
        warnings.warn(
            "SparseMPC is deprecated and will be removed in 0.5.0; use MPC "
            "with formulation=Formulation.Sparse",
            DeprecationWarning,
            stacklevel=2,
        )
        super().__init__(*args, formulation=Formulation.Sparse, **kwargs)
