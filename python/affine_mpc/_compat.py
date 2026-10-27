"""
Former class names, kept for one minor release: use MPC with a Formulation.
"""

from ._bindings import Formulation, MPC


class CondensedMPC(MPC):
    """MPC with Formulation.Condensed; use MPC instead."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, formulation=Formulation.Condensed, **kwargs)


class SparseMPC(MPC):
    """MPC with Formulation.Sparse; use MPC instead."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, formulation=Formulation.Sparse, **kwargs)
