import pytest

import affine_mpc as ampc


def test_options_interface():
    opts = ampc.Options()
    assert opts.state_bounds == ampc.StateBoundMode.Off
    assert repr(opts) == "Options()"
    opts = ampc.Options(input_cost=ampc.InputCostMode.Off)
    opts = ampc.Options(use_input_rate_cost=False)
    opts = ampc.Options(slew_initial_input=False)
    opts = ampc.Options(slew_control_points=False)
    opts = ampc.Options(input_bounds=ampc.InputBoundMode.ControlPoints)
    opts = ampc.Options(refined_bound_interval=16)
    opts = ampc.Options(state_bounds=ampc.StateBoundMode.Off)
    opts = ampc.Options(bounded_states=[])
    opts = ampc.Options(num_general_constraints=0)
    opts = ampc.Options(num_terminal_constraints=0)


def test_options_bound_modes():
    opts = ampc.Options(state_bounds=ampc.StateBoundMode.Soft, bounded_states=[1])
    assert opts.state_bounds == ampc.StateBoundMode.Soft
    assert repr(opts) == "Options(state_bounds=StateBoundMode.Soft, bounded_states=[1])"
    opts.input_bounds = ampc.InputBoundMode.Exact
    assert opts.input_bounds == ampc.InputBoundMode.Exact
    assert repr(opts) == (
        "Options(input_bounds=InputBoundMode.Exact, state_bounds=StateBoundMode.Soft, "
        "bounded_states=[1])"
    )
    assert "input_bounds=Exact" in str(opts)
    assert "state_bounds=Soft" in str(opts)
    opts.state_bounds = ampc.StateBoundMode.Hard
    assert opts.state_bounds == ampc.StateBoundMode.Hard
    assert list(ampc.InputBoundMode) == [
        ampc.InputBoundMode.ControlPoints,
        ampc.InputBoundMode.Exact,
        ampc.InputBoundMode.Refined,
    ]
    assert list(ampc.StateBoundMode) == [
        ampc.StateBoundMode.Off,
        ampc.StateBoundMode.Hard,
        ampc.StateBoundMode.Soft,
    ]


def test_options_refined_bounds():
    opts = ampc.Options(input_bounds=ampc.InputBoundMode.Refined)
    assert opts.refined_bound_interval == 16
    assert repr(opts) == "Options(input_bounds=InputBoundMode.Refined)"
    opts.refined_bound_interval = 3
    assert opts.refined_bound_interval == 3
    assert repr(opts) == (
        "Options(input_bounds=InputBoundMode.Refined, refined_bound_interval=3)"
    )
    assert "input_bounds=Refined, refined_bound_interval=3," in str(opts)


def test_options_input_cost():
    opts = ampc.Options()
    assert opts.input_cost == ampc.InputCostMode.Off
    opts = ampc.Options(input_cost=ampc.InputCostMode.Trajectory)
    assert opts.input_cost == ampc.InputCostMode.Trajectory
    assert repr(opts) == "Options(input_cost=InputCostMode.Trajectory)"
    assert str(opts).startswith("Options(input_cost=Trajectory, use_input_rate_cost=False,")
    opts.input_cost = ampc.InputCostMode.ControlPoints
    opts.use_input_rate_cost = True
    assert repr(opts) == (
        "Options(input_cost=InputCostMode.ControlPoints, use_input_rate_cost=True)"
    )
    assert list(ampc.InputCostMode) == [
        ampc.InputCostMode.Off,
        ampc.InputCostMode.ControlPoints,
        ampc.InputCostMode.Trajectory,
    ]


def test_options_are_keyword_only():
    with pytest.raises(TypeError):
        ampc.Options(True)


@pytest.mark.parametrize(
    "name", ["saturate_states", "saturate_input_trajectory", "use_input_cost"]
)
def test_options_reject_removed_bools(name):
    with pytest.raises(TypeError, match=name):
        ampc.Options(**{name: True})
    with pytest.raises(AttributeError):
        setattr(ampc.Options(), name, True)
