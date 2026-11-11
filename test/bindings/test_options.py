import pytest

import affine_mpc as ampc


def test_options_interface():
    opts = ampc.Options()
    assert opts.state_bounds == ampc.StateBoundMode.Off
    assert repr(opts) == "Options()"
    opts = ampc.Options(input_cost=ampc.InputCostMode.Off)
    opts = ampc.Options(use_input_rate_cost=False)
    opts = ampc.Options(slew_initial_input=False)
    opts = ampc.Options(slew_bounds=ampc.SlewBoundMode.Off)
    opts = ampc.Options(input_bounds=ampc.InputBoundMode.ControlPoints)
    opts = ampc.Options(min_refined_intervals=16)
    opts = ampc.Options(state_bounds=ampc.StateBoundMode.Off)
    opts = ampc.Options(bounded_states=[])
    opts = ampc.Options(num_general_constraints=0)
    opts = ampc.Options(num_terminal_constraints=0)


def test_options_bound_modes():
    opts = ampc.Options(state_bounds=ampc.StateBoundMode.Soft, bounded_states=[1])
    assert opts.state_bounds == ampc.StateBoundMode.Soft
    assert repr(opts) == "Options(state_bounds=StateBoundMode.Soft, bounded_states=[1])"
    opts.input_bounds = ampc.InputBoundMode.Trajectory
    assert opts.input_bounds == ampc.InputBoundMode.Trajectory
    assert repr(opts) == (
        "Options(input_bounds=InputBoundMode.Trajectory, state_bounds=StateBoundMode.Soft, "
        "bounded_states=[1])"
    )
    assert "input_bounds=Trajectory" in str(opts)
    assert "state_bounds=Soft" in str(opts)
    opts.state_bounds = ampc.StateBoundMode.Hard
    assert opts.state_bounds == ampc.StateBoundMode.Hard
    assert list(ampc.InputBoundMode) == [
        ampc.InputBoundMode.ControlPoints,
        ampc.InputBoundMode.Trajectory,
        ampc.InputBoundMode.Refined,
    ]


def test_options_slew_bounds():
    opts = ampc.Options(slew_bounds=ampc.SlewBoundMode.Trajectory)
    assert opts.slew_bounds == ampc.SlewBoundMode.Trajectory
    assert repr(opts) == "Options(slew_bounds=SlewBoundMode.Trajectory)"
    assert "slew_bounds=Trajectory" in str(opts)
    opts.slew_bounds = ampc.SlewBoundMode.ControlPoints
    assert repr(opts) == "Options(slew_bounds=SlewBoundMode.ControlPoints)"
    assert ampc.Options().slew_bounds == ampc.SlewBoundMode.Off
    assert "slew_bounds=Off" in str(ampc.Options())
    assert list(ampc.SlewBoundMode) == [
        ampc.SlewBoundMode.Off,
        ampc.SlewBoundMode.ControlPoints,
        ampc.SlewBoundMode.Trajectory,
    ]
    with pytest.raises(TypeError):
        ampc.Options(slew_control_points=True)
    assert list(ampc.StateBoundMode) == [
        ampc.StateBoundMode.Off,
        ampc.StateBoundMode.Hard,
        ampc.StateBoundMode.Soft,
    ]


def test_options_refined_bounds():
    opts = ampc.Options(input_bounds=ampc.InputBoundMode.Refined)
    assert opts.min_refined_intervals == 16
    assert repr(opts) == "Options(input_bounds=InputBoundMode.Refined)"
    opts.min_refined_intervals = 3
    assert opts.min_refined_intervals == 3
    assert repr(opts) == (
        "Options(input_bounds=InputBoundMode.Refined, min_refined_intervals=3)"
    )
    assert "input_bounds=Refined, min_refined_intervals=3," in str(opts)


def test_options_input_cost():
    opts = ampc.Options()
    assert opts.input_cost == ampc.InputCostMode.Off
    opts = ampc.Options(input_cost=ampc.InputCostMode.Trajectory)
    assert opts.input_cost == ampc.InputCostMode.Trajectory
    assert repr(opts) == "Options(input_cost=InputCostMode.Trajectory)"
    assert str(opts).startswith(
        "Options(input_cost=Trajectory, use_input_rate_cost=False,"
    )
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
