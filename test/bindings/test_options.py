import pytest

import affine_mpc as ampc


def test_options_interface():
    opts = ampc.Options()
    assert opts.state_bounds == ampc.StateBoundMode.Off
    assert repr(opts) == "Options()"
    opts = ampc.Options(use_input_cost=False)
    opts = ampc.Options(use_input_rate_cost=False)
    opts = ampc.Options(slew_initial_input=False)
    opts = ampc.Options(slew_control_points=False)
    opts = ampc.Options(saturate_input_trajectory=False)
    opts = ampc.Options(state_bounds=ampc.StateBoundMode.Off)
    opts = ampc.Options(bounded_states=[])
    opts = ampc.Options(num_general_constraints=0)
    opts = ampc.Options(num_terminal_constraints=0)


def test_options_state_bounds():
    opts = ampc.Options(state_bounds=ampc.StateBoundMode.Soft, bounded_states=[1])
    assert opts.state_bounds == ampc.StateBoundMode.Soft
    assert repr(opts) == "Options(state_bounds=StateBoundMode.Soft, bounded_states=[1])"
    assert "state_bounds=Soft" in str(opts)
    opts.state_bounds = ampc.StateBoundMode.Hard
    assert opts.state_bounds == ampc.StateBoundMode.Hard
    assert list(ampc.StateBoundMode) == [
        ampc.StateBoundMode.Off,
        ampc.StateBoundMode.Hard,
        ampc.StateBoundMode.Soft,
    ]


def test_options_are_keyword_only():
    with pytest.raises(TypeError):
        ampc.Options(True)


def test_options_reject_removed_saturate_states():
    with pytest.raises(TypeError, match="saturate_states"):
        ampc.Options(saturate_states=True)
    with pytest.raises(AttributeError):
        ampc.Options().saturate_states = True
