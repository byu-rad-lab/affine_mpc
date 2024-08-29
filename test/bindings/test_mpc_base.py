import affine_mpc_py as ampc


def test_mpc_base_constructor():
    try:
        mpc = ampc.MPCBase(num_states=2, num_inputs=1,
                           len_horizon=10, num_control_points=5)
    except:
        assert False
