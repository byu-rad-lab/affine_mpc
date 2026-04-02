import affine_mpc as ampc


def test_options_interface():
    try:
        opts = ampc.Options()
        opts = ampc.Options(use_input_cost=False)
        opts = ampc.Options(slew_initial_input=False)
        opts = ampc.Options(slew_control_points=False)
        opts = ampc.Options(saturate_states=False)
        opts = ampc.Options(saturate_input_trajectory=False)
    except:
        assert False


if __name__ == "__main__":
    test_options_interface()
