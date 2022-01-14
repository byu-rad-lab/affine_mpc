#include "affine_mpc_py_module.hpp"
#include <osqp/osqp.h>

void moduleAddOsqpSettings(py::module& m)
{
  py::class_<OSQPSettings> s(m, "Settings");
  s.def(py::init<>([]()
    {
      OSQPSettings self;
      osqp_set_default_settings(&self);
      self.verbose=false;
      return self;
    }), "Constructor sets all default values");
  s.def_readwrite("adaptive_rho", &OSQPSettings::adaptive_rho);
  s.def_readwrite("adaptive_rho_fraction", &OSQPSettings::adaptive_rho_fraction);
  s.def_readwrite("adaptive_rho_interval", &OSQPSettings::adaptive_rho_interval);
  s.def_readwrite("adaptive_rho_tolerance", &OSQPSettings::adaptive_rho_tolerance);
  s.def_readwrite("alpha", &OSQPSettings::alpha);
  s.def_readwrite("check_termination", &OSQPSettings::check_termination);
  s.def_readwrite("delta", &OSQPSettings::delta);
  s.def_readwrite("eps_abs", &OSQPSettings::eps_abs);
  s.def_readwrite("eps_dual_inf", &OSQPSettings::eps_dual_inf);
  s.def_readwrite("eps_prim_inf", &OSQPSettings::eps_prim_inf);
  s.def_readwrite("eps_rel", &OSQPSettings::eps_rel);
  s.def_readwrite("linsys_solver", &OSQPSettings::linsys_solver);
  s.def_readwrite("max_iter", &OSQPSettings::max_iter);
  s.def_readwrite("polish", &OSQPSettings::polish);
  s.def_readwrite("polish_refine_iter", &OSQPSettings::polish_refine_iter);
  s.def_readwrite("rho", &OSQPSettings::rho);
  s.def_readwrite("scaled_termination", &OSQPSettings::scaled_termination);
  s.def_readwrite("scaling", &OSQPSettings::scaling);
  s.def_readwrite("sigma", &OSQPSettings::sigma);
  s.def_readwrite("time_limit", &OSQPSettings::time_limit);
  s.def_readwrite("verbose", &OSQPSettings::verbose);
  s.def_readwrite("warm_start", &OSQPSettings::warm_start);
}
