# Compiler warnings for targets built from this repository (not dependencies).
#
# Warnings are enabled for GCC and Clang. AFFINE_MPC_WARNINGS_AS_ERRORS turns
# them into errors and is meant for CI, so builds with other compilers or newer
# compiler versions are not broken by new warnings.

option(AFFINE_MPC_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

function(affine_mpc_enable_warnings target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra)
    if(AFFINE_MPC_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
      # GCC 15 reports out-of-bounds reads in Eigen 3.4's vectorized loops for
      # small fixed-size vectors (e.g. VectorXd = Vector3d with AVX), in loop
      # iterations that never run, so these stay warnings (Eigen 5 does not
      # trigger them)
      if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND DEFINED Eigen3_VERSION
         AND Eigen3_VERSION VERSION_LESS 5)
        target_compile_options(${target} PRIVATE
          -Wno-error=array-bounds -Wno-error=stringop-overread
        )
      endif()
    endif()
  elseif(MSVC AND AFFINE_MPC_WARNINGS_AS_ERRORS)
    target_compile_options(${target} PRIVATE /WX)
  endif()
endfunction()
