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
    endif()
  elseif(MSVC AND AFFINE_MPC_WARNINGS_AS_ERRORS)
    target_compile_options(${target} PRIVATE /WX)
  endif()
endfunction()
