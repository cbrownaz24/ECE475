#=========================================================================
# imuldiv Subpackage
#=========================================================================

imuldiv_deps = vc

imuldiv_srcs = \
  imuldiv-DivReqMsg.v \
  imuldiv-MulDivReqMsg.v \
  imuldiv-IntMulDivSingleCycle.v \
  imuldiv-IntMulIterative.v \
  imuldiv-IntDivIterative.v \
  imuldiv-IntMulDivIterative.v \
  imuldiv-IntMulDivIterative3Input.v \
  imuldiv-IntMulDivIterativeIntegrated.v \

imuldiv_test_srcs = \
  imuldiv-DivReqMsg.t.v \
  imuldiv-MulDivReqMsg.t.v \
  imuldiv-IntMulDivSingleCycle.t.v \
  imuldiv-IntMulIterative.t.v \
  imuldiv-IntDivIterative.t.v \
  imuldiv-IntMulDivIterative.t.v \
  imuldiv-IntMulDivIterativeIntegrated.t.v \
  imuldiv-IntMulDivIterative3Input.t.v \


imuldiv_prog_srcs = \
  imuldiv-singcyc-sim.v \
  imuldiv-iterative-sim.v \
  imuldiv-iterative-3-input-sim.v \
  imuldiv-iterative-integrated-sim.v \

