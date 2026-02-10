`ifndef RISCV_INT_MULDIV_ITERATIVE_3INPUT_V
`define RISCV_INT_MULDIV_ITERATIVE_3INPUT_V
`include "imuldiv-IntMulDivIterative.v"

module imuldiv_IntMulDivIterative3Input
(
input clk,
input reset,

input [2:0] fn1,
input [2:0] fn2,

input [31:0] msg_a,
input [31:0] msg_b,
input [31:0] msg_c,

input in_val,
output out_rdy,

output [63:0] out_msg,
output out_val,
input in_rdy
);
wire [63:0] res1;
wire mid_rdy, mid_val;
wire [31:0] mid_msg = (fn1 == `IMULDIV_MULDIVREQ_MSG_FUNC_REM) ? res1[63:32] : res1[31:0];

imuldiv_IntMulDivIterative unit1 (
.clk (clk),
.reset (reset),
.muldivreq_msg_fn (fn1),
.muldivreq_msg_a (msg_a),
.muldivreq_msg_b (msg_b),
.muldivreq_val (in_val), // input
.muldivreq_rdy (out_rdy), //output
.muldivresp_msg_result (res1),
.muldivresp_val (mid_val), // output (intermediate)
.muldivresp_rdy (mid_rdy) // input (intermediate)
);

imuldiv_IntMulDivIterative unit2 (
.clk (clk),
.reset (reset),
.muldivreq_msg_fn (fn2),
.muldivreq_msg_a (mid_msg),
.muldivreq_msg_b (msg_c),
.muldivreq_val (mid_val), // input (intermediate)
.muldivreq_rdy (mid_rdy), // output (intermediate)
.muldivresp_msg_result (out_msg),
.muldivresp_val (out_val), // output
.muldivresp_rdy (in_rdy) // input
);

endmodule
`endif