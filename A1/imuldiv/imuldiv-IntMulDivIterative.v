`ifndef RISCV_INT_MULDIV_ITERATIVE_V
`define RISCV_INT_MULDIV_ITERATIVE_V

`include "imuldiv-MulDivReqMsg.v"
`include "imuldiv-IntMulIterative.v"
`include "imuldiv-IntDivIterative.v"

module imuldiv_IntMulDivIterative
(
  input         clk, reset,
  input   [2:0] muldivreq_msg_fn,
  input  [31:0] muldivreq_msg_a, muldivreq_msg_b,
  input         muldivreq_val,
  output        muldivreq_rdy,
  output [63:0] muldivresp_msg_result,
  output        muldivresp_val,
  input         muldivresp_rdy
);

  wire mulreq_rdy, divreq_rdy, mulresp_val, divresp_val;
  wire [63:0] mul_res, div_res;

  wire is_mul = (muldivreq_msg_fn == `IMULDIV_MULDIVREQ_MSG_FUNC_MUL);
  wire is_div = !is_mul;

  assign muldivreq_rdy = is_mul ? mulreq_rdy : divreq_rdy;
  assign muldivresp_val = mulresp_val || divresp_val;
  assign muldivresp_msg_result = ( mulresp_val ) ? mul_res : ( divresp_val ) ? div_res : 64'bx;

  imuldiv_IntMulIterative imul (
    .clk(clk), .reset(reset),
    .mulreq_msg_a(muldivreq_msg_a), .mulreq_msg_b(muldivreq_msg_b),
    .mulreq_val(muldivreq_val && is_mul), .mulreq_rdy(mulreq_rdy),
    .mulresp_msg_result(mul_res), .mulresp_val(mulresp_val), .mulresp_rdy(muldivresp_rdy)
  );

  imuldiv_IntDivIterative idiv (
    .clk(clk), .reset(reset),
    // Pass 1 for Signed (DIV/REM), 0 for Unsigned (DIVU/REMU)
    .divreq_msg_fn(muldivreq_msg_fn == `IMULDIV_MULDIVREQ_MSG_FUNC_DIV || 
                   muldivreq_msg_fn == `IMULDIV_MULDIVREQ_MSG_FUNC_REM),
    .divreq_msg_a(muldivreq_msg_a), 
    .divreq_msg_b(muldivreq_msg_b),
    .divreq_val(muldivreq_val && is_div), 
    .divreq_rdy(divreq_rdy),
    .divresp_msg_result(div_res), 
    .divresp_val(divresp_val), 
    .divresp_rdy(muldivresp_rdy)
  );

endmodule
`endif