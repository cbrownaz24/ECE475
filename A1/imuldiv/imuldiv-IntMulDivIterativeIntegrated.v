`ifndef IMULDIV_ITERATIVE_INTEGRATED_V
`define IMULDIV_ITERATIVE_INTEGRATED_V

`include "imuldiv-MulDivReqMsg.v"

module imuldiv_IntMulDivIterativeIntegrated
(
  input clk,
  input reset,
  input [2:0] muldivreq_msg_fn,
  input [31:0] muldivreq_msg_a,
  input [31:0] muldivreq_msg_b,
  input muldivreq_val,
  output reg muldivreq_rdy,
  output [63:0] muldivresp_msg_result,
  output reg muldivresp_val,
  input muldivresp_rdy
);

  localparam STATE_IDLE = 2'd0;
  localparam STATE_CALC = 2'd1;
  localparam STATE_DONE = 2'd2;

  reg [1:0] state, next_state;
  reg [5:0] count;

  reg [64:0] a_reg;
  reg [64:0] b_reg;
  reg [63:0] res_reg;
  reg sign_q, sign_r;
  reg sign_mul;
  reg [2:0] fn_reg;

  wire is_mul = (muldivreq_msg_fn == `IMULDIV_MULDIVREQ_MSG_FUNC_MUL);
  wire is_signed_div = (muldivreq_msg_fn == `IMULDIV_MULDIVREQ_MSG_FUNC_DIV || 
                        muldivreq_msg_fn == `IMULDIV_MULDIVREQ_MSG_FUNC_REM);

  wire [31:0] abs_a = ( (is_mul || is_signed_div) && muldivreq_msg_a[31] ) ? (~muldivreq_msg_a + 1'b1) : muldivreq_msg_a;
  wire [31:0] abs_b = ( (is_mul || is_signed_div) && muldivreq_msg_b[31] ) ? (~muldivreq_msg_b + 1'b1) : muldivreq_msg_b;

  wire [64:0] div_shifted_a = a_reg << 1;
  wire [64:0] div_diff = div_shifted_a - b_reg;
  wire div_diff_neg  = div_diff[64];

  always @(posedge clk) begin
    if (reset) begin
      state <= STATE_IDLE;
      count <= 0;
    end else begin
      state <= next_state;

      if (state == STATE_IDLE && muldivreq_val) begin
        fn_reg <= muldivreq_msg_fn;
        count  <= 6'd32;
        if (is_mul) begin
          a_reg <= {33'b0, abs_a};
          b_reg <= {33'b0, abs_b};
          res_reg <= 64'b0;
          sign_mul <= muldivreq_msg_a[31] ^ muldivreq_msg_b[31];
        end else begin
          a_reg <= {33'b0, abs_a};
          b_reg <= {1'b0, abs_b, 32'b0};
          sign_q <= is_signed_div && (muldivreq_msg_a[31] ^ muldivreq_msg_b[31]);
          sign_r <= is_signed_div && muldivreq_msg_a[31];
        end
      end 

      else if (state == STATE_CALC) begin
        count <= count - 1;
        if (fn_reg == `IMULDIV_MULDIVREQ_MSG_FUNC_MUL) begin
          if (b_reg[0]) res_reg <= res_reg + a_reg[63:0];
          a_reg <= a_reg << 1;
          b_reg <= b_reg >> 1;
        end else begin
          if (div_diff_neg) a_reg <= div_shifted_a;
          else              a_reg <= { div_diff[64:1], 1'b1 };
        end
      end
    end
  end

  always @(*) begin
    next_state = state;
    muldivreq_rdy = 0;
    muldivresp_val = 0;

    case (state)
      STATE_IDLE: begin
        muldivreq_rdy = 1;
        if (muldivreq_val) next_state = STATE_CALC;
      end
      STATE_CALC: if (count == 1) next_state = STATE_DONE;
      STATE_DONE: begin
        muldivresp_val = 1;
        if (muldivresp_rdy) next_state = STATE_IDLE;
      end
    endcase
  end

  wire [31:0] final_q = sign_q ? (~a_reg[31:0] + 1'b1)  : a_reg[31:0];
  wire [31:0] final_r = sign_r ? (~a_reg[63:32] + 1'b1) : a_reg[63:32];
  wire [63:0] final_mul = sign_mul ? (~res_reg + 1'b1) : res_reg;

  assign muldivresp_msg_result = (fn_reg == `IMULDIV_MULDIVREQ_MSG_FUNC_MUL) ? final_mul : {final_r, final_q};

endmodule
`endif