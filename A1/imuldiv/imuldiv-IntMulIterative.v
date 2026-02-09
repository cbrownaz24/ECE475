`ifndef RISCV_INT_MUL_ITERATIVE_V
`define RISCV_INT_MUL_ITERATIVE_V

module imuldiv_IntMulIterative
(
  input                clk,
  input                reset,
  input  [31:0]        mulreq_msg_a,
  input  [31:0]        mulreq_msg_b,
  input                mulreq_val,
  output               mulreq_rdy,
  output [63:0]        mulresp_msg_result,
  output               mulresp_val,
  input                mulresp_rdy
);

  wire cs_do_load, cs_do_shift, ds_b_lsb;

  imuldiv_IntMulIterativeDpath dpath (
    .clk(clk), .reset(reset),
    .mulreq_msg_a(mulreq_msg_a), .mulreq_msg_b(mulreq_msg_b),
    .mulresp_msg_result(mulresp_msg_result),
    .cs_do_load(cs_do_load), .cs_do_shift(cs_do_shift), .ds_b_lsb(ds_b_lsb)
  );

  imuldiv_IntMulIterativeCtrl ctrl (
    .clk(clk), .reset(reset),
    .mulreq_val(mulreq_val), .mulreq_rdy(mulreq_rdy),
    .mulresp_val(mulresp_val), .mulresp_rdy(mulresp_rdy),
    .cs_do_load(cs_do_load), .cs_do_shift(cs_do_shift), .ds_b_lsb(ds_b_lsb)
  );
endmodule

module imuldiv_IntMulIterativeDpath
(
  input         clk, reset,
  input  [31:0] mulreq_msg_a, mulreq_msg_b,
  output [63:0] mulresp_msg_result,
  input         cs_do_load, cs_do_shift,
  output        ds_b_lsb
);
  reg [63:0] a_reg, result_reg;
  reg [31:0] b_reg;
  reg        sign_reg;

  wire [31:0] unsign_a = mulreq_msg_a[31] ? (~mulreq_msg_a + 1'b1) : mulreq_msg_a;
  wire [31:0] unsign_b = mulreq_msg_b[31] ? (~mulreq_msg_b + 1'b1) : mulreq_msg_b;
  assign ds_b_lsb = b_reg[0];

  always @(posedge clk) begin
    if (cs_do_load) begin
      a_reg      <= {32'b0, unsign_a};
      b_reg      <= unsign_b;
      result_reg <= 64'b0;
      sign_reg   <= mulreq_msg_a[31] ^ mulreq_msg_b[31];
    end else if (cs_do_shift) begin
      if (b_reg[0]) result_reg <= result_reg + a_reg;
      a_reg <= a_reg << 1;
      b_reg <= b_reg >> 1;
    end
  end

  assign mulresp_msg_result = sign_reg ? (~result_reg + 1'b1) : result_reg;
endmodule

module imuldiv_IntMulIterativeCtrl
(
  input clk, reset, mulreq_val, mulresp_rdy, ds_b_lsb,
  output reg mulreq_rdy, mulresp_val, cs_do_load, cs_do_shift
);
  localparam STATE_IDLE = 2'd0, STATE_CALC = 2'd1, STATE_DONE = 2'd2;
  reg [1:0] state, next_state;
  reg [5:0] count;

  always @(posedge clk) begin
    if (reset) {state, count} <= 0;
    else begin state <= next_state; if (cs_do_shift) count <= count - 1; else if (cs_do_load) count <= 32; end
  end

  always @(*) begin
    next_state = state; cs_do_load = 0; cs_do_shift = 0; mulreq_rdy = 0; mulresp_val = 0;
    case (state)
      STATE_IDLE: begin mulreq_rdy = 1; if (mulreq_val) begin cs_do_load = 1; next_state = STATE_CALC; end end
      STATE_CALC: begin cs_do_shift = 1; if (count == 1) next_state = STATE_DONE; end
      STATE_DONE: begin mulresp_val = 1; if (mulresp_rdy) next_state = STATE_IDLE; end
    endcase
  end
endmodule
`endif