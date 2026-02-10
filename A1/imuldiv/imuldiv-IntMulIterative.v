`ifndef RISCV_INT_MUL_ITERATIVE_V
`define RISCV_INT_MUL_ITERATIVE_V

// module imuldiv_IntMulIterative
// (
//   input clk,
//   input reset,
//   input [31:0] mulreq_msg_a,
//   input [31:0] mulreq_msg_b,
//   input mulreq_val,
//   output mulreq_rdy,
//   output [63:0] mulresp_msg_result,
//   output mulresp_val,
//   input mulresp_rdy
// );

//   wire cs_do_load, cs_do_shift, ds_b_lsb;

//   imuldiv_IntMulIterativeDpath dpath (
//     .clk(clk), .reset(reset),
//     .mulreq_msg_a(mulreq_msg_a), .mulreq_msg_b(mulreq_msg_b),
//     .mulresp_msg_result(mulresp_msg_result),
//     .cs_do_load(cs_do_load), .cs_do_shift(cs_do_shift), .ds_b_lsb(ds_b_lsb)
//   );

//   imuldiv_IntMulIterativeCtrl ctrl (
//     .clk(clk), .reset(reset),
//     .mulreq_val(mulreq_val), .mulreq_rdy(mulreq_rdy),
//     .mulresp_val(mulresp_val), .mulresp_rdy(mulresp_rdy),
//     .cs_do_load(cs_do_load), .cs_do_shift(cs_do_shift), .ds_b_lsb(ds_b_lsb)
//   );
// endmodule

// module imuldiv_IntMulIterativeDpath
// (
//   input clk, reset,
//   input [31:0] mulreq_msg_a, mulreq_msg_b,
//   output [63:0] mulresp_msg_result,
//   input cs_do_load, cs_do_shift,
//   output ds_b_lsb
// );
//   reg [63:0] a_reg, result_reg;
//   reg [31:0] b_reg;
//   reg sign_reg;

//   wire [31:0] unsign_a = mulreq_msg_a[31] ? (~mulreq_msg_a + 1'b1) : mulreq_msg_a;
//   wire [31:0] unsign_b = mulreq_msg_b[31] ? (~mulreq_msg_b + 1'b1) : mulreq_msg_b;
//   assign ds_b_lsb = b_reg[0];

//   always @(posedge clk) begin
//     if (cs_do_load) begin
//       a_reg  <= {32'b0, unsign_a};
//       b_reg  <= unsign_b;
//       result_reg <= 64'b0;
//       sign_reg  <= mulreq_msg_a[31] ^ mulreq_msg_b[31];
//     end else if (cs_do_shift) begin
//       if (b_reg[0]) result_reg <= result_reg + a_reg;
//       a_reg <= a_reg << 1;
//       b_reg <= b_reg >> 1;
//     end
//   end

//   assign mulresp_msg_result = sign_reg ? (~result_reg + 1'b1) : result_reg;
// endmodule

// module imuldiv_IntMulIterativeCtrl
// (
//   input clk, reset, mulreq_val, mulresp_rdy, ds_b_lsb,
//   output reg mulreq_rdy, mulresp_val, cs_do_load, cs_do_shift
// );
//   localparam STATE_IDLE = 2'd0, STATE_CALC = 2'd1, STATE_DONE = 2'd2;
//   reg [1:0] state, next_state;
//   reg [5:0] count;

//   always @(posedge clk) begin
//     if (reset) {state, count} <= 0;
//     else begin state <= next_state; if (cs_do_shift) count <= count - 1; else if (cs_do_load) count <= 32; end
//   end

//   always @(*) begin
//     next_state = state; cs_do_load = 0; cs_do_shift = 0; mulreq_rdy = 0; mulresp_val = 0;
//     case (state)
//       STATE_IDLE: begin mulreq_rdy = 1; if (mulreq_val) begin cs_do_load = 1; next_state = STATE_CALC; end end
//       STATE_CALC: begin cs_do_shift = 1; if (count == 1) next_state = STATE_DONE; end
//       STATE_DONE: begin mulresp_val = 1; if (mulresp_rdy) next_state = STATE_IDLE; end
//     endcase
//   end
// endmodule

module imuldiv_IntMulIterative
(
  input clk,
  input reset,
  input [31:0] mulreq_msg_a,
  input [31:0] mulreq_msg_b,
  input mulreq_val,
  output mulreq_rdy,
  output [63:0] mulresp_msg_result,
  output mulresp_val,
  input mulresp_rdy
);

  wire [5:0] ds_tzc;
  wire cs_do_load, cs_do_shift, ds_b_is_zero;

  imuldiv_IntMulIterativeDpath dpath (
    .clk(clk), .reset(reset),
    .mulreq_msg_a(mulreq_msg_a), .mulreq_msg_b(mulreq_msg_b),
    .mulresp_msg_result(mulresp_msg_result),
    .cs_do_load(cs_do_load), .cs_do_shift(cs_do_shift), 
    .ds_tzc(ds_tzc), .ds_b_is_zero(ds_b_is_zero)
  );

  imuldiv_IntMulIterativeCtrl ctrl (
    .clk(clk), .reset(reset),
    .mulreq_val(mulreq_val), .mulreq_rdy(mulreq_rdy),
    .mulresp_val(mulresp_val), .mulresp_rdy(mulresp_rdy),
    .cs_do_load(cs_do_load), .cs_do_shift(cs_do_shift),
    .ds_tzc(ds_tzc), .ds_b_is_zero(ds_b_is_zero)
  );
endmodule

module imuldiv_IntMulIterativeDpath
(
  input clk, reset,
  input [31:0] mulreq_msg_a, mulreq_msg_b,
  output [63:0] mulresp_msg_result,
  input cs_do_load, cs_do_shift,
  output [5:0] ds_tzc,
  output ds_b_is_zero
);
  reg [63:0] a_reg, result_reg;
  reg [31:0] b_reg;
  reg sign_reg;

  wire [31:0] unsign_a = mulreq_msg_a[31] ? (~mulreq_msg_a + 1'b1) : mulreq_msg_a;
  wire [31:0] unsign_b = mulreq_msg_b[31] ? (~mulreq_msg_b + 1'b1) : mulreq_msg_b;

  assign ds_b_is_zero = (b_reg == 0);
  assign ds_tzc = (b_reg[0])  ? 6'd0  : (b_reg[1])  ? 6'd1  : (b_reg[2])  ? 6'd2  : 
                  (b_reg[3])  ? 6'd3  : (b_reg[4])  ? 6'd4  : (b_reg[5])  ? 6'd5  :
                  (b_reg[6])  ? 6'd6  : (b_reg[7])  ? 6'd7  : (b_reg[8])  ? 6'd8  :
                  (b_reg[9])  ? 6'd9  : (b_reg[10]) ? 6'd10 : (b_reg[11]) ? 6'd11 :
                  (b_reg[12]) ? 6'd12 : (b_reg[13]) ? 6'd13 : (b_reg[14]) ? 6'd14 :
                  (b_reg[15]) ? 6'd15 : (b_reg[16]) ? 6'd16 : (b_reg[17]) ? 6'd17 :
                  (b_reg[18]) ? 6'd18 : (b_reg[19]) ? 6'd19 : (b_reg[20]) ? 6'd20 :
                  (b_reg[21]) ? 6'd21 : (b_reg[22]) ? 6'd22 : (b_reg[23]) ? 6'd23 :
                  (b_reg[24]) ? 6'd24 : (b_reg[25]) ? 6'd25 : (b_reg[26]) ? 6'd26 :
                  (b_reg[27]) ? 6'd27 : (b_reg[28]) ? 6'd28 : (b_reg[29]) ? 6'd29 :
                  (b_reg[30]) ? 6'd30 : (b_reg[31]) ? 6'd31 : 6'd32;

  always @(posedge clk) begin
    if (cs_do_load) begin
      a_reg <= {32'b0, unsign_a};
      b_reg <= unsign_b;
      result_reg <= 64'b0;
      sign_reg <= mulreq_msg_a[31] ^ mulreq_msg_b[31];
    end else if (cs_do_shift) begin
      result_reg <= result_reg + (a_reg << ds_tzc);
      a_reg <= a_reg << (ds_tzc + 1);
      b_reg <= b_reg >> (ds_tzc + 1);
    end
  end

  assign mulresp_msg_result = sign_reg ? (~result_reg + 1'b1) : result_reg;
endmodule

module imuldiv_IntMulIterativeCtrl
(
  input clk, reset, mulreq_val, mulresp_rdy, ds_b_is_zero,
  input [5:0] ds_tzc,
  output reg mulreq_rdy, mulresp_val, cs_do_load, cs_do_shift
);
  localparam STATE_IDLE = 2'd0, STATE_CALC = 2'd1, STATE_DONE = 2'd2;
  reg [1:0] state, next_state;

  always @(posedge clk) begin
    if (reset) state <= STATE_IDLE;
    else state <= next_state;
  end

  always @(*) begin
    next_state = state; cs_do_load = 0; cs_do_shift = 0; mulreq_rdy = 0; mulresp_val = 0;
    case (state)
      STATE_IDLE: begin 
        mulreq_rdy = 1; 
        if (mulreq_val) begin 
          cs_do_load = 1; 
          next_state = STATE_CALC; 
        end 
      end
      STATE_CALC: begin 
        if (ds_b_is_zero) next_state = STATE_DONE;
        else              cs_do_shift = 1;
      end
      STATE_DONE: begin 
        mulresp_val = 1; 
        if (mulresp_rdy) next_state = STATE_IDLE; 
      end
    endcase
  end
endmodule
`endif