//========================================================================
// imuldiv-IntDivIterative.v
//========================================================================

`ifndef RISCV_INT_DIV_ITERATIVE_V
`define RISCV_INT_DIV_ITERATIVE_V

`include "imuldiv-DivReqMsg.v"

module imuldiv_IntDivIterative
(
  input         clk,
  input         reset,

  input         divreq_msg_fn, // 1 for signed, 0 for unsigned
  input  [31:0] divreq_msg_a,
  input  [31:0] divreq_msg_b,
  input         divreq_val,
  output        divreq_rdy,

  output [63:0] divresp_msg_result,
  output        divresp_val,
  input         divresp_rdy
);

  // Control Signals
  wire cs_do_load;
  wire cs_do_shift;
  wire ds_diff_neg;

  imuldiv_IntDivIterativeDpath dpath (
    .clk                (clk),
    .reset              (reset),
    .divreq_msg_fn      (divreq_msg_fn),
    .divreq_msg_a       (divreq_msg_a),
    .divreq_msg_b       (divreq_msg_b),
    .divresp_msg_result (divresp_msg_result),
    .cs_do_load         (cs_do_load),
    .cs_do_shift        (cs_do_shift),
    .ds_diff_neg        (ds_diff_neg)
  );

  imuldiv_IntDivIterativeCtrl ctrl (
    .clk                (clk),
    .reset              (reset),
    .divreq_val         (divreq_val),
    .divreq_rdy         (divreq_rdy),
    .divresp_val        (divresp_val),
    .divresp_rdy        (divresp_rdy),
    .cs_do_load         (cs_do_load),
    .cs_do_shift        (cs_do_shift),
    .ds_diff_neg        (ds_diff_neg)
  );

endmodule

//------------------------------------------------------------------------
// Datapath
//------------------------------------------------------------------------

module imuldiv_IntDivIterativeDpath
(
  input         clk,
  input         reset,
  input         divreq_msg_fn,
  input  [31:0] divreq_msg_a,
  input  [31:0] divreq_msg_b,
  output [63:0] divresp_msg_result,
  input         cs_do_load,
  input         cs_do_shift,
  output        ds_diff_neg
);

  // 1. Sign Detection and Operand Conversion
  // Signed instructions treat operands as signed values; unsigned do not[cite: 416, 425].
  wire is_signed = (divreq_msg_fn == 1'b1);
  wire sign_a    = divreq_msg_a[31];
  wire sign_b    = divreq_msg_b[31];

  // Convert signed operands to absolute values[cite: 402, 424].
  wire [31:0] op_a = (is_signed && sign_a) ? (~divreq_msg_a + 1'b1) : divreq_msg_a;
  wire [31:0] op_b = (is_signed && sign_b) ? (~divreq_msg_b + 1'b1) : divreq_msg_b;

  // 2. Registers (A and B from flowchart)
  reg [64:0] rem_quot_reg;  // Register A: 65 bits to handle unsigned overflow 
  reg [64:0] b_reg_aligned; // Register B
  reg        sign_q;
  reg        sign_r;

  // 3. Combinational Logic for Flowchart Steps
  // Step: A << 1
  wire [64:0] shifted_A = rem_quot_reg << 1;
  // Step: diff = A - B
  wire [64:0] diff      = shifted_A - b_reg_aligned;
  // Step: diff < 0? (Check MSB of 65-bit subtraction) [cite: 436, 439]
  assign ds_diff_neg    = diff[64];

  always @(posedge clk) begin
    if (cs_do_load) begin
      // Start Box: Initialize A with dividend in right-half 
      rem_quot_reg  <= { 33'b0, op_a };
      // Start Box: Initialize B with divisor in left-half 
      b_reg_aligned <= { 1'b0, op_b, 32'b0 };
      
      // Calculate final signs: Quotient is XOR of signs; Remainder matches dividend[cite: 441, 442].
      sign_q <= is_signed && (sign_a ^ sign_b);
      sign_r <= is_signed && sign_a; 
    end 
    else if (cs_do_shift) begin
      if (ds_diff_neg)
        // Flowchart "Yes" Path: Keep the shifted value (restoring)
        rem_quot_reg <= shifted_A;
      else
        // Flowchart "No" Path: Update A with the difference and set LSB to 1
        rem_quot_reg <= { diff[64:1], 1'b1 };
    end
  end

  // 4. Extraction and Final Sign Correction
  // After 32 iterations, remainder is in [63:32] and quotient in [31:0].
  wire [31:0] raw_q = rem_quot_reg[31:0];
  wire [31:0] raw_r = rem_quot_reg[63:32];

  // Re-sign results if necessary[cite: 404, 426].
  wire [31:0] final_q = sign_q ? (~raw_q + 1'b1) : raw_q;
  wire [31:0] final_r = sign_r ? (~raw_r + 1'b1) : raw_r;

  assign divresp_msg_result = { final_r, final_q };

endmodule

//------------------------------------------------------------------------
// Control
//------------------------------------------------------------------------

module imuldiv_IntDivIterativeCtrl
(
  input  clk, reset, divreq_val, divresp_rdy, ds_diff_neg,
  output reg divreq_rdy, divresp_val, cs_do_load, cs_do_shift
);
  localparam STATE_IDLE = 2'd0, STATE_CALC = 2'd1, STATE_DONE = 2'd2;
  reg [1:0] state, next_state;
  reg [5:0] count;

  always @(posedge clk) begin
    if (reset) state <= STATE_IDLE;
    else       state <= next_state;
  end

  always @(posedge clk) begin
    if (cs_do_load)      count <= 6'd32;
    else if (cs_do_shift) count <= count - 1'b1;
  end

  always @(*) begin
    next_state = state;
    cs_do_load = 0; cs_do_shift = 0; divreq_rdy = 0; divresp_val = 0;
    case (state)
      STATE_IDLE: begin 
        divreq_rdy = 1; 
        if (divreq_val) begin 
          cs_do_load = 1; 
          next_state = STATE_CALC; 
        end 
      end
      STATE_CALC: begin 
        cs_do_shift = 1; 
        if (count == 6'd1) next_state = STATE_DONE; 
      end
      STATE_DONE: begin 
        divresp_val = 1; 
        if (divresp_rdy) next_state = STATE_IDLE; 
      end
      default: next_state = STATE_IDLE;
    endcase
  end
endmodule
`endif