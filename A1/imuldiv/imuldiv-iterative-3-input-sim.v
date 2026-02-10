`include "imuldiv-MulDivReqMsg.v"
`include "imuldiv-IntMulDivIterative3Input.v"

module sim;

  reg clk = 1'b0;
  reg reset = 1'b1;
  always #5 clk = ~clk;

  reg [2:0] fn1, fn2;
  reg [31:0] a, b, c;
  reg req_val = 1'b0;
  wire req_rdy, resp_val;
  wire [63:0] resp_msg;

  imuldiv_IntMulDivIterative3Input dut (
    .clk(clk), .reset(reset),
    .fn1(fn1), .fn2(fn2),
    .msg_a(a), .msg_b(b), .msg_c(c),
    .out_val(resp_val), .out_rdy(req_rdy),
    .out_msg(resp_msg), .in_val(req_val), .in_rdy(1'b1)
  );

  reg [1023:0] op1_str, op2_str;

  function [2:0] get_fn(input [1023:0] s);
    case (s)
      "mul": get_fn = `IMULDIV_MULDIVREQ_MSG_FUNC_MUL;
      "div": get_fn = `IMULDIV_MULDIVREQ_MSG_FUNC_DIV;
      "divu": get_fn = `IMULDIV_MULDIVREQ_MSG_FUNC_DIVU;
      "rem": get_fn = `IMULDIV_MULDIVREQ_MSG_FUNC_REM;
      "remu": get_fn = `IMULDIV_MULDIVREQ_MSG_FUNC_REMU;
      default: get_fn = 3'bxxx;
    endcase
  endfunction

  initial begin
    $dumpfile("dump.vcd"); $dumpvars;
    #10 reset = 1'b0;

    if (!$value$plusargs("op1=%s", op1_str)) $finish;
    if (!$value$plusargs("op2=%s", op2_str)) $finish;
    if (!$value$plusargs("a=%h", a)) $finish;
    if (!$value$plusargs("b=%h", b)) $finish;
    if (!$value$plusargs("c=%h", c)) $finish;

    fn1 = get_fn(op1_str);
    fn2 = get_fn(op2_str);
    req_val = 1'b1;
  end

  reg busy = 1'b0;
  reg [31:0] cycles = 0;

  always @(posedge clk) begin
    if (req_val && req_rdy) begin
      busy <= 1'b1;
      req_val <= 1'b0;
    end

    if (busy || (req_val && req_rdy)) cycles <= cycles + 1;

    if (resp_val) begin
      $display("Result: 0x%h", resp_msg);
      $display("Total Cycles: %d", cycles);
      $finish;
    end
  end

endmodule