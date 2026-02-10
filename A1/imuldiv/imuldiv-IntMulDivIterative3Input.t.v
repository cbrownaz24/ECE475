//========================================================================
// Final Corrected 3-Input MulDiv Tester
//========================================================================

`include "imuldiv-MulDivReqMsg.v"
`include "imuldiv-IntMulDivIterative3Input.v"
`include "vc-TestSource.v"
`include "vc-TestSink.v"
`include "vc-Test.v"

module imuldiv_IntMulDivIterative3Input_helper
(
  input clk, reset,
  output done
);

  wire [101:0] src_msg;
  wire [2:0] fn1 = src_msg[101:99];
  wire [2:0] fn2 = src_msg[98:96];
  wire [31:0] a = src_msg[95:64];
  wire [31:0] b = src_msg[63:32];
  wire [31:0] c = src_msg[31:0];
  wire src_val, src_rdy, src_done;
  wire [63:0] sink_msg;
  wire sink_val, sink_rdy, sink_done;

  assign done = src_done && sink_done;

  vc_TestSource#(102, 6) src (
    .clk(clk), .reset(reset), .bits(src_msg),
    .val(src_val), .rdy(src_rdy), .done(src_done)
  );

  imuldiv_IntMulDivIterative3Input imuldiv_3in (
    .clk(clk), .reset(reset),
    .fn1(fn1), .fn2(fn2),
    .msg_a(a), .msg_b(b), .msg_c(c),
    .out_val(sink_val), .out_rdy(src_rdy),
    .out_msg(sink_msg), .in_val(src_val), .in_rdy(sink_rdy)
  );

  vc_TestSink#(64, 6) sink (
    .clk(clk), .reset(reset), .bits(sink_msg),
    .val(sink_val), .rdy(sink_rdy), .done(sink_done)
  );

endmodule

module tester;

  initial begin
    $dumpfile("imuldiv-IntMulDivIterative3Input.t.vcd");
    $dumpvars;
  end

  `VC_TEST_SUITE_BEGIN( "imuldiv-IntMulDivIterative3Input" )

  reg  t0_reset = 1'b1;
  wire t0_done;

  imuldiv_IntMulDivIterative3Input_helper t0 ( .clk(clk), .reset(t0_reset), .done(t0_done) );

  `define TEST_3IN(idx_, f1_, f2_, a_, b_, c_, exp_) \
    t0.src.m[idx_] = {3'h``f1_, 3'h``f2_, 32'h``a_, 32'h``b_, 32'h``c_}; \
    t0.sink.m[idx_] = 64'h``exp_;

  `VC_TEST_CASE_BEGIN( 1, "Handshake_And_Logic_Check" )
  `VC_TEST_CASE_INIT(102, 64)
  begin
    // 2. Load test vectors
    `TEST_3IN( 0, 0, 0, 00000005, 00000004, 00000002, 0000000000000028 )
    `TEST_3IN( 1, 0, 1, 0000000a, 0000000a, 00000005, 0000000000000014 )
    `TEST_3IN( 2, 3, 1, 00000023, 00000008, 00000002, 0000000100000001 )
    `TEST_3IN( 3, 0, 0, fffffffb, 00000004, 00000002, ffffffffffffffd8 )
    `TEST_3IN( 4, 0, 0, 00000002, 00000002, 40000000, 0000000100000000 )
    `TEST_3IN( 5, 0, 1, 0000000a, 00000002, 00000000, 00000014ffffffff )

    // 3. Wait for completion
    // 10000 cycles is plenty for 6 * 64 cycles
    #5;   t0_reset = 1'b1;
    #20;  t0_reset = 1'b0;
    #10000; `VC_TEST_CHECK( "Is sink finished?", t0_done )

  end
  `VC_TEST_CASE_END

  `VC_TEST_SUITE_END( 1 )
endmodule