//=========================================================================
// riscv-byp-long.S
//=========================================================================
// Tests the long-distance bypass paths unique to the 7-stage pipeline
// (F, D, X, M, A, B, W). Standard tests only insert up to 2 nops between
// a producer and consumer, covering X/M/A-stage bypasses. This file adds
// 3-nop and 4-nop gaps to exercise the B-stage (X3) and W-stage bypasses.
//
// Bug targeted: missing bypass coverage for the two extra pipeline stages
// (A=X2 and B=X3 in the code, labeled "A" and "B" in the diagram).
// If rs*_X3_byp_Dhl or rs*_W_byp_Dhl is wired incorrectly (wrong priority,
// wrong data source, or wrong enable condition), the processor will read a
// stale zero from the register file and produce the wrong result, but only
// when there are exactly 3 or 4 nops between the producer and consumer.

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        //-----------------------------------------------------------------
        // B-stage (X3) bypass: 3 nops between producer and consumer.
        // When the consumer's addi is in D, the producer's add is in X3.
        // Bypass sel=4 (rs*_X3_byp) must forward wb_mux_out_X3hl.
        //-----------------------------------------------------------------

        TEST_RR_DEST_BYP( 3, add, 13, 11, 24 )
        TEST_RR_DEST_BYP( 3, add, 14, 11, 25 )

        TEST_RR_SRC01_BYP( 3, 0, add, 13, 11, 24 )
        TEST_RR_SRC01_BYP( 0, 3, add, 14, 11, 25 )

        TEST_RR_SRC10_BYP( 3, 0, add, 13, 11, 24 )
        TEST_RR_SRC10_BYP( 0, 3, add, 14, 11, 25 )

        //-----------------------------------------------------------------
        // W-stage bypass: 4 nops between producer and consumer.
        // When the consumer's addi is in D, the producer's add is in W.
        // Bypass sel=5 (rs*_W_byp) must forward wb_mux_out_Whl.
        //-----------------------------------------------------------------

        TEST_RR_DEST_BYP( 4, add, 13, 11, 24 )
        TEST_RR_DEST_BYP( 4, add, 14, 11, 25 )

        TEST_RR_SRC01_BYP( 4, 0, add, 13, 11, 24 )
        TEST_RR_SRC01_BYP( 0, 4, add, 14, 11, 25 )

        TEST_RR_SRC10_BYP( 4, 0, add, 13, 11, 24 )
        TEST_RR_SRC10_BYP( 0, 4, add, 14, 11, 25 )

        //-----------------------------------------------------------------
        // MUL-use hazard: 0 nops between mul and a dependent add.
        // The pipeline must stall D while mul is in X/M/A/B, then use
        // the W-stage bypass (sel=5) to forward the muldiv result.
        // This is the canonical test for stall_muldiv_use_Dhl + W bypass.
        //-----------------------------------------------------------------

        TEST_RR_DEST_BYP( 0, mul, 6, 7, 42 )
        TEST_RR_DEST_BYP( 0, mul, 3, 3,  9 )

        TEST_RISCV_END
