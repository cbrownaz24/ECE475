//=========================================================================
// 7-Stage RISCV Control Unit — Part 2: Dual-Issue Superscalar
//=========================================================================

`ifndef RISCV_CORE_CTRL_V
`define RISCV_CORE_CTRL_V

`include "riscvssc-InstMsg.v"

module riscv_CoreCtrl
(
  input clk,
  input reset,

  // Instruction Memory Port
  output        imemreq0_val,
  input         imemreq0_rdy,
  input  [31:0] imemresp0_msg_data,
  input         imemresp0_val,

  // Instruction Memory Port
  output        imemreq1_val,
  input         imemreq1_rdy,
  input  [31:0] imemresp1_msg_data,
  input         imemresp1_val,

  // Data Memory Port

  output        dmemreq_msg_rw,
  output  [1:0] dmemreq_msg_len,
  output        dmemreq_val,
  input         dmemreq_rdy,
  input         dmemresp_val,

  // Controls Signals (ctrl->dpath)

  output  [1:0] pc_mux_sel_Phl,
  output        steering_mux_sel_Dhl,
  output  [3:0] opA0_byp_mux_sel_Dhl,
  output  [1:0] opA0_mux_sel_Dhl,
  output  [3:0] opA1_byp_mux_sel_Dhl,
  output  [2:0] opA1_mux_sel_Dhl,
  output  [3:0] opB0_byp_mux_sel_Dhl,
  output  [1:0] opB0_mux_sel_Dhl,
  output  [3:0] opB1_byp_mux_sel_Dhl,
  output  [2:0] opB1_mux_sel_Dhl,
  output [31:0] instA_Dhl,
  output [31:0] instB_Dhl,
  output  [3:0] aluA_fn_X0hl,
  output  [3:0] aluB_fn_X0hl,
  output  [2:0] muldivreq_msg_fn_Dhl,
  output        muldivreq_val,
  input         muldivreq_rdy,
  input         muldivresp_val,
  output        muldivresp_rdy,
  output        muldiv_stall_mult1,
  output reg [2:0] dmemresp_mux_sel_X1hl,
  output        dmemresp_queue_en_X1hl,
  output reg       dmemresp_queue_val_X1hl,
  output reg       muldiv_mux_sel_X3hl,
  output reg       execute_mux_sel_X3hl,
  output reg       memex_mux_sel_X1hl,
  output        rfA_wen_out_Whl,
  output  [4:0] rfA_waddr_Whl,
  output        rfB_wen_out_Whl,
  output  [4:0] rfB_waddr_Whl,
  output        stall_Fhl,
  output        stall_Dhl,
  output        stall_X0hl,
  output        stall_X1hl,
  output        stall_X2hl,
  output        stall_X3hl,
  output        stall_Whl,

  // Control Signals (dpath->ctrl)

  input         branch_cond_eq_X0hl,
  input         branch_cond_ne_X0hl,
  input         branch_cond_lt_X0hl,
  input         branch_cond_ltu_X0hl,
  input         branch_cond_ge_X0hl,
  input         branch_cond_geu_X0hl,
  input  [31:0] proc2csr_data_Whl,

  // CSR Status

  output reg [31:0] csr_status
);

  //----------------------------------------------------------------------
  // PC Stage: Instruction Memory Request
  //----------------------------------------------------------------------

  assign pc_mux_sel_Phl
    = brj_taken_X0hl    ? pm_b
    : brj_taken_Dhl     ? pc_mux_sel_Dhl
    :                     pm_p;

  wire   imemreq_val_Phl = reset || !stall_Phl;
  assign imemreq0_val     = imemreq_val_Phl;
  assign imemreq1_val     = imemreq_val_Phl;

  wire squash_Phl = 1'b0;
  wire stall_Phl = stall_Fhl;
  wire bubble_next_Phl = ( squash_Phl || stall_Phl );

  //----------------------------------------------------------------------
  // F <- P
  //----------------------------------------------------------------------

  reg imemreq_val_Fhl;
  reg bubble_Fhl;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      imemreq_val_Fhl <= 1'b0;
      bubble_Fhl <= 1'b0;
    end
    else if( !stall_Fhl ) begin
      imemreq_val_Fhl <= imemreq_val_Phl;
      bubble_Fhl <= bubble_next_Phl;
    end
    else begin
      imemreq_val_Fhl <= imemreq_val_Phl;
    end
  end

  //----------------------------------------------------------------------
  // Fetch Stage: Instruction Memory Response
  //----------------------------------------------------------------------

  wire inst_val_Fhl = ( !bubble_Fhl && !squash_Fhl );

  wire squash_Fhl
    = ( inst_val_Dhl && brj_taken_Dhl )
   || ( inst_val_X0hl && brj_taken_X0hl );

  assign stall_Fhl = stall_Dhl || stall_0_Dhl;

  wire bubble_sel_Fhl  = ( squash_Fhl || stall_Fhl );
  wire bubble_next_Fhl = ( !bubble_sel_Fhl ) ? bubble_Fhl
                       : ( bubble_sel_Fhl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // Queue for instruction memory response
  //----------------------------------------------------------------------

  wire imemresp0_queue_en_Fhl = ( stall_Fhl && imemresp0_val );
  wire imemresp0_queue_val_next_Fhl
    = stall_Fhl && ( imemresp0_val || imemresp0_queue_val_Fhl );

  wire imemresp1_queue_en_Fhl = ( stall_Fhl && imemresp1_val );
  wire imemresp1_queue_val_next_Fhl
    = stall_Fhl && ( imemresp1_val || imemresp1_queue_val_Fhl );

  reg [31:0] imemresp0_queue_reg_Fhl;
  reg        imemresp0_queue_val_Fhl;
  reg [31:0] imemresp1_queue_reg_Fhl;
  reg        imemresp1_queue_val_Fhl;

  always @ ( posedge clk ) begin
    if ( imemresp0_queue_en_Fhl )
      imemresp0_queue_reg_Fhl <= imemresp0_msg_data;
    if ( imemresp1_queue_en_Fhl )
      imemresp1_queue_reg_Fhl <= imemresp1_msg_data;
    imemresp0_queue_val_Fhl <= imemresp0_queue_val_next_Fhl;
    imemresp1_queue_val_Fhl <= imemresp1_queue_val_next_Fhl;
  end

  wire [31:0] imemresp0_queue_mux_out_Fhl
    = ( !imemresp0_queue_val_Fhl ) ? imemresp0_msg_data
    : ( imemresp0_queue_val_Fhl )  ? imemresp0_queue_reg_Fhl
    :                               32'bx;

  wire [31:0] imemresp1_queue_mux_out_Fhl
    = ( !imemresp1_queue_val_Fhl ) ? imemresp1_msg_data
    : ( imemresp1_queue_val_Fhl )  ? imemresp1_queue_reg_Fhl
    :                               32'bx;

  //----------------------------------------------------------------------
  // D <- F
  //----------------------------------------------------------------------

  reg [31:0] ir0_Dhl;
  reg [31:0] ir1_Dhl;
  reg        bubble_Dhl;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_Dhl <= 1'b1;
    end
    else begin
      if ( squash_Dhl )
        bubble_Dhl <= 1'b1;
      else if ( !stall_Fhl )
        bubble_Dhl <= bubble_next_Fhl;

      if ( !stall_Fhl ) begin
        ir0_Dhl <= imemresp0_queue_mux_out_Fhl;
        ir1_Dhl <= imemresp1_queue_mux_out_Fhl;
      end
    end
  end

  //----------------------------------------------------------------------
  // Decode Stage: Constants
  //----------------------------------------------------------------------

  localparam n = 1'd0;
  localparam y = 1'd1;

  localparam rx = 5'bx;
  localparam r0 = 5'd0;

  localparam br_x    = 3'bx;
  localparam br_none = 3'd0;
  localparam br_beq  = 3'd1;
  localparam br_bne  = 3'd2;
  localparam br_blt  = 3'd3;
  localparam br_bltu = 3'd4;
  localparam br_bge  = 3'd5;
  localparam br_bgeu = 3'd6;

  localparam pm_x   = 2'bx;
  localparam pm_p   = 2'd0;
  localparam pm_b   = 2'd1;
  localparam pm_j   = 2'd2;
  localparam pm_r   = 2'd3;

  localparam am_r0      = 4'd0;
  localparam am_AX0_byp = 4'd1;
  localparam am_AX1_byp = 4'd2;
  localparam am_AX2_byp = 4'd3;
  localparam am_AX3_byp = 4'd4;
  localparam am_AW_byp  = 4'd5;
  localparam am_BX0_byp = 4'd6;
  localparam am_BX1_byp = 4'd7;
  localparam am_BX2_byp = 4'd8;
  localparam am_BX3_byp = 4'd9;
  localparam am_BW_byp  = 4'd10;

  localparam am_x     = 2'bx;
  localparam am_rdat  = 2'd0;
  localparam am_pc    = 2'd1;
  localparam am_pc4   = 2'd2;
  localparam am_0     = 2'd3;

  localparam bm_r1      = 4'd0;
  localparam bm_AX0_byp = 4'd1;
  localparam bm_AX1_byp = 4'd2;
  localparam bm_AX2_byp = 4'd3;
  localparam bm_AX3_byp = 4'd4;
  localparam bm_AW_byp  = 4'd5;
  localparam bm_BX0_byp = 4'd6;
  localparam bm_BX1_byp = 4'd7;
  localparam bm_BX2_byp = 4'd8;
  localparam bm_BX3_byp = 4'd9;
  localparam bm_BW_byp  = 4'd10;

  localparam bm_x      = 3'bx;
  localparam bm_rdat   = 3'd0;
  localparam bm_shamt  = 3'd1;
  localparam bm_imm_u  = 3'd2;
  localparam bm_imm_sb = 3'd3;
  localparam bm_imm_i  = 3'd4;
  localparam bm_imm_s  = 3'd5;
  localparam bm_0      = 3'd6;

  localparam alu_x    = 4'bx;
  localparam alu_add  = 4'd0;
  localparam alu_sub  = 4'd1;
  localparam alu_sll  = 4'd2;
  localparam alu_or   = 4'd3;
  localparam alu_lt   = 4'd4;
  localparam alu_ltu  = 4'd5;
  localparam alu_and  = 4'd6;
  localparam alu_xor  = 4'd7;
  localparam alu_nor  = 4'd8;
  localparam alu_srl  = 4'd9;
  localparam alu_sra  = 4'd10;

  localparam md_x    = 3'bx;
  localparam md_mul  = 3'd0;
  localparam md_div  = 3'd1;
  localparam md_divu = 3'd2;
  localparam md_rem  = 3'd3;
  localparam md_remu = 3'd4;

  localparam mdm_x = 1'bx;
  localparam mdm_l = 1'd0;
  localparam mdm_u = 1'd1;

  localparam em_x   = 1'bx;
  localparam em_alu = 1'd0;
  localparam em_md  = 1'd1;

  localparam nr = 2'b0;
  localparam ld = 2'd1;
  localparam st = 2'd2;

  localparam ml_x  = 2'bx;
  localparam ml_w  = 2'd0;
  localparam ml_b  = 2'd1;
  localparam ml_h  = 2'd2;

  localparam dmm_x  = 3'bx;
  localparam dmm_w  = 3'd0;
  localparam dmm_b  = 3'd1;
  localparam dmm_bu = 3'd2;
  localparam dmm_h  = 3'd3;
  localparam dmm_hu = 3'd4;

  localparam wm_x   = 1'bx;
  localparam wm_alu = 1'd0;
  localparam wm_mem = 1'd1;

  //----------------------------------------------------------------------
  // Decode Stage: Logic
  //----------------------------------------------------------------------

  wire inst_val_Dhl = ( !bubble_Dhl && !squash_Dhl );

  wire   [4:0] inst0_rs1_Dhl;
  wire   [4:0] inst0_rs2_Dhl;
  wire   [4:0] inst0_rd_Dhl;

  riscv_InstMsgFromBits inst0_msg_from_bits
  (
    .msg (ir0_Dhl), .opcode (), .rs1 (inst0_rs1_Dhl), .rs2 (inst0_rs2_Dhl),
    .rd (inst0_rd_Dhl), .funct3 (), .funct7 (), .shamt (),
    .imm_i (), .imm_s (), .imm_sb (), .imm_u (), .imm_uj ()
  );

  wire   [4:0] inst1_rs1_Dhl;
  wire   [4:0] inst1_rs2_Dhl;
  wire   [4:0] inst1_rd_Dhl;

  riscv_InstMsgFromBits inst1_msg_from_bits
  (
    .msg (ir1_Dhl), .opcode (), .rs1 (inst1_rs1_Dhl), .rs2 (inst1_rs2_Dhl),
    .rd (inst1_rd_Dhl), .funct3 (), .funct7 (), .shamt (),
    .imm_i (), .imm_s (), .imm_sb (), .imm_u (), .imm_uj ()
  );

  wire [4:0] rs10 = inst0_rs1_Dhl;
  wire [4:0] rs20 = inst0_rs2_Dhl;
  wire [4:0] rd0  = inst0_rd_Dhl;
  wire [4:0] rs11 = inst1_rs1_Dhl;
  wire [4:0] rs21 = inst1_rs2_Dhl;
  wire [4:0] rd1  = inst1_rd_Dhl;

  localparam cs_sz = 39;
  reg [cs_sz-1:0] cs0;
  reg [cs_sz-1:0] cs1;

  always @ (*) begin
    cs0 = {cs_sz{1'bx}};
    casez ( ir0_Dhl )
      `RISCV_INST_MSG_LUI     :cs0={ y,  n,    br_none, pm_p,   am_0,    n,  bm_imm_u, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_AUIPC   :cs0={ y,  n,    br_none, pm_p,   am_pc,   n,  bm_imm_u, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_ADDI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_ORI     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLTI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLTIU   :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_XORI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_ANDI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLLI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SRLI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SRAI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_ADD     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SUB     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLL     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLT     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLTU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_XOR     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SRL     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SRA     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_OR      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_AND     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_LW      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_w, dmm_w,  wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_LB      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_b,  wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_LH      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_h,  wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_LBU     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_bu, wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_LHU     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_hu, wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_SW      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_w, dmm_w,  wm_mem, n,  rx, n   };
      `RISCV_INST_MSG_SB      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_b, dmm_b,  wm_mem, n,  rx, n   };
      `RISCV_INST_MSG_SH      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_h, dmm_h,  wm_mem, n,  rx, n   };
      `RISCV_INST_MSG_JAL     :cs0={ y,  y,    br_none, pm_j,   am_pc4,  n,  bm_0,     n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_JALR    :cs0={ y,  y,    br_none, pm_r,   am_pc4,  y,  bm_0,     n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_BNE     :cs0={ y,  n,    br_bne,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BEQ     :cs0={ y,  n,    br_beq,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BLT     :cs0={ y,  n,    br_blt,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BGE     :cs0={ y,  n,    br_bge,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BLTU    :cs0={ y,  n,    br_bltu, pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BGEU    :cs0={ y,  n,    br_bgeu, pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_MUL     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_mul,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_DIV     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_div,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_REM     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_rem,  y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_DIVU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_divu, y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_REMU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_remu, y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_CSRW    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_0,     y,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, n,  rx, y   };
    endcase
  end

  always @ (*) begin
    cs1 = {cs_sz{1'bx}};
    casez ( ir1_Dhl )
      `RISCV_INST_MSG_LUI     :cs1={ y,  n,    br_none, pm_p,   am_0,    n,  bm_imm_u, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_AUIPC   :cs1={ y,  n,    br_none, pm_p,   am_pc,   n,  bm_imm_u, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_ADDI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_ORI     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLTI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLTIU   :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_XORI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_ANDI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLLI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SRLI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SRAI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_ADD     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SUB     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLL     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLT     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLTU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_XOR     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SRL     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SRA     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_OR      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_AND     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_LW      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_w, dmm_w,  wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_LB      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_b,  wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_LH      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_h,  wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_LBU     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_bu, wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_LHU     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_hu, wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_SW      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_w, dmm_w,  wm_mem, n,  rx, n   };
      `RISCV_INST_MSG_SB      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_b, dmm_b,  wm_mem, n,  rx, n   };
      `RISCV_INST_MSG_SH      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_h, dmm_h,  wm_mem, n,  rx, n   };
      `RISCV_INST_MSG_JAL     :cs1={ y,  y,    br_none, pm_j,   am_pc4,  n,  bm_0,     n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_JALR    :cs1={ y,  y,    br_none, pm_r,   am_pc4,  y,  bm_0,     n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_BNE     :cs1={ y,  n,    br_bne,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BEQ     :cs1={ y,  n,    br_beq,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BLT     :cs1={ y,  n,    br_blt,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BGE     :cs1={ y,  n,    br_bge,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BLTU    :cs1={ y,  n,    br_bltu, pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_BGEU    :cs1={ y,  n,    br_bgeu, pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `RISCV_INST_MSG_MUL     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_mul,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_DIV     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_div,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_REM     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_rem,  y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_DIVU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_divu, y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_REMU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_remu, y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_CSRW    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_0,     y,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, n,  rx, y   };
    endcase
  end

  //----------------------------------------------------------------------
  // Instruction Type Classification
  //----------------------------------------------------------------------
  // An instruction is can go to B if it's a load/store, branch, jump, muldiv, or CSR write

  wire inst0_is_alu = cs0[`RISCV_INST_MSG_INST_VAL]
    && (cs0[`RISCV_INST_MSG_MEM_REQ] == nr)
    && !cs0[`RISCV_INST_MSG_J_EN]
    && (cs0[`RISCV_INST_MSG_BR_SEL] == br_none)
    && !cs0[`RISCV_INST_MSG_MULDIV_EN]
    && !cs0[`RISCV_INST_MSG_CSR_WEN];

  wire inst1_is_alu = cs1[`RISCV_INST_MSG_INST_VAL]
    && (cs1[`RISCV_INST_MSG_MEM_REQ] == nr)
    && !cs1[`RISCV_INST_MSG_J_EN]
    && (cs1[`RISCV_INST_MSG_BR_SEL] == br_none)
    && !cs1[`RISCV_INST_MSG_MULDIV_EN]
    && !cs1[`RISCV_INST_MSG_CSR_WEN];

  // Control-flow instruction (branch or jump) —- stalls inst1 when in inst0
  wire inst0_is_control = cs0[`RISCV_INST_MSG_J_EN]
    || (cs0[`RISCV_INST_MSG_BR_SEL] != br_none);

  //----------------------------------------------------------------------
  // Steering Logic: Dual-Issue Superscalar (Part 2)
  //----------------------------------------------------------------------
  // Steering table:
  //   ALU+ALU       → inst0→A, inst1→B  (swap=0)
  //   ALU+Non-ALU   → inst0→B, inst1→A  (swap=1)
  //   Non-ALU+ALU   → inst0→A, inst1→B  (swap=0)
  //   Non-ALU+Non-ALU → inst0→A, stall inst1

  wire swap = inst0_is_alu && !inst1_is_alu && !second_inst_pending;

  // tracks when inst1 from a pair is waiting to issue
  reg second_inst_pending;

  always @(posedge clk) begin
    if ( reset || squash_Dhl || brj_taken_Dhl ) begin
      second_inst_pending <= 1'b0;
    end
    else if ( inst_val_Dhl && !stall_Dhl ) begin
      if ( !second_inst_pending && stall_inst1 )
        second_inst_pending <= 1'b1;
      else
        second_inst_pending <= 1'b0;
    end
  end

  // both non-ALU
  wire stall_structural = !inst0_is_alu && !inst1_is_alu;

  // RAW --> inst1 reads what inst0 writes
  wire [4:0] inst0_waddr_raw = cs0[`RISCV_INST_MSG_RF_WADDR];
  wire       inst0_wen_raw   = cs0[`RISCV_INST_MSG_RF_WEN];
  wire       inst1_rs1_en_raw = cs1[`RISCV_INST_MSG_RS1_EN];
  wire       inst1_rs2_en_raw = cs1[`RISCV_INST_MSG_RS2_EN];

  wire raw_hazard = inst0_wen_raw && (inst0_waddr_raw != 5'd0) && (
    (inst1_rs1_en_raw && (inst1_rs1_Dhl == inst0_waddr_raw)) ||
    (inst1_rs2_en_raw && (inst1_rs2_Dhl == inst0_waddr_raw))
  );

  // WAW ---> both write same non-zero register
  wire [4:0] inst1_waddr_raw = cs1[`RISCV_INST_MSG_RF_WADDR];
  wire       inst1_wen_raw   = cs1[`RISCV_INST_MSG_RF_WEN];

  wire waw_hazard = inst0_wen_raw && inst1_wen_raw
    && (inst0_waddr_raw == inst1_waddr_raw) && (inst0_waddr_raw != 5'd0);

  // Combined stall for inst1 (only evaluated when second_inst_pending=0)
  wire stall_inst1 = stall_structural || raw_hazard || waw_hazard
                   || inst0_is_control || stall_inst1_scoreboard;

  // Stall F/P when only inst0 issues this cycle
  wire stall_0_Dhl = inst_val_Dhl && !second_inst_pending && stall_inst1
                   && !stall_Dhl && !brj_taken_Dhl && !squash_Dhl;

  wire dual_issue = inst_val_Dhl && !stall_Dhl && !second_inst_pending && !stall_inst1;

  assign steering_mux_sel_Dhl = second_inst_pending ? 1'b1
                              : (stall_inst1 || !inst_val_Dhl) ? 1'b0
                              : swap;

  //----------------------------------------------------------------------
  // csA / csB selection and instruction output
  //----------------------------------------------------------------------

  reg [cs_sz-1:0] csA;
  reg [cs_sz-1:0] csB;
  reg [4:0] instA_rs1_steer;
  reg [4:0] instA_rs2_steer;
  reg [4:0] instB_rs1_steer;
  reg [4:0] instB_rs2_steer;

  always @(*) begin
    if ( second_inst_pending ) begin
      // Issue inst1 through pipeline A only
      csA = cs1;
      csB = {cs_sz{1'b0}};
      instA_rs1_steer = inst1_rs1_Dhl;
      instA_rs2_steer = inst1_rs2_Dhl;
      instB_rs1_steer = 5'd0;
      instB_rs2_steer = 5'd0;
    end
    else if ( stall_inst1 || !inst_val_Dhl ) begin
      // single issue inst0 through pipeline A
      csA = cs0;
      csB = {cs_sz{1'b0}};
      instA_rs1_steer = inst0_rs1_Dhl;
      instA_rs2_steer = inst0_rs2_Dhl;
      instB_rs1_steer = 5'd0;
      instB_rs2_steer = 5'd0;
    end
    else if ( swap ) begin
      // dual issue -- swap inst0→B, inst1→A
      csA = cs1;
      csB = cs0;
      instA_rs1_steer = inst1_rs1_Dhl;
      instA_rs2_steer = inst1_rs2_Dhl;
      instB_rs1_steer = inst0_rs1_Dhl;
      instB_rs2_steer = inst0_rs2_Dhl;
    end
    else begin
      // dual issue --- inst0→A, inst1→B
      csA = cs0;
      csB = cs1;
      instA_rs1_steer = inst0_rs1_Dhl;
      instA_rs2_steer = inst0_rs2_Dhl;
      instB_rs1_steer = inst1_rs1_Dhl;
      instB_rs2_steer = inst1_rs2_Dhl;
    end
  end

  assign instA_Dhl = (steering_mux_sel_Dhl == 1'b0) ? ir0_Dhl : ir1_Dhl;
  assign instB_Dhl = (dual_issue) ?
                     ((swap) ? ir0_Dhl : ir1_Dhl)
                     : 32'h00000013; // NOP

  // Jump and Branch Controls (pipe A)
  wire       brj_taken_Dhl = ( inst_val_Dhl && csA[`RISCV_INST_MSG_J_EN] );
  wire [2:0] br_sel_Dhl    = csA[`RISCV_INST_MSG_BR_SEL];
  wire [1:0] pc_mux_sel_Dhl = csA[`RISCV_INST_MSG_PC_SEL];

  //----------------------------------------------------------------------
  // Pipeline A decode signals
  //----------------------------------------------------------------------

  wire [3:0] alu0_fn_Dhl      = csA[`RISCV_INST_MSG_ALU_FN];
  assign muldivreq_msg_fn_Dhl = csA[`RISCV_INST_MSG_MULDIV_FN];
  wire muldivreq_val_Dhl      = csA[`RISCV_INST_MSG_MULDIV_EN];
  wire muldiv_mux_sel_Dhl     = csA[`RISCV_INST_MSG_MULDIV_SEL];
  wire execute_mux_sel_Dhl    = csA[`RISCV_INST_MSG_MULDIV_EN];
  wire is_load_Dhl            = ( csA[`RISCV_INST_MSG_MEM_REQ] == ld );
  wire dmemreq_msg_rw_Dhl     = ( csA[`RISCV_INST_MSG_MEM_REQ] == st );
  wire [1:0] dmemreq_msg_len_Dhl = csA[`RISCV_INST_MSG_MEM_LEN];
  wire dmemreq_val_Dhl        = ( csA[`RISCV_INST_MSG_MEM_REQ] != nr );
  wire [2:0] dmemresp_mux_sel_Dhl = csA[`RISCV_INST_MSG_MEM_SEL];
  wire memex_mux_sel_Dhl      = csA[`RISCV_INST_MSG_WB_SEL];
  wire rf0_wen_Dhl            = csA[`RISCV_INST_MSG_RF_WEN];
  wire [4:0] rf0_waddr_Dhl    = csA[`RISCV_INST_MSG_RF_WADDR];
  wire csr_wen_Dhl            = csA[`RISCV_INST_MSG_CSR_WEN];
  wire [11:0] csr_addr_Dhl    = instA_Dhl[31:20];

  //----------------------------------------------------------------------
  // Pipeline B decode signals
  //----------------------------------------------------------------------

  wire [3:0] alu1_fn_Dhl   = csB[`RISCV_INST_MSG_ALU_FN];
  wire rf1_wen_Dhl          = csB[`RISCV_INST_MSG_RF_WEN];
  wire [4:0] rf1_waddr_Dhl  = csB[`RISCV_INST_MSG_RF_WADDR];

  //----------------------------------------------------------------------
  // Operand Mux Selects
  //----------------------------------------------------------------------

  assign opA0_mux_sel_Dhl = csA[`RISCV_INST_MSG_OP0_SEL];
  assign opA1_mux_sel_Dhl = csA[`RISCV_INST_MSG_OP1_SEL];
  assign opB0_mux_sel_Dhl = csB[`RISCV_INST_MSG_OP0_SEL];
  assign opB1_mux_sel_Dhl = csB[`RISCV_INST_MSG_OP1_SEL];

  //----------------------------------------------------------------------
  // Scoreboard bypass logic
  //----------------------------------------------------------------------
  // The scoreboard is implemented as per-stage tracking registers for
  // both pipelines. Bypass priority: AX0 > BX0 > AX1 > BX1 > ... > BW.

  wire [4:0] rsA1 = instA_rs1_steer;
  wire [4:0] rsA2 = instA_rs2_steer;
  wire       rsA1_en = csA[`RISCV_INST_MSG_RS1_EN];
  wire       rsA2_en = csA[`RISCV_INST_MSG_RS2_EN];
  wire [4:0] rsB1 = instB_rs1_steer;
  wire [4:0] rsB2 = instB_rs2_steer;
  wire       rsB1_en = csB[`RISCV_INST_MSG_RS1_EN];
  wire       rsB2_en = csB[`RISCV_INST_MSG_RS2_EN];

  // Pipeline A operand 0 bypass
  wire a0_AX0 = rsA1_en && rf0_wen_X0hl && (rsA1==rf0_waddr_X0hl) && (rf0_waddr_X0hl!=5'd0) && inst_val_X0hl;
  wire a0_BX0 = rsA1_en && rf1_wen_X0hl && (rsA1==rf1_waddr_X0hl) && (rf1_waddr_X0hl!=5'd0) && inst_val_B_X0hl;
  wire a0_AX1 = rsA1_en && rf0_wen_X1hl && (rsA1==rf0_waddr_X1hl) && (rf0_waddr_X1hl!=5'd0) && inst_val_X1hl;
  wire a0_BX1 = rsA1_en && rf1_wen_X1hl && (rsA1==rf1_waddr_X1hl) && (rf1_waddr_X1hl!=5'd0) && inst_val_B_X1hl;
  wire a0_AX2 = rsA1_en && rf0_wen_X2hl && (rsA1==rf0_waddr_X2hl) && (rf0_waddr_X2hl!=5'd0) && inst_val_X2hl;
  wire a0_BX2 = rsA1_en && rf1_wen_X2hl && (rsA1==rf1_waddr_X2hl) && (rf1_waddr_X2hl!=5'd0) && inst_val_B_X2hl;
  wire a0_AX3 = rsA1_en && rf0_wen_X3hl && (rsA1==rf0_waddr_X3hl) && (rf0_waddr_X3hl!=5'd0) && inst_val_X3hl;
  wire a0_BX3 = rsA1_en && rf1_wen_X3hl && (rsA1==rf1_waddr_X3hl) && (rf1_waddr_X3hl!=5'd0) && inst_val_B_X3hl;
  wire a0_AW  = rsA1_en && rf0_wen_Whl  && (rsA1==rf0_waddr_Whl)  && (rf0_waddr_Whl !=5'd0) && inst_val_Whl;
  wire a0_BW  = rsA1_en && rf1_wen_Whl  && (rsA1==rf1_waddr_Whl)  && (rf1_waddr_Whl !=5'd0) && inst_val_B_Whl;

  assign opA0_byp_mux_sel_Dhl
    = a0_AX0 ? am_AX0_byp : a0_BX0 ? am_BX0_byp
    : a0_AX1 ? am_AX1_byp : a0_BX1 ? am_BX1_byp
    : a0_AX2 ? am_AX2_byp : a0_BX2 ? am_BX2_byp
    : a0_AX3 ? am_AX3_byp : a0_BX3 ? am_BX3_byp
    : a0_AW  ? am_AW_byp  : a0_BW  ? am_BW_byp
    :           am_r0;

  // Pipeline A operand 1 bypass
  wire a1_AX0 = rsA2_en && rf0_wen_X0hl && (rsA2==rf0_waddr_X0hl) && (rf0_waddr_X0hl!=5'd0) && inst_val_X0hl;
  wire a1_BX0 = rsA2_en && rf1_wen_X0hl && (rsA2==rf1_waddr_X0hl) && (rf1_waddr_X0hl!=5'd0) && inst_val_B_X0hl;
  wire a1_AX1 = rsA2_en && rf0_wen_X1hl && (rsA2==rf0_waddr_X1hl) && (rf0_waddr_X1hl!=5'd0) && inst_val_X1hl;
  wire a1_BX1 = rsA2_en && rf1_wen_X1hl && (rsA2==rf1_waddr_X1hl) && (rf1_waddr_X1hl!=5'd0) && inst_val_B_X1hl;
  wire a1_AX2 = rsA2_en && rf0_wen_X2hl && (rsA2==rf0_waddr_X2hl) && (rf0_waddr_X2hl!=5'd0) && inst_val_X2hl;
  wire a1_BX2 = rsA2_en && rf1_wen_X2hl && (rsA2==rf1_waddr_X2hl) && (rf1_waddr_X2hl!=5'd0) && inst_val_B_X2hl;
  wire a1_AX3 = rsA2_en && rf0_wen_X3hl && (rsA2==rf0_waddr_X3hl) && (rf0_waddr_X3hl!=5'd0) && inst_val_X3hl;
  wire a1_BX3 = rsA2_en && rf1_wen_X3hl && (rsA2==rf1_waddr_X3hl) && (rf1_waddr_X3hl!=5'd0) && inst_val_B_X3hl;
  wire a1_AW  = rsA2_en && rf0_wen_Whl  && (rsA2==rf0_waddr_Whl)  && (rf0_waddr_Whl !=5'd0) && inst_val_Whl;
  wire a1_BW  = rsA2_en && rf1_wen_Whl  && (rsA2==rf1_waddr_Whl)  && (rf1_waddr_Whl !=5'd0) && inst_val_B_Whl;

  assign opA1_byp_mux_sel_Dhl
    = a1_AX0 ? bm_AX0_byp : a1_BX0 ? bm_BX0_byp
    : a1_AX1 ? bm_AX1_byp : a1_BX1 ? bm_BX1_byp
    : a1_AX2 ? bm_AX2_byp : a1_BX2 ? bm_BX2_byp
    : a1_AX3 ? bm_AX3_byp : a1_BX3 ? bm_BX3_byp
    : a1_AW  ? bm_AW_byp  : a1_BW  ? bm_BW_byp
    :           bm_r1;

  // Pipeline B operand 0 bypass
  wire b0_AX0 = rsB1_en && rf0_wen_X0hl && (rsB1==rf0_waddr_X0hl) && (rf0_waddr_X0hl!=5'd0) && inst_val_X0hl;
  wire b0_BX0 = rsB1_en && rf1_wen_X0hl && (rsB1==rf1_waddr_X0hl) && (rf1_waddr_X0hl!=5'd0) && inst_val_B_X0hl;
  wire b0_AX1 = rsB1_en && rf0_wen_X1hl && (rsB1==rf0_waddr_X1hl) && (rf0_waddr_X1hl!=5'd0) && inst_val_X1hl;
  wire b0_BX1 = rsB1_en && rf1_wen_X1hl && (rsB1==rf1_waddr_X1hl) && (rf1_waddr_X1hl!=5'd0) && inst_val_B_X1hl;
  wire b0_AX2 = rsB1_en && rf0_wen_X2hl && (rsB1==rf0_waddr_X2hl) && (rf0_waddr_X2hl!=5'd0) && inst_val_X2hl;
  wire b0_BX2 = rsB1_en && rf1_wen_X2hl && (rsB1==rf1_waddr_X2hl) && (rf1_waddr_X2hl!=5'd0) && inst_val_B_X2hl;
  wire b0_AX3 = rsB1_en && rf0_wen_X3hl && (rsB1==rf0_waddr_X3hl) && (rf0_waddr_X3hl!=5'd0) && inst_val_X3hl;
  wire b0_BX3 = rsB1_en && rf1_wen_X3hl && (rsB1==rf1_waddr_X3hl) && (rf1_waddr_X3hl!=5'd0) && inst_val_B_X3hl;
  wire b0_AW  = rsB1_en && rf0_wen_Whl  && (rsB1==rf0_waddr_Whl)  && (rf0_waddr_Whl !=5'd0) && inst_val_Whl;
  wire b0_BW  = rsB1_en && rf1_wen_Whl  && (rsB1==rf1_waddr_Whl)  && (rf1_waddr_Whl !=5'd0) && inst_val_B_Whl;

  assign opB0_byp_mux_sel_Dhl
    = b0_AX0 ? am_AX0_byp : b0_BX0 ? am_BX0_byp
    : b0_AX1 ? am_AX1_byp : b0_BX1 ? am_BX1_byp
    : b0_AX2 ? am_AX2_byp : b0_BX2 ? am_BX2_byp
    : b0_AX3 ? am_AX3_byp : b0_BX3 ? am_BX3_byp
    : b0_AW  ? am_AW_byp  : b0_BW  ? am_BW_byp
    :           am_r0;

  // Pipeline B operand 1 bypass
  wire b1_AX0 = rsB2_en && rf0_wen_X0hl && (rsB2==rf0_waddr_X0hl) && (rf0_waddr_X0hl!=5'd0) && inst_val_X0hl;
  wire b1_BX0 = rsB2_en && rf1_wen_X0hl && (rsB2==rf1_waddr_X0hl) && (rf1_waddr_X0hl!=5'd0) && inst_val_B_X0hl;
  wire b1_AX1 = rsB2_en && rf0_wen_X1hl && (rsB2==rf0_waddr_X1hl) && (rf0_waddr_X1hl!=5'd0) && inst_val_X1hl;
  wire b1_BX1 = rsB2_en && rf1_wen_X1hl && (rsB2==rf1_waddr_X1hl) && (rf1_waddr_X1hl!=5'd0) && inst_val_B_X1hl;
  wire b1_AX2 = rsB2_en && rf0_wen_X2hl && (rsB2==rf0_waddr_X2hl) && (rf0_waddr_X2hl!=5'd0) && inst_val_X2hl;
  wire b1_BX2 = rsB2_en && rf1_wen_X2hl && (rsB2==rf1_waddr_X2hl) && (rf1_waddr_X2hl!=5'd0) && inst_val_B_X2hl;
  wire b1_AX3 = rsB2_en && rf0_wen_X3hl && (rsB2==rf0_waddr_X3hl) && (rf0_waddr_X3hl!=5'd0) && inst_val_X3hl;
  wire b1_BX3 = rsB2_en && rf1_wen_X3hl && (rsB2==rf1_waddr_X3hl) && (rf1_waddr_X3hl!=5'd0) && inst_val_B_X3hl;
  wire b1_AW  = rsB2_en && rf0_wen_Whl  && (rsB2==rf0_waddr_Whl)  && (rf0_waddr_Whl !=5'd0) && inst_val_Whl;
  wire b1_BW  = rsB2_en && rf1_wen_Whl  && (rsB2==rf1_waddr_Whl)  && (rf1_waddr_Whl !=5'd0) && inst_val_B_Whl;

  assign opB1_byp_mux_sel_Dhl
    = b1_AX0 ? bm_AX0_byp : b1_BX0 ? bm_BX0_byp
    : b1_AX1 ? bm_AX1_byp : b1_BX1 ? bm_BX1_byp
    : b1_AX2 ? bm_AX2_byp : b1_BX2 ? bm_BX2_byp
    : b1_AX3 ? bm_AX3_byp : b1_BX3 ? bm_BX3_byp
    : b1_AW  ? bm_AW_byp  : b1_BW  ? bm_BW_byp
    :           bm_r1;

  //----------------------------------------------------------------------
  // Scoreboard stall logic
  //----------------------------------------------------------------------
  // Stalls the issuing instruction (pipeline A) if its sources depend on a load (at AX0 or AX1) or muldiv (at AX0-AX3) in pipeline A
  // No stall needed from B (only ALU instr)

  wire [4:0] issuing_rs1 = instA_rs1_steer;
  wire [4:0] issuing_rs2 = instA_rs2_steer;
  wire       issuing_rs1_en = csA[`RISCV_INST_MSG_RS1_EN];
  wire       issuing_rs2_en = csA[`RISCV_INST_MSG_RS2_EN];

  wire stall_load_use_A = inst_val_Dhl && (
      ( inst_val_X0hl && issuing_rs1_en && rf0_wen_X0hl && (issuing_rs1==rf0_waddr_X0hl) && (rf0_waddr_X0hl!=5'd0) && is_load_X0hl )
   || ( inst_val_X1hl && issuing_rs1_en && rf0_wen_X1hl && (issuing_rs1==rf0_waddr_X1hl) && (rf0_waddr_X1hl!=5'd0) && is_load_X1hl )
   || ( inst_val_X0hl && issuing_rs2_en && rf0_wen_X0hl && (issuing_rs2==rf0_waddr_X0hl) && (rf0_waddr_X0hl!=5'd0) && is_load_X0hl )
   || ( inst_val_X1hl && issuing_rs2_en && rf0_wen_X1hl && (issuing_rs2==rf0_waddr_X1hl) && (rf0_waddr_X1hl!=5'd0) && is_load_X1hl )
  );

  wire stall_muldiv_use_A = inst_val_Dhl && (
      ( inst_val_X0hl && issuing_rs1_en && rf0_wen_X0hl && (issuing_rs1==rf0_waddr_X0hl) && (rf0_waddr_X0hl!=5'd0) && is_muldiv_X0hl )
   || ( inst_val_X1hl && issuing_rs1_en && rf0_wen_X1hl && (issuing_rs1==rf0_waddr_X1hl) && (rf0_waddr_X1hl!=5'd0) && is_muldiv_X1hl )
   || ( inst_val_X2hl && issuing_rs1_en && rf0_wen_X2hl && (issuing_rs1==rf0_waddr_X2hl) && (rf0_waddr_X2hl!=5'd0) && is_muldiv_X2hl )
   || ( inst_val_X3hl && issuing_rs1_en && rf0_wen_X3hl && (issuing_rs1==rf0_waddr_X3hl) && (rf0_waddr_X3hl!=5'd0) && is_muldiv_X3hl )
   || ( inst_val_X0hl && issuing_rs2_en && rf0_wen_X0hl && (issuing_rs2==rf0_waddr_X0hl) && (rf0_waddr_X0hl!=5'd0) && is_muldiv_X0hl )
   || ( inst_val_X1hl && issuing_rs2_en && rf0_wen_X1hl && (issuing_rs2==rf0_waddr_X1hl) && (rf0_waddr_X1hl!=5'd0) && is_muldiv_X1hl )
   || ( inst_val_X2hl && issuing_rs2_en && rf0_wen_X2hl && (issuing_rs2==rf0_waddr_X2hl) && (rf0_waddr_X2hl!=5'd0) && is_muldiv_X2hl )
   || ( inst_val_X3hl && issuing_rs2_en && rf0_wen_X3hl && (issuing_rs2==rf0_waddr_X3hl) && (rf0_waddr_X3hl!=5'd0) && is_muldiv_X3hl )
  );

  // Scoreboard stall for inst1
  wire inst1_s1_en = cs1[`RISCV_INST_MSG_RS1_EN];
  wire inst1_s2_en = cs1[`RISCV_INST_MSG_RS2_EN];

  wire stall_inst1_scoreboard = inst_val_Dhl && (
      ( inst_val_X0hl && inst1_s1_en && rf0_wen_X0hl && (inst1_rs1_Dhl==rf0_waddr_X0hl) && (rf0_waddr_X0hl!=5'd0) && (is_load_X0hl||is_muldiv_X0hl) )
   || ( inst_val_X1hl && inst1_s1_en && rf0_wen_X1hl && (inst1_rs1_Dhl==rf0_waddr_X1hl) && (rf0_waddr_X1hl!=5'd0) && (is_load_X1hl||is_muldiv_X1hl) )
   || ( inst_val_X2hl && inst1_s1_en && rf0_wen_X2hl && (inst1_rs1_Dhl==rf0_waddr_X2hl) && (rf0_waddr_X2hl!=5'd0) && is_muldiv_X2hl )
   || ( inst_val_X3hl && inst1_s1_en && rf0_wen_X3hl && (inst1_rs1_Dhl==rf0_waddr_X3hl) && (rf0_waddr_X3hl!=5'd0) && is_muldiv_X3hl )
   || ( inst_val_X0hl && inst1_s2_en && rf0_wen_X0hl && (inst1_rs2_Dhl==rf0_waddr_X0hl) && (rf0_waddr_X0hl!=5'd0) && (is_load_X0hl||is_muldiv_X0hl) )
   || ( inst_val_X1hl && inst1_s2_en && rf0_wen_X1hl && (inst1_rs2_Dhl==rf0_waddr_X1hl) && (rf0_waddr_X1hl!=5'd0) && (is_load_X1hl||is_muldiv_X1hl) )
   || ( inst_val_X2hl && inst1_s2_en && rf0_wen_X2hl && (inst1_rs2_Dhl==rf0_waddr_X2hl) && (rf0_waddr_X2hl!=5'd0) && is_muldiv_X2hl )
   || ( inst_val_X3hl && inst1_s2_en && rf0_wen_X3hl && (inst1_rs2_Dhl==rf0_waddr_X3hl) && (rf0_waddr_X3hl!=5'd0) && is_muldiv_X3hl )
  );

  // Squash and aggregate stall
  wire squash_Dhl = ( inst_val_X0hl && brj_taken_X0hl );

  assign stall_Dhl = stall_X0hl || stall_load_use_A || stall_muldiv_use_A;

  wire bubble_sel_Dhl  = ( squash_Dhl || stall_Dhl );
  wire bubble_next_Dhl = ( !bubble_sel_Dhl ) ? bubble_Dhl
                       : ( bubble_sel_Dhl )  ? 1'b1
                       :                       1'bx;

  // Pipeline B bubble: only valid during dual issue
  wire bubble_next_B_Dhl = dual_issue ? bubble_Dhl : 1'b1;

  //----------------------------------------------------------------------
  // X0 <- D
  //----------------------------------------------------------------------

  reg [31:0] irA_X0hl;
  reg [31:0] irB_X0hl;
  reg  [2:0] br_sel_X0hl;
  reg  [3:0] alu0_fn_X0hl;
  reg  [3:0] alu1_fn_X0hl;
  reg        muldivreq_val_X0hl;
  reg  [2:0] muldivreq_msg_fn_X0hl;
  reg        muldiv_mux_sel_X0hl;
  reg        execute_mux_sel_X0hl;
  reg        is_load_X0hl;
  reg        is_muldiv_X0hl;
  reg        dmemreq_msg_rw_X0hl;
  reg  [1:0] dmemreq_msg_len_X0hl;
  reg        dmemreq_val_X0hl;
  reg  [2:0] dmemresp_mux_sel_X0hl;
  reg        memex_mux_sel_X0hl;
  reg        rf0_wen_X0hl;
  reg  [4:0] rf0_waddr_X0hl;
  reg        rf1_wen_X0hl;
  reg  [4:0] rf1_waddr_X0hl;
  reg        csr_wen_X0hl;
  reg [11:0] csr_addr_X0hl;
  reg        bubble_X0hl;
  reg        bubble_B_X0hl;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_X0hl   <= 1'b1;
      bubble_B_X0hl <= 1'b1;
    end
    else if( !stall_X0hl ) begin
      irA_X0hl              <= instA_Dhl;
      irB_X0hl              <= instB_Dhl;
      br_sel_X0hl           <= br_sel_Dhl;
      alu0_fn_X0hl          <= alu0_fn_Dhl;
      alu1_fn_X0hl          <= alu1_fn_Dhl;
      muldivreq_val_X0hl    <= muldivreq_val_Dhl;
      muldivreq_msg_fn_X0hl <= muldivreq_msg_fn_Dhl;
      muldiv_mux_sel_X0hl   <= muldiv_mux_sel_Dhl;
      execute_mux_sel_X0hl  <= execute_mux_sel_Dhl;
      is_load_X0hl          <= is_load_Dhl;
      is_muldiv_X0hl        <= muldivreq_val_Dhl;
      dmemreq_msg_rw_X0hl   <= dmemreq_msg_rw_Dhl;
      dmemreq_msg_len_X0hl  <= dmemreq_msg_len_Dhl;
      dmemreq_val_X0hl      <= dmemreq_val_Dhl;
      dmemresp_mux_sel_X0hl <= dmemresp_mux_sel_Dhl;
      memex_mux_sel_X0hl    <= memex_mux_sel_Dhl;
      rf0_wen_X0hl          <= rf0_wen_Dhl;
      rf0_waddr_X0hl        <= rf0_waddr_Dhl;
      rf1_wen_X0hl          <= rf1_wen_Dhl;
      rf1_waddr_X0hl        <= rf1_waddr_Dhl;
      csr_wen_X0hl          <= csr_wen_Dhl;
      csr_addr_X0hl         <= csr_addr_Dhl;
      bubble_X0hl           <= bubble_next_Dhl;
      bubble_B_X0hl         <= bubble_next_B_Dhl;
    end
  end

  assign aluA_fn_X0hl = alu0_fn_X0hl;
  assign aluB_fn_X0hl = alu1_fn_X0hl;

  //----------------------------------------------------------------------
  // Execute Stage
  //----------------------------------------------------------------------

  wire inst_val_X0hl   = ( !bubble_X0hl && !squash_X0hl );
  wire inst_val_B_X0hl = ( !bubble_B_X0hl && !squash_X0hl );

  assign muldivreq_val = muldivreq_val_Dhl && inst_val_Dhl && (!bubble_next_Dhl);
  assign muldivresp_rdy = 1'b1;
  assign muldiv_stall_mult1 = stall_X1hl;

  assign dmemreq_msg_rw  = dmemreq_msg_rw_X0hl;
  assign dmemreq_msg_len = dmemreq_msg_len_X0hl;
  assign dmemreq_val     = ( inst_val_X0hl && !stall_X0hl && dmemreq_val_X0hl );

  wire bne_taken_X0hl  = ( ( br_sel_X0hl == br_bne ) && branch_cond_ne_X0hl );
  wire beq_taken_X0hl  = ( ( br_sel_X0hl == br_beq ) && branch_cond_eq_X0hl );
  wire blt_taken_X0hl  = ( ( br_sel_X0hl == br_blt ) && branch_cond_lt_X0hl );
  wire bltu_taken_X0hl = ( ( br_sel_X0hl == br_bltu) && branch_cond_ltu_X0hl);
  wire bge_taken_X0hl  = ( ( br_sel_X0hl == br_bge ) && branch_cond_ge_X0hl );
  wire bgeu_taken_X0hl = ( ( br_sel_X0hl == br_bgeu) && branch_cond_geu_X0hl);

  wire any_br_taken_X0hl = beq_taken_X0hl || bne_taken_X0hl || blt_taken_X0hl
                        || bltu_taken_X0hl || bge_taken_X0hl || bgeu_taken_X0hl;

  wire brj_taken_X0hl = ( inst_val_X0hl && any_br_taken_X0hl );

  wire squash_X0hl = 1'b0;
  wire stall_muldiv_X0hl = 1'b0;
  wire stall_imem_X0hl = !imemreq0_rdy || !imemreq1_rdy;
  wire stall_dmem_X0hl = ( dmemreq_val_X0hl && inst_val_X0hl && !dmemreq_rdy );

  assign stall_X0hl = ( stall_X1hl || stall_muldiv_X0hl || stall_imem_X0hl || stall_dmem_X0hl );

  wire bubble_sel_X0hl  = ( squash_X0hl || stall_X0hl );
  wire bubble_next_X0hl   = ( !bubble_sel_X0hl ) ? bubble_X0hl   : 1'b1;
  wire bubble_next_B_X0hl = ( !bubble_sel_X0hl ) ? bubble_B_X0hl : 1'b1;

  //----------------------------------------------------------------------
  // X1 <- X0
  //----------------------------------------------------------------------

  reg [31:0] irA_X1hl;
  reg [31:0] irB_X1hl;
  reg        is_load_X1hl;
  reg        is_muldiv_X1hl;
  reg        dmemreq_val_X1hl;
  reg        execute_mux_sel_X1hl;
  reg        muldiv_mux_sel_X1hl;
  reg        rf0_wen_X1hl;
  reg  [4:0] rf0_waddr_X1hl;
  reg        rf1_wen_X1hl;
  reg  [4:0] rf1_waddr_X1hl;
  reg        csr_wen_X1hl;
  reg  [4:0] csr_addr_X1hl;
  reg        bubble_X1hl;
  reg        bubble_B_X1hl;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      dmemreq_val_X1hl <= 1'b0;
      bubble_X1hl   <= 1'b1;
      bubble_B_X1hl <= 1'b1;
    end
    else if( !stall_X1hl ) begin
      irA_X1hl              <= irA_X0hl;
      irB_X1hl              <= irB_X0hl;
      is_load_X1hl          <= is_load_X0hl;
      is_muldiv_X1hl        <= is_muldiv_X0hl;
      dmemreq_val_X1hl      <= dmemreq_val;
      dmemresp_mux_sel_X1hl <= dmemresp_mux_sel_X0hl;
      memex_mux_sel_X1hl    <= memex_mux_sel_X0hl;
      execute_mux_sel_X1hl  <= execute_mux_sel_X0hl;
      muldiv_mux_sel_X1hl   <= muldiv_mux_sel_X0hl;
      rf0_wen_X1hl          <= rf0_wen_X0hl;
      rf0_waddr_X1hl        <= rf0_waddr_X0hl;
      rf1_wen_X1hl          <= rf1_wen_X0hl;
      rf1_waddr_X1hl        <= rf1_waddr_X0hl;
      csr_wen_X1hl          <= csr_wen_X0hl;
      csr_addr_X1hl         <= csr_addr_X0hl;
      bubble_X1hl           <= bubble_next_X0hl;
      bubble_B_X1hl         <= bubble_next_B_X0hl;
    end
  end

  //----------------------------------------------------------------------
  // X1 Stage
  //----------------------------------------------------------------------

  wire inst_val_X1hl   = ( !bubble_X1hl && !squash_X1hl );
  wire inst_val_B_X1hl = ( !bubble_B_X1hl && !squash_X1hl );

  assign dmemresp_queue_en_X1hl = ( stall_X1hl && dmemresp_val );
  wire dmemresp_queue_val_next_X1hl
    = stall_X1hl && ( dmemresp_val || dmemresp_queue_val_X1hl );

  wire squash_X1hl = 1'b0;

  wire stall_dmem_X1hl
    = ( !reset && dmemreq_val_X1hl && inst_val_X1hl && !dmemresp_val && !dmemresp_queue_val_X1hl );

  wire stall_imem_X1hl
    = ( !reset && !stall_0_Dhl && imemreq_val_Fhl && inst_val_Fhl && !imemresp0_val && !imemresp0_queue_val_Fhl )
   || ( !reset && !stall_0_Dhl && imemreq_val_Fhl && inst_val_Fhl && !imemresp1_val && !imemresp1_queue_val_Fhl );

  assign stall_X1hl = ( stall_imem_X1hl || stall_dmem_X1hl );

  wire bubble_sel_X1hl  = ( squash_X1hl || stall_X1hl );
  wire bubble_next_X1hl   = ( !bubble_sel_X1hl ) ? bubble_X1hl   : 1'b1;
  wire bubble_next_B_X1hl = ( !bubble_sel_X1hl ) ? bubble_B_X1hl : 1'b1;

  //----------------------------------------------------------------------
  // X2 <- X1
  //----------------------------------------------------------------------

  reg [31:0] irA_X2hl;
  reg [31:0] irB_X2hl;
  reg        is_muldiv_X2hl;
  reg        rf0_wen_X2hl;
  reg  [4:0] rf0_waddr_X2hl;
  reg        rf1_wen_X2hl;
  reg  [4:0] rf1_waddr_X2hl;
  reg        csr_wen_X2hl;
  reg  [4:0] csr_addr_X2hl;
  reg        execute_mux_sel_X2hl;
  reg        muldiv_mux_sel_X2hl;
  reg        bubble_X2hl;
  reg        bubble_B_X2hl;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_X2hl   <= 1'b1;
      bubble_B_X2hl <= 1'b1;
    end
    else if( !stall_X2hl ) begin
      irA_X2hl              <= irA_X1hl;
      irB_X2hl              <= irB_X1hl;
      is_muldiv_X2hl        <= is_muldiv_X1hl;
      muldiv_mux_sel_X2hl   <= muldiv_mux_sel_X1hl;
      rf0_wen_X2hl          <= rf0_wen_X1hl;
      rf0_waddr_X2hl        <= rf0_waddr_X1hl;
      rf1_wen_X2hl          <= rf1_wen_X1hl;
      rf1_waddr_X2hl        <= rf1_waddr_X1hl;
      csr_wen_X2hl          <= csr_wen_X1hl;
      csr_addr_X2hl         <= csr_addr_X1hl;
      execute_mux_sel_X2hl  <= execute_mux_sel_X1hl;
      bubble_X2hl           <= bubble_next_X1hl;
      bubble_B_X2hl         <= bubble_next_B_X1hl;
    end
    dmemresp_queue_val_X1hl <= dmemresp_queue_val_next_X1hl;
  end

  wire inst_val_X2hl   = ( !bubble_X2hl && !squash_X2hl );
  wire inst_val_B_X2hl = ( !bubble_B_X2hl && !squash_X2hl );
  wire squash_X2hl = 1'b0;
  assign stall_X2hl = 1'b0;

  wire bubble_next_X2hl   = ( stall_X2hl ) ? 1'b1 : bubble_X2hl;
  wire bubble_next_B_X2hl = ( stall_X2hl ) ? 1'b1 : bubble_B_X2hl;

  //----------------------------------------------------------------------
  // X3 <- X2
  //----------------------------------------------------------------------

  reg [31:0] irA_X3hl;
  reg [31:0] irB_X3hl;
  reg        is_muldiv_X3hl;
  reg        rf0_wen_X3hl;
  reg  [4:0] rf0_waddr_X3hl;
  reg        rf1_wen_X3hl;
  reg  [4:0] rf1_waddr_X3hl;
  reg        csr_wen_X3hl;
  reg  [4:0] csr_addr_X3hl;
  reg        bubble_X3hl;
  reg        bubble_B_X3hl;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_X3hl   <= 1'b1;
      bubble_B_X3hl <= 1'b1;
    end
    else if( !stall_X3hl ) begin
      irA_X3hl              <= irA_X2hl;
      irB_X3hl              <= irB_X2hl;
      is_muldiv_X3hl        <= is_muldiv_X2hl;
      muldiv_mux_sel_X3hl   <= muldiv_mux_sel_X2hl;
      rf0_wen_X3hl          <= rf0_wen_X2hl;
      rf0_waddr_X3hl        <= rf0_waddr_X2hl;
      rf1_wen_X3hl          <= rf1_wen_X2hl;
      rf1_waddr_X3hl        <= rf1_waddr_X2hl;
      csr_wen_X3hl          <= csr_wen_X2hl;
      csr_addr_X3hl         <= csr_addr_X2hl;
      execute_mux_sel_X3hl  <= execute_mux_sel_X2hl;
      bubble_X3hl           <= bubble_next_X2hl;
      bubble_B_X3hl         <= bubble_next_B_X2hl;
    end
  end

  wire inst_val_X3hl   = ( !bubble_X3hl && !squash_X3hl );
  wire inst_val_B_X3hl = ( !bubble_B_X3hl && !squash_X3hl );
  wire squash_X3hl = 1'b0;
  assign stall_X3hl = 1'b0;

  wire bubble_next_X3hl   = ( stall_X3hl ) ? 1'b1 : bubble_X3hl;
  wire bubble_next_B_X3hl = ( stall_X3hl ) ? 1'b1 : bubble_B_X3hl;

  //----------------------------------------------------------------------
  // W <- X3
  //----------------------------------------------------------------------

  reg [31:0] irA_Whl;
  reg [31:0] irB_Whl;
  reg        rf0_wen_Whl;
  reg  [4:0] rf0_waddr_Whl;
  reg        rf1_wen_Whl;
  reg  [4:0] rf1_waddr_Whl;
  reg        csr_wen_Whl;
  reg  [4:0] csr_addr_Whl;
  reg        bubble_Whl;
  reg        bubble_B_Whl;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_Whl   <= 1'b1;
      bubble_B_Whl <= 1'b1;
    end
    else if( !stall_Whl ) begin
      irA_Whl          <= irA_X3hl;
      irB_Whl          <= irB_X3hl;
      rf0_wen_Whl      <= rf0_wen_X3hl;
      rf0_waddr_Whl    <= rf0_waddr_X3hl;
      rf1_wen_Whl      <= rf1_wen_X3hl;
      rf1_waddr_Whl    <= rf1_waddr_X3hl;
      csr_wen_Whl      <= csr_wen_X3hl;
      csr_addr_Whl     <= csr_addr_X3hl;
      bubble_Whl       <= bubble_next_X3hl;
      bubble_B_Whl     <= bubble_next_B_X3hl;
    end
  end

  //----------------------------------------------------------------------
  // Writeback Stage
  //----------------------------------------------------------------------

  wire inst_val_Whl   = ( !bubble_Whl && !squash_Whl );
  wire inst_val_B_Whl = ( !bubble_B_Whl && !squash_Whl );

  assign rfA_wen_out_Whl = ( inst_val_Whl   && !stall_Whl && rf0_wen_Whl );
  assign rfA_waddr_Whl   = rf0_waddr_Whl;
  assign rfB_wen_out_Whl = ( inst_val_B_Whl && !stall_Whl && rf1_wen_Whl );
  assign rfB_waddr_Whl   = rf1_waddr_Whl;

  wire squash_Whl = 1'b0;
  assign stall_Whl = 1'b0;

  //----------------------------------------------------------------------
  // Debug registers for instruction disassembly
  //----------------------------------------------------------------------

  reg [31:0] irA_debug;
  reg [31:0] irB_debug;
  reg        inst_val_debug;

  always @ ( posedge clk ) begin
    irA_debug      <= irA_Whl;
    inst_val_debug <= inst_val_Whl;
    irB_debug      <= irB_Whl;
  end

  //----------------------------------------------------------------------
  // CSR register
  //----------------------------------------------------------------------

  reg csr_stats;

  always @ ( posedge clk ) begin
    if ( csr_wen_Whl && inst_val_Whl ) begin
      case ( csr_addr_Whl )
        12'd10 : csr_stats  <= proc2csr_data_Whl[0];
        12'd21 : csr_status <= proc2csr_data_Whl;
      endcase
    end
  end

//========================================================================
// Disassemble instructions
//========================================================================

  `ifndef SYNTHESIS

  riscv_InstMsgDisasm inst0_msg_disasm_D  ( .msg ( ir0_Dhl ) );
  riscv_InstMsgDisasm instA_msg_disasm_X0 ( .msg ( irA_X0hl ) );
  riscv_InstMsgDisasm instA_msg_disasm_X1 ( .msg ( irA_X1hl ) );
  riscv_InstMsgDisasm instA_msg_disasm_X2 ( .msg ( irA_X2hl ) );
  riscv_InstMsgDisasm instA_msg_disasm_X3 ( .msg ( irA_X3hl ) );
  riscv_InstMsgDisasm instA_msg_disasm_W  ( .msg ( irA_Whl ) );
  riscv_InstMsgDisasm instA_msg_disasm_debug ( .msg ( irA_debug ) );
  riscv_InstMsgDisasm inst1_msg_disasm_D  ( .msg ( ir1_Dhl ) );
  riscv_InstMsgDisasm instB_msg_disasm_X0 ( .msg ( irB_X0hl ) );
  riscv_InstMsgDisasm instB_msg_disasm_X1 ( .msg ( irB_X1hl ) );
  riscv_InstMsgDisasm instB_msg_disasm_X2 ( .msg ( irB_X2hl ) );
  riscv_InstMsgDisasm instB_msg_disasm_X3 ( .msg ( irB_X3hl ) );
  riscv_InstMsgDisasm instB_msg_disasm_W  ( .msg ( irB_Whl ) );
  riscv_InstMsgDisasm instB_msg_disasm_debug ( .msg ( irB_debug ) );

  `endif

//========================================================================
// Assertions
//========================================================================

  `ifndef SYNTHESIS

  reg overload = 1'b0;

  always @ ( posedge clk ) begin
    if ( inst_val_Dhl && !reset ) begin
      if ( !cs0[`RISCV_INST_MSG_INST_VAL] || !cs1[`RISCV_INST_MSG_INST_VAL] ) begin
        $display(" RTL-ERROR : %m : Illegal instruction!");
        if ( overload == 1'b1 ) $finish;
        overload = 1'b1;
      end
      else begin
        overload = 1'b0;
      end
    end
    else begin
      overload = 1'b0;
    end
  end

  `endif

//========================================================================
// Stats
//========================================================================

  `ifndef SYNTHESIS

  reg [31:0] num_inst    = 32'b0;
  reg [31:0] num_cycles  = 32'b0;
  reg        stats_en    = 1'b0;

  always @( posedge clk ) begin
    if ( !reset ) begin
      if ( stats_en || csr_stats ) begin
        num_cycles = num_cycles + 1;
        if ( inst_val_Dhl && !stall_Dhl ) begin
          if ( dual_issue )
            num_inst = num_inst + 2;
          else
            num_inst = num_inst + 1;
        end
      end
    end
  end

  `endif

endmodule

`endif

// vim: set textwidth=0 ts=2 sw=2 sts=2 :