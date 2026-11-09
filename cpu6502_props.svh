// ---------------- environment ----------------
logic f_past_valid = 1'b0;
logic f_rdy_q = 1'b0, f_rst_q = 1'b0;
always @(posedge clk) begin
  f_past_valid <= 1'b1;
  f_rdy_q      <= rdy;
  f_rst_q      <= rst_n;
end

always @(*) if (!f_past_valid) assume (!rst_n);

wire f_step  = f_past_valid && f_rst_q && rst_n &&  f_rdy_q;
wire f_stall = f_past_valid && f_rst_q && rst_n && !f_rdy_q;

// ---------------- shadow state ----------------
// record what CPU fetched so later checks can be stated
// independently of the core's own address arithmetic
logic [15:0] f_insn_pc;
logic [7:0]  f_op1, f_op2, f_ilo, f_ihi, f_rlo;
wire f_irq_take = nmi_pend || (!irq_n && !fI);

always @(posedge clk) if (rst_n && rdy) begin
  if (state == S_FETCH && !f_irq_take) f_insn_pc <= pc;
  if (state == S_DECODE) f_op1 <= din;
  if (state == S_ABS_HI) f_op2 <= din;
  if (state == S_IND_LO) f_ilo <= din;
  if (state == S_IND_HI) f_ihi <= din;
  if (state == S_RET_LO) f_rlo <= din;
end

// ---------------- reference models ----------------
function automatic [7:0] f_bcd(input [7:0] v);
  logic [7:0] t, u;
  t = v / 8'd10;
  u = v % 8'd10;
  f_bcd = {t[3:0], u[3:0]};
endfunction

logic [8:0]        f_add, f_sub;
logic signed [9:0] f_sadd, f_ssub;
logic signed [8:0] f_ddif;
logic [7:0]        f_adec, f_mdec, f_dsum, f_dadd_r, f_dsub_r;
logic              f_vadd, f_vsub, f_bcd_ok, f_dadd_c, f_dsub_c, f_taken;
logic              f_ctrl, f_memmode;
logic [1:0]        f_len;
logic [15:0]       f_ea_ref, f_ptr_ref, f_ret;

always @(*) begin
  // binary ADC/SBC; overflow from signed arithmetic
  f_add  = {1'b0, A} + {1'b0, din} + {8'd0, fC};
  f_sub  = {1'b0, A} - {1'b0, din} - {8'd0, ~fC};
  f_sadd = $signed({{2{A[7]}}, A}) + $signed({{2{din[7]}}, din}) + $signed({9'd0, fC});
  f_ssub = $signed({{2{A[7]}}, A}) - $signed({{2{din[7]}}, din}) - $signed({9'd0, ~fC});
  f_vadd = (f_sadd > 10'sd127) || (f_sadd < -10'sd128);
  f_vsub = (f_ssub > 10'sd127) || (f_ssub < -10'sd128);

  // decimal ADC/SBC: convert to integers, do the math, convert back
  f_bcd_ok = (A[7:4] <= 4'd9) && (A[3:0] <= 4'd9) &&
             (din[7:4] <= 4'd9) && (din[3:0] <= 4'd9);
  f_adec   = {4'd0, A[7:4]}   * 8'd10 + {4'd0, A[3:0]};
  f_mdec   = {4'd0, din[7:4]} * 8'd10 + {4'd0, din[3:0]};
  f_dsum   = f_adec + f_mdec + {7'd0, fC};
  f_dadd_c = (f_dsum >= 8'd100);
  f_dadd_r = f_bcd(f_dadd_c ? f_dsum - 8'd100 : f_dsum);
  f_ddif   = $signed({1'b0, f_adec}) - $signed({1'b0, f_mdec}) - $signed({8'd0, ~fC});
  f_dsub_c = ~f_ddif[8];
  f_dsub_r = f_bcd(f_dsub_c ? f_ddif[7:0] : f_ddif[7:0] + 8'd100);

  // branch condition straight from the opcode table
  case (ir)
    8'h10: f_taken = !fN;  8'h30: f_taken = fN;
    8'h50: f_taken = !fV;  8'h70: f_taken = fV;
    8'h90: f_taken = !fC;  8'hB0: f_taken = fC;
    8'hD0: f_taken = !fZ;  8'hF0: f_taken = fZ;
    default: f_taken = 1'b0;
  endcase

  // instruction length
  case (mode)
    M_IMP, M_ACC:               f_len = 2'd1;
    M_ABS, M_ABX, M_ABY, M_IND: f_len = 2'd3;
    default:                    f_len = 2'd2;
  endcase

  // effective address from the raw operand bytes
  f_memmode = 1'b1;
  case (mode)
    M_ZP:    f_ea_ref = {8'h00, f_op1};
    M_ZPX:   f_ea_ref = {8'h00, f_op1 + X};
    M_ZPY:   f_ea_ref = {8'h00, f_op1 + Y};
    M_ABS:   f_ea_ref = {f_op2, f_op1};
    M_ABX:   f_ea_ref = {f_op2, f_op1} + {8'h00, X};
    M_ABY:   f_ea_ref = {f_op2, f_op1} + {8'h00, Y};
    M_IZX:   f_ea_ref = {f_ihi, f_ilo};
    M_IZY:   f_ea_ref = {f_ihi, f_ilo} + {8'h00, Y};
    default: begin f_ea_ref = 16'h0000; f_memmode = 1'b0; end
  endcase

  case (mode)
    M_IZX:   f_ptr_ref = {8'h00, f_op1 + X};
    M_IZY:   f_ptr_ref = {8'h00, f_op1};
    default: f_ptr_ref = {f_op2, f_op1}; // JMP (ind)
  endcase

  f_ret  = f_insn_pc + 16'd2; // JSR/BRK push value
  f_ctrl = (op == O_BR)  || (op == O_JMP) || (op == O_JSR) ||
           (op == O_RTS) || (op == O_RTI) || (op == O_BRK);
end

wire f_exec_rd = (state == S_DECODE && mode == M_IMM) ||
                 (state == S_OPER && !is_store && !is_rmw);

// ---------------- invariants (also make k-induction pass) ----------------
always @(*) if (f_past_valid && rst_n) begin
  assert (state <= S_VEC_HI);
  assert (vec == 16'hFFFA || vec == 16'hFFFC || vec == 16'hFFFE);

  // writes only from write states; stack traffic only in page 1
  if (we)
    assert (state == S_OPER    || state == S_RMW_WR  || state == S_PUSH ||
            state == S_JSR_PCH || state == S_JSR_PCL ||
            state == S_INT_PCH || state == S_INT_PCL || state == S_INT_P);
  if (state == S_PUSH    || state == S_PULL    || state == S_RTI_P   ||
      state == S_RET_LO  || state == S_RET_HI  ||
      state == S_JSR_PCH || state == S_JSR_PCL ||
      state == S_INT_PCH || state == S_INT_PCL || state == S_INT_P)
    assert (addr[15:8] == 8'h01);

  // PC position within current instruction
  case (state)
    S_DECODE: assert (pc == f_insn_pc + 16'd1);
    S_ABS_HI, S_JSR_PCH, S_JSR_PCL, S_JSR_HI:
              assert (pc == f_insn_pc + 16'd2);
    S_IND_LO, S_IND_HI, S_OPER, S_RMW_WR, S_PUSH, S_PULL:
              assert (pc == f_insn_pc + {14'd0, f_len});
    default: ;
  endcase
  if (brk && (state == S_INT_PCH || state == S_INT_PCL || state == S_INT_P))
    assert (pc == f_ret); // BRK skips its padding byte

  // address generation
  if (state == S_ABS_HI || state == S_JSR_PCH || state == S_JSR_PCL || state == S_JSR_HI)
    assert (ea[7:0] == f_op1);
  if (state == S_IND_LO) assert (addr == f_ptr_ref);
  if (state == S_IND_HI) begin
    assert (addr == {f_ptr_ref[15:8], f_ptr_ref[7:0] + 8'd1});  // page wrap
    assert (ea[7:0] == f_ilo);
  end
  if ((state == S_OPER || state == S_RMW_WR) && f_memmode)
    assert (ea == f_ea_ref);
  if (state == S_RET_HI) assert (ea[7:0] == f_rlo);

  // JSR pushes the address of its own last byte
  if (state == S_JSR_PCH) assert (dout == f_ret[15:8]);
  if (state == S_JSR_PCL) assert (dout == f_ret[7:0]);
end

// ---------------- stall: rdy=0 freezes the CPU ----------------
always @(posedge clk) if (f_stall) begin
  assert (state == $past(state) && pc == $past(pc) && ir == $past(ir));
  assert (A == $past(A) && X == $past(X) && Y == $past(Y) && sp == $past(sp));
  assert (ea == $past(ea));
  assert ({fN, fV, fD, fI, fZ, fC} == $past({fN, fV, fD, fI, fZ, fC}));
end

// ---------------- one-step behavioral checks ----------------
always @(posedge clk) if (f_step) begin
  // fetch and interrupt masking
  if ($past(state) == S_FETCH) begin
    if ($past(fI) && !$past(nmi_pend)) assert (state != S_INT_PCH);
    if ($past(f_irq_take)) assert (state == S_INT_PCH);
    else begin
      assert (state == S_DECODE);
      assert (ir == $past(din));
      assert (pc == $past(pc) + 16'd1);
    end
  end
  if ($past(state) == S_INT_P) assert (fI && state == S_VEC_LO);

  // read-type instructions
  if ($past(f_exec_rd)) begin
    if ($past(op) == O_LDA) assert (A == $past(din));
    if ($past(op) == O_LDX) assert (X == $past(din));
    if ($past(op) == O_LDY) assert (Y == $past(din));
    if ($past(op) == O_AND) assert (A == $past(A & din));
    if ($past(op) == O_ORA) assert (A == $past(A | din));
    if ($past(op) == O_EOR) assert (A == $past(A ^ din));

    if ($past(op) == O_ADC) begin
      if (!$past(fD)) begin
        assert ({fC, A} == $past(f_add));
        assert (fV == $past(f_vadd));
      end else if ($past(f_bcd_ok)) begin
        assert (A  == $past(f_dadd_r));
        assert (fC == $past(f_dadd_c));
      end
    end
    if ($past(op) == O_SBC) begin
      if (!$past(fD)) begin
        assert (A  == $past(f_sub[7:0]));
        assert (fC == !$past(f_sub[8]));
        assert (fV == $past(f_vsub));
      end else if ($past(f_bcd_ok)) begin
        assert (A  == $past(f_dsub_r));
        assert (fC == $past(f_dsub_c));
      end
    end

    if ($past(op) == O_CMP)
      assert (fC == $past(A >= din) && fZ == $past(A == din) && A == $past(A));
    if ($past(op) == O_CPX)
      assert (fC == $past(X >= din) && fZ == $past(X == din));
    if ($past(op) == O_CPY)
      assert (fC == $past(Y >= din) && fZ == $past(Y == din));
    if ($past(op) == O_BIT)
      assert (fZ == $past((A & din) == 8'h00) && fN == $past(din[7]) &&
              fV == $past(din[6]) && A == $past(A));

    // N/Z mirror the destination register
    if ($past(op) == O_LDA || $past(op) == O_AND || $past(op) == O_ORA ||
        $past(op) == O_EOR || $past(op) == O_ADC || $past(op) == O_SBC)
      assert (fZ == (A == 8'h00) && fN == A[7]);
    if ($past(op) == O_LDX) assert (fZ == (X == 8'h00) && fN == X[7]);
    if ($past(op) == O_LDY) assert (fZ == (Y == 8'h00) && fN == Y[7]);
  end

  // read-modify-write on memory
  if ($past(state) == S_OPER && $past(is_rmw)) begin
    assert (state == S_RMW_WR);
    if ($past(op) == O_ASL) assert ({fC, dout} == {$past(din), 1'b0});
    if ($past(op) == O_ROL) assert ({fC, dout} == {$past(din), $past(fC)});
    if ($past(op) == O_LSR) assert ({dout, fC} == {1'b0, $past(din)});
    if ($past(op) == O_ROR) assert ({dout, fC} == {$past(fC), $past(din)});
    if ($past(op) == O_INC) assert (dout == $past(din) + 8'd1);
    if ($past(op) == O_DEC) assert (dout == $past(din) - 8'd1);
    assert (fZ == (dout == 8'h00) && fN == dout[7]);
  end

  // accumulator shifts
  if ($past(state) == S_DECODE && $past(mode) == M_ACC) begin
    if ($past(op) == O_ASL) assert ({fC, A} == {$past(A), 1'b0});
    if ($past(op) == O_ROL) assert ({fC, A} == {$past(A), $past(fC)});
    if ($past(op) == O_LSR) assert ({A, fC} == {1'b0, $past(A)});
    if ($past(op) == O_ROR) assert ({A, fC} == {$past(fC), $past(A)});
  end

  // branch target relative to the branch's own address
  if ($past(state) == S_DECODE && $past(op) == O_BR)
    assert (pc == f_insn_pc + 16'd2 +
            ($past(f_taken) ? {{8{$past(din[7])}}, $past(din)} : 16'd0));

  // jumps and returns
  if ($past(state) == S_ABS_HI && $past(op) == O_JMP && $past(mode) == M_ABS)
    assert (pc == {$past(din), f_op1});
  if ($past(state) == S_IND_HI && $past(mode) == M_IND)
    assert (pc == {$past(din), f_ilo});
  if ($past(state) == S_JSR_HI)
    assert (pc == {$past(din), f_op1});
  if ($past(state) == S_RET_HI)
    assert (pc == {$past(din), f_rlo} + (($past(op) == O_RTS) ? 16'd1 : 16'd0));

  // straight-line instructions advance PC by exactly their length
  if (state == S_FETCH && !f_ctrl &&
      ($past(state) == S_DECODE || $past(state) == S_OPER || $past(state) == S_RMW_WR ||
       $past(state) == S_PUSH   || $past(state) == S_PULL))
    assert (pc == f_insn_pc + {14'd0, f_len});

  // stack
  if ($past(state) == S_PUSH) assert (sp == $past(sp) - 8'd1);
  if ($past(state) == S_PULL && $past(op) == O_PLA) assert (A == $past(din));
end

// ---------------- reachability ----------------
always @(posedge clk) if (f_step) begin
  cover ($past(state) == S_RET_HI && $past(op) == O_RTI);
  cover ($past(state) == S_RET_HI && $past(op) == O_RTS);
  cover ($past(state) == S_INT_P  && $past(vec) == 16'hFFFA); // NMI taken
  cover ($past(f_exec_rd) && $past(op) == O_ADC && $past(fD) && fC); // BCD carry
  cover ($past(state) == S_IND_HI && $past(mode) == M_IND &&
         $past(ptr[7:0]) == 8'hFF); // JMP ($xxFF)
end
