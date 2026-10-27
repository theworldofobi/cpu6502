module cpu6502 (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        rdy,      // 1 = run, 0 = stall
  input  logic        irq_n,    // level-sensitive
  input  logic        nmi_n,    // falling-edge sensitive
  output logic [15:0] addr,
  input  logic [7:0]  din,
  output logic [7:0]  dout,
  output logic        we
);

  // ---------------- types ----------------
  typedef enum logic [3:0] {
    M_IMP, M_ACC, M_IMM, M_ZP, M_ZPX, M_ZPY, M_ABS, M_ABX, M_ABY,
    M_IZX, M_IZY, M_IND, M_REL
  } mode_t;

  typedef enum logic [5:0] {
    O_NOP, O_ADC, O_AND, O_ASL, O_BIT, O_BR,  O_BRK, O_CMP, O_CPX, O_CPY,
    O_DEC, O_DEX, O_DEY, O_EOR, O_INC, O_INX, O_INY, O_JMP, O_JSR, O_LDA,
    O_LDX, O_LDY, O_LSR, O_ORA, O_PHA, O_PHP, O_PLA, O_PLP, O_ROL, O_ROR,
    O_RTI, O_RTS, O_SBC, O_STA, O_STX, O_STY, O_TAX, O_TAY, O_TSX, O_TXA,
    O_TXS, O_TYA, O_CLC, O_SEC, O_CLI, O_SEI, O_CLV, O_CLD, O_SED
  } op_t;

  typedef enum logic [4:0] {
    S_FETCH, S_DECODE, S_ABS_HI, S_IND_LO, S_IND_HI, S_OPER, S_RMW_WR,
    S_PUSH, S_PULL, S_RTI_P, S_RET_LO, S_RET_HI,
    S_JSR_PCH, S_JSR_PCL, S_JSR_HI,
    S_INT_PCH, S_INT_PCL, S_INT_P, S_VEC_LO, S_VEC_HI
  } state_t;

  // ---------------- registers ----------------
  state_t      state;
  logic [15:0] pc, ea, ptr, vec;
  logic [7:0]  A, X, Y, sp, ir, dl;
  logic        fN, fV, fD, fI, fZ, fC;
  logic        brk, nmi_q, nmi_pend;

  mode_t mode;
  op_t   op;

  // ---------------- decoder (aaabbbcc structure) ----------------
  wire [2:0] ia = ir[7:5];
  wire [2:0] ib = ir[4:2];

  always_comb begin
    mode = M_IMP;
    op   = O_NOP;
    case (ir[1:0])
      2'b01: begin
        case (ib)
          3'd0: mode = M_IZX;  3'd1: mode = M_ZP;
          3'd2: mode = M_IMM;  3'd3: mode = M_ABS;
          3'd4: mode = M_IZY;  3'd5: mode = M_ZPX;
          3'd6: mode = M_ABY;  3'd7: mode = M_ABX;
        endcase
        case (ia)
          3'd0: op = O_ORA;  3'd1: op = O_AND;
          3'd2: op = O_EOR;  3'd3: op = O_ADC;
          3'd4: op = (ib == 3'd2) ? O_NOP : O_STA;   // $89 = NOP #imm
          3'd5: op = O_LDA;  3'd6: op = O_CMP;
          3'd7: op = O_SBC;
        endcase
      end

      2'b10: begin
        case (ia)
          3'd0: op = O_ASL;  3'd1: op = O_ROL;
          3'd2: op = O_LSR;  3'd3: op = O_ROR;
          3'd4: op = O_STX;  3'd5: op = O_LDX;
          3'd6: op = O_DEC;  3'd7: op = O_INC;
        endcase
        case (ib)
          3'd0: begin mode = M_IMM; if (ia != 3'd5) op = O_NOP; end
          3'd1: mode = M_ZP;
          3'd2: begin
            mode = (ia < 3'd4) ? M_ACC : M_IMP;
            case (ia)
              3'd4: op = O_TXA;  3'd5: op = O_TAX;
              3'd6: op = O_DEX;  3'd7: op = O_NOP;
              default: ;
            endcase
          end
          3'd3: mode = M_ABS;
          3'd5: mode = (ia == 3'd4 || ia == 3'd5) ? M_ZPY : M_ZPX;
          3'd6: op = (ir == 8'h9A) ? O_TXS : (ir == 8'hBA) ? O_TSX : O_NOP;
          3'd7: begin
            mode = (ia == 3'd5) ? M_ABY : M_ABX;
            if (ia == 3'd4) op = O_NOP;               // $9E undocumented
          end
          default: op = O_NOP;
        endcase
      end

      2'b00: begin
        case (ib)
          3'd0: mode = M_IMM;  3'd1: mode = M_ZP;
          3'd3: mode = M_ABS;  3'd5: mode = M_ZPX;
          3'd7: mode = M_ABX;
          default: mode = M_IMP;
        endcase
        case (ia)
          3'd1: op = (ib == 3'd1 || ib == 3'd3) ? O_BIT : O_NOP;
          3'd2: op = (ib == 3'd3) ? O_JMP : O_NOP;
          3'd3: if (ib == 3'd3) begin op = O_JMP; mode = M_IND; end
          3'd4: op = (ib == 3'd1 || ib == 3'd3 || ib == 3'd5) ? O_STY : O_NOP;
          3'd5: op = O_LDY;
          3'd6: op = (ib == 3'd0 || ib == 3'd1 || ib == 3'd3) ? O_CPY : O_NOP;
          3'd7: op = (ib == 3'd0 || ib == 3'd1 || ib == 3'd3) ? O_CPX : O_NOP;
          default: op = O_NOP;
        endcase
        if (ib == 3'd4) begin op = O_BR; mode = M_REL; end
        case (ir)
          8'h00: begin op = O_BRK; mode = M_IMP; end
          8'h20: begin op = O_JSR; mode = M_ABS; end
          8'h40: begin op = O_RTI; mode = M_IMP; end
          8'h60: begin op = O_RTS; mode = M_IMP; end
          8'h08: op = O_PHP;  8'h28: op = O_PLP;
          8'h48: op = O_PHA;  8'h68: op = O_PLA;
          8'h88: op = O_DEY;  8'hA8: op = O_TAY;
          8'hC8: op = O_INY;  8'hE8: op = O_INX;
          8'h18: op = O_CLC;  8'h38: op = O_SEC;
          8'h58: op = O_CLI;  8'h78: op = O_SEI;
          8'h98: op = O_TYA;  8'hB8: op = O_CLV;
          8'hD8: op = O_CLD;  8'hF8: op = O_SED;
          default: ;
        endcase
      end

      default: ;  // cc = 11: undocumented, NOP
    endcase
  end

  // ---------------- ALU helpers ----------------
  // returns {C, V, result}
  function automatic logic [9:0] adc(input logic [7:0] a, m, input logic ci, d);
    logic [8:0] s; logic [4:0] lo; logic [5:0] hi; logic v;
    s = {1'b0, a} + {1'b0, m} + {8'd0, ci};
    v = ~(a[7] ^ m[7]) & (a[7] ^ s[7]);
    if (!d) return {s[8], v, s[7:0]};
    lo = {1'b0, a[3:0]} + {1'b0, m[3:0]} + {4'd0, ci};
    if (lo > 5'd9) lo = lo + 5'd6;
    hi = {2'b0, a[7:4]} + {2'b0, m[7:4]} + {5'd0, (lo > 5'd15)};
    if (hi > 6'd9) hi = hi + 6'd6;
    return {(hi > 6'd15), v, hi[3:0], lo[3:0]};
  endfunction

  function automatic logic [9:0] sbc(input logic [7:0] a, m, input logic ci, d);
    logic [8:0] s; logic [4:0] lo, hi; logic v;
    s = {1'b0, a} + {1'b0, ~m} + {8'd0, ci};
    v = (a[7] ^ m[7]) & (a[7] ^ s[7]);
    if (!d) return {s[8], v, s[7:0]};
    lo = {1'b0, a[3:0]} - {1'b0, m[3:0]} - {4'd0, ~ci};
    hi = {1'b0, a[7:4]} - {1'b0, m[7:4]} - {4'd0, lo[4]};
    if (lo[4]) lo = lo - 5'd6;
    if (hi[4]) hi = hi - 5'd6;
    return {s[8], v, hi[3:0], lo[3:0]};
  endfunction

  // returns {C, result}
  function automatic logic [8:0] shift(input op_t o, input logic [7:0] v, input logic c);
    case (o)
      O_ASL:   return {v[7], v[6:0], 1'b0};
      O_ROL:   return {v[7], v[6:0], c};
      O_LSR:   return {v[0], 1'b0, v[7:1]};
      O_ROR:   return {v[0], c, v[7:1]};
      O_INC:   return {c, v + 8'd1};
      O_DEC:   return {c, v - 8'd1};
      default: return {c, v};
    endcase
  endfunction

  logic [8:0] sh_a, sh_m;
  assign sh_a = shift(op, A,   fC);
  assign sh_m = shift(op, din, fC);

  logic br_flag, taken;
  always_comb begin
    case (ir[7:6])
      2'd0: br_flag = fN;  2'd1: br_flag = fV;
      2'd2: br_flag = fC;  2'd3: br_flag = fZ;
    endcase
  end
  assign taken = (br_flag == ir[5]);

  wire is_store = (op == O_STA) || (op == O_STX) || (op == O_STY);
  wire is_rmw   = (op == O_ASL) || (op == O_ROL) || (op == O_LSR) ||
                  (op == O_ROR) || (op == O_INC) || (op == O_DEC);

  // ---------------- tasks used by the sequential block ----------------
  task automatic set_nz(input logic [7:0] v);
    fN <= v[7];
    fZ <= (v == 8'h00);
  endtask

  task automatic load_p(input logic [7:0] v);
    fN <= v[7]; fV <= v[6]; fD <= v[3];
    fI <= v[2]; fZ <= v[1]; fC <= v[0];
  endtask

  task automatic compare(input logic [7:0] r, m);
    logic [7:0] d;
    d  = r - m;
    fC <= (r >= m);
    fZ <= (r == m);
    fN <= d[7];
  endtask

  task automatic read_op(input logic [7:0] m);
    logic [9:0] r;
    case (op)
      O_LDA: begin A <= m; set_nz(m); end
      O_LDX: begin X <= m; set_nz(m); end
      O_LDY: begin Y <= m; set_nz(m); end
      O_ORA: begin A <= A | m; set_nz(A | m); end
      O_AND: begin A <= A & m; set_nz(A & m); end
      O_EOR: begin A <= A ^ m; set_nz(A ^ m); end
      O_ADC: begin
        r = adc(A, m, fC, fD);
        A <= r[7:0]; fC <= r[9]; fV <= r[8]; set_nz(r[7:0]);
      end
      O_SBC: begin
        r = sbc(A, m, fC, fD);
        A <= r[7:0]; fC <= r[9]; fV <= r[8]; set_nz(r[7:0]);
      end
      O_CMP: compare(A, m);
      O_CPX: compare(X, m);
      O_CPY: compare(Y, m);
      O_BIT: begin fZ <= ((A & m) == 8'h00); fN <= m[7]; fV <= m[6]; end
      default: ;
    endcase
  endtask

  // ---------------- bus outputs ----------------
  always_comb begin
    addr = pc;
    we   = 1'b0;
    dout = 8'h00;
    case (state)
      S_IND_LO:  addr = ptr;
      S_IND_HI:  addr = {ptr[15:8], ptr[7:0] + 8'd1};  // page wrap (NMOS JMP bug)
      S_OPER: begin
        addr = ea;
        if (is_store) begin
          we   = 1'b1;
          dout = (op == O_STA) ? A : (op == O_STX) ? X : Y;
        end
      end
      S_RMW_WR:  begin addr = ea; we = 1'b1; dout = dl; end
      S_PUSH: begin
        addr = {8'h01, sp}; we = 1'b1;
        dout = (op == O_PHA) ? A : {fN, fV, 2'b11, fD, fI, fZ, fC};
      end
      S_PULL, S_RTI_P, S_RET_LO, S_RET_HI: addr = {8'h01, sp};
      S_JSR_PCH, S_INT_PCH: begin addr = {8'h01, sp}; we = 1'b1; dout = pc[15:8]; end
      S_JSR_PCL, S_INT_PCL: begin addr = {8'h01, sp}; we = 1'b1; dout = pc[7:0];  end
      S_INT_P: begin
        addr = {8'h01, sp}; we = 1'b1;
        dout = {fN, fV, 1'b1, brk, fD, fI, fZ, fC};
      end
      S_VEC_LO:  addr = vec;
      S_VEC_HI:  addr = {vec[15:1], 1'b1};
      default:   addr = pc;   // FETCH, DECODE, ABS_HI, JSR_HI
    endcase
  end

  // ---------------- Sequencer ----------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= S_VEC_LO;  vec <= 16'hFFFC;
      pc <= '0; ea <= '0; ptr <= '0;
      A <= '0; X <= '0; Y <= '0; sp <= 8'hFD;
      ir <= 8'hEA; dl <= '0;
      fN <= 0; fV <= 0; fD <= 0; fZ <= 0; fC <= 0; fI <= 1;
      brk <= 0; nmi_q <= 1; nmi_pend <= 0;
    end else begin
      if (rdy) begin
        case (state)
          S_FETCH: begin
            if (nmi_pend || (!irq_n && !fI)) begin
              brk   <= 1'b0;
              vec   <= nmi_pend ? 16'hFFFA : 16'hFFFE;
              if (nmi_pend) nmi_pend <= 1'b0;
              state <= S_INT_PCH;
            end else begin
              ir    <= din;
              pc    <= pc + 16'd1;
              state <= S_DECODE;
            end
          end

          S_DECODE: begin
            state <= S_FETCH;
            case (mode)
              M_IMP, M_ACC: case (op)
                O_BRK: begin pc <= pc + 16'd1; brk <= 1'b1; vec <= 16'hFFFE; state <= S_INT_PCH; end
                O_RTI: begin sp <= sp + 8'd1; state <= S_RTI_P;  end
                O_RTS: begin sp <= sp + 8'd1; state <= S_RET_LO; end
                O_PHA, O_PHP: state <= S_PUSH;
                O_PLA, O_PLP: begin sp <= sp + 8'd1; state <= S_PULL; end
                O_TAX: begin X <= A;  set_nz(A);  end
                O_TAY: begin Y <= A;  set_nz(A);  end
                O_TXA: begin A <= X;  set_nz(X);  end
                O_TYA: begin A <= Y;  set_nz(Y);  end
                O_TSX: begin X <= sp; set_nz(sp); end
                O_TXS: sp <= X;
                O_INX: begin X <= X + 8'd1; set_nz(X + 8'd1); end
                O_INY: begin Y <= Y + 8'd1; set_nz(Y + 8'd1); end
                O_DEX: begin X <= X - 8'd1; set_nz(X - 8'd1); end
                O_DEY: begin Y <= Y - 8'd1; set_nz(Y - 8'd1); end
                O_CLC: fC <= 1'b0;  O_SEC: fC <= 1'b1;
                O_CLI: fI <= 1'b0;  O_SEI: fI <= 1'b1;
                O_CLD: fD <= 1'b0;  O_SED: fD <= 1'b1;
                O_CLV: fV <= 1'b0;
                O_ASL, O_ROL, O_LSR, O_ROR: begin
                  {fC, A} <= sh_a; set_nz(sh_a[7:0]);
                end
                default: ;
              endcase
              M_IMM: begin pc <= pc + 16'd1; read_op(din); end
              M_ZP:  begin pc <= pc + 16'd1; ea <= {8'h00, din};      state <= S_OPER; end
              M_ZPX: begin pc <= pc + 16'd1; ea <= {8'h00, din + X};  state <= S_OPER; end
              M_ZPY: begin pc <= pc + 16'd1; ea <= {8'h00, din + Y};  state <= S_OPER; end
              M_ABS, M_ABX, M_ABY, M_IND: begin
                pc <= pc + 16'd1;
                ea[7:0] <= din;
                state <= (op == O_JSR) ? S_JSR_PCH : S_ABS_HI;
              end
              M_IZX: begin pc <= pc + 16'd1; ptr <= {8'h00, din + X}; state <= S_IND_LO; end
              M_IZY: begin pc <= pc + 16'd1; ptr <= {8'h00, din};     state <= S_IND_LO; end
              M_REL: pc <= pc + 16'd1 + (taken ? {{8{din[7]}}, din} : 16'd0);
              default: ;
            endcase
          end

          S_ABS_HI: begin
            pc    <= pc + 16'd1;
            state <= S_OPER;
            case (mode)
              M_ABX:   ea <= {din, ea[7:0]} + {8'h00, X};
              M_ABY:   ea <= {din, ea[7:0]} + {8'h00, Y};
              default: ea <= {din, ea[7:0]};
            endcase
            if (mode == M_IND) begin
              ptr   <= {din, ea[7:0]};
              state <= S_IND_LO;
            end else if (op == O_JMP) begin
              pc    <= {din, ea[7:0]};
              state <= S_FETCH;
            end
          end

          S_IND_LO: begin ea[7:0] <= din; state <= S_IND_HI; end

          S_IND_HI: begin
            if (mode == M_IND) begin
              pc    <= {din, ea[7:0]};
              state <= S_FETCH;
            end else begin
              ea    <= {din, ea[7:0]} + ((mode == M_IZY) ? {8'h00, Y} : 16'h0000);
              state <= S_OPER;
            end
          end

          S_OPER: begin
            state <= S_FETCH;
            if (is_rmw) begin
              dl <= sh_m[7:0]; fC <= sh_m[8]; set_nz(sh_m[7:0]);
              state <= S_RMW_WR;
            end else if (!is_store) begin
              read_op(din);
            end
          end

          S_RMW_WR: state <= S_FETCH;

          S_PUSH:   begin sp <= sp - 8'd1; state <= S_FETCH; end

          S_PULL: begin
            if (op == O_PLA) begin A <= din; set_nz(din); end
            else load_p(din);
            state <= S_FETCH;
          end

          S_RTI_P:  begin load_p(din); sp <= sp + 8'd1; state <= S_RET_LO; end
          S_RET_LO: begin ea[7:0] <= din; sp <= sp + 8'd1; state <= S_RET_HI; end
          S_RET_HI: begin
            pc    <= {din, ea[7:0]} + ((op == O_RTS) ? 16'd1 : 16'd0);
            state <= S_FETCH;
          end

          S_JSR_PCH: begin sp <= sp - 8'd1; state <= S_JSR_PCL; end
          S_JSR_PCL: begin sp <= sp - 8'd1; state <= S_JSR_HI;  end
          S_JSR_HI:  begin pc <= {din, ea[7:0]}; state <= S_FETCH; end

          S_INT_PCH: begin sp <= sp - 8'd1; state <= S_INT_PCL; end
          S_INT_PCL: begin sp <= sp - 8'd1; state <= S_INT_P;   end
          S_INT_P:   begin sp <= sp - 8'd1; fI <= 1'b1; state <= S_VEC_LO; end
          S_VEC_LO:  begin ea[7:0] <= din; state <= S_VEC_HI; end
          S_VEC_HI:  begin pc <= {din, ea[7:0]}; state <= S_FETCH; end

          default: state <= S_FETCH;
        endcase
      end

      // NMI falling-edge detector
      nmi_q <= nmi_n;
      if (nmi_q && !nmi_n) nmi_pend <= 1'b1;
    end
  end

endmodule
