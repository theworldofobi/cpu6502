module tb_cpu6502;
  logic clk = 0, rst_n = 0;
  logic [15:0] addr;
  logic [7:0]  din, dout, mem [0:65535];
  logic        we;

  cpu6502 dut (.clk, .rst_n, .rdy(1'b1), .irq_n(1'b1), .nmi_n(1'b1),
               .addr, .din, .dout, .we);

  assign din = mem[addr];
  always @(posedge clk) if (we) mem[addr] <= dout;
  always #5 clk = ~clk;

  initial begin
    foreach (mem[i]) mem[i] = 8'h00;
    // $0400: LDX #10 / LDA #0 / CLC / loop: STX $10 / ADC $10 / DEX / BNE loop
    //        STA $0200 / JMP *
    mem[16'h0400:16'h0411] = '{8'hA2,8'h0A, 8'hA9,8'h00, 8'h18,
                               8'h86,8'h10, 8'h65,8'h10, 8'hCA, 8'hD0,8'hF9,
                               8'h8D,8'h00,8'h02, 8'h4C,8'h0F,8'h04};
    mem[16'hFFFC] = 8'h00;  mem[16'hFFFD] = 8'h04;   // reset vector

    #22 rst_n = 1;
    repeat (500) @(posedge clk);
    if (mem[16'h0200] == 8'h37) $display("PASS: sum = %0d", mem[16'h0200]);
    else                        $display("FAIL: got %h", mem[16'h0200]);
    $finish;
  end
endmodule
