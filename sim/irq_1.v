`include "defines.vh"
`timescale 1ns/100ps

`define BUG_ON(cond, msg)     \
if (cond) begin               \
   $error(msg);               \
   $display("TESTS FAILED!"); \
   $finish;                   \
end

module tb();
   initial begin
      $dumpfile("waves.vcd");
      $dumpvars(0, tb);
   end

   // Clock generation
   reg clk;
   reg reset;

   initial begin
      clk = 1'b0;
   end

   // 25MHz clock
   always begin
      #20 clk = !clk;
   end

   // PIO inputs
   reg [31:0]  din;
   reg [4:0]   index;
   reg [5:0]   action;
   reg [1:0]   mindex;
   reg [31:0]  gpio_in;

   // PIO outputs
   wire [31:0] gpio_out;
   wire [31:0] dout;
   wire        irq0;
   wire        irq1;

   // IRQ program
   reg [15:0]  prog [0:31];

   wire [5:0]  plen = 8;                 // Program length 8
   wire [23:0] div = 24'h0 ;             // Clock divider 0
   wire [31:0] pin_grps = 32'h04000000;  // SET group is pin 0
   wire [31:0] exec_ctrl = 32'h00007000; // Wrap top

   integer i;

   // Task to send action to PIO
   task act (
      input [5:0]  a,
      input [31:0] d);
      begin
         @(negedge clk);
         action = a;
         din = d;
         @(posedge clk);
      end
   endtask

   // Configure and run the PIO program
   initial begin
      $readmemh("irq_1.mem", prog);
      gpio_in = 0;

      // Do reset
      reset = 1'b1;
      repeat(2) @(posedge clk);
      reset = 1'b0;

      // Set the instructions
      for (i = 0; i < plen; i++) begin
         index = i;
         act(`INSTR, prog[i]);
      end

      // Set wrap for machine 1
      mindex = 0;
      act(`PEND, exec_ctrl);

      // Set fractional clock divider
      act(`DIV, div);

      // Set pin groups
      act(`GRPS, pin_grps);

      // isr threshold and autopush
      act(`SHIFT, 32'h00810000);

      // Enable status of IRQ0, IRQ1, IRQ2 interrupts
      act(`WR_IRQ0_INTE, 3'b111);

      // Enable machine 1
      act(`EN, 1);

      // Configuration done
      act(`NONE, 0);

      // Run for a while
      repeat(10) @(posedge clk);

      // Pin set, execution blocked in 'IRQ WAIT 0'
      `BUG_ON(gpio_out[0] != 1, "GPIO[0] != 1");

      // IRQ0 status set, IRQ1 cleared
      `BUG_ON(irq0 && !irq1, "irq0 && !irq1");

      // Clear IRQ0, next cycle
      act(`WR_IRQ, 2'b01);
      act(`NONE, 0);
      @(posedge clk);

      // Execute next instruction
      @(posedge clk);
      @(negedge clk);

      // Pin cleared
      `BUG_ON(gpio_out[0] != 0, "GPIO[0] != 0");

      // IRQ0 cleared, IRQ1 cleared
      `BUG_ON(!irq0 && !irq1, "!irq0 && !irq1");

      // Run for a while
      repeat(10) @(posedge clk);

      // Pin set, execution blocked in 'WAIT 0 IRQ 1'
      `BUG_ON(gpio_out[0] != 1, "GPIO[0] != 1");

      // IRQ0 status set, IRQ1 cleared
      `BUG_ON(irq0 && !irq1, "irq0 && !irq1");

      // Clear IRQ1, next cycle
      act(`WR_IRQ, 2'b10);
      act(`NONE, 0);
      @(posedge clk);

      // Execute next instruction
      @(posedge clk);
      @(negedge clk);

      // Pin cleared
      `BUG_ON(gpio_out[0] != 0, "GPIO[0] != 0");

      // IRQ0 cleared, IRQ1 cleared
      `BUG_ON(!irq0 && !irq1, "!irq0 && !irq1");

      // Run for a while
      repeat(10) @(posedge clk);

      // Pin cleared, execution blocked in 'WAIT 1 IRQ 2'
      `BUG_ON(gpio_out[0] != 0, "GPIO[0] != 0");

      // IRQ0 cleared, IRQ1 cleared
      `BUG_ON(!irq0 && !irq1, "!irq0 && !irq1");

      // Set IRQ2, next cycle
      act(`WR_IRQ_FORCE, 3'b100);
      act(`NONE, 0);
      @(posedge clk);

      // Execute next instruction
      @(posedge clk);
      @(negedge clk);

      // Pin set, execution blocked in 'IRQ WAIT 0'
      `BUG_ON(gpio_out[0] != 1, "GPIO[0] != 1");

      // IRQ0 status set, IRQ1 cleared
      `BUG_ON(irq0 && !irq1, "irq0 && !irq1");

      $display("SUCCESS: TESTS PASSED!");
      $finish;
   end

   pio pio_1 (
    .clk(clk),
    .reset(reset),
    .action(action),
    .index(index),
    .mindex(mindex),
    .din(din),
    .dout(dout),
    .gpio_in(gpio_in),
    .gpio_out(gpio_out),
    .irq0(irq0),
    .irq1(irq1)
  );

endmodule
