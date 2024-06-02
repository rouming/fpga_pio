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
   wire [5:0]  plen = 8;                 // Program length
   wire [23:0] div = 24'h0 ;             // Clock divider 0

   // SM1 configuration
   wire [31:0] pin_grps_sm1  = 32'h04000000;  // SET group is pin 0
   wire [31:0] exec_ctrl_sm1 = 32'h00003000;  // Wrap top

   // SM2 configuration
   wire [31:0] pin_grps_sm2  = 32'h04000020;  // SET group is pin 1
   wire [31:0] exec_ctrl_sm2 = 32'h00003000;  // Wrap top

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
      $readmemh("irq_2.mem", prog);
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

      //
      //
      // Common configuration

      // Enable status of IRQ0, IRQ1, IRQ2 interrupts
      act(`WR_IRQ0_INTE, 3'b111);

      //
      // SM1 configuration
      //
      mindex = 0;

      // Set wrap
      act(`PEND, exec_ctrl_sm1);

      // Set pin groups
      act(`GRPS, pin_grps_sm1);

      // Set fractional clock divider
      act(`DIV, div);

      //
      // SM2 configuration
      //
      mindex = 1;

      // Set wrap
      act(`PEND, exec_ctrl_sm2);

      // Set pin groups
      act(`GRPS, pin_grps_sm2);

      // Set fractional clock divider
      act(`DIV, div);

      // Execute 'JMP 0x4' to SM2 program
      act(`IMM, 4);
      // Execute JMP instruction keeping 'din' set, because
      // 'din' is multiplexed to a SM directly, without FF.
      @(negedge clk);
      @(posedge clk);
      action = `NONE;

      //
      // Enable both SMs
      //

      // Enable both machines
      act(`EN, 3);

      // Configuration done
      act(`NONE, 0);

      // Run for a while
      repeat(10) @(posedge clk);

      // SM1, pin 0 set, execution blocked in 'IRQ WAIT 0'
      `BUG_ON(gpio_out[0] != 1, "GPIO[0] != 1");

      // SM2, pin 1 set, execution blocked in 'IRQ WAIT 1'
      `BUG_ON(gpio_out[1] != 1, "GPIO[1] != 1");

      // Get IRQ
      act(`RD_IRQ, 0);
      @(negedge clk);

      `BUG_ON(dout != 2'h3, "irq != 0x3");

      // Clear IRQ0 to wakeup SM1, next cycle
      act(`WR_IRQ, 1'b1);
      act(`NONE, 0);
      @(posedge clk);

      // Run for a while
      repeat(10) @(posedge clk);

      // SM1, pin 0 still set, execution blocked in 'IRQ WAIT 0'
      `BUG_ON(gpio_out[0] != 1, "GPIO[0] != 1");

      // SM2, pin 1 cleared, execution blocked in 'IRQ WAIT 1'
      `BUG_ON(gpio_out[1] != 0, "GPIO[1] != 0");

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
