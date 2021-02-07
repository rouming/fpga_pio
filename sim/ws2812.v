`timescale 1ns/100ps
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
  reg [3:0]   action;
  reg [1:0]   mindex;
  reg [31:0]  gpio_in = 0; 
  
  // PIO outputs
  wire [31:0] gpio_out; 
  wire[31:0]  gpio_dir; 
  wire [31:0] dout;

  // Configuration
  reg [15:0] program [0:31];
  initial $readmemh("ws2812.mem", program);

  wire [5:0]  plen = 4;                 // Program length
  wire [23:0] div = 24'h0535;           // Clock divider 25M / (6 * 800K)
  wire [31:0] pin_grps = 32'h01000000;  // SIDE grp pin 0
  wire [5:0]  sideset_bits = 6'b100001; // Side-set bits, enabled=0
  integer i;

  // Actions
  localparam NONE  = 0;
  localparam INSTR = 1;
  localparam PEND  = 2;
  localparam PULL  = 3;
  localparam PUSH  = 4;
  localparam GRPS  = 5;
  localparam EN    = 6;
  localparam DIV   = 7;
  localparam SIDES = 8;
  localparam IMM   = 9;
  localparam APUSH = 10;
  localparam APULL = 11;
  localparam IPINS = 12;
  localparam IDIRS = 13;
  localparam ISRT  = 14;
  localparam OSRT  = 15;

  // Task to send action to PIO
  task act (
    input [3:0]  a,
    input [31:0] d
  );
    begin
      @(negedge clk);
      action = a;
      din = d;
      @(posedge clk);
    end
  endtask

  // Configure and run the PIO program
  initial begin
    // Do reset
    reset = 1'b1;
    repeat(2) @(posedge clk);
    reset = 1'b0;

    // Set the instructions
    for(i=0;i<plen;i++) begin
      index = i;
      act(INSTR, program[i]);
    end

    // Set wrap for machine 1
    mindex = 0;
    act(PEND, plen - 1);

    // Set fractional clock divider
    act(DIV, div);
    
    // Set pin groups
    act(GRPS, pin_grps);

    // Configure side-set bits
    act(SIDES, sideset_bits);

    // Configure auto-pull 24 and shift direction left
    act(OSRT, 6'b111000);

    // Configure auto-pull
    act(APULL, 1);
     
    // Enable machine 1
    act(EN, 1);

    // Configuration done
    act(NONE, 0);
    
    // Small gap
    repeat(50) @(posedge clk);

    for(i=0;i<1;i++) begin
      // Send GRB value
      act(PUSH, 32'hff00ff00);
      action = 0;

      // Wait for it to be sent
      repeat(768) @(posedge clk);
    end

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
    .gpio_dir(gpio_dir)
  );

endmodule 
