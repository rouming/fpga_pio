// SPDX-FileCopyrightText: 2022 Lawrie Griffiths
// SPDX-License-Identifier: BSD-2-Clause

`default_nettype none
module pc (
  input        clk,
  input        penable,
  input        reset,
  input [4:0]  din,
  input        imm,
  input        jmp,
  input [4:0]  pend,
  input        stalled,
  input [4:0]  wrap_target,
  output reg [4:0] dout
);

  reg [4:0] index;

  // Combinational logic
  always @(*) begin
     if (imm) begin
       // Immediate is special and can be executed even SM is disabled
       // with only one major difference from normal instructions execution:
       // PC does not advance and only instruction can manipulate PC.
       // For details see the "3.5.7. Forced and EXEC’d Instructions".
       if (jmp)
         dout = din;
       else
         dout = index;
     end else if (penable && !stalled) begin
       if (jmp)
         dout = din;
       else if (index == pend)
         dout = wrap_target;
       else
         dout = index + 1;
     end else begin
       dout = index;
     end
  end

  always @(posedge clk) begin
    if (reset)
      index <= 0;
    else
      index <= dout;
  end

endmodule
