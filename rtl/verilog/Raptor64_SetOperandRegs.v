`include "Raptor64_opcodes.v"
`timescale 1ns / 1ps
//=============================================================================
//        __
//   \\__/ o\    (C) 2011,2012  Robert Finch
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@opencores.org
//       ||
//  
//	Raptor64_SetOperandRegs.v
//  
// This source file is free software: you can redistribute it and/or modify 
// it under the terms of the GNU Lesser General Public License as published 
// by the Free Software Foundation, either version 3 of the License, or     
// (at your option) any later version.                                      
//                                                                          
// This source file is distributed in the hope that it will be useful,      
// but WITHOUT ANY WARRANTY; without even the implied warranty of           
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            
// GNU General Public License for more details.                             
//                                                                          
// You should have received a copy of the GNU General Public License        
// along with this program.  If not, see <http://www.gnu.org/licenses/>.    
//                                                                          
//
//=============================================================================

module Raptor64_SetOperandRegs(rst, clk, advanceI, advanceR, advanceX, b, AXC, insn, xIR, dRa, dRb, dRc);
input rst;
input clk;
input advanceI;
input advanceR;
input advanceX;
input [63:0] b;
input [3:0] AXC;
input [41:0] insn;
input [41:0] xIR;
output [8:0] dRa;
reg [8:0] dRa;
output [8:0] dRb;
reg [8:0] dRb;
output [8:0] dRc;
reg [8:0] dRc;

wire [6:0] iOpcode = insn[41:35];
wire [6:0] xOpcode = xIR[41:35];
wire [6:0] xFunc = xIR[6:0];

always @(posedge clk)
if (rst) begin
	dRa <= 9'd0;
	dRb <= 9'd0;
	dRc <= 9'd0;
end
else begin
	if (advanceI) begin
		// Default settings, to be overridden
		dRa <= {AXC,insn[34:30]};
		dRb <= {AXC,insn[29:25]};
		dRc <= {AXC,insn[24:20]};
		casex(iOpcode)
		`RET:		begin
					dRa <= {AXC,5'd30};
					dRb <= {AXC,5'd31};
					end
		`SETLO:		dRa <= {AXC,insn[36:32]};
		`SETHI:		dRa <= {AXC,insn[36:32]};
		`SM,`LM:
			begin
			dRa <= {AXC,1'b1,insn[34:31]};
			casex(insn[30:0])
			31'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx1:	dRb <= {AXC,5'd1};
			31'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxx10:	dRb <= {AXC,5'd2};
			31'bxxxxxxxxxxxxxxxxxxxxxxxxxxxx100:	dRb <= {AXC,5'd3};
			31'bxxxxxxxxxxxxxxxxxxxxxxxxxxx1000:	dRb <= {AXC,5'd4};
			31'bxxxxxxxxxxxxxxxxxxxxxxxxxx10000:	dRb <= {AXC,5'd5};
			31'bxxxxxxxxxxxxxxxxxxxxxxxxx100000:	dRb <= {AXC,5'd6};
			31'bxxxxxxxxxxxxxxxxxxxxxxxx1000000:	dRb <= {AXC,5'd7};
			31'bxxxxxxxxxxxxxxxxxxxxxxx10000000:	dRb <= {AXC,5'd8};
			31'bxxxxxxxxxxxxxxxxxxxxxx100000000:	dRb <= {AXC,5'd9};
			31'bxxxxxxxxxxxxxxxxxxxxx1000000000:	dRb <= {AXC,5'd10};
			31'bxxxxxxxxxxxxxxxxxxxx10000000000:	dRb <= {AXC,5'd11};
			31'bxxxxxxxxxxxxxxxxxxx100000000000:	dRb <= {AXC,5'd12};
			31'bxxxxxxxxxxxxxxxxxx1000000000000:	dRb <= {AXC,5'd13};
			31'bxxxxxxxxxxxxxxxxx10000000000000:	dRb <= {AXC,5'd14};
			31'bxxxxxxxxxxxxxxxx100000000000000:	dRb <= {AXC,5'd15};
			31'bxxxxxxxxxxxxxxx1000000000000000:	dRb <= {AXC,5'd16};
			31'bxxxxxxxxxxxxxx10000000000000000:	dRb <= {AXC,5'd17};
			31'bxxxxxxxxxxxxx100000000000000000:	dRb <= {AXC,5'd18};
			31'bxxxxxxxxxxxx1000000000000000000:	dRb <= {AXC,5'd19};
			31'bxxxxxxxxxxx10000000000000000000:	dRb <= {AXC,5'd20};
			31'bxxxxxxxxxx100000000000000000000:	dRb <= {AXC,5'd21};
			31'bxxxxxxxxx1000000000000000000000:	dRb <= {AXC,5'd22};
			31'bxxxxxxxx10000000000000000000000:	dRb <= {AXC,5'd23};
			31'bxxxxxxx100000000000000000000000:	dRb <= {AXC,5'd24};
			31'bxxxxxx1000000000000000000000000:	dRb <= {AXC,5'd25};
			31'bxxxxx10000000000000000000000000:	dRb <= {AXC,5'd26};
			31'bxxxx100000000000000000000000000:	dRb <= {AXC,5'd27};
			31'bxxx1000000000000000000000000000:	dRb <= {AXC,5'd28};
			31'bxx10000000000000000000000000000:	dRb <= {AXC,5'd29};
			31'bx100000000000000000000000000000:	dRb <= {AXC,5'd30};
			31'b1000000000000000000000000000000:	dRb <= {AXC,5'd31};
			default:	dRb <= {AXC,5'd0};
			endcase
			end
		default:	dRa <= {AXC,insn[34:30]};
		endcase
	end
	else if (advanceR) begin
		dRa <= 9'd0;
		dRb <= 9'd0;
		dRc <= 9'd0;
	end
	// no else here
	if (advanceX) begin
		if (xOpcode==`R) begin
			if (xFunc==`EXEC) begin
				dRa <= b[34:30];
				dRb <= b[29:25];
				dRc <= b[24:20];
			end
		end
	end
end

endmodule
