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
// If a register field is not used by an instruction, then the register
// selected is forced to r0 for that field. This causes load stalls to be
// avoided, which would otherwise occur.
//=============================================================================

module Raptor64_SetOperandRegs(rst, clk, advanceI, advanceR, advanceX, b, AXC, insn, xIR, dRa, dRb, dRc);
input rst;
input clk;
input advanceI;
input advanceR;
input advanceX;
input [63:0] b;
input [3:0] AXC;
input [31:0] insn;
input [31:0] xIR;
output [8:0] dRa;
reg [8:0] dRa;
output [8:0] dRb;
reg [8:0] dRb;
output [8:0] dRc;
reg [8:0] dRc;

wire [6:0] iOpcode = insn[31:25];
wire [6:0] xOpcode = xIR[31:25];
wire [5:0] xFunc = xIR[5:0];
wire [6:0] iFunc7 = insn[6:0];

always @(posedge clk)
if (rst) begin
	dRa <= 9'd0;
	dRb <= 9'd0;
	dRc <= 9'd0;
end
else begin
	if (advanceI) begin
		// Default settings, to be overridden
		dRa <= {AXC,insn[24:20]};
		dRb <= {AXC,insn[19:15]};
		dRc <= {AXC,insn[14:10]};
		casex(iOpcode)
		`MISC:
			case(iFunc7)
			`IRET:	begin
					dRa <= {AXC,5'd25};
					dRb <= 9'd0;
					dRc <= 9'd0;
					end
			`ERET:	begin
					dRa <= {AXC,5'd24};
					dRb <= 9'd0;
					dRc <= 9'd0;
					end
			default:
					begin
					dRa <= 9'd0;
					dRb <= 9'd0;
					dRc <= 9'd0;
					end
			endcase
		`CALL,`JMP,`NOPI:
					begin
					dRa <= 9'd0;
					dRb <= 9'd0;
					dRc <= 9'd0;
					end
		`RET:		begin
					dRa <= {AXC,5'd30};
					dRb <= {AXC,5'd31};
					dRc <= 9'd0;
					end
		`BLTI,`BLEI,`BGTI,`BGEI,
		`BLTUI,`BLEUI,`BGTUI,`BGEUI,
		`BEQI,`BNEI:
					begin
					dRb <= 9'd0;
					dRc <= 9'd0;
					end
		`SLTI,`SLEI,`SGTI,`SGEI,
		`SLTUI,`SLEUI,`SGTUI,`SGEUI,
		`SEQI,`SNEI:
					begin
					dRb <= 9'd0;
					dRc <= 9'd0;
					end
		`SETLO:		dRa <= {AXC,insn[26:22]};
		`SETMID:	dRa <= {AXC,insn[26:22]};
		`SETHI:		dRa <= {AXC,insn[26:22]};
		default:	dRa <= {AXC,insn[24:20]};
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
				dRa <= b[24:20];
				dRb <= b[19:15];
				dRc <= b[14:10];
			end
		end
	end
end

endmodule
