`include "Raptor64_opcodes.v"
`timescale 1ns / 1ps
//=============================================================================
//        __
//   \\__/ o\    (C) 2011,2012  Robert Finch
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@opencores.org
//       ||
//  
//	Raptor64_SetTargetRegister.v
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

module Raptor64_SetTargetRegister(rst,clk,advanceR,advanceX,dIR,dAXC,xRt);
input rst;
input clk;
input advanceR;
input advanceX;
input [41:0] dIR;
input [3:0] dAXC;
output [8:0] xRt;
reg [8:0] xRt;

wire [6:0] dOpcode = dIR[41:35];
wire [6:0] dFunc = dIR[6:0];

always @(posedge clk)
if (rst) begin
	xRt <= 9'd0;
end
else begin
if (advanceR) begin
	casex(dOpcode)
	`R:
		case(dFunc)
		`MTSPR,`CMG,`CMGI,`EXEC:
					xRt <= 9'd0;
		`MYST:		xRt <= {dAXC,dIR[19:15]};
		default:	xRt <= {dAXC,dIR[29:25]};
		endcase
	`SETLO:		xRt <= {dAXC,dIR[36:32]};
	`SETHI:		xRt <= {dAXC,dIR[36:32]};
	`RR,`FP:	xRt <= {dAXC,dIR[24:20]};
	`BTRI:		xRt <= 9'd0;
	`BTRR:
		case(dIR[4:0])
		`LOOP:	xRt <= {dAXC,dIR[29:25]};
		default: xRt <= 9'd0;
		endcase
	`TRAPcc:	xRt <= 9'd0;
	`TRAPcci:	xRt <= 9'd0;
	`JMP:		xRt <= 9'd00;
	`CALL:		xRt <= {dAXC,5'd31};
	`RET:		xRt <= {dAXC,5'd30};
	`MEMNDX:
		case(dFunc[5:0])
		`SWX,`SHX,`SCX,`SBX,`SFX,`SFDX,`SPX,`SFPX,`SFDPX,`SSHX,`SSWX,
		`OUTWX,`OUTHX,`OUTCX,`OUTBX:
				xRt <= 9'd0;
		default:	xRt <= {dAXC,dIR[24:20]};
		endcase
	`LSH,`LSW,
	`SW,`SH,`SC,`SB,`SF,`SFD,`SSH,`SSW,`SP,`SFP,`SFDP,	// but not SWC!
	`OUTW,`OUTH,`OUTC,`OUTB:
				xRt <= 9'd0;
	`NOPI:		xRt <= 9'd0;
	`BEQI,`BNEI,`BLTI,`BLEI,`BGTI,`BGEI,`BLTUI,`BLEUI,`BGTUI,`BGEUI:
				xRt <= 9'd0;
	`SM:		xRt <= 9'd0;
	`LM:
		casex(dIR[30:0])
		31'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx1:	xRt <= {dAXC,5'd1};
		31'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxx10:	xRt <= {dAXC,5'd2};
		31'bxxxxxxxxxxxxxxxxxxxxxxxxxxxx100:	xRt <= {dAXC,5'd3};
		31'bxxxxxxxxxxxxxxxxxxxxxxxxxxx1000:	xRt <= {dAXC,5'd4};
		31'bxxxxxxxxxxxxxxxxxxxxxxxxxx10000:	xRt <= {dAXC,5'd5};
		31'bxxxxxxxxxxxxxxxxxxxxxxxxx100000:	xRt <= {dAXC,5'd6};
		31'bxxxxxxxxxxxxxxxxxxxxxxxx1000000:	xRt <= {dAXC,5'd7};
		31'bxxxxxxxxxxxxxxxxxxxxxxx10000000:	xRt <= {dAXC,5'd8};
		31'bxxxxxxxxxxxxxxxxxxxxxx100000000:	xRt <= {dAXC,5'd9};
		31'bxxxxxxxxxxxxxxxxxxxxx1000000000:	xRt <= {dAXC,5'd10};
		31'bxxxxxxxxxxxxxxxxxxxx10000000000:	xRt <= {dAXC,5'd11};
		31'bxxxxxxxxxxxxxxxxxxx100000000000:	xRt <= {dAXC,5'd12};
		31'bxxxxxxxxxxxxxxxxxx1000000000000:	xRt <= {dAXC,5'd13};
		31'bxxxxxxxxxxxxxxxxx10000000000000:	xRt <= {dAXC,5'd14};
		31'bxxxxxxxxxxxxxxxx100000000000000:	xRt <= {dAXC,5'd15};
		31'bxxxxxxxxxxxxxxx1000000000000000:	xRt <= {dAXC,5'd16};
		31'bxxxxxxxxxxxxxx10000000000000000:	xRt <= {dAXC,5'd17};
		31'bxxxxxxxxxxxxx100000000000000000:	xRt <= {dAXC,5'd18};
		31'bxxxxxxxxxxxx1000000000000000000:	xRt <= {dAXC,5'd19};
		31'bxxxxxxxxxxx10000000000000000000:	xRt <= {dAXC,5'd20};
		31'bxxxxxxxxxx100000000000000000000:	xRt <= {dAXC,5'd21};
		31'bxxxxxxxxx1000000000000000000000:	xRt <= {dAXC,5'd22};
		31'bxxxxxxxx10000000000000000000000:	xRt <= {dAXC,5'd23};
		31'bxxxxxxx100000000000000000000000:	xRt <= {dAXC,5'd24};
		31'bxxxxxx1000000000000000000000000:	xRt <= {dAXC,5'd25};
		31'bxxxxx10000000000000000000000000:	xRt <= {dAXC,5'd26};
		31'bxxxx100000000000000000000000000:	xRt <= {dAXC,5'd27};
		31'bxxx1000000000000000000000000000:	xRt <= {dAXC,5'd28};
		31'bxx10000000000000000000000000000:	xRt <= {dAXC,5'd29};
		31'bx100000000000000000000000000000:	xRt <= {dAXC,5'd30};
		31'b1000000000000000000000000000000:	xRt <= {dAXC,5'd31};
		default:	xRt <= 9'h000;
		endcase

	default:	xRt <= {dAXC,dIR[29:25]};
	endcase
	if (dOpcode[6:4]==`IMM)
		xRt <= 9'd0;
end
else if (advanceX)
	xRt <= 9'd0;
end

endmodule
