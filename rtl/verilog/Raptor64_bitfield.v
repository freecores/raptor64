`include "Raptor64_opcodes.v"
`timescale 1ns / 1ps
//=============================================================================
//        __
//   \\__/ o\    (C) 2012  Robert Finch
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@opencores.org
//       ||
//  
//	Raptor64_bitfield.v
//  - bitfield datapath operations
//
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
//
module Raptor64_bitfield(xIR, rolo, b, o, masko);
input [31:0] xIR;
input [63:0] rolo;
input [63:0] b;
output [63:0] o;
reg [63:0] o;
output [63:0] masko;

reg [63:0] o1;
wire [6:0] xOpcode = xIR[31:25];
wire [4:0] xFunc5 = xIR[4:0];

// generate mask
reg [63:0] mask;
assign masko = mask;
wire [5:0] mb = xIR[10:5];
wire [5:0] me = xIR[16:11];
integer nn,n;
always @(mb or me or nn)
	for (nn = 0; nn < 64; nn = nn + 1)
		mask[nn] <= (nn >= mb) ^ (nn <= me) ^ (me >= mb);

always @(xOpcode,xFunc5,mask,b,rolo,mb)
case (xOpcode)
`SHFTI:
	case(xFunc5)
	`BFINS: 	begin for (n = 0; n < 64; n = n + 1) o[n] = mask[n] ? rolo[n] : b[n]; end
	`BFSET: 	begin for (n = 0; n < 64; n = n + 1) o[n] = mask[n] ? 1'b1 : b[n]; end
	`BFCLR: 	begin for (n = 0; n < 64; n = n + 1) o[n] = mask[n] ? 1'b0 : b[n]; end
	`BFCHG: 	begin for (n = 0; n < 64; n = n + 1) o[n] = mask[n] ? ~b[n] : b[n]; end
	`BFEXT:		begin
					for (n = 0; n < 64; n = n + 1)
						o1[n] = mask[n] ? b[n] : 1'b0;
					o = o1 >> mb;
				end
	default:	o = 64'd0;
	endcase
default:	o = 64'd0;
endcase

endmodule
