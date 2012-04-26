// ============================================================================
// (C) 2012 Robert Finch
// All Rights Reserved.
// robfinch<remove>@opencores.org
//
// Raptor64.v - dcache_ram
//  - 64 bit CPU data cache ram
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
// ============================================================================
//
module Raptor64_dcache_ram(clk,wr,sel,wadr,i,radr,o);
input clk;
input wr;
input [3:0] sel;
input [13:2] wadr;
input [31:0] i;
input [13:3] radr;
output [63:0] o;

reg [7:0] mem0 [2047:0];
reg [31:0] memH [2047:0];

syncRam2kx8_1rw1r u1
(
	.wrst(1'b0),
	.wclk(clk),
	.wce(sel[0] && !wadr[2]),
	.we(wr),
	.wadr(wadr[13:3]),
	.i(i[7:0]),
	.wo(),
	.rrst(1'b0),
	.rclk(~clk),
	.rce(1'b1),
	.radr(radr[13:3]),
	.o(o[7:0])
);

syncRam2kx8_1rw1r u2
(
	.wrst(1'b0),
	.wclk(clk),
	.wce(sel[1] && !wadr[2]),
	.we(wr),
	.wadr(wadr[13:3]),
	.i(i[15:8]),
	.wo(),
	.rrst(1'b0),
	.rclk(~clk),
	.rce(1'b1),
	.radr(radr[13:3]),
	.o(o[15:8])
);

syncRam2kx8_1rw1r u3
(
	.wrst(1'b0),
	.wclk(clk),
	.wce(sel[2] && !wadr[2]),
	.we(wr),
	.wadr(wadr[13:3]),
	.i(i[23:16]),
	.wo(),
	.rrst(1'b0),
	.rclk(~clk),
	.rce(1'b1),
	.radr(radr[13:3]),
	.o(o[23:16])
);

syncRam2kx8_1rw1r u4
(
	.wrst(1'b0),
	.wclk(clk),
	.wce(sel[3] && !wadr[2]),
	.we(wr),
	.wadr(wadr[13:3]),
	.i(i[31:24]),
	.wo(),
	.rrst(1'b0),
	.rclk(~clk),
	.rce(1'b1),
	.radr(radr[13:3]),
	.o(o[31:24])
);

syncRam2kx8_1rw1r u5
(
	.wrst(1'b0),
	.wclk(clk),
	.wce(sel[0] && wadr[2]),
	.we(wr),
	.wadr(wadr[13:3]),
	.i(i[7:0]),
	.wo(),
	.rrst(1'b0),
	.rclk(~clk),
	.rce(1'b1),
	.radr(radr[13:3]),
	.o(o[39:32])
);

syncRam2kx8_1rw1r u6
(
	.wrst(1'b0),
	.wclk(clk),
	.wce(sel[1] && wadr[2]),
	.we(wr),
	.wadr(wadr[13:3]),
	.i(i[15:8]),
	.wo(),
	.rrst(1'b0),
	.rclk(~clk),
	.rce(1'b1),
	.radr(radr[13:3]),
	.o(o[47:40])
);

syncRam2kx8_1rw1r u7
(
	.wrst(1'b0),
	.wclk(clk),
	.wce(sel[2] && wadr[2]),
	.we(wr),
	.wadr(wadr[13:3]),
	.i(i[23:16]),
	.wo(),
	.rrst(1'b0),
	.rclk(~clk),
	.rce(1'b1),
	.radr(radr[13:3]),
	.o(o[55:48])
);

syncRam2kx8_1rw1r u8
(
	.wrst(1'b0),
	.wclk(clk),
	.wce(sel[3] && wadr[2]),
	.we(wr),
	.wadr(wadr[13:3]),
	.i(i[31:24]),
	.wo(),
	.rrst(1'b0),
	.rclk(~clk),
	.rce(1'b1),
	.radr(radr[13:3]),
	.o(o[63:56])
);

endmodule

