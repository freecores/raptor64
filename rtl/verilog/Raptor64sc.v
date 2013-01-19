`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2011-2013  Robert Finch, Stratford
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@opencores.org
//       ||
//
// Raptor64sc.v
//  - 64 bit CPU
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
// 15848 LUT's / 3591 ff's / 48.215 MHz
// 29 Block RAMs
// ============================================================================
//
//`define ADDRESS_RESERVATION	1
//`define FLOATING_POINT		1
//`define BTB					1
//`define TLB		1

`define RESET_VECTOR	64'hFFFF_FFFF_FFFF_FFF0

`define ITLB_MissHandler	64'hFFFF_FFFF_FFFF_FFC0
`define DTLB_MissHandler	64'hFFFF_FFFF_FFFF_FFB0

`define EX_NON			9'd000
`define EX_TRAP			9'd32	// Trap exception
`define EX_IRQ			9'd449	// interrupt
`define EX_DBZ			9'd488	// divide by zero
`define EX_OFL			9'd489	// overflow
`define EX_UNIMP_INSN	9'd495	// unimplemented instruction
`define EX_PRIV			9'd496	// priviledge violation
`define EX_TLBD			9'd506	// TLB exception - data
`define EX_TLBI			9'd507	// TLB exception - ifetch
`define EX_DBERR		9'd508	// Bus Error - load or store or I/O
`define EX_IBERR		9'd509	// Bus Error - instruction fetch
`define EX_NMI			9'd510	// non-maskable interrupt
`define EX_RST			9'd511	// Reset

`include "Raptor64_opcodes.v"

module Raptor64sc(rst_i, clk_i, nmi_i, irq_i, bte_o, cti_o, bl_o, iocyc_o,
	cyc_o, stb_o, ack_i, err_i, we_o, sel_o, rsv_o, adr_o, dat_i, dat_o, sys_adv, sys_adr
);
parameter IDLE = 5'd1;
parameter ICACT = 5'd2;
parameter ICACT1 = 5'd4;
parameter ICACT2 = 5'd5;
parameter DCIDLE = 5'd20;
parameter DCACT = 5'd21;
parameter AMSB = 31;
parameter RESET = 4'd0;
parameter RUN = 4'd1;
input rst_i;
input clk_i;
input nmi_i;
input irq_i;

output [1:0] bte_o;		// burst type
reg [1:0] bte_o;
output [2:0] cti_o;		// cycle type
reg [2:0] cti_o;
output [4:0] bl_o;		// burst length (non-WISHBONE)
reg [4:0] bl_o;
output iocyc_o;			// I/O cycle is valid
reg iocyc_o;
output cyc_o;			// cycle is valid
reg cyc_o;
output stb_o;			// data strobe
reg stb_o;
input ack_i;			// data transfer acknowledge
input err_i;			// bus error
output we_o;			// write enable
reg we_o;
output [7:0] sel_o;		// byte lane selects
reg [7:0] sel_o;
output rsv_o;			// reserve the address (non-WISHBONE)
reg rsv_o;
output [63:0] adr_o;	// address
reg [63:0] adr_o;
input [63:0] dat_i;		// data input
output [63:0] dat_o;	// data output
reg [63:0] dat_o;

input sys_adv;
input [63:5] sys_adr;

reg [3:0] state;
reg [5:0] fltctr;
wire fltdone = fltctr==6'd0;
reg resetA;
reg im,bu_im;			// interrupt mask
reg im1;			// temporary interrupt mask for LM/SM
reg [1:0] rm;		// fp rounding mode
reg FXE;			// fp exception enable
wire KernelMode;
wire [31:0] sr = {bu_im,15'd0,im,1'b0,KernelMode,FXE,2'b00,10'b0};
reg [31:0] dIR,xIR,m1IR,m2IR,wIR;
reg [31:0] ndIR;		// next dIR
reg [63:0] pc;			// ipc
wire [63:0] pchistoric;
reg pccap;
reg [63:0] ErrorEPC;
reg [63:0] EPC [0:15];
reg [63:0] IPC [0:15];
reg [63:0] PCS [0:15];
reg [15:0] StatusEXL;
reg [63:0] dpc,xpc,m1pc,m2pc,wpc;
reg ipcv,dpcv,xpcv,m1pcv,m2pcv,wpcv;	// PC valid indicators
wire [63:0] rfoa,rfob,rfoc;
wire [8:0] dRa,dRb,dRc;
reg [8:0] wRt,m1Rt,m2Rt,tRt;
wire [8:0] xRt;
reg xRtZero;
reg [63:0] ea;
reg [4:0] cstate;
reg dbranch_taken,xbranch_taken;
reg [63:0] mutex_gate;
reg [63:0] TBA;		// Trap Base Address
reg [8:0] dextype,xextype,m1extype,m2extype,wextype,textype;
reg [3:0] epat [0:255];
reg [7:0] eptr;
reg [3:0] dAXC,xAXC,m1AXC,m2AXC,wAXC;
wire [3:0] AXC = (eptr==8'h00) ? 4'h0 : epat[eptr];
reg dtinit;
reg dcache_on;
reg [63:32] nonICacheSeg;
reg [1:0] FPC_rm;
reg FPC_SL;			// result is negative (and non-zero)
reg FPC_SE;			// result is zero
reg FPC_SG;			// result is positive (and non-zero)
reg FPC_SI;			// result is infinite or NaN
reg FPC_overx;
reg fp_iop;
reg fp_ovr;
reg fp_uf;
wire [31:0] FPC = {FPC_rm,1'b0,
			9'd0,
			FPC_SL,
			FPC_SG,
			FPC_SE,
			FPC_SI,
			16'd0
			};
wire [63:0] cdat;
reg [63:0] wr_addr;
reg [31:0] insn;
reg clk_en;
reg cpu_clk_en;
reg StatusERL;		// 1= in error processing
//reg StatusEXL;		// 1= in exception processing
reg StatusHWI;		// 1= in interrupt processing
reg StatusUM;		// 1= user mode
reg [7:0] ASID;		// address space identifier (process ID)
integer n;
reg [63:13] BadVAddr;
reg [63:13] PageTableAddr;
reg [63:0] errorAddress;

wire [6:0] iOpcode = insn[31:25];
wire [6:0] iFunc = insn[6:0];
wire [6:0] dOpcode = dIR[31:25];
wire [6:0] dFunc = dIR[6:0];
wire [5:0] dFunc6 = dIR[5:0];
wire [6:0] xOpcode = xIR[31:25];
wire [6:0] xFunc = xIR[6:0];
wire [5:0] xFunc6 = xIR[5:0];
wire [4:0] xFunc5 = xIR[4:0];
reg [6:0] m1Opcode,m2Opcode,wOpcode;
reg [6:0] m1Func,m2Func,wFunc;
wire [5:0] m1Func6 = m1Func[5:0];
wire [5:0] m2Func6 = m2Func[5:0];
wire [5:0] wFunc6 = wIR[5:0];
reg [63:0] m1Data,m2Data,wData,tData;
reg [63:0] m2Addr;
reg [63:0] tick;
reg [63:0] a,b,c,imm,m1b;
reg [1:0] scale;
reg prev_ihit;
reg rsf;
reg [63:5] resv_address;
reg dirqf,rirqf,m1irqf,m2irqf,wirqf,tirqf;
reg xirqf;
wire advanceX_edge;
wire takb;
wire advanceI,advanceR,advanceX,advanceM1,advanceW,advanceT;
reg m1clkoff,m2clkoff,m3clkoff,m4clkoff,wclkoff;
reg dFip,xFip,m1Fip,m2Fip,m3Fip,m4Fip,wFip;
reg cyc1;
reg LoadNOPs;
reg dIRvalid,xIRvalid,m1IRvalid,m2IRvalid,wIRvalid,tIRvalid;

function [63:0] fnIncPC;
input [63:0] fpc;
begin
fnIncPC = fpc + 64'd4;
end
endfunction

assign KernelMode = StatusEXL[xAXC]|StatusHWI;

wire iIsLSPair = iOpcode==`SP || iOpcode==`LP || iOpcode==`SFP || iOpcode==`LFP || iOpcode==`SFDP || iOpcode==`LFDP;
wire dIsLSPair = dOpcode==`SP || dOpcode==`LP || dOpcode==`SFP || dOpcode==`LFP || dOpcode==`SFDP || dOpcode==`LFDP;

//-----------------------------------------------------------------------------
// TLB
// The TLB contains 64 entries, that are 8 way set associative.
// The TLB is dual ported and shared between the instruction and data streams.
//-----------------------------------------------------------------------------
wire [63:0] ppc;
wire [63:0] pea;
wire [63:0] tlbo;
`ifdef TLB
wire [63:0] TLBVirtPage;
wire wTlbp = wIRvalid && advanceW && wOpcode==`MISC && wFunc==`TLBP;
wire wTlbrd = wIRvalid && advanceW && wOpcode==`MISC && wFunc==`TLBR;
wire wTlbwr = wIRvalid && advanceW && wOpcode==`MISC && wFunc==`TLBWR;
wire wTlbwi = wIRvalid && advanceW && wOpcode==`MISC && wFunc==`TLBWI;
wire wMtspr = wIRvalid && advanceW && wOpcode==`R && wFunc==`MTSPR;
wire xTlbrd = xIRvalid && advanceX && xOpcode==`MISC && xFunc==`TLBR;
wire xTlbwr = xIRvalid && advanceX && xOpcode==`MISC && xFunc==`TLBWR;
wire xTlbwi = xIRvalid && advanceX && xOpcode==`MISC && xFunc==`TLBWI;
wire ITLBMiss,DTLBMiss;

Raptor64_TLB u22
(
	.rst(rst_i),
	.clk(clk),
	.pc(pc),
	.ea(ea),
	.ppc(ppc),
	.pea(pea),
	.m1IsStore(advanceM1 && m1IsStore),
	.ASID(ASID),
	.wTlbp(wTlbp),
	.wTlbrd(wTlbrd),
	.wTlbwr(wTlbwr),
	.wTlbwi(wTlbwi),
	.xTlbrd(xTlbrd),
	.xTlbwr(xTlbwr),
	.xTlbwi(xTlbwi),
	.wr(wMtspr),
	.wregno(wIR[12:7]),
	.dati(wData),
	.xregno(xIR[12:7]),
	.dato(tlbo),
	.ITLBMiss(ITLBMiss),
	.DTLBMiss(DTLBMiss),
	.HTLBVirtPage(TLBVirtPage)
);

`else
assign ppc = pc;
assign pea = ea;
`endif

//-----------------------------------------------------------------------------
// Clock control
// - reset or NMI reenables the clock
// - this circuit must be under the clk_i domain
//-----------------------------------------------------------------------------
//
BUFGCE u20 (.CE(cpu_clk_en), .I(clk_i), .O(clk) );

always @(posedge clk_i)
if (rst_i) begin
	cpu_clk_en <= 1'b1;
end
else begin
	if (nmi_i)
		cpu_clk_en <= 1'b1;
	else
		cpu_clk_en <= clk_en;
end

//-----------------------------------------------------------------------------
// Random number register:
//
// Uses George Marsaglia's multiply method.
//-----------------------------------------------------------------------------
reg [63:0] m_z;
reg [63:0] m_w;
reg [63:0] next_m_z;
reg [63:0] next_m_w;

always @(m_z or m_w)
begin
	next_m_z <= (36'h3696936969 * m_z[31:0]) + m_z[63:32];
	next_m_w <= (36'h1800018000 * m_w[31:0]) + m_w[63:32];
end

wire [63:0] rand = {m_z[31:0],32'd0} + m_w;

wire [10:0] bias = 11'h3FF;				// bias amount (eg 127)
wire [10:0] xl = rand[62:53];
wire sgn = 1'b0;								// floating point: always generate a positive number
wire [10:0] exp = xl > bias-1 ? bias-1 : xl;	// 2^-1 otherwise number could be over 1
wire [52:0] man = rand[52:0];					// a leading '1' will be assumed
wire [63:0] randfd = {sgn,exp,man};
reg [63:0] rando;

//-----------------------------------------------------------------------------
// Instruction Cache / Instruction buffer
// 
// On a bus error, the instruction cache / buffer is loaded with a SYSCALL 509
// instruction, which is a call to the bus error handler.
// 
//-----------------------------------------------------------------------------
//reg lfdir;
reg icaccess;
reg ICacheOn;
wire ibufrdy;
wire [31:0] insnbundle;
reg [31:0] insnbuf;
reg [63:0] ibufadr;
wire isICached = ppc[63:32]!=nonICacheSeg;
//wire isEncrypted = ppc[63:32]==encryptedArea;
wire ICacheAct = ICacheOn & isICached;
reg [31:0] insn1;
reg [31:0] insnkey;
reg [63:0] icadr;

// SYSCALL 509
wire syscall509 = 32'b0000000_11000_0000_11111110_10010111;
wire [63:0] bevect = {syscall509,syscall509};

// Xilinx Core Generator Component
Raptor64_icache_ram u1
(
	.clka(clk), // input clka
	.wea(icaccess & (ack_i|err_i)), // input [0 : 0] wea
	.addra(icadr[12:3]), // input [9 : 0] addra
	.dina(err_i ? bevect : dat_i), // input [63 : 0] dina
	.clkb(~clk), // input clkb
	.addrb(pc[12:2]), // input [8 : 0] addrb
	.doutb(insnbundle) // output [127 : 0] doutb
);

always @(insnbundle or ICacheAct or insnbuf)
begin
	case(ICacheAct)
	1'b0:	insn1 <= insnbuf;
	1'b1:	insn1 <= insnbundle;
	endcase
end

// Decrypt the instruction set.
always @(insn1,insnkey)
	insn <= insn1 ^ insnkey;

reg [63:13] tmem [127:0];
reg [127:0] tvalid;

initial begin
	for (n=0; n < 128; n = n + 1)
		tmem[n] = 0;
	for (n=0; n < 128; n = n + 1)
		tvalid[n] = 0;
end

wire [64:13] tgout;
assign tgout = {tvalid[pc[12:6]],tmem[pc[12:6]]};
assign ihit = (tgout=={1'b1,ppc[63:13]});
assign ibufrdy = ibufadr[63:2]==ppc[63:2];

//-----------------------------------------------------------------------------
// Data Cache
// No-allocate on write
//-----------------------------------------------------------------------------
reg dcaccess;
wire dhit;
wire [64:15] dtgout;
reg wrhit;
reg wr_dcache;
reg [14:0] dcadr;

// cache RAM 32Kb
// Xilinx Core Generator Component
Raptor64_dcache_ram u10
(
	.clka(clk), // input clka
	.ena(1'b1),
	.wea(dcaccess ? {8{ack_i}} : wrhit ? sel_o : 8'h00), // input [7 : 0] wea
	.addra(dcaccess ? dcadr[14:3] : adr_o[14:3]), // input [11 : 0] addra
	.dina(dcaccess ? dat_i : dat_o), // input [63 : 0] dina

	.clkb(~clk), // input clkb
	.addrb(pea[14:3]), // input [11 : 0] addrb
	.doutb(cdat) // output [63 : 0] doutb
);


// Xilinx Core Generator Component
// tag RAM 512 b
Raptor64_dcache_tagram u11
(
	.clka(clk), // input clka
	.ena(dtinit | (dcadr[5:3]==3'b111)), // input ena
	.wea(dtinit | (dcaccess & ack_i)), // input [0 : 0] wea
	.addra(dcadr[14:6]), // input [8 : 0] addra
	.dina({~dtinit,adr_o[63:15]}), // input [49 : 0] dina

	.clkb(~clk), // input clkb
	.addrb(pea[14:6]), // input [8 : 0] addrb
	.doutb(dtgout) // output [49 : 0] doutb
);

assign dhit = (dtgout=={1'b1,pea[63:15]});

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

reg [64:0] xData;
wire xisCacheElement = (xData[63:52] != 12'hFFD && xData[63:52]!=12'hFFF) && dcache_on;
reg m1IsCacheElement;


wire [127:0] mult_out;
wire [63:0] sqrt_out;
wire [63:0] div_q;
wire [63:0] div_r;
wire sqrt_done,mult_done,div_done;
wire isSqrt = xIRvalid && xOpcode==`R && xFunc==`SQRT;

isqrt #(64) u14
(
	.rst(rst_i),
	.clk(clk),
	.ce(1'b1),
	.ld(isSqrt),
	.a(a),
	.o(sqrt_out),
	.done(sqrt_done)
);

wire isMulu = xIRvalid && xOpcode==`RR && xFunc==`MULU;
wire isMuls = xIRvalid && ((xOpcode==`RR && xFunc==`MULS) || xOpcode==`MULSI);
wire isMuli = xIRvalid && (xOpcode==`MULSI || xOpcode==`MULUI);
wire isMult = xIRvalid && (xOpcode==`MULSI || xOpcode==`MULUI || (xOpcode==`RR && (xFunc==`MULS || xFunc==`MULU)));
wire isDivu = xIRvalid && (xOpcode==`RR && xFunc==`DIVU);
wire isDivs = xIRvalid && ((xOpcode==`RR && xFunc==`DIVS) || xOpcode==`DIVSI);
wire isDivi = xIRvalid && (xOpcode==`DIVSI || xOpcode==`DIVUI);
wire isDiv = xIRvalid && (xOpcode==`DIVSI || xOpcode==`DIVUI || (xOpcode==`RR && (xFunc==`DIVS || xFunc==`DIVU)));
wire isModu = xIRvalid && (xOpcode==`RR && xFunc==`MODU);
wire isMods = xIRvalid && (xOpcode==`RR && xFunc==`MODS);
wire isMod = isModu|isMods;

Raptor64Mult u18
(
	.rst(rst_i),
	.clk(clk),
	.ld(isMult),
	.sgn(isMuls),
	.isMuli(isMuli),
	.a(a),
	.b(b),
	.imm(imm),
	.o(mult_out),
	.done(mult_done)
);

Raptor64Div u19
(
	.rst(rst_i),
	.clk(clk),
	.ld(isDiv|isMod),
	.sgn(isDivs|isMods),
	.isDivi(isDivi),
	.a(a),
	.b(b),
	.imm(imm),
	.qo(div_q),
	.ro(div_r),
	.dvByZr(),
	.done(div_done)
);

//-----------------------------------------------------------------------------
// Floating point
//-----------------------------------------------------------------------------

wire [63:0] fpZLOut;
wire [63:0] fpLooOut;
wire fpLooDone;

/*
fpZLUnit #(64) u30 
(
	.op(xFunc[5:0]),
	.a(a),
	.b(b),	// for fcmp
	.o(fpZLOut),
	.nanx()
);

fpLOOUnit #(64) u31
(
	.clk(clk),
	.ce(1'b1),
	.rm(rm),
	.op(xFunc[5:0]),
	.a(a),
	.o(fpLooOut),
	.done(fpLooDone)
);

*/
wire dcmp_result;
wire [63:0] daddsub_result;
wire [63:0] ddiv_result;
wire [63:0] dmul_result;
wire [63:0] i2f_result;
wire [63:0] f2i_result;
wire [63:0] f2d_result;
wire [63:0] d2f_result;

wire f2i_iop,fpmul_iop,fpdiv_iop,fpaddsub_iop,fpcmp_iop;
wire f2i_ovr,fpmul_ovr,fpdiv_ovr,fpaddsub_ovr;
wire fpmul_uf,fpaddsub_uf,fpdiv_uf;


`ifdef FLOATING_POINT
// Xilinx Core Generator Components

Raptor64_fpCmp u60
(
	.a(a), // input [63 : 0] a
	.b(b), // input [63 : 0] b
	.operation(xFunc6), // input [5 : 0] operation
	.clk(clk), // input clk
	.result(dcmp_result), // ouput [0 : 0] result
	.invalid_op(fpcmp_iop)
); // ouput invalid_op

Raptor64_fpAddsub u61
(
	.a(a), // input [63 : 0] a
	.b(b), // input [63 : 0] b
	.operation(xFunc6), // input [5 : 0] operation
	.clk(clk), // input clk
	.result(daddsub_result), // ouput [63 : 0] result
	.underflow(fpaddsub_uf), // ouput underflow
	.overflow(fpaddsub_ovr), // ouput overflow
	.invalid_op(fpaddsub_iop)
); // ouput invalid_op

Raptor64_fpDiv u62
(
	.a(a), // input [63 : 0] a
	.b(b), // input [63 : 0] b
	.clk(clk), // input clk
	.result(ddiv_result), // ouput [63 : 0] result
	.underflow(fpdiv_uf), // ouput underflow
	.overflow(fpdiv_ovr), // ouput overflow
	.invalid_op(fpdiv_iop), // ouput invalid_op
	.divide_by_zero()
); // ouput divide_by_zero

Raptor64_fpMul u63
(
	.a(a), // input [63 : 0] a
	.b(b), // input [63 : 0] b
	.clk(clk), // input clk
	.result(dmul_result), // ouput [63 : 0] result
	.underflow(fpmul_uf), // ouput underflow
	.overflow(fpmul_ovr), // ouput overflow
	.invalid_op(fpmul_iop)
); // ouput invalid_op

Raptor64_fpItoF u64
(
	.a(a), // input [63 : 0] a
	.clk(clk), // input clk
	.result(i2f_result)
); // ouput [63 : 0] result

Raptor64_fpFtoI u65
(
	.a(a), // input [63 : 0] a
	.clk(clk), // input clk
	.result(f2i_result), // ouput [63 : 0] result
	.overflow(f2i_ovr), // ouput overflow
	.invalid_op(f2i_iop)
); // ouput invalid_op

`endif

always @(posedge clk)
if (rst_i) begin
	fltctr <= 6'd0;
end
else begin
	if (fltdone) begin
		FPC_overx <= fp_ovr;
	end
	if (advanceX) begin
		if (xOpcode==`FP && xIRvalid) begin
			if (xFunc6==`FDADD)	// FDADD
				fltctr <= 6'd12;
			else if (xFunc6==`FDSUB)	// FDSUB
				fltctr <= 6'd12;
			else if (xFunc6==`FDMUL)	// FDMUL
				fltctr <= 6'd12;
			else if (xFunc6==`FDDIV)	// FDDIV
				fltctr <= 6'd12;
			else if (xFunc6==6'b000100)	// unordered
				fltctr <= 6'd2;
			else if (xFunc6==6'b001100)	// less than
				fltctr <= 6'd2;
			else if (xFunc6==6'b010100)	// equal
				fltctr <= 6'd2;
			else if (xFunc6==6'b011100)	// less than or equal
				fltctr <= 6'd2;
			else if (xFunc6==6'b100100)	// greater than
				fltctr <= 6'd2;
			else if (xFunc6==6'b101100)	// not equal
				fltctr <= 6'd2;
			else if (xFunc6==6'b110100)	// greater than or equal
				fltctr <= 6'd2;
			else if (xFunc6==`FDI2F)	// ItoFD
				fltctr <= 6'd7;
			else if (xFunc6==6'b000110)	// FFtoI
				fltctr <= 6'd6;
			else if (xFunc6==6'b000111)	// FtoD
				fltctr <= 6'd2;
			else if (xFunc6==6'b001000) // DtoF
				fltctr <= 6'd2;
			else
				fltctr <= 6'd0;
		end
	end
	else begin
		if (fltctr > 6'd0)
			fltctr <= fltctr - 6'd1;
	end
end
		
function [2:0] popcnt6;
input [5:0] a;
begin
case(a)
6'b000000:	popcnt6 = 3'd0;
6'b000001:	popcnt6 = 3'd1;
6'b000010:	popcnt6 = 3'd1;
6'b000011:	popcnt6 = 3'd2;
6'b000100:	popcnt6 = 3'd1;
6'b000101:	popcnt6 = 3'd2;
6'b000110:	popcnt6 = 3'd2;
6'b000111:	popcnt6 = 3'd3;
6'b001000:	popcnt6 = 3'd1;
6'b001001:	popcnt6 = 3'd2;
6'b001010:	popcnt6 = 3'd2;
6'b001011:	popcnt6 = 3'd3;
6'b001100:	popcnt6 = 3'd2;
6'b001101:	popcnt6 = 3'd3;
6'b001110:	popcnt6 = 3'd3;
6'b001111:  popcnt6 = 3'd4;
6'b010000:	popcnt6 = 3'd1;
6'b010001:	popcnt6 = 3'd2;
6'b010010:  popcnt6 = 3'd2;
6'b010011:	popcnt6 = 3'd3;
6'b010100:  popcnt6 = 3'd2;
6'b010101:  popcnt6 = 3'd3;
6'b010110:  popcnt6 = 3'd3;
6'b010111:	popcnt6 = 3'd4;
6'b011000:	popcnt6 = 3'd2;
6'b011001:	popcnt6 = 3'd3;
6'b011010:	popcnt6 = 3'd3;
6'b011011:	popcnt6 = 3'd4;
6'b011100:	popcnt6 = 3'd3;
6'b011101:	popcnt6 = 3'd4;
6'b011110:	popcnt6 = 3'd4;
6'b011111:	popcnt6 = 3'd5;
6'b100000:	popcnt6 = 3'd1;
6'b100001:	popcnt6 = 3'd2;
6'b100010:	popcnt6 = 3'd2;
6'b100011:	popcnt6 = 3'd3;
6'b100100:	popcnt6 = 3'd2;
6'b100101:	popcnt6 = 3'd3;
6'b100110:	popcnt6 = 3'd3;
6'b100111:	popcnt6 = 3'd4;
6'b101000:	popcnt6 = 3'd2;
6'b101001:	popcnt6 = 3'd3;
6'b101010:	popcnt6 = 3'd3;
6'b101011:	popcnt6 = 3'd4;
6'b101100:	popcnt6 = 3'd3;
6'b101101:	popcnt6 = 3'd4;
6'b101110:	popcnt6 = 3'd4;
6'b101111:	popcnt6 = 3'd5;
6'b110000:	popcnt6 = 3'd2;
6'b110001:	popcnt6 = 3'd3;
6'b110010:	popcnt6 = 3'd3;
6'b110011: 	popcnt6 = 3'd4;
6'b110100:	popcnt6 = 3'd3;
6'b110101:	popcnt6 = 3'd4;
6'b110110:	popcnt6 = 3'd4;
6'b110111:	popcnt6 = 3'd5;
6'b111000:	popcnt6 = 3'd3;
6'b111001:	popcnt6 = 3'd4;
6'b111010: 	popcnt6 = 3'd4;
6'b111011:	popcnt6 = 3'd5;
6'b111100:	popcnt6 = 3'd4;
6'b111101:	popcnt6 = 3'd5;
6'b111110:	popcnt6 = 3'd5;
6'b111111:	popcnt6 = 3'd6;
endcase
end
endfunction

function [5:0] popcnt36;
input [35:0] a;
begin
popcnt36 = popcnt6(a[5:0]) + 
			popcnt6(a[11:6]) +
			popcnt6(a[17:12]) +
			popcnt6(a[23:18]) +
			popcnt6(a[29:24]) +
			popcnt6(a[35:30]);
end
endfunction

wire [63:0] jmp_tgt = {pc[63:27],insn[24:0],2'b00};

//-----------------------------------------------------------------------------
// Stack for return address predictor
//-----------------------------------------------------------------------------
reg [63:0] ras [63:0];	// return address stack, return predictions
reg [5:0] ras_sp;		// stack pointer
initial begin
	for (n = 0; n < 64; n = n + 1)
		ras[n] = 0;
end
`ifdef BTB
reg [63:0] btb [63:0];	// branch target buffer
`endif

//-----------------------------------------------------------------------------
// Branch history table.
// The history table is updated by the EX stage and read in
// both the EX and IF stages.
//-----------------------------------------------------------------------------
wire predict_taken;

Raptor64_BranchHistory u6
(
	.rst(rst_i),
	.clk(clk),
	.advanceX(advanceX),
	.xIRvalid(xIRvalid),
	.xIR(xIR),
	.pc(pc),
	.xpc(xpc),
	.takb(takb),
	.predict_taken(predict_taken)
);

//-----------------------------------------------------------------------------
// Evaluate branch conditions.
//-----------------------------------------------------------------------------

Raptor64_EvaluateBranch u4
(
	.ir(xIR),
	.a(a),
	.b(b),
	.imm(imm),
	.rsf(rsf),
	.takb(takb)
);

//-----------------------------------------------------------------------------
// Datapath (ALU) operations.
//-----------------------------------------------------------------------------
reg [63:0] xData1;
wire [63:0] xBitfieldo,xSeto,xLogico,xShifto,xAddsubo;

wire [6:0] cntlzo,cntloo;
cntlz64 u12 (.clk(clk), .i(a),  .o(cntlzo) );
cntlo64 u13 (.clk(clk), .i(a),  .o(cntloo) );
//cntlz64 u12 (.i(a),  .o(cntlzo) );
//cntlo64 u13 (.i(a),  .o(cntloo) );

reg [1:0] shftop;
wire [63:0] shfto;
wire [63:0] masko;
reg [63:0] bfextd;
wire [63:0] rolo;
wire [15:0] bcdmulo;

Raptor64_addsub u21 (xIR,a,b,imm,xAddsubo);
Raptor64_logic   u9 (xIR,a,b,imm,xLogico);
Raptor64_set    u15 (xIR,a,b,imm,xSeto);
Raptor64_bitfield u16(xIR, a, b, xBitfieldo, masko);
Raptor64_shift  u17 (xIR, a, b, masko, xShifto, rolo);
BCDMul2 u22 (a[7:0],b[7:0],bcdmulo);

wire aeqz = a==64'd0;
wire eq = a==b;
wire eqi = a==imm;
wire lt = $signed(a) < $signed(b);
wire lti = $signed(a) < $signed(imm);
wire ltu = a < b;
wire ltui = a < imm;

always @(xOpcode or xFunc or xFunc5 or a or b or imm or xpc or aeqz or
	sqrt_out or cntlzo or cntloo or tick or AXC or
	lt or eq or ltu or mult_out or lti or eqi or ltui or xIR or div_q or div_r or
	shfto or masko or bcdmulo or fpLooOut or fpZLOut or m_z or m_w or
`ifdef TLB
	PageTableAddr or BadVAddr or ASID or tlbo or
`endif
	ASID or EPC or mutex_gate or IPC or TBA or xAXC or nonICacheSeg or rm or
	rando or errorAddress
)
casex(xOpcode)
`MISC:
	case(xFunc)
	`SYSCALL:
		if (xIR[16])
			xData1 = fnIncPC(xpc);
		else
			xData1 = xpc;
	default:	xData1 = 64'd0;
	endcase
`R:
	casex(xFunc6)
	`COM:	xData1 = ~a;
	`NOT:	xData1 = ~|a;
	`NEG:	xData1 = -a;
	`ABS:	xData1 = a[63] ? -a : a;
	`SGN:	xData1 = a[63] ? 64'hFFFFFFFF_FFFFFFFF : aeqz ? 64'd0 : 64'd1;
	`MOV:	xData1 = a;
	`SQRT:	xData1 = sqrt_out;
	`SWAP:	xData1 = {a[31:0],a[63:32]};
	`RBO:	xData1 = {a[7:0],a[15:8],a[23:16],a[31:24],a[39:32],a[47:40],a[55:48],a[63:56]};
	
	`REDOR:		xData1 = |a;
	`REDAND:	xData1 = &a;

	`CTLZ:	xData1 = cntlzo;
	`CTLO:	xData1 = cntloo;
	`CTPOP:	xData1 = {4'd0,popcnt6(a[5:0])} +
					{4'd0,popcnt6(a[11:6])} +
					{4'd0,popcnt6(a[17:12])} +
					{4'd0,popcnt6(a[23:18])} +
					{4'd0,popcnt6(a[29:24])} +
					{4'd0,popcnt6(a[35:30])} +
					{4'd0,popcnt6(a[41:36])} +
					{4'd0,popcnt6(a[47:42])} +
					{4'd0,popcnt6(a[53:48])} +
					{4'd0,popcnt6(a[59:54])} +
					{4'd0,popcnt6(a[63:60])}
					;
	`SEXT8:		xData1 = {{56{a[7]}},a[7:0]};	
	`SEXT16:	xData1 = {{48{a[15]}},a[15:0]};
	`SEXT32:	xData1 = {{32{a[31]}},a[31:0]};

	`MTSPR:		xData1 = a;
	`MFSPR:
		case(xIR[11:6])
`ifdef TLB
		`TLBWired:		xData1 = tlbo;
		`TLBIndex:		xData1 = tlbo;
		`TLBRandom:		xData1 = tlbo;
		`TLBPhysPage0:	xData1 = tlbo;
		`TLBPhysPage1:	xData1 = tlbo;
		`TLBVirtPage:	xData1 = tlbo;
		`TLBPageMask:	xData1 = tlbo;
		`TLBASID:	begin
					xData1 = 64'd0;
					xData1[0] = tlbo[0];
					xData1[1] = tlbo[1];
					xData1[2] = tlbo[2];
					xData1[15:8] = tlbo[15:8];
					end
		`PageTableAddr:	xData1 = {PageTableAddr,13'd0};
		`BadVAddr:		xData1 = {BadVAddr,13'd0};
`endif
		`ASID:			xData1 = ASID;
		`Tick:			xData1 = tick;
		`EPC:			xData1 = EPC[xAXC];
		`IPC:			xData1 = IPC[xAXC];
		`TBA:			xData1 = TBA;
		`ERRADR:		xData1 = errorAddress;
		`AXC:			xData1 = xAXC;
		`NON_ICACHE_SEG:	xData1 = nonICacheSeg;
		`FPCR:			xData1 = FPC;
		`RAND:			xData1 = rando;
		`SRAND1:		xData1 = m_z;
		`SRAND2:		xData1 = m_w;
		`INSNKEY:		xData1 = insnkey;
		`PCHISTORIC:	xData1 = pchistoric;
		default:	xData1 = 64'd0;
		endcase
	`OMG:		xData1 = mutex_gate[a[5:0]];
	`CMG:		xData1 = mutex_gate[a[5:0]];
	`OMGI:		begin
				xData1 = mutex_gate[xIR[11:6]];
				$display("mutex_gate[%d]=%d",xIR[11:6],mutex_gate[xIR[11:6]]);
				end
	`CMGI:		xData1 = mutex_gate[xIR[11:6]];
	default:	xData1 = 64'd0;
	endcase
`RR:
	case(xFunc6)
	`CMP:	xData1 = lt ? 64'hFFFFFFFFFFFFF : eq ? 64'd0 : 64'd1;
	`CMPU:	xData1 = ltu ? 64'hFFFFFFFFFFFFF : eq ? 64'd0 : 64'd1;
	`MIN:	xData1 = lt ? a : b;
	`MAX:	xData1 = lt ? b : a;
	`MOVZ:	xData1 = b;
	`MOVNZ:	xData1 = b;
	`MOVPL:	xData1 = b;
	`MOVMI:	xData1 = b;
	`MULS:	xData1 = mult_out[63:0];
	`MULU:	xData1 = mult_out[63:0];
	`DIVS:	xData1 = div_q;
	`DIVU:	xData1 = div_q;
	`MODU:	xData1 = div_r;
	`MODS:	xData1 = div_r;
	`BCD_MUL:	xData1 = bcdmulo;
	`MFEP:	xData1 = epat[a[7:0]];
 	default:	xData1 = 64'd0;
	endcase
`BTRR:
	case(xFunc5)
	`LOOP:	xData1 = b - 64'd1;
	default:	xData1 = 64'd0;
	endcase
`MUX:
	begin
		for (n = 0; n < 64; n = n + 1)
			xData1[n] = c[n] ? b[n] : a[n];
	end
`SETLO:		xData1 = {{42{xIR[21]}},xIR[21:0]};
`SETMID:	xData1 = {{20{xIR[21]}},xIR[21:0],a[21:0]};
`SETHI:		xData1 = {xIR[19:0],a[43:0]};
`CMPI:	xData1 = lti ? 64'hFFFFFFFFFFFFF : eqi ? 64'd0 : 64'd1;
`CMPUI:	xData1 = ltui ? 64'hFFFFFFFFFFFFF : eqi ? 64'd0 : 64'd1;
`MULSI:	xData1 = mult_out[63:0];
`MULUI:	xData1 = mult_out[63:0];
`DIVSI:	xData1 = div_q;
`DIVUI:	xData1 = div_q;
`INB,`INCH,`INH,`INW,`INCU,`INHU,`INBU:
		xData1 = a + imm;
`OUTB,`OUTC,`OUTH,`OUTW,`OUTBC:
		xData1 = a + imm;
`LW,`LH,`LC,`LB,`LHU,`LCU,`LBU,`LEA:
		xData1 = a + imm;
`SW,`SH,`SC,`SB:
		xData1 = a + imm;
`ifdef ADDRESS_RESERVATION
`LWR:	xData1 = a + imm;
`SWC:	xData1 = a + imm;
`endif
`ifdef FLOATING_POINT
`LF,`LFD:	xData1 = a + imm;
`SF,`SFD:	xData1 = a + imm;
`LFP,`LFDP:	xData1 = a + imm + xIR[15];
`SFP,`SFDP:	xData1 = a + imm + xIR[15];
`endif
//`LP:	xData1 = a + imm + xIR[15];
//`SP:	xData1 = a + imm + xIR[15];
`MEMNDX:
		case(xFunc6)
//		`LPX,`LFPX,`LFDPX,`SPX,`SFPX,`SFDPX:
//			xData1 = a + (b << scale) + imm + xIR[15];
		default:
			xData1 = a + (b << scale) + imm;
		endcase
`TRAPcc:	xData1 = fnIncPC(xpc);
`TRAPcci:	xData1 = fnIncPC(xpc);
`CALL:		xData1 = fnIncPC(xpc);
`JAL:		xData1 = fnIncPC(xpc);//???xpc + {xIR[19:15],2'b00};
`RET:	xData1 = a + imm;
`FPLOO:	xData1 = fpLooOut;
`FPZL:	xData1 = fpZLOut;
`ifdef FLOATING_POINT
`FP:
	case(xFunc6)
	`FDADD:	xData1 = daddsub_result;
	`FDSUB:	xData1 = daddsub_result;
	`FDMUL:	xData1 = dmul_result;
	`FDDIV:	xData1 = ddiv_result;
	`FDI2F:	xData1 = i2f_result;
	`FDF2I:	xData1 = f2i_result;
	`FDCUN:	xData1 = dcmp_result;
	`FDCEQ:	xData1 = dcmp_result;
	`FDCNE:	xData1 = dcmp_result;
	`FDCLT:	xData1 = dcmp_result;
	`FDCLE:	xData1 = dcmp_result;
	`FDCGT:	xData1 = dcmp_result;
	`FDCGE:	xData1 = dcmp_result;
	default:	xData1 = 64'd0;
	endcase
`endif
default:	xData1 = 64'd0;
endcase

always @(xData1,xBitfieldo,xLogico,xShifto,xSeto,xAddsubo)
	xData = xData1|xBitfieldo|xLogico|xShifto|xSeto|xAddsubo;

wire v_ri,v_rr;
overflow u2 (.op(xOpcode==`SUBI), .a(a[63]), .b(imm[63]), .s(xAddsubo[63]), .v(v_ri));
overflow u3 (.op(xOpcode==`RR && xFunc==`SUB), .a(a[63]), .b(b[63]), .s(xAddsubo[63]), .v(v_rr));

wire dbz_error = xIRvalid && (((xOpcode==`DIVSI||xOpcode==`DIVUI) && imm==64'd0) || (xOpcode==`RR && (xFunc6==`DIVS || xFunc6==`DIVU) && b==64'd0));
wire ovr_error = xIRvalid && (((xOpcode==`ADDI || xOpcode==`SUBI) && v_ri) || ((xOpcode==`RR && (xFunc6==`SUB || xFunc6==`ADD)) && v_rr));
// ToDo: add more priv violations
wire priv_violation = xIRvalid && !KernelMode && (xOpcode==`MISC &&
	(xFunc==`IRET || xFunc==`ERET || xFunc==`CLI || xFunc==`SEI ||
	 xFunc==`TLBP || xFunc==`TLBR || xFunc==`TLBWR || xFunc==`TLBWI || xFunc==`IEPP
	));
// ToDo: detect illegal instructions in the hives (sub-opcodes)
wire illegal_insn = xIRvalid && (
		xOpcode==7'd19 ||
		xOpcode==7'd20 ||
		xOpcode==7'd28 ||
		xOpcode==7'd29 ||
		xOpcode==7'd30 ||
		xOpcode==7'd31 ||
		xOpcode==7'd47 ||
		xOpcode==7'd55 ||
		xOpcode==7'd63 ||
		xOpcode==7'd90 ||
		xOpcode==7'd91 ||
		xOpcode==7'd92 ||
		xOpcode==7'd93 ||
		xOpcode==7'd106 ||
		xOpcode==7'd107 ||
		xOpcode==7'd124 ||
		xOpcode==7'd125 ||
		xOpcode==7'd126 ||
		xOpcode==7'd127
		)
		;

//-----------------------------------------------------------------------------
// For performance and core size reasons, the following should really decode
// the opcodes in the decode stage, then pass the decoding information forward
// using regs. However the core is trickier to get working that way; decoding
// in multiple stages is simpler.
//-----------------------------------------------------------------------------
//wire dIsLoad = dIRvalid && (
//	dOpcode==`LW || dOpcode==`LH || dOpcode==`LB || dOpcode==`LWR ||
//	dOpcode==`LHU || dOpcode==`LBU ||
//	dOpcode==`LC || dOpcode==`LCU || dOpcode==`LM ||
//	dOpcode==`LF || dOpcode==`LFD || dOpcode==`LP || dOpcode==`LFP || dOpcode==`LFDP ||
//	dOpcode==`LSH || dOpcode==`LSW ||
//	(dOpcode==`MEMNDX && (
//		dFunc6==`LWX || dFunc6==`LHX || dFunc6==`LBX || dFunc6==`LWRX ||
//		dFunc6==`LHUX || dFunc6==`LBUX ||
//		dFunc6==`LCX || dFunc6==`LCUX ||
//		dFunc6==`LFX || dFunc6==`LFDX || dFunc6==`LPX ||
//		dFunc6==`LSHX || dFunc6==`LSWX
//	)) ||
//	(dOpcode==`MISC && (dFunc==`SYSJMP || dFunc==`SYSCALL || dFunc==`SYSINT)))
//	;
//wire dIsStore = dIRvalid && (
//	dOpcode==`SW || dOpcode==`SH || dOpcode==`SB || dOpcode==`SC || dOpcode==`SWC || dOpcode==`SM ||
//	dOpcode==`SF || dOpcode==`SFD || dOpcode==`SP || dOpcode==`SFP || dOpcode==`SFDP ||
//	dOpcode==`SSH || dOpcode==`SSW ||
//	(dOpcode==`MEMNDX && (
//		dFunc6==`SWX || dFunc6==`SHX || dFunc6==`SBX || dFunc6==`SCX || dFunc6==`SWCX ||
//		dFunc6==`SFX || dFunc6==`SFDX || dFunc6==`SPX ||
//		dFunc6==`SSHX || dFunc6==`SSWX
//	)))
//	;
//wire dIsIn = dIRvalid && (
//	dOpcode==`INW || dOpcode==`INH || dOpcode==`INCH || dOpcode==`INB ||
//	dOpcode==`INHU || dOpcode==`INCU || dOpcode==`INBU ||
//	(dOpcode==`MEMNDX && (
//		dFunc6==`INWX || dFunc6==`INHX || dFunc6==`INCX || dFunc6==`INBX ||
//		dFunc6==`INHUX || dFunc6==`INCUX || dFunc6==`INBUX
//	)))
//	;
//wire dIsOut = dIRvalid && (dOpcode==`OUTW || dOpcode==`OUTH || dOpcode==`OUTC || dOpcode==`OUTB ||
//	(dOpcode==`MEMNDX && (
//		dFunc6==`OUTWX || dFunc6==`OUTHX || dFunc6==`OUTCX || dFunc6==`OUTBX
//	)))
//	;


//-----------------------------------------------------------------------------
// Pipeline advance and stall logic
//-----------------------------------------------------------------------------
wire xIsSqrt = xIRvalid && xOpcode==`R && xFunc6==`SQRT;
wire xIsMult = xIRvalid && ((xOpcode==`RR && (xFunc6==`MULU || xFunc6==`MULS)) || xOpcode==`MULSI || xOpcode==`MULUI);
wire xIsDiv = xIRvalid && ((xOpcode==`RR && (xFunc6==`DIVU || xFunc6==`DIVS || xFunc6==`MODU || xFunc6==`MODS)) || xOpcode==`DIVSI || xOpcode==`DIVUI);
wire xIsCnt = xIRvalid && (xOpcode==`R && (xFunc6==`CTLZ || xFunc6==`CTLO || xFunc6==`CTPOP));
reg m1IsCnt,m2IsCnt;
reg m2IsCacheElement;

// Have to set the xIsLoad/xIsStore flag to false when xIR is nopped out
wire xIsLoad = xIRvalid && (
	xOpcode==`LW || xOpcode==`LH || xOpcode==`LB || xOpcode==`LWR ||
	xOpcode==`LHU || xOpcode==`LBU ||
	xOpcode==`LC || xOpcode==`LCU || xOpcode==`LM ||
	xOpcode==`LF || xOpcode==`LFD || xOpcode==`LP || xOpcode==`LFP || xOpcode==`LFDP ||
	xOpcode==`LSH || xOpcode==`LSW ||
	(xOpcode==`MEMNDX && (
		xFunc6==`LWX || xFunc6==`LHX || xFunc6==`LBX || xFunc6==`LWRX ||
		xFunc6==`LHUX || xFunc6==`LBUX ||
		xFunc6==`LCX || xFunc6==`LCUX ||
		xFunc6==`LFX || xFunc6==`LFDX || xFunc6==`LPX ||
		xFunc6==`LSHX || xFunc6==`LSWX
	)) ||
	(xOpcode==`MISC && (xFunc==`SYSCALL))
	)
	;
wire xIsStore = xIRvalid && (
	xOpcode==`SW || xOpcode==`SH || xOpcode==`SB || xOpcode==`SC || xOpcode==`SWC || xOpcode==`SM ||
	xOpcode==`SF || xOpcode==`SFD || xOpcode==`SP || xOpcode==`SFP || xOpcode==`SFDP ||
	xOpcode==`SSH || xOpcode==`SSW || xOpcode==`STBC ||
	(xOpcode==`MEMNDX && (
		xFunc6==`SWX || xFunc6==`SHX || xFunc6==`SBX || xFunc6==`SCX || xFunc6==`SWCX ||
		xFunc6==`SFX || xFunc6==`SFDX || xFunc6==`SPX ||
		xFunc6==`SSHX || xFunc6==`SSWX
	))
	)
	;
wire xIsSWC = xOpcode==`SWC && xIRvalid;
wire xIsIn = xIRvalid && (
	xOpcode==`INW || xOpcode==`INH || xOpcode==`INCH || xOpcode==`INB ||
	xOpcode==`INHU || xOpcode==`INCU || xOpcode==`INBU ||
	(xOpcode==`MEMNDX && (
		xFunc6==`INWX || xFunc6==`INHX || xFunc6==`INCX || xFunc6==`INBX ||
		xFunc6==`INHUX || xFunc6==`INCUX || xFunc6==`INBUX
	))
	)
	;
wire xIsOut = xIRvalid && (
	xOpcode==`OUTW || xOpcode==`OUTH || xOpcode==`OUTC || xOpcode==`OUTB || 
	(xOpcode==`MEMNDX && (
		xFunc6==`OUTWX || xFunc6==`OUTHX || xFunc6==`OUTCX || xFunc6==`OUTBX
	)))
	;
reg m1IsLoad,m1IsStore;
reg m2IsLoad,m2IsStore;
reg wIsStore;
reg m1IsOut,m1IsIn;

//wire mIsSWC = mOpcode==`SWC;
//reg m1IsIn;

wire m2IsInW = m2Opcode==`INW;
wire xIsIO = xIsIn || xIsOut;
wire m1IsIO = m1IsIn || m1IsOut;

wire xIsFPLoo = xIRvalid && xOpcode==`FPLOO;
wire xIsFP = xIRvalid && xOpcode==`FP;
wire xneedBus = xIsIO;
//wire m1needBus = (m1IsLoad & !m1IsCacheElement) || m1IsStore || m1IsIO;
wire m1needBus = m1IsLoad || m1IsStore || m1IsIO;
wire m2needBus = m2IsLoad || m2IsStore;

wire xRtz = xRt[4:0]==5'd0;
wire m1Rtz = m1Rt[4:0]==5'd0;
wire m2Rtz = m2Rt[4:0]==5'd0;

//wire StallI = dIsLSPair & ~dIR[15];
wire StallI = 1'b0;

// Stall on SWC allows rsf flag to be loaded for the next instruction
// Could check for dRa,dRb,dRc==0, for non-stalling
wire StallR =  	((( xIsLoad||xIsIn||xIsCnt) &&   (( xRt==dRa)||( xRt==dRb)||( xRt==dRc)) && !xRtz) || xIsSWC) ||
				(((m1IsLoad||m1IsIn||m1IsCnt) && ((m1Rt==dRa)||(m1Rt==dRb)||(m1Rt==dRc)) && !m1Rtz)) ||
				(((m2IsLoad||m2IsCnt) &&         ((m2Rt==dRa)||(m2Rt==dRb)||(m2Rt==dRc)) && !m2Rtz))
				;
wire StallX = (xneedBus||xIsLoad||xIsStore) & (m1needBus|m2needBus|icaccess|dcaccess);
wire StallM1 = (m1needBus & (m2needBus|icaccess|dcaccess)) ||
				( m1IsLoad & m1IsCacheElement & (m2IsStore|wIsStore))	// wait for a preceding store to complete
				;
wire StallM2 =  m2needBus & (icaccess|dcaccess);

assign advanceT = !resetA;
assign advanceW = advanceT;
assign advanceM2 = advanceW && (cyc_o ? (ack_i|err_i) : 1'b1) && !StallM2;
assign advanceM1 = advanceM2 &
					(iocyc_o ? (ack_i|err_i) : 1'b1) &
					((m1IsLoad & m1IsCacheElement) ? dhit : 1'b1) & 
					!StallM1
					;
assign advanceX = advanceM1 & (
					xIsSqrt ? sqrt_done :
					xIsMult ? mult_done :
					xIsDiv ? div_done :
					xIsFPLoo ? fpLooDone :
					xIsFP ? fltdone :
					1'b1) &
					!StallX;
assign advanceR = advanceX & !StallR;
assign advanceI = advanceR & (ICacheAct ? ihit : ibufrdy) & !StallI;

//-----------------------------------------------------------------------------
// Cache loading control
//-----------------------------------------------------------------------------
wire pipelineEmpty = 
						(dOpcode==`NOPI || !dIRvalid) &&			// and the pipeline is flushed
						(xOpcode==`NOPI || !xIRvalid) &&
						(m1Opcode==`NOPI || !m1IRvalid) &&
						(m2Opcode==`NOPI || !m2IRvalid)
						;
wire triggerDCacheLoad = (m1IsLoad & m1IsCacheElement & !dhit) &&	// there is a miss
						!(icaccess | dcaccess) && 	// caches are not active
						(m2Opcode==`NOPI || !m2IRvalid);		// and the pipeline is free of memory-ops
						;
wire triggerICacheLoad1 = ICacheAct && !ihit && !triggerDCacheLoad &&	// There is a miss
						!(icaccess | dcaccess) && 	// caches are not active
						pipelineEmpty;
wire triggerICacheLoad2 = (!ICacheAct && !ibufrdy) && !triggerDCacheLoad &&	// There is a miss
						!(icaccess | dcaccess) &&	// caches are not active
						pipelineEmpty;

wire triggerICacheLoad = triggerICacheLoad1 | triggerICacheLoad2;

wire EXexception_pending = ovr_error || dbz_error || priv_violation || xOpcode==`TRAPcci || xOpcode==`TRAPcc;
`ifdef TLB
wire M1exception_pending = advanceM1 & (m1IsLoad|m1IsStore) & DTLBMiss;
`else
wire M1exception_pending = 1'b0;
`endif
wire exception_pending = EXexception_pending | M1exception_pending;

reg prev_nmi,nmi_edge;

//-----------------------------------------------------------------------------
// Register file.
//-----------------------------------------------------------------------------

wire [63:0] nxt_a, nxt_b, nxt_c;

Raptor64_regfile u5
(
	.clk(clk),
	.advanceR(advanceR),
	.advanceW(advanceW),
	.wIRvalid(wIRvalid),
	.dRa(dRa),
	.dRb(dRb),
	.dRc(dRc),
	.dpc(dpc),
	.xRt(xRt),
	.m1Rt(m1Rt),
	.m2Rt(m2Rt),
	.wRt(wRt),
	.tRt(tRt),
	.xData(xData[63:0]),
	.m1Data(m1Data),
	.m2Data(m2Data),
	.wData(wData),
	.tData(tData),
	.nxt_a(nxt_a),
	.nxt_b(nxt_b),
	.nxt_c(nxt_c)
);

Raptor64_SetOperandRegs u7
(
	.rst(rst_i),
	.clk(clk),
	.advanceI(advanceI),
	.advanceR(advanceR),
	.advanceX(advanceX),
	.b(b),
	.AXC(AXC),
	.xAXC(xAXC),
	.insn(insn),
	.xIR(xIR),
	.dRa(dRa),
	.dRb(dRb),
	.dRc(dRc)
);

Raptor64_SetTargetRegister u8
(
	.rst(rst_i),
	.clk(clk),
	.advanceR(advanceR),
	.advanceX(advanceX),
	.dIRvalid(dIRvalid),
	.dIR(dIR),
	.dAXC(dAXC),
	.xRt(xRt)
);

reg [5:0] pchi;
vtdl #(64,64) u23
(
	.clk(clk),
	.ce(advanceI & pccap),
	.a(pchi),
	.d(pc),
	.q(pchistoric)
);

always @(posedge clk)
if (rst_i) begin
	bte_o <= 2'b00;
	cti_o <= 3'b000;
	iocyc_o <= 1'b0;
	cyc_o <= 1'b0;
	stb_o <= 1'b0;
	we_o <= 1'b0;
	sel_o <= 8'h00;
	adr_o <= 64'd0;
	dat_o <= 64'd0;

	state <= RESET;
	cstate <= IDLE;
	pccap <= 1'b1;
	nonICacheSeg <= 32'hFFFF_FFFD;
	TBA <= 64'd0;
	pc <= `RESET_VECTOR;
	m1Opcode <= `NOPI;
	m2Opcode <= `NOPI;
	m1IR <= `NOP_INSN;
	m2IR <= `NOP_INSN;
	wIR <= `NOP_INSN;
	wOpcode <= `NOPI;
	dIR <= `NOP_INSN;
	m1IsLoad <= 1'b0;
	m1IsStore <= 1'b0;
	m2IsLoad <= 1'b0;
	m2IsStore <= 1'b0;
	wIsStore <= 1'b0;
	m1IsOut <= 1'b0;
	m1IsIn <= 1'b0;
	xRtZero <= 1'b0;
	tRt <= 9'd0;
	wRt <= 9'd0;
	m1Rt <= 9'd0;
	m2Rt <= 9'd0;
	tData <= 64'd0;
	wData <= 64'd0;
	m1Data <= 64'd0;
	m2Data <= 64'd0;
	icaccess <= 1'b0;
	dcaccess <= 1'b0;
	prev_ihit <= 1'b0;
	wFip <= 1'b0;
	m2Fip <= 1'b0;
	m1Fip <= 1'b0;
	xFip <= 1'b0;
	dFip <= 1'b0;
	dirqf <= 1'b0;
	ipcv <= 1'b1;
	dpcv <= 1'b0;
	xpcv <= 1'b0;
	m1pcv <= 1'b0;
	m2pcv <= 1'b0;
	wpcv <= 1'b0;
	tick <= 64'd0;
	cstate <= IDLE;
	dAXC <= 4'd0;
	xAXC <= 4'd0;
	m1AXC <= 4'd0;
	m2AXC <= 4'd0;
	wAXC <= 4'd0;
	xirqf <= 1'b0;
	dextype <= 9'h00;
	xextype <= 9'h00;
	m1extype <= 9'h00;
	m2extype <= 9'h00;
	wextype <= 9'h00;
	textype <= 9'h00;
	xIR <= `NOP_INSN;
	xpc <= 64'd0;
	a <= 64'd0;
	b <= 64'd0;
	imm <= 64'd0;
	clk_en <= 1'b1;
	StatusEXL <= 16'hFFFF;
	StatusHWI <= 1'b0;
	resetA <= 1'b1;
	mutex_gate <= 64'h0;
	dcache_on <= 1'b0;
	ICacheOn <= 1'b0;
	ibufadr <= 64'h0;
	m1IsCacheElement <= 1'b0;
	dtinit <= 1'b1;
	ras_sp <= 6'd63;
	im <= 1'b1;
	im1 <= 1'b1;
// These must be non-zero in order to produce random numbers
// We set them here in case the user doesn't bother to set them.
	m_z <= 64'h0123456789ABCDEF;
	m_w <= 64'h8888888877777777;
	insnkey <= 32'd0;
	LoadNOPs <= 1'b0;
	eptr <= 8'h00;
	dIRvalid <= 1'b1;
	xIRvalid <= 1'b1;
	m1IRvalid <= 1'b0;
	m2IRvalid <= 1'b0;
	wIRvalid <= 1'b0;
end
else begin

//---------------------------------------------------------
// Initialize program counters
// Initialize data tags to zero.
// Initialize execution pattern register to zero.
//---------------------------------------------------------
case(state)
RESET:
	begin
		pc <= `RESET_VECTOR;
		ipcv <= 1'b1;
		adr_o[14:6] <= adr_o[14:6]+9'd1;
		if (adr_o[14:6]==9'h1FF) begin
			dtinit <= 1'b0;
			resetA <= 1'b0;
			state <= RUN;
		end
		epat[a[7:0]] <= b[3:0];		/// b=0, to make this line the same as MTEP
		a[7:0] <= a[7:0] + 8'h1;
	end
RUN:
begin

tick <= tick + 64'd1;

prev_nmi <= nmi_i;
if (!prev_nmi & nmi_i)
	nmi_edge <= 1'b1;


`ifdef ADDRESS_RESERVATION
// A store by any device in the system to a reserved address blcok
// clears the reservation.

if (sys_adv && sys_adr[63:5]==resv_address)
	resv_address <= 59'd0;
`endif

wrhit <= 1'b0;

//---------------------------------------------------------
// IFETCH:
// - check for external hardware interrupt
// - fetch instruction
// - increment PC
// - set special register defaults for some instructions
// Outputs:
// - d???? signals
//---------------------------------------------------------
if (advanceI) begin
	dAXC <= AXC;
	dextype <= `EX_NON;
	// record instruction and associated pc value
	dIR <= insn;
	dIRvalid <= 1'b1;
	dpc <= pc;
	dpcv <= ipcv;
	if (nmi_edge & !StatusHWI) begin
		$display("*****************");
		$display("NMI edge detected");
		$display("*****************");
		StatusHWI <= 1'b1;
		nmi_edge <= 1'b0;
		dextype <= `EX_NMI;
		dIRvalid <= 1'b0;
		LoadNOPs <= 1'b1;
	end
	else if (irq_i & !im & !StatusHWI) begin
		$display("*****************");
		$display("IRQ detected");
		$display("*****************");
		bu_im <= 1'b0;
		im <= 1'b1;
		StatusHWI <= 1'b1;
		dextype <= `EX_IRQ;
		dIRvalid <= 1'b0;
		LoadNOPs <= 1'b1;
	end
	// Are we filling the pipeline with NOP's as a result of a previous
	// hardware interrupt ?
	else if (|dFip|LoadNOPs)
		dIRvalid <= 1'b0;
`ifdef TLB
	else if (ITLBMiss)
		dIRvalid <= 1'b0;
`endif
	else begin
`include "insn_dumpsc.v"
	end
`ifdef TLB
	if (ITLBMiss) begin
		$display("TLB miss on instruction fetch.");
		StatusEXL <= 1'b1;
		BadVAddr <= pc[63:13];
		dextype <= `EX_TLBI;
		EPC <= pc;
	end
	else
`endif
	begin
		dbranch_taken <= 1'b0;
		pc <= fnIncPC(pc);
		case(iOpcode)
		// We predict the return address by storing it in a return address stack
		// during a call instruction, then popping it off the stack in a return
		// instruction. The prediction will not always be correct, if it's wrong
		// it's corrected by the EX stage branching to the right address.
		`CALL:
			begin
				ras[ras_sp] <= fnIncPC(pc);
				ras_sp <= ras_sp - 6'd1;
				pc <= jmp_tgt;
				ipcv <= 1'b1;
			end
		`RET:
			begin
				pc <= ras[ras_sp + 6'd1];
				ras_sp <= ras_sp + 6'd1;
				ipcv <= 1'b1;
			end
		`JMP:
			begin
				pc <= jmp_tgt;
				ipcv <= 1'b1;
			end
		`BTRR:
			case(insn[4:0])
			`BEQ,`BNE,`BLT,`BLE,`BGT,`BGE,`BLTU,`BLEU,`BGTU,`BGEU,`BAND,`BOR,`BRA,`BNR,`BRN,`LOOP:
				if (predict_taken) begin
//					$display("Taking predicted branch: %h",{pc[63:4] + {{42{insn[24]}},insn[24:7]},insn[6:5],2'b00});
					dbranch_taken <= 1'b1;
					pc <= pc + {{52{insn[14]}},insn[14:5],2'b00};
					ipcv <= 1'b1;
				end
			default:	;
			endcase

		// If doing a JAL that stores a return address in the link register, save off the return address
		// in the return address predictor stack.
		`JAL:
			if (insn[19:15]==5'd31) begin
				ras[ras_sp] <= fnIncPC(pc);
				ras_sp <= ras_sp - 6'd1;
			end
`ifdef BTB
		`JAL:	pc <= btb[pc[7:2]];
		`BTRI:
			if (predict_taken) begin
				dbranch_taken <= 1'b1;
				pc <= btb[pc[7:2]];
				ipcv <= 1'b1;
			end
`endif
		`BEQI,`BNEI,`BLTI,`BLEI,`BGTI,`BGEI,`BLTUI,`BLEUI,`BGTUI,`BGEUI:
			begin
				if (predict_taken) begin
					dbranch_taken <= 1'b1;
					pc <= pc + {{50{insn[19]}},insn[19:8],2'b00};
					ipcv <= 1'b1;
				end
			end
		default:	;
		endcase
	end
end
// Stage tail
// Pipeline annul for when a bubble in the pipeline occurs.
else if (advanceR) begin
	dbranch_taken <= 1'b0;
	dextype <= `EX_NON;
	dIRvalid <= 1'b0;
	dpcv <= 1'b0;
end


//-----------------------------------------------------------------------------
// RFETCH:
// Register fetch stage
//
// Inputs:
// - d???? signals
// Outputs:
// - x???? signals to EX stage
//-----------------------------------------------------------------------------
//
if (advanceR) begin
	xAXC <= dAXC;
	xFip <= dFip;
	xextype <= dextype;
	xpc <= dpc;
	xpcv <= dpcv;
	xbranch_taken <= dbranch_taken;
	xRtZero <= 1'b0;
	xIRvalid <= dIRvalid;
	if (dOpcode==`R && dFunc==`MYST && dIRvalid)
		xIR <= nxt_c;
	else
		xIR <= dIR;
	a <= nxt_a;
	b <= nxt_b;
	if (dOpcode==`SHFTI)
		b <= {58'd0,dIR[14:9]};
	c <= nxt_c;

	case(dOpcode)
	`BTRI:	imm <= {{53{dIR[10]}},dIR[10:0]};
	`BEQI,`BNEI,`BLTI,`BLEI,`BGTI,`BGEI,`BLTUI,`BLEUI,`BGTUI,`BGEUI:
		imm <= {{56{dIR[7]}},dIR[7:0]};
	`RET:	imm <= {49'h00000000,dIR[14:3],3'b000};
	`MEMNDX:	imm <= dIR[7:6];
	default:	imm <= {{49{dIR[14]}},dIR[14:0]};
	endcase
	scale <= dIR[9:8];
//	if (dIsLSPair & ~dIR[15])
//		dIR <= dIR|32'h8000;
end
// Stage tail
// Pipeline annul for when a bubble in the pipeline occurs.
else if (advanceX) begin
	xRtZero <= #1 1'b1;
	xextype <= #1 `EX_NON;
	xbranch_taken <= #1 1'b0;
	xIRvalid <= #1 1'b0;
	xpcv <= #1 1'b0;
	xFip <= #1 1'b0;
end

//---------------------------------------------------------
// EXECUTE:
// - perform datapath operation
// - perform virtual to physical address translation.
// Outputs:
// - m1???? signals to M1 stage
//---------------------------------------------------------
if (advanceX) begin
	m1extype <= xextype;
	m1Fip <= xFip;
	m1Func <= xFunc;
	m1pcv <= xpcv;
	m1pc <= xpc;
	m1IR <= xIR;
	m1IsCnt <= xIsCnt;
	m1Opcode <= xOpcode;
	m1IsLoad <= xIsLoad & xIRvalid;
	m1IsStore <= xIsStore & xIRvalid;
	m1IsOut <= xIsOut & xIRvalid;
	m1IsIn <= xIsIn & xIRvalid;
	m1Rt <= xRtZero ? 9'd0 : xRt;
	m1Data <= xData;
	m1IsCacheElement <= xisCacheElement;
	m1IRvalid <= xIRvalid;
	m1AXC <= xAXC;
	if (xOpcode==`RR && xIRvalid) begin
		if (xFunc6==`MOVZ && !aeqz) begin
			m1Rt <= 9'd0;
			m1Data <= 64'd0;
		end
		if (xFunc6==`MOVNZ && aeqz) begin
			m1Rt <= 9'd0;
			m1Data <= 64'd0;
		end
		if (xFunc6==`MOVPL && a[63]) begin
			m1Rt <= 9'd0;
			m1Data <= 64'd0;
		end
		if (xFunc6==`MOVMI && !a[63]) begin
			m1Rt <= 9'd0;
			m1Data <= 64'd0;
		end
	end

	if (xIRvalid) begin
		case(xOpcode)
		`MISC:
			case(xFunc)
			`SEI:	im <= 1'b1;
			`CLI:	im <= 1'b0;
			`WAIT:	m1clkoff <= 1'b1;
			`ICACHE_ON:		ICacheOn <= 1'b1;
			`ICACHE_OFF:	ICacheOn <= 1'b0;
			`DCACHE_ON:		dcache_on <= 1'b1;
			`DCACHE_OFF:	dcache_on <= 1'b0;
			`FIP:	begin
					dIRvalid <= 1'b0;
					xIRvalid <= 1'b0;
					xRtZero <= 1'b1;
					xpcv <= 1'b0;
					dpcv <= 1'b0;
					ipcv <= 1'b0;
					dFip <= 1'b1;
					xFip <= 1'b1;
					m1Fip <= 1'b1;
					end
			`IEPP:	begin
					eptr <= eptr + 8'd1;
					dIRvalid <= 1'b0;
					xIRvalid <= 1'b0;
					xRtZero <= 1'b1;
					xpcv <= 1'b0;
					dpcv <= 1'b0;
					ipcv <= 1'b0;
					dFip <= 1'b1;
					xFip <= 1'b1;
					m1Fip <= 1'b1;
					end
			`GRAN:	begin
					rando <= rand;
					m_z <= next_m_z;
					m_w <= next_m_w;
					end
			`GRAFD:	begin
					rando <= randfd;
					m_z <= next_m_z;
					m_w <= next_m_w;
					end
			`IRET:
				if (StatusHWI) begin
					StatusHWI <= 1'b0;
					im <= 1'b0;
					pc <= IPC[xAXC];	//a;
					dIRvalid <= 1'b0;
					xIRvalid <= 1'b0;
					xRtZero <= 1'b1;
					xpcv <= 1'b0;
					dpcv <= 1'b0;
					ipcv <= 1'b1;
				end
			`ERET:
				if (StatusEXL[xAXC]) begin
					StatusEXL[xAXC]	<= 1'b0;
					pc <= EPC[xAXC];
					dIRvalid <= 1'b0;
					xIRvalid <= 1'b0;
					xRtZero <= 1'b1;
					xpcv <= 1'b0;
					dpcv <= 1'b0;
					ipcv <= 1'b1;
				end
			// Note: we can't mask off the interrupts in the I-stage because this
			// instruction might not be valid. Eg. a branch could occur causing
			// the instruction to not be executed. But we don't want to allow
			// nested interrupts. We would need a stack of return addresses to 
			// implement nested interrupts. We don't want a real IRQ that's following this
			// instruction in the pipeline to interfere with it's operation. So...
			// we check the pipeline and if if the IRQ SYSCALL is being followed by
			// a real IRQ, then we merge the two IRQ's into a single one by aborting
			// the IRQ SYSCALL. If nested interrupts were happening, the IRET at the
			// end of the real IRQ routine would re-enable interrupts too soon.
			`SYSCALL:
				begin
					if ((xIR[15:7]==`EX_NMI || xIR[15:7]==`EX_IRQ) && 	// Is this a IRQ SYSCALL ?
						((dextype==`EX_NMI || dextype==`EX_IRQ) ||		// Is there an interrupt in the pipeline ?
						(nmi_edge & !StatusHWI) || (irq_i & ~im & !StatusHWI))) begin	// OR about to happen
						m1Opcode <= `NOPI;								// Then turn this into a NOP
						m1IRvalid <= 1'b0;
					end
					else begin
						if (xIR[15:7]!=`EX_NMI && xIR[15:7]!=`EX_IRQ)
							StatusEXL[xAXC] <= 1'b1;
						else begin
							StatusHWI <= 1'b1;
							im <= 1'b1;
						end
						dIRvalid <= 1'b0;
						xIRvalid <= 1'b0;
						xRtZero <= 1'b1;
						xpcv <= 1'b0;
						dpcv <= 1'b0;
						ipcv <= 1'b0;
						ea <= {TBA[63:12],xIR[15:7],3'b000};
						LoadNOPs <= 1'b1;
						$display("EX SYSCALL thru %h",{TBA[63:12],xIR[15:7],3'b000});
					end
				end
`ifdef TLB
			`TLBP:	ea <= TLBVirtPage;
`endif
			default:	;
			endcase
		`R:
			case(xFunc6)
			`EXEC:
				begin
					pc <= fnIncPC(xpc);
					dIR <= b;
					xIRvalid <= 1'b0;
					xRtZero <= 1'b1;
					xpcv <= 1'b0;
					dpcv <= 1'b0;
					ipcv <= 1'b1;
				end
			`MTSPR:
				begin
				case(xIR[11:6])
`ifdef TLB
				`PageTableAddr:	PageTableAddr <= a[63:13];
				`BadVAddr:		BadVAddr <= a[63:13];
`endif
				`ASID:			ASID <= a[7:0];
				`TBA:			TBA <= {a[63:12],12'h000};
				`NON_ICACHE_SEG:	nonICacheSeg <= a[63:32];
				`FPCR:			rm <= a[31:30];
				`SRAND1:		begin
								m_z <= a;
								end
				`SRAND2:		begin
								m_w <= a;
								end
				`INSNKEY:		insnkey <= a[41:0];
				`PCHI:			pchi <= a[5:0];
				default:	;
				endcase
				end
			`OMG:	mutex_gate[a[5:0]] <= 1'b1;
			`CMG:	mutex_gate[a[5:0]] <= 1'b0;
			`OMGI:	mutex_gate[xIR[11:6]] <= 1'b1;
			`CMGI:	mutex_gate[xIR[11:6]] <= 1'b0;
			default:	;
			endcase
		`RR:
			case(xFunc6)
			`MTEP:	epat[a[7:0]] <= b[3:0];
			default:	;
			endcase
		// JMP and CALL change the program counter immediately in the IF stage.
		// There's no work to do here. The pipeline does not need to be cleared.
		`JMP:	;
		`CALL:	;//m1Data <= fnIncPC(xpc);
		
		`JAL:
`ifdef BTB
			if (dpc[63:2] != a[63:2] + imm[63:2]) begin
				pc[63:2] <= a[63:2] + imm[63:2];
				btb[xpc[7:2]] <= {a[63:2] + imm[63:2],2'b00};
				dIRvalid <= 1'b0;
				xIRvalid <= 1'b0;
				xRtzero <= 1'b1;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
				ipcv <= 1'b1;
			end
`else
			begin
				pc[63:2] <= a[63:2] + imm[63:2];
				dIRvalid <= 1'b0;
				xIRvalid <= 1'b0;
				xRtZero <= 1'b1;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
				ipcv <= 1'b1;
			end
`endif
		// Check the pc of the instruction after the RET instruction (the dpc), to
		// see if it's equal to the RET target. If it's the same as the target then
		// we predicted the RET return correctly, so there's nothing to do. Otherwise
		// we need to branch to the RET location.
		`RET:
			if (dpc[63:2]!=b[63:2]) begin
				pc[63:2] <= b[63:2];
				dIRvalid <= 1'b0;
				xIRvalid <= 1'b0;
				xRtZero <= 1'b1;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
				ipcv <= 1'b1;
			end
		`BTRR:
			case(xFunc5)
		// BEQ r1,r2,label
			`BEQ,`BNE,`BLT,`BLE,`BGT,`BGE,`BLTU,`BLEU,`BGTU,`BGEU,`BAND,`BOR,`BNR,`LOOP,`BRA,`BRN:
				if (!takb & xbranch_taken) begin
					$display("Taking mispredicted branch %h",fnIncPC(xpc));
					pc <= fnIncPC(xpc);
					dIRvalid <= 1'b0;
					xIRvalid <= 1'b0;
					xRtZero <= 1'b1;
					xpcv <= 1'b0;
					dpcv <= 1'b0;
					ipcv <= 1'b1;
				end
				else if (takb & !xbranch_taken) begin
					$display("Taking branch %h",{xpc[63:2] + {{52{xIR[14]}},xIR[14:5]},2'b00});
					pc[63:2] <= xpc[63:2] + {{52{xIR[14]}},xIR[14:5]};
					dIRvalid <= 1'b0;
					xIRvalid <= 1'b0;
					xRtZero <= 1'b1;
					xpcv <= 1'b0;
					dpcv <= 1'b0;
					ipcv <= 1'b1;
				end
		// BEQ r1,r2,r10
			`BEQR,`BNER,`BLTR,`BLER,`BGTR,`BGER,`BLTUR,`BLEUR,`BGTUR,`BGEUR://,`BANDR,`BORR,`BNRR:
				if (takb) begin
					pc[63:2] <= c[63:2];
					pc[1:0] <= 2'b00;
`ifdef BTB
					btb[xpc[7:2]] <= c;
`endif
					dIRvalid <= 1'b0;
					xIRvalid <= 1'b0;
					xRtZero <= 1'b1;
					xpcv <= 1'b0;
					dpcv <= 1'b0;
					ipcv <= 1'b1;
				end
			default:	;
			endcase
		// BEQ r1,#3,r10
		`BTRI:
`ifdef BTB
			if (takb) begin
				if ((xbranch_taken && b[63:2]!=dpc[63:2]) ||	// took branch, but not to right target
					!xbranch_taken) begin					// didn't take branch, and were supposed to
					pc[63:2] <= b[63:2];
					pc[1:0] <= 2'b00;
					btb[xpc[7:2]] <= b;
					dIRvalid <= 1'b0;
					xIRvalid <= 1'b0;
					xRtzero <= 1'b1;
					xpcv <= 1'b0;
					dpcv <= 1'b0;
					ipcv <= 1'b1;
				end
			end
			else if (xbranch_taken)	begin	// took the branch, and weren't supposed to
				pc <= fnIncPC(xpc);
				dIRvalid <= 1'b0;
				xIRvalid <= 1'b0;
				xRtzero <= 1'b1;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
				ipcv <= 1'b1;
			end
`else
			if (takb) begin
				pc[63:2] <= b[63:2];
				pc[1:0] <= 2'b00;
				dIRvalid <= 1'b0;
				xIRvalid <= 1'b0;
				xRtZero <= 1'b1;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
				ipcv <= 1'b1;
			end
`endif
		// BEQI r1,#3,label
		`BEQI,`BNEI,`BLTI,`BLEI,`BGTI,`BGEI,`BLTUI,`BLEUI,`BGTUI,`BGEUI:
			if (takb) begin
				if (!xbranch_taken) begin
					pc[63:2] <= xpc[63:2] + {{50{xIR[19]}},xIR[19:8]};
					dIRvalid <= 1'b0;
					xIRvalid <= 1'b0;
					xRtZero <= 1'b1;
					xpcv <= 1'b0;
					dpcv <= 1'b0;
					ipcv <= 1'b1;
				end
			end
			else begin
				if (xbranch_taken) begin
					$display("Taking mispredicted branch %h",fnIncPC(xpc));
					pc <= fnIncPC(xpc);
					dIRvalid <= 1'b0;
					xIRvalid <= 1'b0;
					xRtZero <= 1'b1;
					xpcv <= 1'b0;
					dpcv <= 1'b0;
					ipcv <= 1'b1;
				end
			end
		`TRAPcc,`TRAPcci:
			if (takb) begin
				StatusEXL[xAXC] <= 1'b1;
				xextype <= `EX_TRAP;
				dIRvalid <= 1'b0;
				xIRvalid <= 1'b0;
				xRtZero <= 1'b1;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
				ipcv <= 1'b0;
				LoadNOPs <= 1'b1;
			end

		`INW:
				begin
				iocyc_o <= 1'b1;
				stb_o <= 1'b1;
				sel_o <= 8'hFF;
				adr_o <= xData1;
				end
		`INH,`INHU:
				begin
				iocyc_o <= 1'b1;
				stb_o <= 1'b1;
				sel_o <= xData1[2] ? 8'b11110000 : 8'b00001111;
				adr_o <= xData1;
				end
		`INCH,`INCU:
				begin
				iocyc_o <= 1'b1;
				stb_o <= 1'b1;
				case(xData1[2:1])
				2'b00:	sel_o <= 8'b00000011;
				2'b01:	sel_o <= 8'b00001100;
				2'b10:	sel_o <= 8'b00110000;
				2'b11:	sel_o <= 8'b11000000;
				endcase
				adr_o <= xData1;
				end
		`INB,`INBU:
				begin
				iocyc_o <= 1'b1;
				stb_o <= 1'b1;
				case(xData1[2:0])
				3'b000:	sel_o <= 8'b00000001;
				3'b001:	sel_o <= 8'b00000010;
				3'b010:	sel_o <= 8'b00000100;
				3'b011:	sel_o <= 8'b00001000;
				3'b100:	sel_o <= 8'b00010000;
				3'b101:	sel_o <= 8'b00100000;
				3'b110:	sel_o <= 8'b01000000;
				3'b111:	sel_o <= 8'b10000000;
				endcase
				adr_o <= xData1;
				end
		`OUTW:
				begin
				iocyc_o <= 1'b1;
				stb_o <= 1'b1;
				we_o <= 1'b1;
				sel_o <= 8'hFF;
				adr_o <= xData1;
				dat_o <= b;
				end
		`OUTH:
				begin
				iocyc_o <= 1'b1;
				stb_o <= 1'b1;
				we_o <= 1'b1;
				sel_o <= xData1[2] ? 8'b11110000 : 8'b00001111;
				adr_o <= xData1;
				dat_o <= {2{b[31:0]}};
				end
		`OUTC:
				begin
				iocyc_o <= 1'b1;
				stb_o <= 1'b1;
				we_o <= 1'b1;
				case(xData1[2:1])
				2'b00:	sel_o <= 8'b00000011;
				2'b01:	sel_o <= 8'b00001100;
				2'b10:	sel_o <= 8'b00110000;
				2'b11:	sel_o <= 8'b11000000;
				endcase
				adr_o <= xData1;
				dat_o <= {4{b[15:0]}};
				end
		`OUTB:
				begin
				iocyc_o <= 1'b1;
				stb_o <= 1'b1;
				we_o <= 1'b1;
				case(xData1[2:0])
				3'b000:	sel_o <= 8'b00000001;
				3'b001:	sel_o <= 8'b00000010;
				3'b010:	sel_o <= 8'b00000100;
				3'b011:	sel_o <= 8'b00001000;
				3'b100:	sel_o <= 8'b00010000;
				3'b101:	sel_o <= 8'b00100000;
				3'b110:	sel_o <= 8'b01000000;
				3'b111:	sel_o <= 8'b10000000;
				endcase
				adr_o <= xData1;
				dat_o <= {8{b[7:0]}};
				end
	//	`OUTBC:
	//			begin
	//			iocyc_o <= 1'b1;
	//			stb_o <= 1'b1;
	//			we_o <= 1'b1;
	//			case(xData1[2:0])
	//			3'b000:	sel_o <= 8'b00000001;
	//			3'b001:	sel_o <= 8'b00000010;
	//			3'b010:	sel_o <= 8'b00000100;
	//			3'b011:	sel_o <= 8'b00001000;
	//			3'b100:	sel_o <= 8'b00010000;
	//			3'b101:	sel_o <= 8'b00100000;
	//			3'b110:	sel_o <= 8'b01000000;
	//			3'b111:	sel_o <= 8'b10000000;
	//			endcase
	//			adr_o <= xData1;
	//			dat_o <= {8{xIR[19:12]}};
	//			end
		`LEA:	begin
				$display("LEA %h", xData1);
				m1Data <= xData1;
				end
		`LB,`LBU,`LC,`LCU,`LH,`LHU,`LW,`LWR,`LF,`LFD,`LM,`LSH,`LSW,`LP,`LFP,`LFDP,
		`SW,`SH,`SC,`SB,`SWC,`SF,`SFD,`SM,`SSW,`SP,`SFP,`SFDP:
				begin
				m1Data <= b;
				ea <= xData1;
				$display("EX MEMOP %h", xData1);
				end
	//	`STBC:
	//			begin
	//			m1Data <= {8{xIR[19:12]}};
	//			ea <= xData1;
	//			end
	//	`SSH:	begin
	//			case(xRt)
	//			`SR:	m1Data <= {2{sr}};
	//			default:	m1Data <= 64'd0;
	//			endcase
	//			ea <= xData1;
	//			end
		`CACHE:
				begin
				m1Data <= b;
				ea <= xData1;
				case(xIR[19:15])
				`INVIL:		;		// handled in M1 stage
				`INVIALL:	tvalid <= 128'd0;
				`ICACHEON:	ICacheOn <= 1'b1;
				`ICACHEOFF:	ICacheOn <= 1'b0;
				`DCACHEON:	dcache_on <= 1'b1;
				`DCACHEOFF:	dcache_on <= 1'b0;
				default:	;
				endcase
				end
		`MEMNDX:
				begin
				m1Opcode <= 7'd32+xFunc6;
				case(xFunc6)
				`LEAX:
					begin
					$display("LEAX %h", xData1);
					m1Data <= xData1;
					end
				`INWX:
						begin
						iocyc_o <= 1'b1;
						stb_o <= 1'b1;
						sel_o <= 8'hFF;
						adr_o <= xData1;
						end
				`INHX,`INHUX:
						begin
						iocyc_o <= 1'b1;
						stb_o <= 1'b1;
						sel_o <= xData1[2] ? 8'b11110000 : 8'b00001111;
						adr_o <= xData1;
						end
				`INCX,`INCUX:
						begin
						iocyc_o <= 1'b1;
						stb_o <= 1'b1;
						case(xData1[2:1])
						2'b00:	sel_o <= 8'b00000011;
						2'b01:	sel_o <= 8'b00001100;
						2'b10:	sel_o <= 8'b00110000;
						2'b11:	sel_o <= 8'b11000000;
						endcase
						adr_o <= xData1;
						end
				`INBX,`INBUX:
						begin
						iocyc_o <= 1'b1;
						stb_o <= 1'b1;
						case(xData1[2:0])
						3'b000:	sel_o <= 8'b00000001;
						3'b001:	sel_o <= 8'b00000010;
						3'b010:	sel_o <= 8'b00000100;
						3'b011:	sel_o <= 8'b00001000;
						3'b100:	sel_o <= 8'b00010000;
						3'b101:	sel_o <= 8'b00100000;
						3'b110:	sel_o <= 8'b01000000;
						3'b111:	sel_o <= 8'b10000000;
						endcase
						adr_o <= xData1;
						end
				`OUTWX:
						begin
						iocyc_o <= 1'b1;
						stb_o <= 1'b1;
						we_o <= 1'b1;
						sel_o <= 8'hFF;
						adr_o <= xData1;
						dat_o <= c;
						end
				`OUTHX:
						begin
						iocyc_o <= 1'b1;
						stb_o <= 1'b1;
						we_o <= 1'b1;
						sel_o <= xData1[2] ? 8'b11110000 : 8'b00001111;
						adr_o <= xData1;
						dat_o <= {2{c[31:0]}};
						end
				`OUTCX:
						begin
						iocyc_o <= 1'b1;
						stb_o <= 1'b1;
						we_o <= 1'b1;
						case(xData1[2:1])
						2'b00:	sel_o <= 8'b00000011;
						2'b01:	sel_o <= 8'b00001100;
						2'b10:	sel_o <= 8'b00110000;
						2'b11:	sel_o <= 8'b11000000;
						endcase
						adr_o <= xData1;
						dat_o <= {4{c[15:0]}};
						end
				`OUTBX:
						begin
						iocyc_o <= 1'b1;
						stb_o <= 1'b1;
						we_o <= 1'b1;
						case(xData1[2:0])
						3'b000:	sel_o <= 8'b00000001;
						3'b001:	sel_o <= 8'b00000010;
						3'b010:	sel_o <= 8'b00000100;
						3'b011:	sel_o <= 8'b00001000;
						3'b100:	sel_o <= 8'b00010000;
						3'b101:	sel_o <= 8'b00100000;
						3'b110:	sel_o <= 8'b01000000;
						3'b111:	sel_o <= 8'b10000000;
						endcase
						adr_o <= xData1;
						dat_o <= {8{c[7:0]}};
						end
				default:
					begin
					m1Data <= c;
					ea <= xData1;
					end
				endcase
				end
		default:	;
		endcase
	end
`ifdef FLOATING_POINT
	if (xOpcode==`FP && xIRvalid) begin
		case (xFunc6)
		`FDADD,`FDSUB:	
				begin
				fp_uf <= fpaddsub_uf;
				fp_ovr <= fpaddsub_ovr;
				fp_iop <= fpaddsub_iop;
				FPC_SL <= xData[63] && xData[62:0]!=63'd0;
				FPC_SG <= !xData[63] && xData[62:0]!=63'd0;
				FPC_SE <= xData[62:0]==63'd0;
				end
		`FPMUL:
				begin
				fp_uf <= fpmul_uf;
				fp_ovr <= fpmul_ovr;
				fp_iop <= fpmul_iop;
				FPC_SL <= xData[63] && xData[62:0]!=63'd0;
				FPC_SG <= !xData[63] && xData[62:0]!=63'd0;
				FPC_SE <= xData[62:0]==63'd0;
				end
		`FPDIV:
				begin
				fp_uf <= fpdiv_uf;
				fp_ovr <= fpdiv_ovr;
				fp_iop <= fpdiv_iop;
				FPC_SL <= xData[63] && xData[62:0]!=63'd0;
				FPC_SG <= !xData[63] && xData[62:0]!=63'd0;
				FPC_SE <= xData[62:0]==63'd0;
				end
		`FDF2I:
				begin
				fp_ovr <= f2i_ovr;
				fp_iop <= f2i_iop;
				end
		`FDCLT,`FDCLE,`FDCEQ,`FDCNE,`FDCGT,`FDCGE,`FDCUN:
				begin
				fp_iop <= fpcmp_iop;
				end
		default:	;
		endcase
	end
`endif
	if (dbz_error) begin
		$display("Divide by zero error");
		m1extype <= #1 `EX_DBZ;
		LoadNOPs <= #1 1'b1;
		xRtZero <= #1 1'b1;
		xpcv <= #1 1'b0;
		dpcv <= #1 1'b0;
		ipcv <= 1'b0;
	end
	else if (ovr_error) begin
		$display("Overflow error");
		m1extype <= `EX_OFL;
		LoadNOPs <= 1'b1;
		xRtZero <= 1'b1;
		xpcv <= 1'b0;
		dpcv <= 1'b0;
		ipcv <= 1'b0;
	end
//	else if (priv_violation) begin
//		$display("Priviledge violation");
//		m1extype <= `EX_PRIV;
//		LoadNOPs <= 1'b1;
//		xRtZero <= 1'b1;
//		xpcv <= 1'b0;
//		dpcv <= 1'b0;
//		ipcv <= 1'b0;
//	end
	else if (illegal_insn) begin
		$display("Unimplemented Instruction");
		m1extype <= `EX_UNIMP_INSN;
		LoadNOPs <= 1'b1;
		xRtZero <= 1'b1;
		xpcv <= 1'b0;
		dpcv <= 1'b0;
		ipcv <= 1'b0;
	end
end
// Stage tail
// Pipeline annul for when a bubble in the pipeline occurs.
else if (advanceM1) begin
	m1IRvalid <= #1 1'b0;
	m1IsLoad <= #1 1'b0;
	m1IsStore <= #1 1'b0;
	m1IsOut <= #1 1'b0;
	m1IsIn <= #1 1'b0;
	m1Rt <= #1 9'd0;
	m1clkoff <= #1 1'b0;
	m1Fip <= #1 1'b0;
	m1pcv <= #1 1'b0;
	m1extype <= #1 `EX_NON;
	m1IsCnt <= #1 1'b0;
	m1IsCacheElement <= #1 1'b0;
end


//-----------------------------------------------------------------------------
// MEMORY:
// - I/O instructions are finished
// - store instructions are started
// - missed loads are started
// On a data cache hit for a load, the load is essentially
// finished in this stage. We switch the opcode to 'NOPI'
// to cause the pipeline to advance as if a NOPs were
// present.
//
// Inputs:
// - m1???? signals
// Outputs:
// - m2???? signals to M2 stage
//-----------------------------------------------------------------------------
if (advanceM1) begin
	m2extype <= m1extype;
	m2Addr <= pea;
	m2Data <= m1Data;
	m2Fip <= m1Fip;
	m2pc <= m1pc;
	m2pcv <= m1pcv;
	m2IR <= m1IR;
	m2Opcode <= m1Opcode;
	m2Func <= m1Func;
	m2IsCnt <= m1IsCnt;
	m2Func <= m1Func;
	m2Rt <= m1Rt;
	m2clkoff <= m1clkoff;
	m2AXC <= m1AXC;
	m2IsCacheElement <= m1IsCacheElement;
	m2IRvalid <= m1IRvalid;
	m2IsLoad <= m1IsLoad & m1IRvalid;
	m2IsStore <= m2IsStore & m1IRvalid;

	if (m1IsIO & err_i & m1IRvalid) begin
		m2extype <= `EX_DBERR;
		errorAddress <= adr_o;
	end

	if (m1extype == `EX_NON && m1IRvalid) begin
		case(m1Opcode)
		`MISC:
			case(m1Func)
			`SYSCALL:
				if (!m1IsCacheElement) begin
					cyc_o <= 1'b1;
					stb_o <= 1'b1;
					sel_o <= 8'hFF;
					adr_o <= pea;
					m2Addr <= pea;
				end
				else begin	// dhit must be true
					$display("fetched vector: %h", {cdat[63:2],2'b00});
					m2Opcode <= `NOPI;
					m2IsLoad <= 1'b0;
					pc <= {cdat[63:2],2'b00};
					ipcv <= 1'b1;
					LoadNOPs <= 1'b0;
				end
			endcase
		`INW:
			begin
				iocyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				m2Data <= dat_i;
			end
		`INH:
			begin
				iocyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				m2Data <= sel_o[7] ? {{32{dat_i[63]}},dat_i[63:32]}:{{32{dat_i[31]}},dat_i[31: 0]};
			end
		`INHU:
			begin
				iocyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				m2Data <= sel_o[7] ? dat_i[63:32] : dat_i[31: 0];
			end
		`INCH:
			begin
				iocyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				case(sel_o)
				8'b00000011:	m2Data <= {{48{dat_i[15]}},dat_i[15: 0]};
				8'b00001100:	m2Data <= {{48{dat_i[31]}},dat_i[31:16]};
				8'b00110000:	m2Data <= {{48{dat_i[47]}},dat_i[47:32]};
				8'b11000000:	m2Data <= {{48{dat_i[63]}},dat_i[63:48]};
				default:	m2Data <= 64'hDEADDEADDEADDEAD;			
				endcase
			end
		`INCU:
			begin
				iocyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				case(sel_o)
				8'b00000011:	m2Data <= dat_i[15: 0];
				8'b00001100:	m2Data <= dat_i[31:16];
				8'b00110000:	m2Data <= dat_i[47:32];
				8'b11000000:	m2Data <= dat_i[63:48];
				default:	m2Data <= 64'hDEADDEADDEADDEAD;			
				endcase
			end
		`INB:
			begin
				iocyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				case(sel_o)
				8'b00000001:	m2Data <= {{56{dat_i[ 7]}},dat_i[ 7: 0]};
				8'b00000010:	m2Data <= {{56{dat_i[15]}},dat_i[15: 8]};
				8'b00000100:	m2Data <= {{56{dat_i[23]}},dat_i[23:16]};
				8'b00001000:	m2Data <= {{56{dat_i[31]}},dat_i[31:24]};
				8'b00010000:	m2Data <= {{56{dat_i[39]}},dat_i[39:32]};
				8'b00100000:	m2Data <= {{56{dat_i[47]}},dat_i[47:40]};
				8'b01000000:	m2Data <= {{56{dat_i[55]}},dat_i[55:48]};
				8'b10000000:	m2Data <= {{56{dat_i[63]}},dat_i[63:56]};
				default:	m2Data <= 64'hDEADDEADDEADDEAD;
				endcase
			end
		`INBU:
			begin
				iocyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				case(sel_o)
				8'b00000001:	m2Data <= dat_i[ 7: 0];
				8'b00000010:	m2Data <= dat_i[15: 8];
				8'b00000100:	m2Data <= dat_i[23:16];
				8'b00001000:	m2Data <= dat_i[31:24];
				8'b00010000:	m2Data <= dat_i[39:32];
				8'b00100000:	m2Data <= dat_i[47:40];
				8'b01000000:	m2Data <= dat_i[55:48];
				8'b10000000:	m2Data <= dat_i[63:56];
				default:	m2Data <= 64'hDEADDEADDEADDEAD;
				endcase
			end
		`OUTW,`OUTH,`OUTC,`OUTB,`OUTBC:
			begin
				iocyc_o <= #1 1'b0;
				stb_o <= #1 1'b0;
				we_o <= #1 1'b0;
				sel_o <= #1 8'h00;
			end
		`CACHE:
			case(m1IR[19:15])
			`INVIL:	tvalid[pea[12:6]] <= 1'b0;
			default:	;
			endcase
			

		`LW,`LM,`LFD,`LSW,`LP,`LFDP:
			if (!m1IsCacheElement) begin
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				sel_o <= 8'hFF;
				adr_o <= pea;
				m2Addr <= pea;
			end
			else begin
				m2IsLoad <= 1'b0;
				m2Opcode <= `NOPI;
				m2Data <= cdat;
			end
`ifdef ADDRESS_RESERVATION
		`LWR:
			if (!m1IsCacheElement) begin
				rsv_o <= 1'b1;
				resv_address <= pea[63:5];
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				sel_o <= 8'hFF;
				adr_o <= pea;
				m2Addr <= pea;
			end
			else begin
				m2IsLoad <= 1'b0;
				m2Opcode <= `NOPI;
				m2Data <= cdat;
				rsv_o <= 1'b1;	// ???
				resv_address <= pea[63:5];
			end
`endif
		`LH,`LF,`LFP:
			if (!m1IsCacheElement) begin
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				sel_o <= pea[2] ? 8'b11110000 : 8'b00001111;
				adr_o <= pea;
				m2Addr <= pea;
			end
			else begin
				m2IsLoad <= 1'b0;
				m2Opcode <= `NOPI;
				if (pea[2])
					m2Data <= {{32{cdat[31]}},cdat[31:0]};
				else
					m2Data <= {{32{cdat[63]}},cdat[63:32]};
			end

		`LHU,`LSH:
			if (!m1IsCacheElement) begin
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				sel_o <= pea[2] ? 8'b11110000 : 8'b00001111;
				adr_o <= pea;
				m2Addr <= pea;
			end
			else begin
				m2IsLoad <= 1'b0;
				m2Opcode <= `NOPI;
				if (pea[2])
					m2Data <= {32'd0,cdat};
				else
					m2Data <= {32'd0,cdat[63:32]};
			end

		`LC:
			if (!m1IsCacheElement) begin
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				case(pea[2:1])
				2'b00:	sel_o <= 8'b00000011;
				2'b01:	sel_o <= 8'b00001100;
				2'b10:	sel_o <= 8'b00110000;
				2'b11:	sel_o <= 8'b11000000;
				endcase
				adr_o <= pea;
				m2Addr <= pea;
			end
			else begin
				$display("dhit=1, cdat=%h",cdat);
				m2IsLoad <= 1'b0;
				m2Opcode <= `NOPI;
				case(pea[2:1])
				2'd0:	m2Data <= {{48{cdat[15]}},cdat[15:0]};
				2'd1:	m2Data <= {{48{cdat[31]}},cdat[31:16]};
				2'd2:	m2Data <= {{48{cdat[47]}},cdat[47:32]};
				2'd3:	m2Data <= {{48{cdat[63]}},cdat[63:48]};
				endcase
			end

		`LCU:
			if (!m1IsCacheElement) begin
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				case(pea[2:1])
				2'b00:	sel_o <= 8'b00000011;
				2'b01:	sel_o <= 8'b00001100;
				2'b10:	sel_o <= 8'b00110000;
				2'b11:	sel_o <= 8'b11000000;
				endcase
				adr_o <= pea;
				m2Addr <= pea;
			end
			else begin
				m2IsLoad <= 1'b0;
				m2Opcode <= `NOPI;
				case(pea[2:1])
				2'd0:	m2Data <= {48'd0,cdat[15: 0]};
				2'd1:	m2Data <= {48'd0,cdat[31:16]};
				2'd2:	m2Data <= {48'd0,cdat[47:32]};
				2'd3:	m2Data <= {48'd0,cdat[63:48]};
				endcase
			end

		`LB:
			if (!m1IsCacheElement) begin
				$display("Load byte:");
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				case(pea[2:0])
				3'b000:	sel_o <= 8'b00000001;
				3'b001:	sel_o <= 8'b00000010;
				3'b010:	sel_o <= 8'b00000100;
				3'b011:	sel_o <= 8'b00001000;
				3'b100:	sel_o <= 8'b00010000;
				3'b101:	sel_o <= 8'b00100000;
				3'b110:	sel_o <= 8'b01000000;
				3'b111:	sel_o <= 8'b10000000;
				endcase
				adr_o <= pea;
				m2Addr <= pea;
			end
			else begin
				m2IsLoad <= 1'b0;
				m2Opcode <= `NOPI;
				case(pea[2:0])
				3'b000:	m2Data <= {{56{cdat[ 7]}},cdat[ 7: 0]};
				3'b001:	m2Data <= {{56{cdat[15]}},cdat[15: 8]};
				3'b010:	m2Data <= {{56{cdat[23]}},cdat[23:16]};
				3'b011:	m2Data <= {{56{cdat[31]}},cdat[31:24]};
				3'b100:	m2Data <= {{56{cdat[39]}},cdat[39:32]};
				3'b101:	m2Data <= {{56{cdat[47]}},cdat[47:40]};
				3'b110:	m2Data <= {{56{cdat[55]}},cdat[55:48]};
				3'b111:	m2Data <= {{56{cdat[63]}},cdat[63:56]};
				endcase
			end

		`LBU:
			if (!m1IsCacheElement) begin
				$display("Load unsigned byte:");
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				case(pea[2:0])
				3'b000:	sel_o <= 8'b00000001;
				3'b001:	sel_o <= 8'b00000010;
				3'b010:	sel_o <= 8'b00000100;
				3'b011:	sel_o <= 8'b00001000;
				3'b100:	sel_o <= 8'b00010000;
				3'b101:	sel_o <= 8'b00100000;
				3'b110:	sel_o <= 8'b01000000;
				3'b111:	sel_o <= 8'b10000000;
				endcase
				adr_o <= pea;
				m2Addr <= pea;
			end
			else begin
				$display("m2IsLoad <= 0");
				m2IsLoad <= 1'b0;
				m2Opcode <= `NOPI;
				case(pea[2:0])
				3'b000:	m2Data <= {56'd0,cdat[ 7: 0]};
				3'b001:	m2Data <= {56'd0,cdat[15: 8]};
				3'b010:	m2Data <= {56'd0,cdat[23:16]};
				3'b011:	m2Data <= {56'd0,cdat[31:24]};
				3'b100:	m2Data <= {56'd0,cdat[39:32]};
				3'b101:	m2Data <= {56'd0,cdat[47:40]};
				3'b110:	m2Data <= {56'd0,cdat[55:48]};
				3'b111:	m2Data <= {56'd0,cdat[63:56]};
				endcase
			end

		`SW,`SM,`SFD,`SSW,`SP,`SFDP:
			begin
				$display("%d SW/SM %h",tick,{pea[63:3],3'b000});
				m2Addr <= pea;
				wrhit <= #1 dhit;
`ifdef ADDRESS_RESERVATION
				if (resv_address==pea[63:5])
					resv_address <= #1 59'd0;
`endif
				cyc_o <= #1 1'b1;
				stb_o <= #1 1'b1;
				we_o <= #1 1'b1;
				sel_o <= #1 8'hFF;
				adr_o <= pea;
				dat_o <= #1 m1Data;
			end

		`SH,`SF,`SSH,`SFP:
			begin
				wrhit <= #1 dhit;
				m2Addr <= pea;
`ifdef ADDRESS_RESERVATION
				if (resv_address==pea[63:5])
					resv_address <= #1 59'd0;
`endif
				cyc_o <= #1 1'b1;
				stb_o <= #1 1'b1;
				we_o <= #1 1'b1;
				sel_o <= #1 pea[2] ? 8'b11110000 : 8'b00001111;
				adr_o <= pea;
				dat_o <= #1 {2{m1Data[31:0]}};
			end

		`SC:
			begin
				$display("Storing char to %h, ea=%h",pea,ea);
				wrhit <= #1 dhit;
				m2Addr <= pea;
`ifdef ADDRESS_RESERVATION
				if (resv_address==pea[63:5])
					resv_address <= #1 59'd0;
`endif
				cyc_o <= #1 1'b1;
				stb_o <= #1 1'b1;
				we_o <= #1 1'b1;
				case(pea[2:1])
				2'b00:	sel_o <= #1 8'b00000011;
				2'b01:	sel_o <= #1 8'b00001100;
				2'b10:	sel_o <= #1 8'b00110000;
				2'b11:	sel_o <= #1 8'b11000000;
				endcase
				adr_o <= pea;
				dat_o <= #1 {4{m1Data[15:0]}};
			end

		`SB,`STBC:
			begin
				wrhit <= #1 dhit;
				m2Addr <= pea;
`ifdef ADDRESS_RESERVATION
				if (resv_address==pea[63:5])
					resv_address <= #1 59'd0;
`endif
				cyc_o <= #1 1'b1;
				stb_o <= #1 1'b1;
				we_o <= #1 1'b1;
				case(pea[2:0])
				3'b000:	sel_o <= #1 8'b00000001;
				3'b001:	sel_o <= #1 8'b00000010;
				3'b010:	sel_o <= #1 8'b00000100;
				3'b011:	sel_o <= #1 8'b00001000;
				3'b100:	sel_o <= #1 8'b00010000;
				3'b101:	sel_o <= #1 8'b00100000;
				3'b110:	sel_o <= #1 8'b01000000;
				3'b111:	sel_o <= #1 8'b10000000;
				endcase
				adr_o <= pea;
				dat_o <= #1 {8{m1Data[7:0]}};
			end

`ifdef ADDRESS_RESERVATION
		`SWC:
			begin
				rsf <= #1 1'b0;
				if (resv_address==pea[63:5]) begin
					wrhit <= #1 dhit;
					m2Addr <= pea;
					cyc_o <= #1 1'b1;
					stb_o <= #1 1'b1;
					we_o <= #1 1'b1;
					sel_o <= #1 8'hFF;
					adr_o <= pea;
					dat_o <= #1 m1Data;
					resv_address <= #1 59'd0;
					rsf <= #1 1'b1;
				end
				else
					m2Opcode <= #1 `NOPI;
			end
`endif
		endcase
	end
	//---------------------------------------------------------
	// Check for a TLB miss.
	// On a prefetch load, just switch the opcode to a NOP
	// instruction and ignore the error. Otherwise set the
	// exception type.
	//---------------------------------------------------------
`ifdef TLB
	if (m1IsLoad && m1Rt[4:0]==5'd0 && DTLBMiss) begin
		m1Opcode <= `NOPI;
	end
	if ((m1IsLoad&&m1Rt[4:0]!=5'd0)|m1IsStore) begin
		if (DTLBMiss) begin
			$display("DTLB miss on address: %h",ea);
			m1extype <= `EX_TLBD;
			StatusEXL <= 1'b1;
			BadVAddr <= ea[63:13];
			m1Opcode <= `NOPI;
			m1Rt <= 9'd0;
			pc <= 64'hC;
			xIR <= `NOP_INSN;
			xRtzero <= 1'b1;
			dIR <= `NOP_INSN;
			m1pcv <= 1'b0;
			xpcv <= 1'b0;
			dpcv <= 1'b0;
			dhwxtype <= 2'b11;
		end
	end
`endif
end
// Stage tail
// Pipeline annul for when a bubble in the pipeline occurs.
else if (advanceM2) begin
	m2Rt <= #1 9'd0;
	m2IRvalid <= 1'b0;
	m2IsCnt <= #1 1'b0;
	m2IsLoad <= #1 1'b0;
	m2IsStore <= #1 1'b0;
	m2Func <= #1 7'd0;
	m2Addr <= 64'd0;
	m2Data <= #1 64'd0;
	m2clkoff <= #1 1'b0;
	m2Fip <= #1 1'b0;
	m2pcv <= #1 1'b0;
	m2extype <= #1 `EX_NON;
	m2IsCacheElement <= 1'b0;
end


//-----------------------------------------------------------------------------
// MEMORY:
// - complete the memory cycle
// - merge load data into pipeline
// Inputs:
// - m2???? type signals
// Outputs:
// - w???? signals to WB stage
//-----------------------------------------------------------------------------
if (advanceM2) begin
	wextype <= #1 m2extype;
	wpc <= #1 m2pc;
	wpcv <= #1 m2pcv;
	wFip <= #1 m2Fip;
	wIR <= #1 m2IR;
	wIsStore <= #1 m2IsStore & m2IRvalid;
	wOpcode <= #1 m2Opcode;
	wFunc <= #1 m2Func;
	wData <= #1 m2Data;
	wRt <= #1 m2Rt;
	wclkoff <= #1 m2clkoff;
	wAXC <= #1 m2AXC;
	wIRvalid <= m2IRvalid;
	
	// There's not an error is a prefetch is taking place (m2Rt=0).
	if (((m2IsLoad&&m2Rt[4:0]!=5'd0)|m2IsStore) & err_i & m2IRvalid) begin
		wextype <= #1 `EX_DBERR;
		errorAddress <= #1 adr_o;
	end
	
	if (m2extype==`EX_NON & m2IRvalid) begin
		case(m2Opcode)
		`MISC:
			if (m2Func==`SYSCALL)
				begin
					cyc_o <= #1 1'b0;
					stb_o <= #1 1'b0;
					sel_o <= #1 8'h00;
					pc <= #1 {dat_i[63:2],2'b00};
					ipcv <= 1'b1;
					LoadNOPs <= 1'b0;
					$display("M2 Fetched vector: %h",{dat_i[63:2],2'b00});
				end
		`SH,`SC,`SB,`SW,`SWC,`SF,`SFD,`SSH,`SSW,`SP,`SFP,`SFDP:
			begin
				cyc_o <= #1 1'b0;
				stb_o <= #1 1'b0;
				we_o <= #1 1'b0;
				sel_o <= #1 8'h00;
			end
		`LH,`LF,`LSH,`LFP:
			begin
				cyc_o <= #1 1'b0;
				stb_o <= #1 1'b0;
				sel_o <= #1 8'h00;
				wData <= #1 sel_o[7] ? {{32{dat_i[63]}},dat_i[63:32]}:{{32{dat_i[31]}},dat_i[31: 0]};
			end
		`LW,`LWR,`LFD,`LSW,`LP,`LFDP:
			begin
				cyc_o <= #1 1'b0;
				stb_o <= #1 1'b0;
				sel_o <= #1 8'h00;
				wData <= #1 dat_i;
			end
		`LHU:
			begin
				cyc_o <= #1 1'b0;
				stb_o <= #1 1'b0;
				sel_o <= #1 8'h00;
				wData <= #1 sel_o[7] ? dat_i[63:32] : dat_i[31: 0];
			end
		`LC:
			begin
				cyc_o <= #1 1'b0;
				stb_o <= #1 1'b0;
				sel_o <= #1 8'h00;
				case(sel_o)
				8'b00000011:	wData <= #1 {{48{dat_i[15]}},dat_i[15: 0]};
				8'b00001100:	wData <= #1 {{48{dat_i[31]}},dat_i[31:16]};
				8'b00110000:	wData <= #1 {{48{dat_i[47]}},dat_i[47:32]};
				8'b11000000:	wData <= #1 {{48{dat_i[63]}},dat_i[63:48]};
				default:	wData <= #1 64'hDEADDEADDEADDEAD;			
				endcase
			end
		`LCU:
			begin
				cyc_o <= #1 1'b0;
				stb_o <= #1 1'b0;
				sel_o <= #1 8'h00;
				case(sel_o)
				8'b00000011:	wData <= #1 dat_i[15: 0];
				8'b00001100:	wData <= #1 dat_i[31:16];
				8'b00110000:	wData <= #1 dat_i[47:32];
				8'b11000000:	wData <= #1 dat_i[63:48];
				default:	wData <= #1 64'hDEADDEADDEADDEAD;
				endcase
			end
		`LB:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				case(sel_o)
				8'b00000001:	wData <= {{56{dat_i[ 7]}},dat_i[ 7: 0]};
				8'b00000010:	wData <= {{56{dat_i[15]}},dat_i[15: 8]};
				8'b00000100:	wData <= {{56{dat_i[23]}},dat_i[23:16]};
				8'b00001000:	wData <= {{56{dat_i[31]}},dat_i[31:24]};
				8'b00010000:	wData <= {{56{dat_i[39]}},dat_i[39:32]};
				8'b00100000:	wData <= {{56{dat_i[47]}},dat_i[47:40]};
				8'b01000000:	wData <= {{56{dat_i[55]}},dat_i[55:48]};
				8'b10000000:	wData <= {{56{dat_i[63]}},dat_i[63:56]};
				default:	wData <= 64'hDEADDEADDEADDEAD;
				endcase
			end
		`LBU:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				case(sel_o)
				8'b00000001:	wData <= dat_i[ 7: 0];
				8'b00000010:	wData <= dat_i[15: 8];
				8'b00000100:	wData <= dat_i[23:16];
				8'b00001000:	wData <= dat_i[31:24];
				8'b00010000:	wData <= dat_i[39:32];
				8'b00100000:	wData <= dat_i[47:40];
				8'b01000000:	wData <= dat_i[55:48];
				8'b10000000:	wData <= dat_i[63:56];
				default:	wData <= 64'hDEADDEADDEADDEAD;
				endcase
			end
		default:	;
		endcase
		// Force stack pointer to word alignment
		if (m2Rt[4:0]==5'b11110)
			wData[2:0] <= 3'b000;
	end
end
// Stage tail
// Pipeline annul for when a bubble in the pipeline occurs.
else if (advanceW) begin
	wextype <= `EX_NON;
	wRt <= 9'd0;
	wData <= 64'd0;
	wIsStore <= 1'b0;
	wIRvalid <= 1'b0;
	wclkoff <= 1'b0;
	wFip <= 1'b0;
	wpcv <= 1'b0;
end


//-----------------------------------------------------------------------------
// WRITEBACK:
// - update the register file with results
// - record exception address and type
// - jump to exception handler routine (below)
// Inputs:
// - w???? type signals
// Outputs:
// - t???? signals
//-----------------------------------------------------------------------------
//
if (advanceW) begin
	textype <= wextype;
	tRt <= wRt;
	tData <= wData;
	if (wRt!=5'd0) begin
		$display("Writing regfile[%d:%d] with %h", wRt[8:5],wRt[4:0], wData);
	end
	if (wIRvalid) begin
		case(wOpcode)
		`LSH:
			case (wRt)
			`SR:	begin
					bu_im <= wData[31];
					im <= wData[15];
					FXE <= wData[12];
					end
			default:	;
			endcase
		`MISC:
			case(wFunc)
			`SYSCALL:
				if (wIR[15:7]==`EX_NMI || wIR[15:7]==`EX_IRQ)
					IPC[wAXC] <= wData;
				else
					EPC[wAXC] <= wData;
			default:	;
			endcase
		`R:
			case(wFunc6)
			`MTSPR:
				case(wIR[11:6])
				`IPC:	begin
						$display("mtspr IPC[%d]=%h",wAXC,wData);
						IPC[wAXC] <= wData;
						end
				`EPC:	EPC[wAXC] <= wData;
				default:	;
				endcase
			endcase
		endcase
	end
	if (wclkoff)
		clk_en <= 1'b0;
	else
		clk_en <= 1'b1;
	// FIP/IEPP:
	// Jump back to the instruction following the FIP/IEPP
	if (wFip) begin
		wFip <= 1'b0;
		m2Fip <= 1'b0;
		m1Fip <= 1'b0;
		xFip <= 1'b0;
		dFip <= 1'b0;
		pc <= fnIncPC(wpc);
		ipcv <= 1'b1;
	end
	//---------------------------------------------------------
	// WRITEBACK (WB') - part two:
	// - vector to exception handler address
	// In the case of a hardware interrupt (NMI/IRQ) we know
	// the pipeline following the interrupt is filled with
	// NOP instructions. This means there is no need to 
	// invalidate the pipeline.
	// 		Also, we have to wait until the WB stage before
	// vectoring so that the pc setting doesn't get trashed
	// by a branch or other exception.
	// 		Tricky because we have to find the first valid
	// PC to record in the IPC register. The interrupt might
	// have occurred in a branch shadow, in which case the
	// current PC isn't valid.
	//---------------------------------------------------------
	case(wextype)
	`EX_NON:	;
	`EX_RST:
		begin
		pc <= `RESET_VECTOR;
		end
	// Hardware exceptions
	`EX_NMI,`EX_IRQ:
		begin
		$display("Stuffing SYSCALL %d",wextype);
		dIR <= {`MISC,5'd25,4'd0,wextype,`SYSCALL};
		dIRvalid <= 1'b1;
		// One of the following pc's MUST be valid.
		// wpc will be valid if the interrupt occurred outside of a branch
		// shadow. m1pc or m2pc (the branch target address) will be valid
		// depending on where in the branch shadow the interrupt falls.
		// Syscall has a larger shadow than a branch because it loads the
		// vector from memory. Eventually syscall flags the pc valid.
		case(1'b1)
		wpcv:	dpc <= wpc;
		m2pcv:	dpc <= m2pc;
		m1pcv: 	dpc <= m1pc;
		xpcv:	dpc <= xpc;
		dpcv:	dpc <= dpc;
		default:	dpc <= pc;
		endcase
		dpcv <= 1'b1;
		end
	// Software exceptions
	// We probably want to return to the excepting instruction.
	`EX_TLBD,`EX_DBERR:
		begin
		pccap <= 1'b0;
		dIR <= {`MISC,5'd24,4'd0,wextype,`SYSCALL};
		dIRvalid <= 1'b1;
		dpc <= wpc;
		dpcv <= 1'b1;
		end
	default:
		begin
		dIR <= {`MISC,5'd24,4'd0,wextype,`SYSCALL};
		dIRvalid <= 1'b1;
		dpc <= wpc;
		dpcv <= 1'b1;
		end
	endcase
end
// Stage tail
// Pipeline annul for when a bubble in the pipeline occurs.
// T and W advance together, meaning an else would never be executed.

//-----------------------------------------------------------------------------
// TRAILER:
// - placeholder to allow the use of synchronous register memory
//-----------------------------------------------------------------------------
if (advanceT) begin
end


//=============================================================================
// Cache loader
//=============================================================================
case(cstate)
IDLE:
	if (triggerDCacheLoad) begin
		dcaccess <= 1'b1;
		bte_o <= 2'b00;			// linear burst
		cti_o <= 3'b001;		// constant address burst access
		bl_o <= 5'd7;
		cyc_o <= 1'b1;
		stb_o <= 1'b1;
		sel_o <= 8'hFF;
		adr_o <= {pea[63:6],6'h00};
		dcadr <= {pea[14:6],6'h00};
		cstate <= DCACT;
	end
	else if (triggerICacheLoad) begin
		icaccess <= 1'b1;
		bte_o <= 2'b00;			// linear burst
		cyc_o <= 1'b1;
		stb_o <= 1'b1;
		sel_o <= 8'hFF;
		if (ICacheAct) begin
			cti_o <= 3'b001;		// constant address burst access
			bl_o <= 5'd7;
			adr_o <= {ppc[63:6],6'h00};
			icadr <= {ppc[63:6],6'h00};
			cstate <= ICACT1;
		end
		else begin
			cti_o <= 3'b000;
			bl_o <= 5'd0;
			adr_o <= {ppc[63:2],2'b00};
			cstate <= ICACT2;
		end
	end
// WISHBONE burst accesses
//
ICACT1:
	if (ack_i|err_i) begin
		icadr[5:3] <= icadr[5:3] + 3'd1;
		if (icadr[5:3]==3'd6)
			cti_o <= 3'b111;	// Last cycle ahead
		if (icadr[5:3]==3'd7) begin
			cti_o <= 3'b000;	// back to non-burst mode
			cyc_o <= 1'b0;
			stb_o <= 1'b0;
			sel_o <= 8'h00;
			tmem[adr_o[12:6]] <= {1'b1,adr_o[63:13]};	// This will cause ihit to go high
			tvalid[adr_o[12:6]] <= 1'b1;
			icaccess <= 1'b0;
			cstate <= IDLE;
		end
	end
//SYSCALL 509:	00000000_00000000_11111110_10010111
ICACT2:
	begin
		if (ack_i|err_i) begin
			ibufadr <= adr_o;
			if (err_i)
				insnbuf <= syscall509;
			else
				insnbuf <= adr_o[2] ? dat_i[63:32] : dat_i[31:0];
			cti_o <= 3'b000;	// back to non-burst mode
			cyc_o <= 1'b0;
			stb_o <= 1'b0;
			sel_o <= 8'h00;
			icaccess <= 1'b0;
			cstate <= IDLE;
		end
	end

DCACT:
	if (ack_i|err_i) begin
		dcadr[5:3] <= dcadr[5:3] + 3'd1;
		if (dcadr[5:3]==3'd6)
			cti_o <= 3'b111;	// Last cycle ahead
		if (dcadr[5:3]==3'h7) begin
			cti_o <= 3'b000;	// back to non-burst mode
			cyc_o <= 1'b0;
			stb_o <= 1'b0;
			sel_o <= 8'h00;
			dcaccess <= 1'b0;
			cstate <= IDLE;
		end
	end

endcase	// cstate
end		// RUN
endcase
end

endmodule
