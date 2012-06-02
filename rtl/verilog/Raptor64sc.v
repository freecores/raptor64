// ============================================================================
// (C) 2012 Robert Finch
// All Rights Reserved.
// robfinch<remove>@opencores.org
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
// ============================================================================
//
`define ADDRESS_RESERVATION	1
`define RAS_PREDICTION		1
//`define FLOATING_POINT		1
//`define BTB					1
//`define TLB		1
//`define BRANCH_PREDICTION_SIMPLE	1

`define RESET_VECTOR	64'hFFFF_FFFF_FFFF_FFF0
`define NMI_VECTOR		64'hFFFF_FFFF_FFFF_FFE0
`define IRQ_VECTOR		64'hFFFF_FFFF_FFFF_FFD0
`define TRAP_VECTOR		64'h0000_0000_0000_0000

`define ITLB_MissHandler	64'hFFFF_FFFF_FFFF_FFC0
`define DTLB_MissHandler	64'hFFFF_FFFF_FFFF_FFB0

`define EX_NON		9'd000
`define EX_TRAP		9'd32	// Trap exception
`define EX_IRQ		9'd449	// interrupt
`define EX_DBZ		9'd488	// divide by zero
`define EX_OFL		9'd489	// overflow
`define EX_PRIV		9'd496	// priviledge violation
`define EX_TLBD		9'd506	// TLB exception - data
`define EX_TLBI		9'd507	// TLB exception - ifetch
`define EX_DBERR	9'd508	// Bus Error - load or store or I/O
`define EX_IBERR	9'd509	// Bus Error - instruction fetch
`define EX_NMI		9'd510	// non-maskable interrupt
`define EX_RST		9'd511	// Reset

`include "Raptor64_opcodes.v"

module Raptor64sc(rst_i, clk_i, nmi_i, irq_i, bte_o, cti_o, bl_o,
	cyc_o, stb_o, ack_i, err_i, we_o, sel_o, rsv_o, adr_o, dat_i, dat_o, sys_adv, sys_adr
);
parameter IDLE = 5'd1;
parameter ICACT = 5'd2;
parameter ICACT0 = 5'd3;
parameter ICACT1 = 5'd4;
parameter ICACT2 = 5'd5;
parameter ICACT3 = 5'd6;
parameter ICACT4 = 5'd7;
parameter ICACT5 = 5'd8;
parameter ICACT6 = 5'd9;
parameter ICACT7 = 5'd10;
parameter ICDLY = 5'd11;
parameter DCIDLE = 5'd20;
parameter DCACT = 5'd21;
parameter DCACT0 = 5'd22;
parameter DCACT1 = 5'd23;
parameter DCACT2 = 5'd24;
parameter DCACT3 = 5'd25;
parameter DCACT4 = 5'd26;
parameter DCACT5 = 5'd27;
parameter DCACT6 = 5'd28;
parameter DCACT7 = 5'd29;
parameter DCDLY = 5'd30;

input rst_i;
input clk_i;
input nmi_i;
input irq_i;

output [1:0] bte_o;
reg [1:0] bte_o;
output [2:0] cti_o;
reg [2:0] cti_o;
output [4:0] bl_o;
reg [4:0] bl_o;
output cyc_o;
reg cyc_o;
output stb_o;
reg stb_o;
input ack_i;
input err_i;
output we_o;
reg we_o;
output [7:0] sel_o;
reg [7:0] sel_o;
output rsv_o;
reg rsv_o;
output [63:0] adr_o;
reg [63:0] adr_o;
input [63:0] dat_i;
output [63:0] dat_o;
reg [63:0] dat_o;

input sys_adv;
input [63:5] sys_adr;

reg [5:0] fltctr;
wire fltdone = fltctr==6'd0;
reg resetA;
reg im,bu_im;			// interrupt mask
reg im1;			// temporary interrupt mask for LM/SM
reg [1:0] vtno;		// vector table number
reg [1:0] rm;		// fp rounding mode
reg FXE;			// fp exception enable
wire KernelMode;
wire [31:0] sr = {bu_im,15'd0,im,1'b0,KernelMode,FXE,vtno,10'b0};
reg [41:0] dIR,xIR,m1IR,m2IR,wIR;
reg [41:0] ndIR;
wire [6:0] dOpcode = dIR[41:35];
reg [63:0] pc;
reg [63:0] ErrorEPC,EPC,IPC;
reg [63:0] dpc,m1pc,m2pc,wpc;
reg dpcv,xpcv,m1pcv,m2pcv,wpcv;	// PC valid indicators
reg [63:0] xpc;
reg [63:0] tlbra;		// return address for a TLB exception
reg [8:0] dRa,dRb,dRc;
reg [8:0] wRt,mRt,m1Rt,m2Rt,tRt,dRt;
reg [8:0] xRt;
reg [63:0] dImm;
reg [63:0] ea;
reg [63:0] iadr_o;
reg [31:0] idat;
reg [4:0] cstate;
reg dbranch_taken,xbranch_taken;
reg [63:0] mutex_gate;
reg [63:0] TBA;
reg [1:0] dhwxtype,xhwxtype,m1hwxtype,m2hwxtype,whwxtype;
reg [3:0] AXC,dAXC,xAXC;
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
reg [41:0] insn;
wire [63:0] rfoa,rfob,rfoc;
reg clk_en;
reg cpu_clk_en;
reg StatusERL;		// 1= in error processing
reg StatusEXL;		// 1= in exception processing
reg StatusHWI;		// 1= in interrupt processing
reg StatusUM;		// 1= user mode
reg [7:0] CauseCode;
reg [7:0] ASID;		// address space identifier (process ID)
integer n;
reg [63:13] BadVAddr;
reg [63:13] PageTableAddr;
reg [63:0] errorAddress;

wire [6:0] iFunc = insn[6:0];
wire [6:0] dFunc = dIR[6:0];
wire [5:0] dFunc6 = dIR[5:0];
wire [6:0] xFunc = xIR[6:0];
wire [5:0] xFunc6 = xIR[5:0];
wire [4:0] xFunc5 = xIR[4:0];
wire [6:0] iOpcode = insn[41:35];
wire [6:0] xOpcode = xIR[41:35];
reg [6:0] m1Opcode,m2Opcode,wOpcode;
reg [6:0] m1Func,m2Func,wFunc;
reg [63:0] m1Data,m2Data,wData,tData;
reg [63:0] m2Addr;
reg [63:0] tick;
reg [63:0] tba;
reg [63:0] exception_address,ipc;
reg [63:0] a,b,c,imm,m1b;
reg prev_ihit;
reg rsf;
reg [63:5] resv_address;
reg dirqf,rirqf,m1irqf,m2irqf,wirqf,tirqf;
reg xirqf;
reg [8:0] dextype,m1extype,m2extype,wextype,textype,exception_type;
reg [8:0] xextype;
reg wLdPC,m2LdPC;
wire advanceX_edge;
reg takb;
wire advanceX,advanceM1,advanceW;
reg m1IsLoad,m2IsLoad;
reg m1IsIO;
reg m1IsStore,m2IsStore,wIsStore;

function [63:0] fnIncPC;
input [63:0] fpc;
begin
case(fpc[3:2])
2'd0:	fnIncPC = {fpc[63:4],4'b0100};
2'd1:	fnIncPC = {fpc[63:4],4'b1000};
2'd2:	fnIncPC = {fpc[63:4]+60'd1,4'b0000};
2'd3:	fnIncPC = {fpc[63:4]+60'd1,4'b0000};
endcase
end
endfunction

assign KernelMode = StatusEXL|StatusHWI;

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
wire wTlbp = advanceW && wOpcode==`MISC && wFunc==`TLBP;
wire wTlbrd = advanceW && wOpcode==`MISC && wFunc==`TLBR;
wire wTlbwr = advanceW && wOpcode==`MISC && wFunc==`TLBWR;
wire wTlbwi = advanceW && wOpcode==`MISC && wFunc==`TLBWI;
wire wMtspr = advanceW && wOpcode==`R && wFunc==`MTSPR;
wire xTlbrd = advanceX && xOpcode==`MISC && xFunc==`TLBR;
wire xTlbwr = advanceX && xOpcode==`MISC && xFunc==`TLBWR;
wire xTlbwi = advanceX && xOpcode==`MISC && xFunc==`TLBWI;
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

wire dram_bus = !pea[63];
wire m2_dram_bus = !m2Addr[63];

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
// Current method is to concatonate two 32 bit generators. This isn't a very
// good way to do it.
//-----------------------------------------------------------------------------
reg [31:0] m_z1,m_z2;
reg [31:0] m_w1,m_w2;
reg [31:0] next_m_z1,next_m_z2;
reg [31:0] next_m_w1,next_m_w2;

always @(m_z1 or m_w1)
begin
	next_m_z1 <= (18'h36969 * m_z1[15:0]) + m_z1[31:16];
	next_m_w1 <= (18'h18000 * m_w1[15:0]) + m_w1[31:16];
end

always @(m_z2 or m_w2)
begin
	next_m_z2 <= (18'h36969 * m_z2[15:0]) + m_z2[31:16];
	next_m_w2 <= (18'h18000 * m_w2[15:0]) + m_w2[31:16];
end

wire [31:0] rand1 = {m_z1[15:0],16'd0} + m_w1;
wire [31:0] rand2 = {m_z2[15:0],16'd0} + m_w2;
wire [63:0] rand = {rand2,rand1};

wire [10:0] bias = 11'h3FF;				// bias amount (eg 127)
wire [10:0] xl = rand[62:53];
wire sgn = 1'b0;								// floating point: always generate a positive number
wire [10:0] exp = xl > bias-1 ? bias-1 : xl;	// 2^-1 otherwise number could be over 1
wire [52:0] man = rand[52:0];					// a leading '1' will be assumed
wire [63:0] randfd = {sgn,exp,man};
reg [63:0] rando;

//-----------------------------------------------------------------------------
// Instruction Cache / Instruction buffer
// 8kB
// 
//-----------------------------------------------------------------------------
//reg lfdir;
wire lfdir = (((dOpcode==`LM || dOpcode==`SM) && dIR[31:0]!=32'd0) && ndIR[31:0]!=32'd0) ||
				((dOpcode==`LP || dOpcode==`SP || dOpcode==`LFP || dOpcode==`SFP || dOpcode==`LFDP || dOpcode==`SFDP) && dIR[25]!=1'b1);
wire ldnop = (((dOpcode==`LM || dOpcode==`SM) && (dIR[31:0]==32'd0 || ndIR[31:0]==32'd0)) ||
				((dOpcode==`LP || dOpcode==`SP || dOpcode==`LFP || dOpcode==`SFP || dOpcode==`LFDP || dOpcode==`SFDP) && dIR[25]==1'b1));
reg icaccess;
reg ICacheOn;
wire ibufrdy;
reg [63:0] tmpbuf;
wire [127:0] insnbundle;
reg [127:0] insnbuf0,insnbuf1;
reg [63:4] ibuftag0,ibuftag1;
wire isICached = ppc[63:32]!=nonICacheSeg;
//wire isEncrypted = ppc[63:32]==encryptedArea;
wire ICacheAct = ICacheOn & isICached;
reg [41:0] insn1;
reg [41:0] insnkey;

// SYSCALL 509
wire [127:0] bevect = 128'b00_00000000_00000000_00000000_11111110_10010111__00_00000000_00000000_00000000_11111110_10010111__00_00000000_00000000_00000000_11111110_10010111;

Raptor64_icache_ram u1
(
	.clka(clk), // input clka
	.wea(icaccess & (ack_i|err_i)), // input [0 : 0] wea
	.addra(adr_o[12:3]), // input [9 : 0] addra
	.dina(err_i ? (adr_o[3] ? bevect[127:64] : bevect[63:0]) : dat_i), // input [63 : 0] dina
	.clkb(~clk), // input clkb
	.addrb(pc[12:4]), // input [8 : 0] addrb
	.doutb(insnbundle) // output [127 : 0] doutb
);

always @(ppc or insnbundle or ICacheAct or insnbuf0 or insnbuf1 or ndIR or lfdir or ldnop)
begin
	casex({ldnop,lfdir,ICacheAct,ibuftag1==ppc[63:4],pc[3:2]})
	6'b1xxxxx:	insn1 <= 42'h37800000000;
	6'b01xxxx:	insn1 <= ndIR;
	6'b001x00:	insn1 <= insnbundle[ 41: 0];
	6'b001x01:	insn1 <= insnbundle[ 83:42];
	6'b001x10:	insn1 <= insnbundle[125:84];
	6'b001x11:	insn1 <= 42'h37800000000;	// NOP instruction
	6'b000000:	insn1 <= insnbuf0[ 41: 0];
	6'b000001:	insn1 <= insnbuf0[ 83:42];
	6'b000010:	insn1 <= insnbuf0[125:84];
	6'b000011:	insn1 <= 42'h37800000000;
	6'b000100:	insn1 <= insnbuf1[ 41: 0];
	6'b000101:	insn1 <= insnbuf1[ 83:42];
	6'b000110:	insn1 <= insnbuf1[125:84];
	6'b000111:	insn1 <= 42'h37800000000;
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
assign ibufrdy = ibuftag0==ppc[63:4] || ibuftag1==ppc[63:4];

//-----------------------------------------------------------------------------
// Data Cache
// No-allocate on write
//-----------------------------------------------------------------------------
reg dcaccess;
wire dhit;
wire [64:15] dtgout;
reg wrhit;
reg [7:0] dsel_o;
reg [63:0] dadr_o;
reg [31:0] ddat;
reg wr_dcache;

// cache RAM 32Kb
Raptor64_dcache_ram u10
(
	.clka(clk), // input clka
	.ena(1'b1),
	.wea(dcaccess ? {8{ack_i}} : wrhit ? sel_o : 8'h00), // input [7 : 0] wea
	.addra(adr_o[14:3]), // input [11 : 0] addra
	.dina(dcaccess ? dat_i : dat_o), // input [63 : 0] dina

	.clkb(~clk), // input clkb
	.addrb(pea[14:3]), // input [11 : 0] addrb
	.doutb(cdat) // output [63 : 0] doutb
);


Raptor64_dcache_tagram u11
(
	.clka(clk), // input clka
	.ena(dtinit | (adr_o[5:3]==3'b111)), // input ena
	.wea(dtinit | (dcaccess & ack_i)), // input [0 : 0] wea
	.addra({1'b0,adr_o[14:6]}), // input [9 : 0] addra
	.dina(dtinit ? {1'b0,adr_o[63:15]} : {1'b1,adr_o[63:15]}), // input [48 : 0] dina

	.clkb(~clk), // input clkb
	.addrb({1'b0,pea[14:6]}), // input [9 : 0] addrb
	.doutb(dtgout) // output [48 : 0] doutb
);

assign dhit = (dtgout=={1'b1,pea[63:15]});

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

reg [64:0] xData;
wire xisCacheElement = (xData[63:52] != 12'hFFD && xData[63:52]!=12'hFFF) && dcache_on;
reg m1IsCacheElement;

reg nopI;



wire [127:0] mult_out;
wire [63:0] sqrt_out;
wire [63:0] div_q;
wire [63:0] div_r;
wire sqrt_done,mult_done,div_done;
wire isSqrt = xOpcode==`R && xFunc==`SQRT;
wire [7:0] bcdaddo,bcdsubo;

BCDAdd u40(.ci(1'b0),.a(a[7:0]),.b(b[7:0]),.o(bcdaddo),.c());
BCDSub u41(.ci(1'b0),.a(a[7:0]),.b(b[7:0]),.o(bcdsubo),.c());

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

wire isMulu = xOpcode==`RR && xFunc==`MULU;
wire isMuls = (xOpcode==`RR && xFunc==`MULS) || xOpcode==`MULSI;
wire isMuli = xOpcode==`MULSI || xOpcode==`MULUI;
wire isMult = xOpcode==`MULSI || xOpcode==`MULUI || (xOpcode==`RR && (xFunc==`MULS || xFunc==`MULU));
wire isDivu = xOpcode==`RR && xFunc==`DIVU;
wire isDivs = (xOpcode==`RR && xFunc==`DIVS) || xOpcode==`DIVSI;
wire isDivi = xOpcode==`DIVSI || xOpcode==`DIVUI;
wire isDiv = xOpcode==`DIVSI || xOpcode==`DIVUI || (xOpcode==`RR && (xFunc==`DIVS || xFunc==`DIVU));

wire disRRShift = dOpcode==`RR && (
	dFunc==`SHL || dFunc==`ROL || dFunc==`SHR ||
	dFunc==`SHRU || dFunc==`ROR || dFunc==`ROLAM
	);
wire disRightShift = dOpcode==`RR && (
	dFunc==`SHR || dFunc==`SHRU || dFunc==`ROR
	);

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
	.ld(isDiv),
	.sgn(isDivs),
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
		if (xOpcode==`FP) begin
			if (xFunc[5:0]==6'b000000)	// FDADD
				fltctr <= 6'd12;
			else if (xFunc[5:0]==6'b000001)	// FDSUB
				fltctr <= 6'd12;
			else if (xFunc[5:0]==6'b000010)	// FDMUL
				fltctr <= 6'd12;
			else if (xFunc[5:0]==6'b000011)	// FDDIV
				fltctr <= 6'd12;
			else if (xFunc[5:0]==6'b000100)	// unordered
				fltctr <= 6'd2;
			else if (xFunc[5:0]==6'b001100)	// less than
				fltctr <= 6'd2;
			else if (xFunc[5:0]==6'b010100)	// equal
				fltctr <= 6'd2;
			else if (xFunc[5:0]==6'b011100)	// less than or equal
				fltctr <= 6'd2;
			else if (xFunc[5:0]==6'b100100)	// greater than
				fltctr <= 6'd2;
			else if (xFunc[5:0]==6'b101100)	// not equal
				fltctr <= 6'd2;
			else if (xFunc[5:0]==6'b110100)	// greater than or equal
				fltctr <= 6'd2;
			else if (xFunc[5:0]==6'b000101)	// ItoFD
				fltctr <= 6'd7;
			else if (xFunc[5:0]==6'b000110)	// FFtoI
				fltctr <= 6'd6;
			else if (xFunc[5:0]==6'b000111)	// FtoD
				fltctr <= 6'd2;
			else if (xFunc[5:0]==6'b001000) // DtoF
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

wire [63:0] jmp_tgt = dOpcode[6:4]==`IMM ? {dIR[26:0],insn[34:0],2'b00} : {pc[63:37],insn[34:0],2'b00};

//-----------------------------------------------------------------------------
// Stack for return address predictor
//-----------------------------------------------------------------------------
`ifdef RAS_PREDICTION
reg [63:0] ras [63:0];	// return address stack, return predictions
reg [5:0] ras_sp;		// stack pointer
`endif
`ifdef BTB
reg [63:0] btb [63:0];	// branch target buffer
`endif

`ifdef BRANCH_PREDICTION_SIMPLE
//-----------------------------------------------------------------------------
// Simple predictor:
// - backwards branches are predicted taken, others predicted not taken.
//-----------------------------------------------------------------------------
reg predict_taken;

always @(iOpcode or insn)
case(iOpcode)
`BTRR:
	case(insn[4:0])
	`BEQ,`BNE,`BLT,`BLE,`BGT,`BGE,`BLTU,`BLEU,`BGTU,`BGEU,`BAND,`BOR,`BNR:
		predict_taken = insn[24];
	default:	predict_taken = 1'd0;
	endcase
`BEQI,`BNEI,`BLTI,`BLEI,`BGTI,`BGEI,`BLTUI,`BLEUI,`BGTUI,`BGEUI:
	predict_taken = insn[29];
default:
	predict_taken = 1'd0;
endcase
`else

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
	.xIR(xIR),
	.pc(pc),
	.xpc(xpc),
	.takb(takb),
	.predict_taken(predict_taken)
);
`endif

//-----------------------------------------------------------------------------
// Evaluate branch conditions.
//-----------------------------------------------------------------------------
wire signed [63:0] as = a;
wire signed [63:0] bs = b;
wire signed [63:0] imms = imm;
wire aeqz = a==64'd0;
wire beqz = b==64'd0;
wire immeqz = imm==64'd0;

wire eqb0 = a[ 7: 0]==b[ 7: 0];
wire eqb1 = a[15: 8]==b[15: 8];
wire eqb2 = a[23:16]==b[23:16];
wire eqb3 = a[31:24]==b[31:24];
wire eqb4 = a[39:32]==b[39:32];
wire eqb5 = a[47:40]==b[47:40];
wire eqb6 = a[55:48]==b[55:48];
wire eqb7 = a[63:56]==b[63:56];

wire eqc0 = eqb0 & eqb1;
wire eqc1 = eqb2 & eqb3;
wire eqc2 = eqb4 & eqb5;
wire eqc3 = eqb6 & eqb7;

wire eqh0 = eqc0 & eqc1;
wire eqh1 = eqc2 & eqc3;

wire eqw = eqh0 & eqh1;
wire eq = eqw;

wire eqi = a==imm;
wire lt = $signed(a) < $signed(b);
wire lti = as < imms;
wire ltu = a < b;
wire ltui = a < imm;

always @(xOpcode or xFunc or a or eq or eqi or lt or lti or ltu or ltui or aeqz or beqz or rsf or xIR)
case (xOpcode)
`BTRR:
	case(xFunc5)
	`BRA:	takb = 1'b1;
	`BRN:	takb = 1'b0;
	`BEQ:	takb = eq;
	`BNE:	takb = !eq;
	`BLT:	takb = lt;
	`BLE:	takb = lt|eq;
	`BGT:	takb = !(lt|eq);
	`BGE:	takb = !lt;
	`BLTU:	takb = ltu;
	`BLEU:	takb = ltu|eq;
	`BGTU:	takb = !(ltu|eq);
	`BGEU:	takb = !ltu;
	`BOR:	takb = !aeqz || !beqz;
	`BAND:	takb = !aeqz && !beqz;
	`BNR:	takb = !rsf;
	`LOOP:	takb = !beqz;
	`BEQR:	takb = eq;
	`BNER:	takb = !eq;
	`BLTR:	takb = lt;
	`BLER:	takb = lt|eq;
	`BGTR:	takb = !(lt|eq);
	`BGER:	takb = !lt;
	`BLTUR:	takb = ltu;
	`BLEUR:	takb = ltu|eq;
	`BGTUR:	takb = !(ltu|eq);
	`BGEUR:	takb = !ltu;
	default:	takb = 1'b0;
	endcase
`BEQI:	takb = eqi;
`BNEI:	takb = !eqi;
`BLTI:	takb = lti;
`BLEI:	takb = lti|eqi;
`BGTI:	takb = !(lti|eqi);
`BGEI:	takb = !lti;
`BLTUI:	takb = ltui;
`BLEUI:	takb = ltui|eqi;
`BGTUI:	takb = !(ltui|eqi);
`BGEUI:	takb = !ltui;
`BTRI:
	case(xIR[24:20])
	`BRA:	takb = 1'b1;
	`BRN:	takb = 1'b0;
	`BEQ:	takb = eqi;
	`BNE:	takb = !eqi;
	`BLT:	takb = lti;
	`BLE:	takb = lti|eqi;
	`BGT:	takb = !(lti|eqi);
	`BGE:	takb = !lti;
	`BLTU:	takb = ltui;
	`BLEU:	takb = ltui|eqi;
	`BGTU:	takb = !(ltui|eqi);
	`BGEU:	takb = !ltui;
	default:	takb = 1'b0;
	endcase
`TRAPcc: 
	case(xFunc5)
	`TEQ:	takb = eq;
	`TNE:	takb = !eq;
	`TLT:	takb = lt;
	`TLE:	takb = lt|eq;
	`TGT:	takb = !(lt|eq);
	`TGE:	takb = !lt;
	`TLTU:	takb = ltu;
	`TLEU:	takb = ltu|eq;
	`TGTU:	takb = !(ltu|eq);
	`TGEU:	takb = !ltu;
	default:	takb = 1'b0;
	endcase
`TRAPcci: 
	case(xIR[29:25])
	`TEQI:	takb = eqi;
	`TNEI:	takb = !eqi;
	`TLTI:	takb = lti;
	`TLEI:	takb = lti|eqi;
	`TGTI:	takb = !(lti|eqi);
	`TGEI:	takb = !lti;
	`TLTUI:	takb = ltui;
	`TLEUI:	takb = ltui|eqi;
	`TGTUI:	takb = !(ltui|eqi);
	`TGEUI:	takb = !ltui;
	default:	takb = 1'b0;
	endcase
default:
	takb = 1'b0;
endcase


//-----------------------------------------------------------------------------
// Datapath (ALU) operations.
//-----------------------------------------------------------------------------
wire [6:0] cntlzo,cntloo;
cntlz64 u12 (.clk(clk), .i(a),  .o(cntlzo) );
cntlo64 u13 (.clk(clk), .i(a),  .o(cntloo) );

reg [1:0] shftop;
wire [63:0] shfto;
reg [63:0] masko;
reg [63:0] bfextd;
wire [127:0] shlxo = {64'd0,a} << b[5:0];
wire [127:0] shruxo = {a,64'd0} >> b[5:0];
wire [63:0] shlo = shlxo[63:0];
wire [63:0] shruo = shruxo[127:64];
wire [63:0] rolo = {shlxo[127:64]|shlxo[63:0]};
wire [63:0] roro = {shruxo[127:64]|shruxo[63:0]};
wire [63:0] shro = ~(~a >> b[5:0]);
// generate mask
wire [5:0] mb = xIR[12:7];
wire [5:0] me = xIR[18:13];
integer nn;
always @(mb or me or nn)
	for (nn = 0; nn < 64; nn = nn + 1)
		masko[nn] <= (nn >= mb) ^ (nn <= me) ^ (me >= mb);

always @(xOpcode or xFunc or a or b or imm or as or bs or imms or xpc or
	sqrt_out or cntlzo or cntloo or tick or ipc or tba or AXC or
	lt or eq or ltu or mult_out or lti or eqi or ltui or xIR or div_q or div_r or
	shfto or masko or bcdaddo or bcdsubo or fpLooOut or fpZLOut or
`ifdef TLB
	PageTableAddr or BadVAddr or ASID or tlbo or
`endif
	ASID or EPC or mutex_gate or IPC or CauseCode or TBA or xAXC or nonICacheSeg or rm or
	rando
)
casex(xOpcode)
`R:
	casex(xFunc)
	`COM:	xData = ~a;
	`NOT:	xData = ~|a;
	`NEG:	xData = -a;
	`ABS:	xData = a[63] ? -a : a;
	`SGN:	xData = a[63] ? 65'h1FFFFFFFF_FFFFFFFF : aeqz ? 65'd0 : 65'd1;
	`MOV:	xData = a;
	`SQRT:	xData = sqrt_out;
	`SWAP:	xData = {a[31:0],a[63:32]};
	
	`REDOR:		xData = |a;
	`REDAND:	xData = &a;

	`CTLZ:	xData = cntlzo;
	`CTLO:	xData = cntloo;
	`CTPOP:	xData = {4'd0,popcnt6(a[5:0])} +
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
	`SEXT8:		xData = {{56{a[7]}},a[7:0]};	
	`SEXT16:	xData = {{48{a[15]}},a[15:0]};
	`SEXT32:	xData = {{32{a[31]}},a[31:0]};

	`MTSPR:		xData = a;
	`MFSPR:
		case(xIR[12:7])
`ifdef TLB
		`TLBWired:		xData = tlbo;
		`TLBIndex:		xData = tlbo;
		`TLBRandom:		xData = tlbo;
		`TLBPhysPage0:	xData = tlbo;
		`TLBPhysPage1:	xData = tlbo;
		`TLBVirtPage:	xData = tlbo;
		`TLBPageMask:	xData = tlbo;
		`TLBASID:	begin
					xData = 65'd0;
					xData[0] = tlbo[0];
					xData[1] = tlbo[1];
					xData[2] = tlbo[2];
					xData[15:8] = tlbo[15:8];
					end
		`PageTableAddr:	xData = {PageTableAddr,13'd0};
		`BadVAddr:		xData = {BadVAddr,13'd0};
`endif
		`ASID:			xData = ASID;
		`Tick:			xData = tick;
		`EPC:			xData = EPC;
		`IPC:			xData = IPC;
		`CauseCode:		xData = CauseCode;
		`TBA:			xData = TBA;
		`AXC:			xData = xAXC;
		`NON_ICACHE_SEG:	xData = nonICacheSeg;
		`FPCR:			xData = FPC;
		`RAND:			xData = rando;
		`SRAND1:		xData = {m_z2,m_z1};
		`SRAND2:		xData = {m_w2,m_w1};
		`INSNKEY:		xData = insnkey;
		default:	xData = 65'd0;
		endcase
	`OMG:		xData = mutex_gate[a[5:0]];
	`CMG:		xData = mutex_gate[a[5:0]];
	`OMGI:		begin
				xData = mutex_gate[xIR[12:7]];
				$display("mutex_gate[%d]=%d",xIR[12:7],mutex_gate[xIR[12:7]]);
				end
	`CMGI:		xData = mutex_gate[xIR[12:7]];
	default:	xData = 65'd0;
	endcase
`RR:
	case(xFunc)
	`ADD:	xData = a + b;
	`ADDU:	xData = a + b;
	`SUB:	xData = a - b;
	`SUBU:	xData = a - b;
	`CMP:	xData = lt ? 64'hFFFFFFFFFFFFF : eq ? 64'd0 : 64'd1;
	`CMPU:	xData = ltu ? 64'hFFFFFFFFFFFFF : eq ? 64'd0 : 64'd1;
	`SEQ:	xData = eq;
	`SNE:	xData = !eq;
	`SLT:	xData = lt;
	`SLE:	xData = lt|eq;
	`SGT:	xData = !(lt|eq);
	`SGE:	xData = !lt;
	`SLTU:	xData = ltu;
	`SLEU:	xData = ltu|eq;
	`SGTU:	xData = !(ltu|eq);
	`SGEU:	xData = !ltu;
	`AND:	xData = a & b;
	`OR:	xData = a | b;
	`XOR:	xData = a ^ b;
	`ANDC:	xData = a & ~b;
	`NAND:	xData = ~(a & b);
	`NOR:	xData = ~(a | b);
	`XNOR:	xData = ~(a ^ b);
	`ORC:	xData = a | ~b;
	`MIN:	xData = lt ? a : b;
	`MAX:	xData = lt ? b : a;
	`MOVZ:	xData = b;
	`MOVNZ:	xData = b;
	`MULS:	xData = mult_out[63:0];
	`MULU:	xData = mult_out[63:0];
	`DIVS:	xData = div_q;
	`DIVU:	xData = div_q;
	`MOD:	xData = div_r;
	`SHL:	xData = shlo;
	`SHRU:	xData = shruo;
	`ROL:	xData = rolo;
	`ROR:	xData = roro;
	`SHR:	xData = shro;
	`ROLAM:	xData = rolo & masko;
	`BCD_ADD:	xData = bcdaddo;
	`BCD_SUB:	xData = bcdsubo;
	default:	xData = 65'd0;
	endcase
`SHFTI:
	case(xFunc5)
	`SHLI:	xData = shlo;
	`SHRUI:	xData = shruo;
	`ROLI:	xData = rolo;
	`RORI:	xData = roro;
	`SHRI:	xData = shro;
	`ROLAMI:	xData = rolo & masko;
	`BFINS: 	begin for (n = 0; n < 64; n = n + 1) xData[n] = masko[n] ? rolo[n] : b[n]; xData[64] = 1'b0; end
	`BFSET: 	begin for (n = 0; n < 64; n = n + 1) xData[n] = masko[n] ? 1'b1 : b[n]; xData[64] = 1'b0; end
	`BFCLR: 	begin for (n = 0; n < 64; n = n + 1) xData[n] = masko[n] ? 1'b0 : b[n]; xData[64] = 1'b0; end
	`BFCHG: 	begin for (n = 0; n < 64; n = n + 1) xData[n] = masko[n] ? ~b[n] : b[n]; xData[64] = 1'b0; end
	`BFEXT:		begin for (n = 0; n < 64; n = n + 1) bfextd[n] = masko[n] ? b[n] : 1'b0; xData = bfextd >> mb; end
	default:	xData = 65'd0;
	endcase
`BTRR:
	case(xFunc5)
	`LOOP:	xData = b - 64'd1;
	default:	xData = 64'd0;
	endcase
`SETLO:	xData = {{32{xIR[31]}},xIR[31:0]};
`SETHI:	xData = {xIR[31:0],a[31:0]};
`ADDI:	xData = a + imm;
`ADDUI:	xData = a + imm;
`SUBI:	xData = a - imm;
`SUBUI:	xData = a - imm;
`CMPI:	xData = lti ? 64'hFFFFFFFFFFFFF : eqi ? 64'd0 : 64'd1;
`CMPUI:	xData = ltui ? 64'hFFFFFFFFFFFFF : eqi ? 64'd0 : 64'd1;
`MULSI:	xData = mult_out[63:0];
`MULUI:	xData = mult_out[63:0];
`DIVSI:	xData = div_q;
`DIVUI:	xData = div_q;
`ANDI:	xData = a & imm;
`ORI:	xData = a | imm;
`XORI:	xData = a ^ imm;
`SEQI:	xData = eqi;
`SNEI:	xData = !eqi;
`SLTI:	xData = lti;
`SLEI:	xData = lti|eqi;
`SGTI:	xData = !(lti|eqi);
`SGEI:	xData = !lti;
`SLTUI:	xData = ltui;
`SLEUI:	xData = ltui|eqi;
`SGTUI:	xData = !(ltui|eqi);
`SGEUI:	xData = !ltui;
`INB,`INCH,`INH,`INW,`INCU,`INHU,`INBU:
		xData = a + imm;
`OUTB,`OUTC,`OUTH,`OUTW:
		xData = a + imm;
`LW,`LH,`LC,`LB,`LHU,`LCU,`LBU,`LWR,`LF,`LFD,`LP,`LFP,`LFDP,`LEA:
		xData = a + imm;
`SW,`SH,`SC,`SB,`SWC,`SF,`SFD,`SP,`SFP,`SFDP:
		xData = a + imm;
`MEMNDX:
		xData = a + b + imm;
`SM:	xData = a + {popcnt36(xIR[31:0]),3'b000}-64'd8;
`LM:	xData = a + {popcnt36(xIR[31:0]),3'b000}-64'd8;
`TRAPcc:	xData = fnIncPC(xpc);
`TRAPcci:	xData = fnIncPC(xpc);
`CALL:		xData = fnIncPC(xpc);
`JAL:		xData = xpc + {xIR[29:25],2'b00};
`RET:	xData = a + imm;
`FPLOO:	xData = fpLooOut;
`FPZL:	xData = fpZLOut;
`ifdef FLOATING_POINT
`FP:
	case(xFunc6)
	`FDADD:	xData = daddsub_result;
	`FDSUB:	xData = daddsub_result;
	`FDMUL:	xData = dmul_result;
	`FDDIV:	xData = ddiv_result;
	`FDI2F:	xData = i2f_result;
	`FDF2I:	xData = f2i_result;
	`FDCUN:	xData = dcmp_result;
	`FDCEQ:	xData = dcmp_result;
	`FDCNE:	xData = dcmp_result;
	`FDCLT:	xData = dcmp_result;
	`FDCLE:	xData = dcmp_result;
	`FDCGT:	xData = dcmp_result;
	`FDCGE:	xData = dcmp_result;
	endcase
`endif
default:	xData = 65'd0;
endcase

wire v_ri,v_rr;
overflow u2 (.op(xOpcode==`SUBI), .a(a[63]), .b(imm[63]), .s(xData[63]), .v(v_ri));
overflow u3 (.op(xOpcode==`RR && xFunc==`SUB), .a(a[63]), .b(b[63]), .s(xData[63]), .v(v_rr));

wire dbz_error = ((xOpcode==`DIVSI||xOpcode==`DIVUI) && imm==64'd0) || (xOpcode==`RR && (xFunc==`DIVS || xFunc==`DIVU) && b==64'd0);
wire ovr_error = ((xOpcode==`ADDI || xOpcode==`SUBI) && v_ri) || ((xOpcode==`RR && (xFunc==`SUB || xFunc==`ADD)) && v_rr);
wire priv_violation = !KernelMode && (xOpcode==`MISC &&
	(xFunc==`IRET || xFunc==`ERET || xFunc==`CLI || xFunc==`SEI ||
	 xFunc==`TLBP || xFunc==`TLBR || xFunc==`TLBWR || xFunc==`TLBWI
	));
wire xIsSqrt = xOpcode==`R && xFunc==`SQRT;
wire xIsMult = (xOpcode==`RR && (xFunc==`MULU || xFunc==`MULS)) ||
	xOpcode==`MULSI || xOpcode==`MULUI;
wire xIsDiv = (xOpcode==`RR && (xFunc==`DIVU || xFunc==`DIVS)) ||
	xOpcode==`DIVSI || xOpcode==`DIVUI;
wire xIsCnt = xOpcode==`R && (xFunc==`CTLZ || xFunc==`CTLO || xFunc==`CTPOP);
reg m1IsCnt,m2IsCnt;

wire xIsLoad =
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
	(xOpcode==`MISC && (xFunc==`SYSJMP || xFunc==`SYSCALL))
	;

wire xIsStore =
	xOpcode==`SW || xOpcode==`SH || xOpcode==`SB || xOpcode==`SC || xOpcode==`SWC || xOpcode==`SM ||
	xOpcode==`SF || xOpcode==`SFD || xOpcode==`SP || xOpcode==`SFP || xOpcode==`SFDP ||
	xOpcode==`SSH || xOpcode==`SSW ||
	(xOpcode==`MEMNDX && (
		xFunc6==`SWX || xFunc6==`SHX || xFunc6==`SBX || xFunc6==`SCX || xFunc6==`SWCX ||
		xFunc6==`SFX || xFunc6==`SFDX || xFunc6==`SPX ||
		xFunc6==`SSHX || xFunc6==`SSWX
	))
	;
wire xIsSWC = xOpcode==`SWC;
wire xIsIn = 
	xOpcode==`INW || xOpcode==`INH || xOpcode==`INCH || xOpcode==`INB ||
	xOpcode==`INHU || xOpcode==`INCU || xOpcode==`INBU
	;
//wire mIsSWC = mOpcode==`SWC;

wire m1IsIn = 
	m1Opcode==`INW || m1Opcode==`INH || m1Opcode==`INCH || m1Opcode==`INB ||
	m1Opcode==`INHU || m1Opcode==`INCU || m1Opcode==`INBU
	;
wire m2IsInW = m2Opcode==`INW;
wire xIsIO = 
	xIsIn ||
	xOpcode==`OUTW || xOpcode==`OUTH || xOpcode==`OUTC || xOpcode==`OUTB
	;


wire xIsFPLoo = xOpcode==`FPLOO;
wire xIsFP = xOpcode==`FP;
wire xneedBus = xIsIO;
wire m1needBus = (m1IsLoad & !m1IsCacheElement) || m1IsStore || m1IsIO;
wire m2needBus = (m2IsLoad | m2IsStore);

// Stall on SWC allows rsf flag to be loaded for the next instruction
// Currently stalls on load of R0, but doesn't need to.
wire StallR =  	(((xIsLoad||xIsIn||xIsCnt) && ((xRt==dRa)||(xRt==dRb)||(xRt==dRt))) || xIsSWC) ||
				(((m1IsLoad||m1IsIn||m1IsCnt) && ((m1Rt==dRa)||(m1Rt==dRb)||(m1Rt==dRt)))) ||
				(((m2IsLoad||m2IsCnt) && ((m2Rt==dRa)||(m2Rt==dRb)||(m2Rt==dRt))))
				;
wire StallX = xneedBus & (m1needBus|m2needBus|icaccess|dcaccess);
wire StallM1 = (m1needBus & (m2needBus|icaccess|dcaccess)) ||
				( m1IsLoad & m1IsCacheElement & (m2IsStore|wIsStore))	// wait for a preceding store to complete
				;
wire StallM2 =  icaccess|dcaccess;

wire advanceT = !resetA;
assign advanceW = advanceT;
wire advanceM2 = advanceW &&
					((m2IsLoad || m2IsStore) ? (ack_i|err_i) : 1'b1) &&
					!StallM2
					;
assign advanceM1 = advanceM2 &
					(m1IsIO ? (ack_i|err_i) : 1'b1) &
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
wire advanceR = advanceX & !StallR;
wire advanceI = advanceR & (ICacheOn ? ihit : ibufrdy);

wire triggerDCacheLoad = (m1IsLoad & m1IsCacheElement & !dhit) &&	// there is a miss
						!(icaccess | dcaccess) && 	// caches are not active
						m2Opcode==`NOPI				// and the pipeline is free of memory-ops
						;
// Since IMM is "sticky" we have to check for it.
wire triggerICacheLoad1 = ICacheAct && !ihit && !triggerDCacheLoad &&	// There is a miss
						!(icaccess | dcaccess) && 	// caches are not active
						(dOpcode==`NOPI || dOpcode[6:4]==`IMM) &&			// and the pipeline is flushed
						(xOpcode==`NOPI || xOpcode[6:4]==`IMM) &&
						m1Opcode==`NOPI &&
						m2Opcode==`NOPI
						;
wire triggerICacheLoad2 = (!ICacheAct && !ibufrdy) && !triggerDCacheLoad &&	// There is a miss
						!(icaccess | dcaccess) 	// caches are not active
						;
wire triggerICacheLoad = triggerICacheLoad1 | triggerICacheLoad2;

wire EXexception_pending = ovr_error || dbz_error || priv_violation || xOpcode==`TRAPcci || xOpcode==`TRAPcc;
`ifdef TLB
wire M1exception_pending = advanceM1 & (m1IsLoad|m1IsStore) & DTLBMiss;
`else
wire M1exception_pending = 1'b0;
`endif
wire exception_pending = EXexception_pending | M1exception_pending;

reg prev_nmi,nmi_edge;

always @(dOpcode or dIR)
begin
	ndIR <= dIR;
	if (dOpcode==`SP || dOpcode==`LP || dOpcode==`SFP || dOpcode==`LFP || dOpcode==`SFDP || dOpcode==`LFDP) begin
		ndIR[25] <= 1'b1;
		ndIR[3] <= 1'b1;
	end
	else if ((dOpcode==`LM || dOpcode==`SM) && dIR[31:0]!=32'd0) begin
		$display("LM/SM %h",dIR[31:0]);
		if (dIR[0]) 
			ndIR[0] <= 1'b0;
		else if (dIR[1])
			ndIR[1] <= 1'b0;
		else if (dIR[2])
			ndIR[2] <= 1'b0;
		else if (dIR[3])
			ndIR[3] <= 1'b0;
		else if (dIR[4])
			ndIR[4] <= 1'b0;
		else if (dIR[5])
			ndIR[5] <= 1'b0;
		else if (dIR[6])
			ndIR[6] <= 1'b0;
		else if (dIR[7])
			ndIR[7] <= 1'b0;
		else if (dIR[8])
			ndIR[8] <= 1'b0;
		else if (dIR[9])
			ndIR[9] <= 1'b0;
		else if (dIR[10])
			ndIR[10] <= 1'b0;
		else if (dIR[11])
			ndIR[11] <= 1'b0;
		else if (dIR[12])
			ndIR[12] <= 1'b0;
		else if (dIR[13])
			ndIR[13] <= 1'b0;
		else if (dIR[14])
			ndIR[14] <= 1'b0;
		else if (dIR[15])
			ndIR[15] <= 1'b0;
		else if (dIR[16])
			ndIR[16] <= 1'b0;
		else if (dIR[17])
			ndIR[17] <= 1'b0;
		else if (dIR[18])
			ndIR[18] <= 1'b0;
		else if (dIR[19])
			ndIR[19] <= 1'b0;
		else if (dIR[20])
			ndIR[20] <= 1'b0;
		else if (dIR[21])
			ndIR[21] <= 1'b0;
		else if (dIR[22])
			ndIR[22] <= 1'b0;
		else if (dIR[23])
			ndIR[23] <= 1'b0;
		else if (dIR[24])
			ndIR[24] <= 1'b0;
		else if (dIR[25])
			ndIR[25] <= 1'b0;
		else if (dIR[26])
			ndIR[26] <= 1'b0;
		else if (dIR[27])
			ndIR[27] <= 1'b0;
		else if (dIR[28])
			ndIR[28] <= 1'b0;
		else if (dIR[29])
			ndIR[29] <= 1'b0;
		else if (dIR[30])
			ndIR[30] <= 1'b0;
		else
			ndIR[31] <= 1'b0;
	end
end

reg m1clkoff,m2clkoff,m3clkoff,m4clkoff,wclkoff;
reg dFip,xFip,m1Fip,m2Fip,m3Fip,m4Fip,wFip;
reg cyc1;

//---------------------------------------------------------
// Register file.
//---------------------------------------------------------

wire [63:0] nxt_a, nxt_b, nxt_c;

Raptor64_regfile u5
(
	.clk(clk),
	.advanceR(advanceR),
	.advanceW(advanceW),
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

always @(posedge clk)
if (rst_i) begin
	bte_o <= 2'b00;
	cti_o <= 3'b000;
	cyc_o <= 1'b0;
	stb_o <= 1'b0;
	we_o <= 1'b0;
	sel_o <= 8'h00;
	adr_o <= 64'd0;
	dat_o <= 64'd0;
	
	nonICacheSeg <= 32'hFFFF_FFFD;
	TBA <= 64'd0;
	pc <= `RESET_VECTOR;
	m1Opcode <= `NOPI;
	m2Opcode <= `NOPI;
	wOpcode <= `NOPI;
	dIR <= `NOP_INSN;
	dRt <= 9'd0;
	tRt <= 9'd0;
	wRt <= 9'd0;
	m1Rt <= 9'd0;
	m2Rt <= 9'd0;
	tData <= 64'd0;
	wData <= 64'd0;
	m1Data <= 64'd0;
	m2Data <= 64'd0;
	m2LdPC <= 1'b0;
	wLdPC <= 1'b0;
	m1IsLoad <= 1'b0;
	m2IsLoad <= 1'b0;
	m1IsStore <= 1'b0;
	m2IsStore <= 1'b0;
	wIsStore <= 1'b0;
	m1IsIO <= 1'b0;
	icaccess <= 1'b0;
	dcaccess <= 1'b0;
	nopI <= 1'b0;
	prev_ihit <= 1'b0;
	dhwxtype <= 2'b00;
	xhwxtype <= 2'b00;
	m1hwxtype <= 2'b00;
	m2hwxtype <= 2'b00;
	whwxtype <= 2'b00;
	wFip <= 1'b0;
	m2Fip <= 1'b0;
	m1Fip <= 1'b0;
	xFip <= 1'b0;
	dFip <= 1'b0;
	dirqf <= 1'b0;
	dpcv <= 1'b0;
	xpcv <= 1'b0;
	m1pcv <= 1'b0;
	m2pcv <= 1'b0;
	wpcv <= 1'b0;
	tick <= 32'd0;
	cstate <= IDLE;
	dImm <= 64'd0;
	AXC <= 4'd0;
	dAXC <= 4'd0;
	xirqf <= 1'b0;
	xextype <= 8'h00;
	xIR <= `NOP_INSN;
	xpc <= 64'd0;
	a <= 64'd0;
	b <= 64'd0;
	imm <= 64'd0;
	xRt <= 9'd0;
	clk_en <= 1'b1;
	StatusEXL <= 1'b1;
	StatusHWI <= 1'b0;
	resetA <= 1'b1;
	mutex_gate <= 64'h0;
	dcache_on <= 1'b0;
	ICacheOn <= 1'b0;
	ibuftag0 <= 64'h0;
	ibuftag1 <= 64'h0;
	m1IsCacheElement <= 1'b0;
	dtinit <= 1'b1;
`ifdef RAS_PREDICTION
	ras_sp <= 6'd63;
`endif
	im <= 1'b1;
	im1 <= 1'b1;
// These must be non-zero in order to produce random numbers
// We set them here in case the user doesn't bother to set them.
	m_z1 <= 32'h01234567;
	m_z2 <= 32'h89ABCDEF;
	m_w1 <= 32'h88888888;
	m_w2 <= 32'h77777777;
	insnkey <= 42'd0;
end
else begin

//---------------------------------------------------------
// Initialize program counters
// Initialize data tags to zero.
//---------------------------------------------------------
if (resetA) begin
	pc <= `RESET_VECTOR;
	adr_o[14:6] <= adr_o[14:6]+9'd1;
	if (adr_o[14:6]==9'h1FF) begin
		dtinit <= 1'b0;
		resetA <= 1'b0;
	end
end

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

//---------------------------------------------------------
// TRAILER:
// - placeholder to allow the use of synchronous register
//   memory
//---------------------------------------------------------
if (advanceT) begin
	tRt <= 9'd0;
	tData <= 64'd0;
end

//---------------------------------------------------------
// WRITEBACK:
// - update the register file with results
// - record exception address and type
// - jump to exception handler routine (below)
//---------------------------------------------------------
if (advanceW) begin
	textype <= wextype;
	wextype <= `EX_NON;
	if (wextype==`EX_IRQ)
		$display("wextype=IRQ");
	tRt <= wRt;
	tData <= wData;
	if (wRt!=5'd0)
		$display("Writing regfile[%d:%d] with %h", wRt[8:5],wRt[4:0], wData);
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
	endcase
	wRt <= 9'd0;
	wData <= 64'd0;
	wOpcode <= `NOPI;
	wIsStore <= 1'b0;
	wLdPC <= 1'b0;
	if (|whwxtype) begin
		dhwxtype <= 2'b00;
		xhwxtype <= 2'b00;
		m1hwxtype <= 2'b00;
		m2hwxtype <= 2'b00;
		whwxtype <= 2'b00;
	end
	clk_en <= 1'b1;
	if (wclkoff)
		clk_en <= 1'b0;
	wclkoff <= 1'b0;
	m1clkoff <= 1'b0;
	m2clkoff <= 1'b0;
	if (wFip) begin
		wFip <= 1'b0;
		m2Fip <= 1'b0;
		m1Fip <= 1'b0;
		xFip <= 1'b0;
		dFip <= 1'b0;
	end
end

//---------------------------------------------------------
// MEMORY:
//---------------------------------------------------------
if (advanceM2) begin
	wIsStore <= m2IsStore;
	wIR <= m2IR;
	wOpcode <= m2Opcode;
	wFunc <= m2Func;
	wData <= m2Data;
	whwxtype <= m2hwxtype;
	wextype <= (m2IsLoad|m2IsStore)&err_i ? `EX_DBERR : m2extype;
	if (m2extype==`EX_IRQ)
		$display("m2extype=IRQ");
	wRt <= m2Rt;
	wpc <= m2pc;
	wpcv <= m2pcv;
	wclkoff <= m2clkoff;
	wFip <= m2Fip;
	wLdPC <= m2LdPC;
	
	m2Rt <= 9'd0;
	m2Opcode <= `NOPI;
	m2IsLoad <= 1'b0;
	m2IsStore <= 1'b0;
	m2IsCnt <= 1'b0;
	m2Func <= 7'd0;
	m2Addr <= 64'd0;
	m2Data <= 64'd0;
	m2clkoff <= 1'b0;
	m2pc <= 64'd0;
	m2extype <= `EX_NON;
	m2LdPC <= 1'b0;
	if ((m2IsLoad|m2IsStore)&err_i)
		errorAddress <= adr_o;
	if (m2extype==`EX_NON) begin
		case(m2Opcode)
		`MISC:
			if (m2Func==`SYSJMP || m2Func==`SYSCALL)
				begin
					cyc_o <= 1'b0;
					stb_o <= 1'b0;
					sel_o <= 8'h00;
					wData <= {dat_i[63:2],2'b00};
				end
		`SH,`SC,`SB,`SW,`SWC,`SM,`SF,`SFD,`SSH,`SSW,`SP,`SFP,`SFDP:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				we_o <= 1'b0;
				sel_o <= 4'h0;
			end
		`LH,`LF,`LSH,`LFP:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				wData <= sel_o[7] ? {{32{dat_i[63]}},dat_i[63:32]}:{{32{dat_i[31]}},dat_i[31: 0]};
			end
		`LW,`LWR,`LM,`LFD,`LSW,`LP,`LFDP:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				wData <= dat_i;
			end
		`LHU:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				wData <= sel_o[7] ? dat_i[63:32] : dat_i[31: 0];
			end
		`LC:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				case(sel_o)
				8'b00000011:	wData <= {{48{dat_i[15]}},dat_i[15: 0]};
				8'b00001100:	wData <= {{48{dat_i[31]}},dat_i[31:16]};
				8'b00110000:	wData <= {{48{dat_i[47]}},dat_i[47:32]};
				8'b11000000:	wData <= {{48{dat_i[63]}},dat_i[63:48]};
				default:	wData <= 64'hDEADDEADDEADDEAD;			
				endcase
			end
		`LCU:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				case(sel_o)
				8'b00000011:	wData <= dat_i[15: 0];
				8'b00001100:	wData <= dat_i[31:16];
				8'b00110000:	wData <= dat_i[47:32];
				8'b11000000:	wData <= dat_i[63:48];
				default:	wData <= 64'hDEADDEADDEADDEAD;			
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
	end
end

wrhit <= 1'b0;
//---------------------------------------------------------
// MEMORY:
// - I/O instructions are finished
// - store instructions are started
// - missed loads are started
// On a data cache hit for a load, the load is essentially
// finished in this stage. We switch the opcode to 'NOPI'
// to cause the pipeline to advance as if a NOPs were
// present.
//---------------------------------------------------------
if (advanceM1) begin
	m2IR <= m1IR;
	m2Opcode <= m1Opcode;
	m2Func <= m1Func;
	m2IsLoad <= m1IsLoad;
	m2IsStore <= m1IsStore;
	m2IsCnt <= m1IsCnt;
	m2Func <= m1Func;
	m2Addr <= pea;
	m2Data <= m1Data;
	m2hwxtype <= m1hwxtype;
	m2extype <= m1IsIO & err_i ? `EX_DBERR : m1extype;
	if (m1extype==`EX_IRQ)
		$display("m1extype=IRQ");
	m2Rt <= m1Rt;
	m2pc <= m1pc;
	m2pcv <= m1pcv;
	m2clkoff <= m1clkoff;
	m2Fip <= m1Fip;

	m1Rt <= 9'd0;
	m1IsLoad <= 1'b0;
	m1IsStore <= 1'b0;
	m1IsCnt <= 1'b0;
	m1IsIO <= 1'b0;
	m1Opcode <= `NOPI;
	m1Func <= 7'd0;
	m1Data <= 64'd0;
	m1clkoff <= 1'b0;
	m1pc <= 64'd0;
	m1IsCacheElement <= 1'b0;
	m1extype <= `EX_NON;

	if (m1IsIO&err_i)
		errorAddress <= adr_o;

	if (m1extype == `EX_NON) begin
		case(m1Opcode)
		`MISC:
			case(m1Func)
			`SYSJMP,`SYSCALL:
				begin
				m2LdPC <= 1'b1;
				if (!m1IsCacheElement) begin
					cyc_o <= 1'b1;
					stb_o <= 1'b1;
					sel_o <= 8'hFF;
					adr_o <= {pea[63:3],3'b000};
					m2Addr <= {pea[63:3],3'b000};
				end
				else if (dhit) begin
					m2IsLoad <= 1'b0;
					m2Opcode <= `NOPI;
					m2Data <= {cdat[63:2],2'b00};
				end
				end
			endcase
		`INW:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				m2Data <= dat_i;
			end
		`INH:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				m2Data <= sel_o[7] ? {{32{dat_i[63]}},dat_i[63:32]}:{{32{dat_i[31]}},dat_i[31: 0]};
			end
		`INHU:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				m2Data <= sel_o[7] ? dat_i[63:32] : dat_i[31: 0];
			end
		`INCH:
			begin
				cyc_o <= 1'b0;
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
				cyc_o <= 1'b0;
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
				cyc_o <= 1'b0;
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
				cyc_o <= 1'b0;
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
		`OUTW,`OUTH,`OUTC,`OUTB:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				we_o <= 1'b0;
				sel_o <= 8'h00;
			end
		`CACHE:
			case(xIR[29:25])
			`INVIL:	tvalid[ea[12:6]] <= 1'b0;
			default:	;
			endcase
			

		`LW,`LM,`LFD,`LSW,`LP,`LFDP:
			if (!m1IsCacheElement) begin
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				sel_o <= 8'hFF;
				adr_o <= {pea[63:3],3'b000};
				m2Addr <= {pea[63:3],3'b000};
			end
			else if (dhit) begin
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
				adr_o <= {pea[63:3],3'b000};
				m2Addr <= {pea[63:3],3'b000};
			end
			else if (dhit) begin
				m2IsLoad <= 1'b0;
				m2Opcode <= `NOPI;
				m2Data <= cdat;
				rsv_o <= 1'b1;
				resv_address <= pea[63:5];
			end
`endif
		`LH,`LF,`LFP:
			if (!m1IsCacheElement) begin
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				sel_o <= pea[2] ? 8'b11110000 : 8'b00001111;
				adr_o <= {pea[63:2],2'b00};
				m2Addr <= {pea[63:2],2'b00};
			end
			else if (dhit) begin
				m2IsLoad <= 1'b0;
				m2Opcode <= `NOPI;
				if (pea[1])
					m2Data <= {{32{cdat[31]}},cdat[31:0]};
				else
					m2Data <= {{32{cdat[63]}},cdat[63:32]};
			end

		`LHU,`LSH:
			if (!m1IsCacheElement) begin
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				sel_o <= pea[2] ? 8'b11110000 : 8'b00001111;
				adr_o <= {pea[63:2],2'b00};
				m2Addr <= {pea[63:2],2'b00};
			end
			else if (dhit) begin
				m2IsLoad <= 1'b0;
				m2Opcode <= `NOPI;
				if (pea[1])
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
				adr_o <= {pea[63:1],1'b0};
				m2Addr <= {pea[63:1],1'b0};
			end
			else if (dhit) begin
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
				adr_o <= {pea[63:1],1'b0};
				m2Addr <= {pea[63:1],1'b0};
			end
			else if (dhit) begin
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
			else if (dhit) begin
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
			else if (dhit) begin
				m2IsLoad <= 1'b0;
				m2Opcode <= `NOPI;
				case(pea[2:0])
				3'b000:	m2Data <= {56'd0,cdat[ 7: 0]};
				3'b001:	m2Data <= {56'd0,cdat[15: 8]};
				3'b010:	m2Data <= {56'd0,cdat[23:16]};
				3'b011:	m2Data <= {56'd0,cdat[31:23]};
				3'b100:	m2Data <= {56'd0,cdat[39:32]};
				3'b101:	m2Data <= {56'd0,cdat[47:40]};
				3'b110:	m2Data <= {56'd0,cdat[55:48]};
				3'b111:	m2Data <= {56'd0,cdat[63:56]};
				endcase
			end

		`SW,`SM,`SFD,`SSW,`SP,`SFDP:
			begin
				$display("SW/SM");
				m2Addr <= {pea[63:3],3'b000};
				wrhit <= dhit;
`ifdef ADDRESS_RESERVATION
				if (resv_address==pea[63:5])
					resv_address <= 59'd0;
`endif
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				we_o <= 1'b1;
				sel_o <= 8'hFF;
				adr_o <= {pea[63:3],3'b000};
				dat_o <= m1Data;
			end

		`SH,`SF,`SSH,`SFP:
			begin
				wrhit <= dhit;
				m2Addr <= {pea[63:2],2'b00};
`ifdef ADDRESS_RESERVATION
				if (resv_address==pea[63:5])
					resv_address <= 59'd0;
`endif
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				we_o <= 1'b1;
				sel_o <= pea[2] ? 8'b11110000 : 8'b00001111;
				adr_o <= {pea[63:2],2'b00};
				dat_o <= {2{m1Data[31:0]}};
			end

		`SC:
			begin
				$display("Storing char to %h, ea=%h",pea,ea);
				wrhit <= dhit;
				m2Addr <= {pea[63:2],2'b00};
`ifdef ADDRESS_RESERVATION
				if (resv_address==pea[63:5])
					resv_address <= 59'd0;
`endif
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				we_o <= 1'b1;
				case(pea[2:1])
				2'b00:	sel_o <= 8'b00000011;
				2'b01:	sel_o <= 8'b00001100;
				2'b10:	sel_o <= 8'b00110000;
				2'b11:	sel_o <= 8'b11000000;
				endcase
				adr_o <= {pea[63:1],1'b0};
				dat_o <= {4{m1Data[15:0]}};
			end

		`SB:
			begin
				wrhit <= dhit;
				m2Addr <= {pea[63:2],2'b00};
`ifdef ADDRESS_RESERVATION
				if (resv_address==pea[63:5])
					resv_address <= 59'd0;
`endif
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				we_o <= 1'b1;
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
				adr_o <= {pea[63:2],2'b00};
				dat_o <= {8{m1Data[7:0]}};
			end

`ifdef ADDRESS_RESERVATION
		`SWC:
			begin
				rsf <= 1'b0;
				if (resv_address==pea[63:5]) begin
					wrhit <= dhit;
					m2Addr <= {pea[63:3],3'b00};
					cyc_o <= 1'b1;
					stb_o <= 1'b1;
					we_o <= 1'b1;
					sel_o <= 8'hFF;
					adr_o <= {pea[63:3],3'b000};
					dat_o <= m1Data;
					resv_address <= 59'd0;
					rsf <= 1'b1;
				end
				else
					m2Opcode <= `NOPI;
			end
`endif
		endcase
	end
end

//---------------------------------------------------------
// EXECUTE:
// - perform datapath operation
// - perform virtual to physical address translation.
//---------------------------------------------------------
if (advanceX) begin
	m1IR <= xIR;
	m1hwxtype <= xhwxtype;
	m1Fip <= xFip;
	m1extype <= xextype;
	if (xextype==`EX_IRQ)
		$display("xextype=IRQ");
	m1IsLoad <= xIsLoad;
	m1IsStore <= xIsStore;
	m1IsCnt <= xIsCnt;
	m1IsIO <= xIsIO;
	m1Opcode <= xOpcode;
	m1Func <= xFunc;
	m1Rt <= xRt;
	m1Data <= xData;
	m1IsCacheElement <= xisCacheElement;
	if (xOpcode==`MOVZ && !aeqz) begin
		m1Rt <= 9'd0;
		m1Data <= 64'd0;
	end
	if (xOpcode==`MOVNZ && aeqz) begin
		m1Rt <= 9'd0;
		m1Data <= 64'd0;
	end
	m1pc <= xpc;
	m1pcv <= xpcv;
	xRt <= 9'd0;
	a <= 64'd0;
	b <= 64'd0;
	imm <= 64'd0;
	xextype <= `EX_NON;
	if (xOpcode[6:4]!=`IMM) begin
		xIR <= `NOP_INSN;
	end
//	xpc <= 64'd0;
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
		`GRAN:	begin
				rando <= rand;
				m_z1 <= next_m_z1;
				m_z2 <= next_m_z2;
				m_w1 <= next_m_w1;
				m_w2 <= next_m_w2;
				end
		`GRAFD:	begin
				rando <= randfd;
				m_z1 <= next_m_z1;
				m_z2 <= next_m_z2;
				m_w1 <= next_m_w1;
				m_w2 <= next_m_w2;
				end
		`SYSJMP,`SYSCALL:
				begin
				ea <= {TBA[63:12],xIR[15:7],3'b000};
				end
`ifdef TLB
		`TLBP:	ea <= TLBVirtPage;
`endif
		default:	;
		endcase
	`R:
		case(xFunc)
		`MTSPR:
			case(xIR[12:7])
`ifdef TLB
			`PageTableAddr:	PageTableAddr <= a[63:13];
			`BadVAddr:		BadVAddr <= a[63:13];
`endif
			`ASID:			ASID <= a[7:0];
			`EPC:			EPC <= a;
			`TBA:			TBA <= {a[63:12],12'h000};
			`AXC:			AXC <= a[3:0];
			`NON_ICACHE_SEG:	nonICacheSeg <= a[63:32];
			`FPCR:			rm <= a[31:30];
			`IPC:			IPC <= a;
			`SRAND1:		begin
							m_z1 <= a[31:0];
							m_z2 <= a[63:32];
							end
			`SRAND2:		begin
							m_w1 <= a[31:0];
							m_w2 <= a[63:32];
							end
			`INSNKEY:		insnkey <= a[41:0];
			default:	;
			endcase
		`OMG:	mutex_gate[a[5:0]] <= 1'b1;
		`CMG:	mutex_gate[a[5:0]] <= 1'b0;
		`OMGI:	mutex_gate[xIR[12:7]] <= 1'b1;
		`CMGI:	mutex_gate[xIR[12:7]] <= 1'b0;
		default:	;
		endcase
	`CALL:	m1Data <= fnIncPC(xpc);
	`INW:
			begin
			cyc_o <= 1'b1;
			stb_o <= 1'b1;
			sel_o <= 8'hFF;
			adr_o <= {xData[63:3],3'b000};
			end
	`INH,`INHU:
			begin
			cyc_o <= 1'b1;
			stb_o <= 1'b1;
			sel_o <= xData[2] ? 8'b11110000 : 8'b00001111;
			adr_o <= {xData[63:2],2'b00};
			end
	`INCH,`INCU:
			begin
			cyc_o <= 1'b1;
			stb_o <= 1'b1;
			case(xData[2:1])
			2'b00:	sel_o <= 8'b00000011;
			2'b01:	sel_o <= 8'b00001100;
			2'b10:	sel_o <= 8'b00110000;
			2'b11:	sel_o <= 8'b11000000;
			endcase
			adr_o <= {xData[63:1],1'b0};
			end
	`INB,`INBU:
			begin
			cyc_o <= 1'b1;
			stb_o <= 1'b1;
			case(xData[2:0])
			3'b000:	sel_o <= 8'b00000001;
			3'b001:	sel_o <= 8'b00000010;
			3'b010:	sel_o <= 8'b00000100;
			3'b011:	sel_o <= 8'b00001000;
			3'b100:	sel_o <= 8'b00010000;
			3'b101:	sel_o <= 8'b00100000;
			3'b110:	sel_o <= 8'b01000000;
			3'b111:	sel_o <= 8'b10000000;
			endcase
			adr_o <= xData;
			end
	`OUTW:
			begin
			cyc_o <= 1'b1;
			stb_o <= 1'b1;
			we_o <= 1'b1;
			sel_o <= 8'hFF;
			adr_o <= {xData[63:3],3'b000};
			dat_o <= b;
			end
	`OUTH:
			begin
			cyc_o <= 1'b1;
			stb_o <= 1'b1;
			we_o <= 1'b1;
			sel_o <= xData[2] ? 8'b11110000 : 8'b00001111;
			adr_o <= {xData[63:2],2'b00};
			dat_o <= {2{b[31:0]}};
			end
	`OUTC:
			begin
			cyc_o <= 1'b1;
			stb_o <= 1'b1;
			we_o <= 1'b1;
			case(xData[2:1])
			2'b00:	sel_o <= 8'b00000011;
			2'b01:	sel_o <= 8'b00001100;
			2'b10:	sel_o <= 8'b00110000;
			2'b11:	sel_o <= 8'b11000000;
			endcase
			adr_o <= {xData[63:1],1'b0};
			dat_o <= {4{b[15:0]}};
			end
	`OUTB:
			begin
			cyc_o <= 1'b1;
			stb_o <= 1'b1;
			we_o <= 1'b1;
			case(xData[2:0])
			3'b000:	sel_o <= 8'b00000001;
			3'b001:	sel_o <= 8'b00000010;
			3'b010:	sel_o <= 8'b00000100;
			3'b011:	sel_o <= 8'b00001000;
			3'b100:	sel_o <= 8'b00010000;
			3'b101:	sel_o <= 8'b00100000;
			3'b110:	sel_o <= 8'b01000000;
			3'b111:	sel_o <= 8'b10000000;
			endcase
			adr_o <= xData;
			dat_o <= {8{b[7:0]}};
			end
	`LEA:	m1Data <= xData;
	`LB,`LBU,`LC,`LCU,`LH,`LHU,`LW,`LWR,`LF,`LFD,`LM,`LSH,`LSW,`LP,`LFP,`LFDP,
	`SW,`SH,`SC,`SB,`SWC,`SF,`SFD,`SM,`SSH,`SSW,`SP,`SFP,`SFDP:
			begin
			m1Data <= b;
			ea <= xData;
			end
	`SSH:	begin
			case(xRt)
			`SR:	m1Data <= {2{sr}};
			default:	m1Data <= 64'd0;
			endcase
			ea <= xData;
			end
	`CACHE:
			begin
			m1Data <= b;
			ea <= xData;
			case(xIR[29:25])
			`INVIALL:	tvalid <= 128'd0;
			default:	;
			endcase
			end
	`MEMNDX:
			begin
			m1Opcode <= 7'd32+xFunc6;
			case(xFunc6)
			`LEAX:
				begin
				m1Data <= xData;
				end
			default:
				begin
				m1Data <= c;
				ea <= xData;
				end
			endcase
			end
	`DIVSI,`DIVUI:
		if (b==64'd0) begin
			xextype <= `EX_DBZ;
		end
	default:	;
	endcase
`ifdef FLOATING_POINT
	if (xOpcode==`FP) begin
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
end

//---------------------------------------------------------
// RFETCH:
// Register fetch stage
//---------------------------------------------------------
if (advanceR) begin
	xAXC <= dAXC;
	xhwxtype <= dhwxtype;
	xFip <= dFip;
	xextype <= dextype;
	if (dextype==`EX_IRQ)
		$display("dextype=IRQ");
	if (dOpcode==`R && dFunc==`MYST)
		xIR <= nxt_c;
	else
		xIR <= dIR;
	xpc <= dpc;
	xpcv <= dpcv;
	xbranch_taken <= dbranch_taken;
	dbranch_taken <= 1'b0;
	dextype <= `EX_NON;
	if (dOpcode[6:4]!=`IMM && dOpcode!=`LM && dOpcode!=`SM && dOpcode!=`SP && dOpcode!=`LP &&
	dOpcode!=`SFP && dOpcode!=`LFP && dOpcode!=`SFDP && dOpcode!=`LFDP)	// IMM is "sticky"
		dIR <= `NOP_INSN;
	dRa <= 9'd0;
	dRb <= 9'd0;

	a <= nxt_a;
	b <= nxt_b;
	if (dOpcode==`SHFTI)
		b <= {58'd0,dIR[24:19]};
	c <= nxt_c;

	// Set the target register
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

	// Set immediate value
	if (xOpcode[6:4]==`IMM) begin
		imm <= {xIR[38:0],dIR[24:0]};
	end
	else
		case(dOpcode)
		`BTRI:	imm <= {{44{dIR[19]}},dIR[19:0]};
		`BEQI,`BNEI,`BLTI,`BLEI,`BGTI,`BGEI,`BLTUI,`BLEUI,`BGTUI,`BGEUI:
			imm <= {{46{dIR[17]}},dIR[17:0]};
		`ORI:	imm <= {39'h0000000000,dIR[24:0]};
		`XORI:	imm <= {39'h0000000000,dIR[24:0]};
		`RET:	imm <= {39'h00000000,dIR[24:3],3'b000};
		`MEMNDX:	imm <= {{50{dIR[19]}},dIR[19:6]};
		default:	imm <= {{39{dIR[24]}},dIR[24:0]};
		endcase

	if ((((dOpcode==`SM || dOpcode==`LM) && dIR[31:0]!=32'd0)) ||
	((dOpcode==`SP || dOpcode==`LP || dOpcode==`SFP || dOpcode==`LFP || dOpcode==`SFDP || dOpcode==`LFDP) && !dIR[25]))
		dIR <= ndIR;
end

//---------------------------------------------------------
// IFETCH:
// - check for external hardware interrupt
// - fetch instruction
// - increment PC
// - set special register defaults for some instructions
//---------------------------------------------------------
if (advanceI) begin
	dAXC <= AXC;
	dextype <= `EX_NON;
	if (nmi_edge & !StatusHWI & !im1) begin
		$display("*****************");
		$display("NMI edge detected");
		$display("*****************");
		StatusHWI <= 1'b1;
		nmi_edge <= 1'b0;
		dhwxtype <= 2'b01;
		dIR <= `NOP_INSN;
		dextype <= `EX_NMI;
	end
	else if (irq_i & !im & !StatusHWI & !im1) begin
		$display("*****************");
		$display("IRQ detected");
		$display("*****************");
		bu_im <= 1'b0;
		im <= 1'b1;
		StatusHWI <= 1'b1;
		dhwxtype <= 2'b10;
		dIR <= `NOP_INSN;
		dextype <= `EX_IRQ;
	end
	// Are we filling the pipeline with NOP's as a result of a previous
	// hardware interrupt ?
	else if (|dhwxtype|dFip) begin
		dIR <= `NOP_INSN;
	end
`ifdef TLB
	else if (ITLBMiss)
		dIR <= `NOP_INSN;
`endif
	else begin
		if (((iOpcode==`SM || iOpcode==`LM) && insn[31:0]!=32'd0) ||
		((iOpcode==`LP || iOpcode==`SP || iOpcode==`SFP || iOpcode==`LFP || iOpcode==`SFDP || iOpcode==`LFDP) && !insn[25]))
			im1 <= 1'b1;
		else
			im1 <= 1'b0;
		if ((iOpcode==`SP || iOpcode==`LP || iOpcode==`SFP || iOpcode==`LFP || iOpcode==`SFDP || iOpcode==`LFDP) && insn[25]==1'b1) begin
			dIR <= `NOP_INSN;
			pc <= fnIncPC(pc);
		end
		else if ((iOpcode==`SM || iOpcode==`LM) && insn[31:0]==32'd0) begin
			dIR <= `NOP_INSN;
			pc <= fnIncPC(pc);
		end
		else
			dIR <= insn;
`include "insn_dumpsc.v"
	end
	nopI <= 1'b0;
	if (dOpcode[6:4]!=`IMM) begin
		dpc <= pc;
		dpcv <= 1'b1;
	end
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
`ifdef TLB
	if (ITLBMiss) begin
		$display("TLB miss on instruction fetch.");
		CauseCode <= `EX_TLBI;
		StatusEXL <= 1'b1;
		BadVAddr <= pc[63:13];
		pc <= `ITLB_MissHandler;
		EPC <= pc;
	end
	else
`endif
	begin
		dbranch_taken <= 1'b0;
		if ((iOpcode==`SP || iOpcode==`LP || iOpcode==`SFP || iOpcode==`LFP || iOpcode==`SFDP || iOpcode==`LFDP) && !insn[25])
			;
		else if ((iOpcode==`LM || iOpcode==`SM) && insn[31:0]!=32'd0)
			;
		else begin
			if (pc!=64'hC)
				pc <= fnIncPC(pc);
		end
		case(iOpcode)
		`MISC:
			case(iFunc)
			`FIP:	dFip <= 1'b1;
			default:	;
			endcase
		// We predict the return address by storing it in a return address stack
		// during a call instruction, then popping it off the stack in a return
		// instruction. The prediction will not always be correct, if it's wrong
		// it's corrected by the EX stage branching to the right address.
		`CALL:
			begin
`ifdef RAS_PREDICTION
				ras[ras_sp] <= fnIncPC(pc);
				ras_sp <= ras_sp - 6'd1;
`endif
				dbranch_taken <= 1'b1;
				pc <= jmp_tgt;
			end
		`RET:
			begin
`ifdef RAS_PREDICTION
//				$display("predicted return address=%h.%h",{ras[ras_sp + 6'd1][63:4],4'b0000},ras[ras_sp + 6'd1][3:2]);
				pc <= ras[ras_sp + 6'd1];
				ras_sp <= ras_sp + 6'd1;
`endif
			end
		`JMP:
			begin
				dbranch_taken <= 1'b1;
				pc <= jmp_tgt;
			end
		`BTRR:
			case(insn[4:0])
			`BEQ,`BNE,`BLT,`BLE,`BGT,`BGE,`BLTU,`BLEU,`BGTU,`BGEU,`BAND,`BOR,`BRA,`BNR,`BRN,`LOOP:
				if (predict_taken) begin
//					$display("Taking predicted branch: %h",{pc[63:4] + {{42{insn[24]}},insn[24:7]},insn[6:5],2'b00});
					dbranch_taken <= 1'b1;
					pc <= {pc[63:4] + {{42{insn[24]}},insn[24:7]},insn[6:5],2'b00};
				end
			default:	;
			endcase
`ifdef BTB
		`BTRI:
			if (predict_taken) begin
				dbranch_taken <= 1'b1;
				pc <= btb[pc[7:2]];
			end
`endif
		`BEQI,`BNEI,`BLTI,`BLEI,`BGTI,`BGEI,`BLTUI,`BLEUI,`BGTUI,`BGEUI:
			begin
				if (predict_taken) begin
					dbranch_taken <= 1'b1;
					pc <= {pc[63:4] + {{50{insn[29]}},insn[29:20]},insn[19:18],2'b00};
				end
			end
		default:	;
		endcase
	end
end

//`include "RPSTAGE.v"
//---------------------------------------------------------
// EXECUTE (EX')- part two:
// - override the default program counter increment for
//   control flow instructions
// - NOP out the instructions following a branch in the
//   pipeline
//---------------------------------------------------------
if (advanceX) begin
	case(xOpcode)
	`MISC:
		case(xFunc)
		`IRET:
			if (StatusHWI) begin
				StatusHWI <= 1'b0;
				im <= 1'b0;
				pc <= IPC;
				dIR <= `NOP_INSN;
				xIR <= `NOP_INSN;
				xRt <= 9'd0;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
			end
		`ERET:
			if (StatusEXL) begin
				StatusEXL <= 1'b0;
				pc <= EPC;
				dIR <= `NOP_INSN;
				xIR <= `NOP_INSN;
				xRt <= 9'd0;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
			end
		`SYSJMP:
			begin
				StatusEXL <= 1'b1;
				pc <= 64'hC;
				dIR <= `NOP_INSN;
				xIR <= `NOP_INSN;
				xRt <= 9'd0;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
			end
		`SYSCALL:
			begin
				StatusEXL <= 1'b1;
				EPC <= fnIncPC(xpc);
				pc <= 64'hC;
				dIR <= `NOP_INSN;
				xIR <= `NOP_INSN;
				xRt <= 9'd0;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
			end
		default:	;
		endcase
	`R:
		case(xFunc)
		`EXEC:
			begin
				pc <= fnIncPC(xpc);
				dRa <= b[34:30];
				dRb <= b[29:25];
				dRc <= b[24:20];
				dIR <= b;
				xIR <= `NOP_INSN;
				xRt <= 9'd0;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
			end
		default:	;
		endcase
	`BTRR:
		case(xFunc5)
	// BEQ r1,r2,label
		`BEQ,`BNE,`BLT,`BLE,`BGT,`BGE,`BLTU,`BLEU,`BGTU,`BGEU,`BAND,`BOR,`BNR,`LOOP,`BRA,`BRN:
			if (!takb & xbranch_taken) begin
				$display("Taking mispredicted branch %h",fnIncPC(xpc));
				pc <= fnIncPC(xpc);
				dIR <= `NOP_INSN;
				xIR <= `NOP_INSN;
				xRt <= 9'd0;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
			end
			else if (takb & !xbranch_taken) begin
				$display("Taking branch %h.%h",{xpc[63:4] + {{42{xIR[24]}},xIR[24:7]},4'b0000},xIR[6:5]);
				pc[63:4] <= xpc[63:4] + {{42{xIR[24]}},xIR[24:7]};
				pc[3:2] <= xIR[6:5];
				dIR <= `NOP_INSN;
				xIR <= `NOP_INSN;
				xRt <= 9'd0;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
			end
	// BEQ r1,r2,r10
		`BEQR,`BNER,`BLTR,`BLER,`BGTR,`BGER,`BLTUR,`BLEUR,`BGTUR,`BGEUR://,`BANDR,`BORR,`BNRR:
			if (takb) begin
				pc[63:2] <= c[63:2];
				pc[1:0] <= 2'b00;
`ifdef BTB
				btb[xpc[7:2]] <= c;
`endif
				dIR <= `NOP_INSN;
				xIR <= `NOP_INSN;
				xRt <= 9'd0;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
			end
		default:	;
		endcase
	// JMP and CALL change the program counter immediately in the IF stage.
	// There's no work to do here. The pipeline does not need to be cleared.
	`JMP:	;
	`CALL:	;	
	`JAL:
		begin
			pc[63:2] <= a[63:2] + imm[63:2];
			dIR <= `NOP_INSN;
			xIR <= `NOP_INSN;
			xRt <= 9'd0;
			xpcv <= 1'b0;
			dpcv <= 1'b0;
		end

	// Check the pc of the instruction after the RET instruction (the dpc), to
	// see if it's equal to the RET target. If it's the same as the target then
	// we predicted the RET return correctly, so there's nothing to do. Otherwise
	// we need to branch to the RET location.
	`RET:
`ifdef RAS_PREDICTION
		if (dpc[63:2]!=b[63:2]) begin
`else
		begin
`endif
//			$display("returning to: %h.%h", {b[63:4],4'b0},b[3:2]);
			pc[63:2] <= b[63:2];
			dIR <= `NOP_INSN;
			xIR <= `NOP_INSN;
			xRt <= 9'd0;
			xpcv <= 1'b0;
			dpcv <= 1'b0;
		end
//		else
//			$display("RET address %h predicted correctly.", {b[63:4],4'b0},b[3:2]);
	// BEQ r1,#3,r10
	`BTRI:
`ifdef BTB
		if (takb) begin
			if ((xbranch_taken && b!=btb[xpc[7:2]]) ||	// took branch, but not to right target
				!xbranch_taken) begin					// didn't take branch, and were supposed to
				pc[63:2] <= b[63:2];
				pc[1:0] <= 2'b00;
				btb[xpc[7:2]] <= b;
				dIR <= `NOP_INSN;
				xIR <= `NOP_INSN;
				xRt <= 9'd0;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
			end
		end
		else if (xbranch_taken)	begin	// took the branch, and weren't supposed to
			pc <= fnIncPC(xpc);
			dIR <= `NOP_INSN;
			xIR <= `NOP_INSN;
			xRt <= 9'd0;
			xpcv <= 1'b0;
			dpcv <= 1'b0;
		end
`else
		if (takb) begin
			pc[63:2] <= b[63:2];
			pc[1:0] <= 2'b00;
			dIR <= `NOP_INSN;
			xIR <= `NOP_INSN;
			xRt <= 9'd0;
			xpcv <= 1'b0;
			dpcv <= 1'b0;
		end
`endif
	// BEQI r1,#3,label
	`BEQI,`BNEI,`BLTI,`BLEI,`BGTI,`BGEI,`BLTUI,`BLEUI,`BGTUI,`BGEUI:
		if (takb) begin
			if (!xbranch_taken) begin
				pc[63:4] <= xpc[63:4] + {{50{xIR[29]}},xIR[29:20]};
				pc[3:2] <= xIR[19:18];
				dIR <= `NOP_INSN;
				xIR <= `NOP_INSN;
				xRt <= 9'd0;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
			end
		end
		else begin
			if (xbranch_taken) begin
				$display("Taking mispredicted branch %h",fnIncPC(xpc));
				pc <= fnIncPC(xpc);
				dIR <= `NOP_INSN;
				xIR <= `NOP_INSN;
				xRt <= 9'd0;
				xpcv <= 1'b0;
				dpcv <= 1'b0;
			end
		end
	`TRAPcc,`TRAPcci:
		if (takb) begin
			StatusEXL <= 1'b1;
			CauseCode <= `EX_TRAP;
			xextype <= `EX_TRAP;
			pc <= 64'hC;
			dIR <= `NOP_INSN;
			xIR <= `NOP_INSN;
			xRt <= 9'd0;
			xpcv <= 1'b0;
			dpcv <= 1'b0;
		end
	default:	;
	endcase
	
	if (dbz_error) begin
		$display("Divide by zero error");
		CauseCode <= `EX_DBZ;
		xextype <= `EX_DBZ;
		StatusEXL <= 1'b1;
		pc <= 64'hC;
		dIR <= `NOP_INSN;
		xIR <= `NOP_INSN;
		xRt <= 9'd0;
		xpcv <= 1'b0;
		dpcv <= 1'b0;
	end
	else if (ovr_error) begin
		$display("Overflow error");
		CauseCode <= `EX_OFL;
		xextype <= `EX_OFL;
		StatusEXL <= 1'b1;
		pc <= 64'hC;
		dIR <= `NOP_INSN;
		xIR <= `NOP_INSN;
		xRt <= 9'd0;
		xpcv <= 1'b0;
		dpcv <= 1'b0;
	end
	else if (priv_violation) begin
		$display("Priviledge violation");
		CauseCode <= `EX_PRIV;
		xextype <= `EX_PRIV;
		StatusEXL <= 1'b1;
		pc <= 64'hC;
		dIR <= `NOP_INSN;
		xIR <= `NOP_INSN;
		xRt <= 9'd0;
		xpcv <= 1'b0;
		dpcv <= 1'b0;
	end
end

//---------------------------------------------------------
// MEMORY1 (M1') - part two:
// Check for a TLB miss.
//---------------------------------------------------------
`ifdef TLB
if (advanceM1) begin
	if (m1IsLoad|m1IsStore) begin
		if (DTLBMiss) begin
			$display("DTLB miss on address: %h",ea);
			m1extype <= `EX_TLBD;
			CauseCode <= `EX_TLBD;
			StatusEXL <= 1'b1;
			BadVAddr <= ea[63:13];
			EPC <= m1pc;
			pc <= `DTLB_MissHandler;
			m1Opcode <= `NOPI;
			m1Rt <= 9'd0;
			xIR <= `NOP_INSN;
			xRt <= 9'd0;
			dIR <= `NOP_INSN;
			m1pcv <= 1'b0;
			xpcv <= 1'b0;
			dpcv <= 1'b0;
		end
	end
end
`endif

//---------------------------------------------------------
// MEMORY2 (M2')
//---------------------------------------------------------
if (advanceM2) begin
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
if (advanceW) begin
	case(wextype)
	`EX_NON:	;
	`EX_RST:
		begin
		pc <= `RESET_VECTOR;
		end
	`EX_NMI,`EX_IRQ,`EX_DBERR:
		begin
		$display("Stuffing SYSJMP");
		xIR <= {`MISC,19'd0,wextype,`SYSJMP};
		pc <= 64'hC;
		case(1'b1)
		wpcv:	IPC <= wpc;
		m2pcv:	IPC <= m2pc;
		m1pcv: 	IPC <= m1pc;
		xpcv:	IPC <= xpc;
		dpcv:	IPC <= dpc;
		default:	IPC <= pc;
		endcase
		end
	`EX_OFL,`EX_DBZ,`EX_PRIV,`EX_TRAP:
		begin
		xIR <= {`MISC,19'd0,wextype,`SYSJMP};
		pc <= 64'hC;
		EPC <= fnIncPC(wpc);
		end
	default:
		begin
		xIR <= {`MISC,19'd0,wextype,`SYSJMP};
		pc <= 64'hC;
		EPC <= fnIncPC(wpc);
		end
	endcase
	if (wLdPC) begin
		$display("Loading PC");
		pc <= wData;
	end
end


//---------------------------------------------------------
// Trailer (TR')
// - no exceptions
//---------------------------------------------------------
if (advanceT) begin
end


//---------------------------------------------------------
// Cache loader
//---------------------------------------------------------
if (rst_i) begin
	cstate <= IDLE;
end
else begin
case(cstate)
IDLE:
	if (triggerDCacheLoad) begin
		dcaccess <= 1'b1;
		bte_o <= 2'b00;			// linear burst
		cti_o <= 3'b010;		// burst access
		bl_o <= 5'd7;
		cyc_o <= 1'b1;
		stb_o <= 1'b1;
		adr_o <= {pea[63:6],6'h00};
		cstate <= DCACT;
	end
	else if (triggerICacheLoad) begin
		icaccess <= 1'b1;
		bte_o <= 2'b00;			// linear burst
		cti_o <= 3'b010;		// burst access
		cyc_o <= 1'b1;
		stb_o <= 1'b1;
		if (ICacheAct) begin
			bl_o <= 5'd7;
			adr_o <= {ppc[63:6],6'h00};
			cstate <= ICACT1;
		end
		else begin
			bl_o <= 5'd1;
			adr_o <= {ppc[63:4],4'b0000};
			cstate <= ICACT2;
		end
	end
// WISHBONE burst accesses
//
ICACT1:
	if (ack_i|err_i) begin
		adr_o[5:3] <= adr_o[5:3] + 3'd1;
		if (adr_o[5:3]==3'd6)
			cti_o <= 3'b111;	// Last cycle ahead
		else if (adr_o[5:3]==3'd7) begin
			cti_o <= 3'b000;	// back to non-burst mode
			cyc_o <= 1'b0;
			stb_o <= 1'b0;
			tmem[adr_o[12:6]] <= {1'b1,adr_o[63:13]};	// This will cause ihit to go high
			tvalid[adr_o[12:6]] <= 1'b1;
			icaccess <= 1'b0;
			cstate <= IDLE;
		end
	end
//SYSCALL 509:	00_00000000_00000000_00000000_11111110_10010111
ICACT2:
	if (ack_i|err_i) begin
		adr_o <= adr_o + 64'd8;
		if (adr_o[3]==1'b0) begin
			cti_o <= 3'b111;	// Last cycle ahead
			tmpbuf <= err_i ? bevect[63:0] : dat_i;
		end
		else begin
			if (tick[0]) begin
				insnbuf0 <= {err_i ? bevect[127:64] : dat_i,tmpbuf};
				ibuftag0 <= adr_o[63:4];
			end
			else begin
				insnbuf1 <= {err_i ? bevect[127:64] : dat_i,tmpbuf};
				ibuftag1 <= adr_o[63:4];
			end
			cti_o <= 3'b000;	// back to non-burst mode
			cyc_o <= 1'b0;
			stb_o <= 1'b0;
			icaccess <= 1'b0;
			cstate <= IDLE;
		end
	end

DCACT:
	if (ack_i|err_i) begin
		adr_o[5:3] <= adr_o[5:3] + 3'd1;
		if (adr_o[5:3]==3'h6)
			cti_o <= 3'b111;	// Last cycle ahead
		if (adr_o[5:3]==3'h7) begin
			cti_o <= 3'b000;	// back to non-burst mode
			cyc_o <= 1'b0;
			stb_o <= 1'b0;
			dcaccess <= 1'b0;
			cstate <= IDLE;
		end
	end

endcase
end

end

endmodule
