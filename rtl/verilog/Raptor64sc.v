// ============================================================================
// (C) 2012 Robert Finch
// All Rights Reserved.
// robfinch<remove>@opencores.org
//
// Raptor64.v
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
`define RESET_VECTOR	64'hFFFF_FFFF_FFFF_FFF0
`define NMI_VECTOR		64'hFFFF_FFFF_FFFF_FFE0
`define IRQ_VECTOR		64'hFFFF_FFFF_FFFF_FFD0
`define TRAP_VECTOR		64'h0000_0000_0000_0000

`define TLBMissPage		52'hFFFF_FFFF_FFFF_F
`define ITLB_MissHandler	64'hFFFF_FFFF_FFFF_FFC0
`define DTLB_MissHandler	64'hFFFF_FFFF_FFFF_FFB0

`define GEN_TRAP_OFFSET		13'h0200
`define DBZ_TRAP_OFFSET		13'h0050
`define OFL_TRAP_OFFSET		13'h0070
`define PRIV_OFFSET			13'h0080

`define EX_NON		8'd0
`define EX_RST		8'd1
`define EX_NMI		8'd2
`define EX_IRQ		8'd3
`define EX_TRAP		8'd4
`define EX_PRIV		8'd5	// priviledge violation
`define EX_OFL		8'd16	// overflow
`define EX_DBZ		8'd17	// divide by zero
`define EX_TLBI		8'd19	// TLB exception - ifetch
`define EX_TLBD		8'd20	// TLB exception - data

`define EXCEPT_Int		5'd00
`define EXCEPT_Mod		5'd01	// TLB modification
`define EXCEPT_TLBL		5'd02	// TLB exception - load or ifetch
`define EXCEPT_TLBS		5'd03	// TLB exception - store
`define EXCEPT_AdEL		5'd04	// Address error - load or ifetch
`define EXCEPT_AdES		5'd05	// Address error - store
`define EXCEPT_IBE		5'd06	// Bus Error - instruction fetch
`define EXCEPT_DBE		5'd07	// Bus Error - load or store
`define EXCEPT_Sys		5'd08
`define EXCEPT_Bp		5'd09
`define EXCEPT_RI		5'd10	// reserved instruction
`define EXCEPT_CpU		5'd11	// Coprocessor unusable
`define EXCEPT_Ov		5'd12	// Integer Overflow
`define EXCEPT_Tr		5'd13	// Trap exception
// 14-22 Reserved
`define EXCEPT_WATCH	5'd23
`define EXCEPT_MCheck	5'd24	// Machine check
// 25-31 Reserved


`define MISC	7'd0
`define		BRK		7'd0
`define		IRQ		7'd1
`define     FIP		7'd20
`define		IRET	7'd32
`define		ERET	7'd33
`define 	WAIT	7'd40
`define		TLBP	7'd49
`define     TLBR	7'd50
`define     TLBWI	7'd51
`define     TLBWR	7'd52
`define		CLI		7'd64
`define 	SEI		7'd65
`define R		7'd1
`define 	COM		7'd4
`define		NOT		7'd5
`define		NEG		7'd6
`define		ABS		7'd7
`define		SWAP	7'd13
`define		CTLZ	7'd16
`define		CTLO	7'd17
`define		CTPOP	7'd18
`define		SEXT8	7'd19
`define		SEXT16	7'd20
`define		SEXT32	7'd21
`define		SQRT	7'd24
`define		REDOR	7'd30
`define		REDAND	7'd31
`define     MFSPR	7'd40
`define     MTSPR	7'd41
`define         TLBIndex    	6'd01
`define         TLBRandom		6'd02
`define         PageTableAddr	6'd04
`define         BadVAddr        6'd08
`define         TLBPhysPage0	6'd10
`define         TLBPhysPage1	6'd11
`define         TLBVirtPage		6'd12
`define			TLBPageMask		6'd13
`define			TLBASID			6'd14
`define         ASID			6'd15
`define			Wired			6'd16
`define         EP0             6'd17
`define         EP1             6'd18
`define         EP2             6'd19
`define         EP3             6'd20
`define         AXC             6'd21
`define			Tick			6'd22
`define 		EPC				6'd23
`define			CauseCode		6'd24
`define			TBA				6'd25
`define		OMG		7'd50
`define 	CMG		7'd51
`define		OMGI	7'd52
`define 	CMGI	7'd53
`define		MFTBA	7'd58
`define		MTTBA	7'd59
`define RR	7'd2
`define 	ADD		7'd2
`define		ADDU	7'd3
`define 	SUB		7'd4
`define 	SUBU	7'd5
`define 	CMP		7'd6
`define 	CMPU	7'd7
`define 	AND		7'd8
`define 	OR		7'd9
`define 	XOR		7'd10
`define 	ANDC	7'd11
`define		NAND	7'd12
`define		NOR		7'd13
`define 	XNOR	7'd14
`define		ORC		7'd15
`define		MIN		7'd20
`define		MAX		7'd21
`define		MULU	7'd24
`define		MULS	7'd25
`define		DIVU	7'd26
`define 	DIVS	7'd27
`define		MOD		7'd28
`define		MOVZ	7'd30
`define		MOVNZ	7'd31

`define 	SHL		7'd40
`define 	SHRU	7'd41
`define		ROL		7'd42
`define		ROR		7'd43
`define		SHR		7'd44
`define 	ROLAM	7'd45

`define		NOP		7'd60

`define 	SLT		7'd96
`define 	SLE		7'd97
`define 	SGT		7'd98
`define 	SGE		7'd99
`define 	SLTU	7'd100
`define 	SLEU	7'd101
`define 	SGTU	7'd102
`define 	SGEU	7'd103
`define 	SEQ		7'd104
`define 	SNE		7'd105

`define     BCD_ADD	7'd110
`define     BCD_SUB 7'd111

`define SHFTI	7'd3
`define	SHLI		7'd0
`define SHRUI		7'd1
`define ROLI		7'd2
`define SHRI		7'd3
`define RORI		7'd4
`define ROLAMI		7'd5
`define BFINS		7'd8
`define BFSET		7'd9
`define BFCLR		7'd10
`define BFCHG		7'd11

`define ADDI	7'd4
`define ADDUI	7'd5
`define SUBI	7'd6
`define SUBUI	7'd7
`define CMPI	7'd8
`define CMPUI	7'd9
`define ANDI	7'd10
`define ORI		7'd11
`define XORI	7'd12

`define MULUI	7'd13
`define MULSI	7'd14
`define DIVUI	7'd15
`define DIVSI	7'd16

`define TRAPcc	7'd17
`define		TEQ		7'd0
`define		TNE		7'd1
`define		TLT		7'd2
`define		TGE		7'd3
`define		TLE		7'd4
`define		TGT		7'd5
`define		TLTU	7'd6
`define		TGEU	7'd7
`define		TLEU	7'd8
`define		TGTU	7'd9
`define		TRAP	7'd10
`define		TRN		7'd11
`define TRAPcci	7'd18
`define		TEQI	5'd0
`define		TNEI	5'd1
`define		TLTI	5'd2
`define		TGEI	5'd3
`define		TLEI	5'd4
`define		TGTI	5'd5
`define		TLTUI	5'd6
`define		TGEUI	5'd7
`define		TLEUI	5'd8
`define		TGTUI	5'd9
`define		TRAI	5'd10
`define		TRNI	5'd11
// SETLO=20 to 23
`define SETLO	7'b00101xx
`define CALL	7'd24
`define JMP		7'd25
`define JAL		7'd26
`define RET		7'd27
// SETLO=28 to 31
`define SETHI	7'b00111xx
`define LB		7'd32
`define LC		7'd33
`define LH		7'd34
`define LW		7'd35
`define LP		7'd36
`define LBU		7'd37
`define LCU		7'd38
`define LHU		7'd39
`define LSH		7'd40
`define LSW		7'd41
`define LF		7'd42
`define LFD		7'd43
`define LFP		7'd44
`define LFDP	7'd45
`define LWR		7'd46
`define LDONE	7'd47

`define SB		7'd48
`define SC		7'd49
`define SH		7'd50
`define SW		7'd51
`define SP		7'd52
`define MEMNDX	7'd53
`define SSH		7'd56
`define SSW		7'd57
`define SF		7'd58
`define SFD		7'd59
`define SFP		7'd60
`define SFDP	7'd61
`define SWC		7'd62

`define INB		7'd64
`define INCH	7'd65
`define INH		7'd66
`define INW		7'd67
`define INBU	7'd68
`define INCU	7'd69
`define INHU	7'd70
`define OUTB	7'd72
`define OUTC	7'd73
`define OUTH	7'd74
`define OUTW	7'd75

`define BLTI	7'd80
`define BGEI	7'd81
`define BLEI	7'd82
`define BGTI	7'd83
`define BLTUI	7'd84
`define BGEUI	7'd85
`define BLEUI	7'd86
`define BGTUI	7'd87
`define BEQI	7'd88
`define BNEI	7'd89
`define BRAI	7'd90
`define BRNI	7'd91

`define BTRI	7'd94
`define 	BLTRI	5'd0
`define 	BGERI	5'd1
`define 	BLERI	5'd2
`define 	BGTRI	5'd3
`define 	BLTURI	5'd4
`define 	BGEURI	5'd5
`define 	BLEURI	5'd6
`define 	BGTURI	5'd7
`define 	BEQRI	5'd8
`define 	BNERI	5'd9
`define		BRARI	5'd10
`define		BRNRI	5'd11
`define		BANDRI	5'd12
`define		BORRI	5'd13
`define BTRR	7'd95
`define 	BLT		5'd0
`define 	BGE		5'd1
`define 	BLE		5'd2
`define 	BGT		5'd3
`define 	BLTU	5'd4
`define 	BGEU	5'd5
`define 	BLEU	5'd6
`define 	BGTU	5'd7
`define 	BEQ		5'd8
`define 	BNE		5'd9
`define		BRA		5'd10
`define		BRN		5'd11
`define		BAND	5'd12
`define		BOR		5'd13
`define		BNR		5'd14
`define 	BLTR	5'd16
`define 	BGER	5'd17
`define 	BLER	5'd18
`define 	BGTR	5'd19
`define 	BLTUR	5'd20
`define 	BGEUR	5'd21
`define 	BLEUR	5'd22
`define 	BGTUR	5'd23
`define 	BEQR	5'd24
`define 	BNER	5'd25
`define		BRAR	5'd26
`define		BRNR	5'd27


`define SLTI	7'd96
`define SLEI	7'd97
`define SGTI	7'd98
`define SGEI	7'd99
`define SLTUI	7'd100
`define SLEUI	7'd101
`define SGTUI	7'd102
`define SGEUI	7'd103
`define SEQI	7'd104
`define SNEI	7'd105

`define FPLOO	7'd109
`define FPZL	7'd110
`define NOPI	7'd111

`define IMM		3'd7

`define NOP_INSN	42'b1101111_000_00000000_00000000_00000000_00000000

module Raptor64sc(rst_i, clk_i, nmi_i, irq_i, bte_o, cti_o, bl_o,
	cyc_o, stb_o, ack_i, we_o, sel_o, rsv_o, adr_o, dat_i, dat_o, sys_adv, sys_adr
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

reg resetA;
reg im,bu_im;			// interrupt mask
reg [1:0] rm;		// fp rounding mode
reg [41:0] dIR;
reg [41:0] xIR;
reg [63:0] pc;
reg [63:0] ErrorEPC,EPC,IPC;
reg [63:0] dpc,m1pc,m2pc,m3pc,m4pc,wpc;
reg dpcv,xpcv,m1pcv,m2pcv,m3pcv,m4pcv,wpcv;	// PC valid indicators
reg [63:0] xpc;
reg [63:0] tlbra;		// return address for a TLB exception
reg [8:0] dRa,dRb,dRc;
reg [8:0] wRt,mRt,m1Rt,m2Rt,m3Rt,m4Rt,tRt,dRt;
reg [8:0] xRt;
reg [63:0] dImm;
reg [63:0] ea;
reg [63:0] iadr_o;
reg [31:0] idat;
reg [4:0] cstate;
reg dbranch_taken,xbranch_taken;
reg [63:0] mutex_gate;
reg [63:0] TBA;
reg [1:0] dhwxtype,xhwxtype,m1hwxtype,m2hwxtype,m3hwxtype,m4hwxtype,whwxtype;
reg [3:0] AXC,dAXC,xAXC;
reg dtinit;

//reg wr_icache;
reg dccyc;
wire [63:0] cdat;
reg [63:0] wr_addr;
reg [41:0] insn;
wire [63:0] rfoa,rfob;
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

wire KernelMode = StatusEXL;

//-----------------------------------------------------------------------------
// TLB
//-----------------------------------------------------------------------------

reg [24:13] TLBPageMask;
reg [63:13] TLBVirtPage;
reg [63:13] TLBPhysPage0;
reg [63:13] TLBPhysPage1;
reg [7:0] TLBASID;
reg TLBG;
reg TLBD;
reg TLBValid;
reg [63:0] Index;
reg [3:0] Random;
reg [3:0] Wired;
reg [15:0] IMatch,DMatch;

reg [4:0] m;
reg [3:0] i;
reg [24:13] ITLBPageMask [15:0];
reg [63:13] ITLBVirtPage [15:0];
reg [63:13] ITLBPhysPage0 [15:0];
reg [63:13] ITLBPhysPage1 [15:0];
reg [15:0] ITLBG;
reg [15:0] ITLBD;
reg [7:0] ITLBASID [15:0];
reg [15:0] ITLBValid;
initial begin
	for (n = 0; n < 16; n = n + 1)
	begin
		ITLBPageMask[n] = 0;
		ITLBVirtPage[n] = 0;
		ITLBPhysPage0[n] = 0;
		ITLBPhysPage1[n] = 0;
		ITLBG[n] = 0;
		ITLBASID[n] = 0;
		ITLBValid[n] = 0;
	end
end
always @*
for (n = 0; n < 16; n = n + 1)
	IMatch[n] = ((pc[63:13]|ITLBPageMask[n])==(ITLBVirtPage[n]|ITLBPageMask[n])) &&
				((ITLBASID[n]==ASID) || ITLBG[n]) &&
				ITLBValid[n];
always @(IMatch)
if (IMatch[0]) m <= 5'd0;
else if (IMatch[1]) m <= 5'd1;
else if (IMatch[2]) m <= 5'd2;
else if (IMatch[3]) m <= 5'd3;
else if (IMatch[4]) m <= 5'd4;
else if (IMatch[5]) m <= 5'd5;
else if (IMatch[6]) m <= 5'd6;
else if (IMatch[7]) m <= 5'd7;
else if (IMatch[8]) m <= 5'd8;
else if (IMatch[9]) m <= 5'd9;
else if (IMatch[10]) m <= 5'd10;
else if (IMatch[11]) m <= 5'd11;
else if (IMatch[12]) m <= 5'd12;
else if (IMatch[13]) m <= 5'd13;
else if (IMatch[14]) m <= 5'd14;
else if (IMatch[15]) m <= 5'd15;
else m <= 5'd31;

wire ioddpage = |({ITLBPageMask[q]+19'd1,13'd0}&pc);
wire [63:13] IPFN = ioddpage ? ITLBPhysPage1[q] : ITLBPhysPage0[q];

wire unmappedArea = pc[63:52]==12'hFFD || pc[63:52]==12'hFFE || pc[63:52]==12'hFFF;
wire [63:0] ppc;
wire ITLBMiss = !unmappedArea & m[4];

assign ppc[63:13] = unmappedArea ? pc[63:13] : m[4] ? `TLBMissPage: IPFN;
assign ppc[12:0] = pc[12:0];

reg [4:0] q;
always @(ea)
for (n = 0; n < 16; n = n + 1)
	DMatch[n] = ((ea[63:13]|ITLBPageMask[n])==(ITLBVirtPage[n]|ITLBPageMask[n])) &&
				((ITLBASID[n]==ASID) || ITLBG[n]) &&
				ITLBValid[n];
always @(DMatch)
if (DMatch[0]) q <= 5'd0;
else if (DMatch[1]) q <= 5'd1;
else if (DMatch[2]) q <= 5'd2;
else if (DMatch[3]) q <= 5'd3;
else if (DMatch[4]) q <= 5'd4;
else if (DMatch[5]) q <= 5'd5;
else if (DMatch[6]) q <= 5'd6;
else if (DMatch[7]) q <= 5'd7;
else if (DMatch[8]) q <= 5'd8;
else if (DMatch[9]) q <= 5'd9;
else if (DMatch[10]) q <= 5'd10;
else if (DMatch[11]) q <= 5'd11;
else if (DMatch[12]) q <= 5'd12;
else if (DMatch[13]) q <= 5'd13;
else if (DMatch[14]) q <= 5'd14;
else if (DMatch[15]) q <= 5'd15;
else q <= 5'd31;

wire doddpage = |({ITLBPageMask[q]+19'd1,13'd0}&ea);
wire [63:13] DPFN = doddpage ? ITLBPhysPage1[q] : ITLBPhysPage0[q];

wire unmappedDataArea = ea[63:52]==12'hFFD || ea[63:52]==12'hFFE || ea[63:52]==12'hFFF;
wire DTLBMiss = !unmappedDataArea & q[4];
wire m1UnmappedDataArea = pea[63:13]>=12'hFFD;

wire [63:0] pea;
assign pea[63:13] = unmappedDataArea ? ea[63:13] : q[4] ? `TLBMissPage: DPFN;
assign pea[12:0] = ea[12:0];
wire dram_bus = !pea[63];
wire m2_dram_bus = !m2Addr[63];
wire m3_dram_bus = !m3Addr[63];
wire m4_dram_bus = !m4Addr[63];

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
// Instruction Cache
// 8kB
// 
//-----------------------------------------------------------------------------
reg icaccess;

//Raptor64_icache_ram_x32 u1
//(
//	.clk(clk),
//	.wr(icaccess & ack_i),
//	.adr_i(adr_o[12:0]),
//	.dat_i(dat_i),
//	.pc(pc),
//	.insn(insn)
//);
wire [127:0] insnbundle;

Raptor64_icache_ram u1
(
	.clka(clk), // input clka
	.wea(icaccess & ack_i), // input [0 : 0] wea
	.addra(adr_o[12:3]), // input [9 : 0] addra
	.dina(dat_i), // input [63 : 0] dina
	.clkb(~clk), // input clkb
	.addrb(pc[12:4]), // input [9 : 0] addrb
	.doutb(insnbundle) // output [63 : 0] doutb
);

always @(pc or insnbundle)
begin
	case(pc[3:2])
	2'd0:	insn <= insnbundle[ 41: 0];
	2'd1:	insn <= insnbundle[ 83:42];
	2'd2:	insn <= insnbundle[125:84];
	2'd3:	insn <= 42'h37800000000;	// NOP instruction
	endcase
end


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

// cache RAM 16Kb
/*Raptor64_dcache_ram u10
(
	.clk(clk),
	.wr(dcaccess ? wr_dcache : wrhit ? (dram_bus ? wr_en: we_o): 1'b0),
	.sel(dcaccess ? 4'b1111 : wrhit ? ~wr_mask : 4'b0000),
	.wadr(dcaccess ? dadr_o[13:2] : wr_addr[13:2]),
	.i(dcaccess ? ddat : wr_data),
	.radr(pea[13:3]),
	.o(cdat)
);
*/
Raptor64_dcache_ram u10
(
	.clka(clk), // input clka
	.ena(1'b1),
	.wea(dcaccess ? {8{ack_i}} : wrhit ? sel_o : 8'h00), // input [7 : 0] wea
	.addra(adr_o[14:3]), // input [11 : 0] addra
	.dina(dcaccess ? dat_i : dat_o), // input [63 : 0] dina

	.clkb(~clk), // input clkb
	.addrb(adr_o[14:3]), // input [11 : 0] addrb
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
// tag ram
//syncRam512x64_1rw1r u11
//(
//	.wrst(1'b0),
//	.wclk(clk),
//	.wce(adr_o[4:2]==3'b111),
//	.we(ack_i),
//	.wadr(adr_o[14:5]),
//	.i({14'h3FFF,dadr_o[63:14]}),
//	.wo(),
//
//	.rrst(1'b0),
//	.rclk(~clk),
//	.rce(1'b1),
//	.radr(pea[13:5]),
//	.ro({dtign,dtgout})
//);

assign dhit = (dtgout=={1'b1,pea[63:15]});

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

reg [64:0] xData;
wire xisCacheElement = xData[63:52] != 12'hFFD;
reg m1IsCacheElement;

reg nopI;
wire [6:0] iFunc = insn[6:0];
wire [6:0] dFunc = dIR[6:0];
wire [6:0] xFunc = xIR[6:0];
wire [6:0] iOpcode = insn[41:35];
wire [6:0] xOpcode = xIR[41:35];
wire [6:0] dOpcode = dIR[41:35];
reg [6:0] m1Opcode,m2Opcode,m3Opcode,m4Opcode;
reg [6:0] m1Func,m2Func,m3Func,m4Func;
reg [63:0] m1Data,m2Data,m3Data,m4Data,wData,tData;
reg [63:0] m2Addr,m3Addr,m4Addr;
reg [63:0] tick;
reg [63:0] tba;
reg [63:0] exception_address,ipc;
reg [63:0] a,b,c,imm,m1b;
reg prev_ihit;
reg rsf;
reg [63:5] resv_address;
reg dirqf,rirqf,m1irqf,m2irqf,m3irqf,m4irqf,wirqf,tirqf;
reg xirqf;
reg [7:0] dextype,m1extype,m2extype,m3extype,m4extype,wextype,textype,exception_type;
reg [7:0] xextype;
wire advanceX_edge;
reg takb;

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

wire [63:0] fpZLOut;
wire [63:0] fpLooOut;
wire fpLooDone;

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

wire [63:0] jmp_tgt = dOpcode[6:4]==`IMM ? {dIR[26:0],insn[34:0],2'b00} : {pc[63:37],insn[34:0],2'b00};

//-----------------------------------------------------------------------------
// Branch history table.
// The history table is updated by the EX stage and read in
// both the EX and IF stages.
//-----------------------------------------------------------------------------
reg [2:0] gbl_branch_hist;
reg [1:0] branch_history_table [255:0];
wire [7:0] bht_wa = {xpc[5:0],gbl_branch_hist[2:1]};		// write address
wire [7:0] bht_ra1 = {xpc[5:0],gbl_branch_hist[2:1]};		// read address (EX stage)
wire [7:0] bht_ra2 = {pc[5:0],gbl_branch_hist[2:1]};	// read address (IF stage)
wire [1:0] bht_xbits = branch_history_table[bht_ra1];
wire [1:0] bht_ibits = branch_history_table[bht_ra2];
wire predict_taken = bht_ibits==2'd0 || bht_ibits==2'd1;

wire isxBranchI = (xOpcode==`BRAI || xOpcode==`BRNI || xOpcode==`BEQI || xOpcode==`BNEI ||
					xOpcode==`BLTI || xOpcode==`BLEI || xOpcode==`BGTI || xOpcode==`BGEI ||
					xOpcode==`BLTUI || xOpcode==`BLEUI || xOpcode==`BGTUI || xOpcode==`BGEUI)
				;
wire isxBranch = isxBranchI || xOpcode==`TRAPcc || xOpcode==`TRAPcci || xOpcode==`BTRI || xOpcode==`BTRR;

reg [1:0] xbits_new;

always @(takb or bht_xbits)
if (takb) begin
	if (bht_xbits != 2'd1)
		xbits_new <= bht_xbits + 2'd1;
	else
		xbits_new <= bht_xbits;
end
else begin
	if (bht_xbits != 2'd2)
		xbits_new <= bht_xbits - 2'd1;
	else
		xbits_new <= bht_xbits;
end

// For simulation only, initialize the history table to zeros.
// In the real world we don't care.
initial begin
	for (n = 0; n < 256; n = n + 1)
		branch_history_table[n] = 0;
end

//-----------------------------------------------------------------------------
// Evaluate branch conditions.
//-----------------------------------------------------------------------------
wire signed [63:0] as = a;
wire signed [63:0] bs = b;
wire signed [63:0] imms = imm;
wire aeqz = a==64'd0;
wire beqz = b==64'd0;
wire immeqz = imm==64'd0;
wire eq = a==b;
wire eqi = a==imm;
wire lt = as < bs;
wire lti = as < imms;
wire ltu = a < b;
wire ltui = a < imm;

always @(xOpcode or xFunc or a or eq or eqi or lt or lti or ltu or ltui or aeqz or beqz or rsf or xIR)
case (xOpcode)
`BTRR:
	case(xIR[4:0])
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
`BRAI:	takb = 1'b1;
`BRNI:	takb = 1'b0;
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
	case(xIR[24:18])
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
	case(xFunc)
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
cntlz64 u12 ( .i(a),  .o(cntlzo) );
cntlo64 u13 ( .i(a),  .o(cntloo) );

reg [1:0] shftop;
wire [63:0] shfto;
always @(xFunc)
	if (xFunc==`SHL)
		shftop = 2'b00;
	else if (xFunc==`ROL || xFunc==`ROR)
		shftop = 2'b01;
	else if (xFunc==`SHRU)
		shftop = 2'b10;
	else if (xFunc==`SHR)
		shftop = 2'b11;
	else
		shftop = 2'b01;

wire [63:0] masko;
shiftAndMask u15
(
	.op(shftop),
	.oz(1'b0),		// zero the output
	.a(a),
	.b(b[5:0]),
	.mb(xIR[12:7]),
	.me(xIR[18:13]),
	.o(shfto),
	.mo(masko)
);

always @(xOpcode or xFunc or a or b or imm or as or bs or imms or xpc or
	sqrt_out or cntlzo or cntloo or tick or ipc or tba or AXC or
	lt or eq or ltu or mult_out or lti or eqi or ltui or xIR or div_q or div_r or
	shfto or masko or bcdaddo or bcdsubo or fpLooOut or fpZLOut or
	Wired or Index or Random or TLBPhysPage0 or TLBPhysPage1 or TLBVirtPage or TLBASID or
	PageTableAddr or BadVAddr or ASID or TLBPageMask
)
casex(xOpcode)
`R:
	casex(xFunc)
	`COM:	xData = ~a;
	`NOT:	xData = ~|a;
	`NEG:	xData = -a;
	`ABS:	xData = a[63] ? -a : a;
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

	`MFSPR:
		case(xIR[12:7])
		`Wired:			xData = Wired;
		`TLBIndex:		xData = Index;
		`TLBRandom:		xData = Random;
		`TLBPhysPage0:	xData = {TLBPhysPage0,13'd0};
		`TLBPhysPage1:	xData = {TLBPhysPage1,13'd0};
		`TLBVirtPage:	xData = {TLBVirtPage,13'd0};
		`TLBPageMask:	xData = {TLBPageMask,13'd0};
		`TLBASID:	begin
					xData = 65'd0;
					xData[0] = TLBValid;
					xData[1] = TLBD;
					xData[2] = TLBG;
					xData[15:8] = TLBASID;
					end
		`PageTableAddr:	xData = {PageTableAddr,13'd0};
		`BadVAddr:		xData = {BadVAddr,13'd0};
		`ASID:			xData = ASID;
		`Tick:			xData = tick;
		`EPC:			xData = EPC;
		`CauseCode:		xData = CauseCode;
		`TBA:			xData = TBA;
		`AXC:			xData = xAXC;
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

	`SHL:	xData = shfto;
	`SHRU:	xData = shfto;
	`ROL:	xData = shfto;
	`ROR:	xData = {a[0],a[63:1]};
	`SHR:	xData = shfto;
	`ROLAM:	xData = shfto & masko;

	`BCD_ADD:	xData = bcdaddo;
	`BCD_SUB:	xData = bcdsubo;

	default:	xData = 65'd0;
	endcase
`SHFTI:
	case(xFunc)
	`SHLI:	xData = shfto;
	`SHRUI:	xData = shfto;
	`ROLI:	xData = shfto;
	`RORI:	xData = {a[0],a[63:1]};
	`SHRI:	xData = shfto;
	`ROLAMI:	xData = shfto & masko;
	`BFINS: 	begin for (n = 0; n < 64; n = n + 1) xData[n] = masko[n] ? shfto[n] : b[n]; xData[64] = 1'b0; end
	`BFSET: 	begin for (n = 0; n < 64; n = n + 1) xData[n] = masko[n] ? 1'b1 : b[n]; xData[64] = 1'b0; end
	`BFCLR: 	begin for (n = 0; n < 64; n = n + 1) xData[n] = masko[n] ? 1'b0 : b[n]; xData[64] = 1'b0; end
	`BFCHG: 	begin for (n = 0; n < 64; n = n + 1) xData[n] = masko[n] ? ~b[n] : b[n]; xData[64] = 1'b0; end
	default:	xData = 65'd0;
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
`INB,`INCH,`INH,`INW:
		xData = a + imm;
`OUTB,`OUTC,`OUTH,`OUTW:
		xData = a + imm;
`LW,`LH,`LC,`LB,`LHU,`LCU,`LBU,`LWR:
		xData = a + imm;
`SW,`SH,`SC,`SB,`SWC:
		xData = a + imm;
`MEMNDX:
		xData = a + b + imm;
`BEQ,`BNE,`BLT,`BLE,`BGT,`BGE,`BLTU,`BLEU,`BGTU,`BGEU,`BOR,`BAND:
		xData = 64'd0;
`TRAPcc:	xData = fnIncPC(xpc);
`TRAPcci:	xData = fnIncPC(xpc);
`CALL:		xData = fnIncPC(xpc);
`JAL:		xData = xpc + {xIR[29:25],2'b00};
`RET:	xData = a + {imm,2'b00};
`FPLOO:	xData = fpLooOut;
`FPZL:	xData = fpZLOut;
default:	xData = 65'd0;
endcase

wire v_ri,v_rr;
overflow u2 (.op(xOpcode==`SUBI), .a(a[63]), .b(imm[63]), .s(xData[63]), .v(v_ri));
overflow u3 (.op(xOpcode==`RR && xFunc==`SUB), .a(a[63]), .b(b[63]), .s(xData[63]), .v(v_rr));

wire dbz_error = (xOpcode==`DIVSI||xOpcode==`DIVUI) && b==64'd0;
wire ovr_error = ((xOpcode==`ADDI || xOpcode==`SUBI) || (xOpcode==`RR && (xFunc==`SUB || xFunc==`ADD))) && v_ri;
wire priv_violation = !KernelMode && (xOpcode==`MISC &&
	(xFunc==`IRET || xFunc==`ERET || xFunc==`CLI || xFunc==`SEI ||
	 xFunc==`TLBP || xFunc==`TLBR || xFunc==`TLBWR || xFunc==`TLBWI
	));
wire xIsSqrt = xOpcode==`R && xFunc==`SQRT;
wire xIsMult = (xOpcode==`RR && (xFunc==`MULU || xFunc==`MULS)) ||
	xOpcode==`MULSI || xOpcode==`MULUI;
wire xIsDiv = (xOpcode==`RR && (xFunc==`DIVU || xFunc==`DIVS)) ||
	xOpcode==`DIVSI || xOpcode==`DIVUI;

wire xIsLoad =
	xOpcode==`LW || xOpcode==`LH || xOpcode==`LB || xOpcode==`LWR ||
	xOpcode==`LHU || xOpcode==`LBU ||
	xOpcode==`LC || xOpcode==`LCU ||
	xOpcode==`INW || xOpcode==`INB || xOpcode==`INH || xOpcode==`INCH
	;
wire xIsStore =
	xOpcode==`SW || xOpcode==`SH || xOpcode==`SB || xOpcode==`SC || xOpcode==`SWC ||
	xOpcode==`OUTW || xOpcode==`OUTH || xOpcode==`OUTB || xOpcode==`OUTC
	;
wire xIsSWC = xOpcode==`SWC;
wire xIsIn = 
	xOpcode==`INW || xOpcode==`INH || xOpcode==`INCH || xOpcode==`INB
	;
//wire mIsSWC = mOpcode==`SWC;

//wire mIsLoad =
//	mOpcode==`LW || mOpcode==`LH || mOpcode==`LB || mOpcode==`LC || mOpcode==`LWR ||
//	mOpcode==`LHU || mOpcode==`LBU || mOpcode==`LCU ||
//	mOpcode==`INW || mOpcode==`INB || mOpcode==`INH
//	;
wire m1IsLoad =
	m1Opcode==`LW || m1Opcode==`LH || m1Opcode==`LB || m1Opcode==`LC || m1Opcode==`LWR ||
	m1Opcode==`LHU || m1Opcode==`LBU || m1Opcode==`LCU
	;
wire m1IsIn = 
	m1Opcode==`INW || m1Opcode==`INH || m1Opcode==`INCH || m1Opcode==`INB
	;
wire m2IsInW = m2Opcode==`INW;
wire m1IsStore =
	m1Opcode==`SW || m1Opcode==`SH || m1Opcode==`SB || m1Opcode==`SC || m1Opcode==`SWC
	;
wire xIsIO = 
	xIsIn ||
	xOpcode==`OUTW || xOpcode==`OUTH || xOpcode==`OUTC || xOpcode==`OUTB
	;
wire m1IsIO = 
	m1IsIn ||
	m1Opcode==`OUTW || m1Opcode==`OUTH || m1Opcode==`OUTC || m1Opcode==`OUTB
	;

wire m2IsLoad =
	m2Opcode==`LW || m2Opcode==`LH || m2Opcode==`LB || m2Opcode==`LC || m2Opcode==`LWR ||
	m2Opcode==`LHU || m2Opcode==`LBU || m2Opcode==`LCU
	;
wire m2IsStore = 
	m2Opcode==`SW || m2Opcode==`SWC || m2Opcode==`SH || m2Opcode==`SC || m2Opcode==`SB;

wire xIsFPLoo = xOpcode==`FPLOO;

wire xneedBus = xIsIO;
wire m1needBus = ((m1IsLoad & !dhit)| m1IsStore) || m1IsIO;
wire m2needBus = (m2IsLoad | m2IsStore);

// Stall on SWC allows rsf flag to be loaded for the next instruction
// Currently stalls on load of R0, but doesn't need to.
wire StallR =  	(((xIsLoad||xIsIn) && ((xRt==dRa)||(xRt==dRb)||(xRt==dRt))) || xIsSWC) ||
				(((m1IsLoad||m1IsIn) && ((m1Rt==dRa)||(m1Rt==dRb)||(m1Rt==dRt)))) ||
				(((m2IsLoad||m2IsInW) && ((m2Rt==dRa)||(m2Rt==dRb)||(m2Rt==dRt))))
				;
wire StallX = xneedBus & (m1needBus|m2needBus|icaccess|dcaccess);
wire StallM1 = m1needBus & (m2needBus|icaccess|dcaccess);
wire StallM2 =  icaccess|dcaccess;

wire advanceT = !resetA;
wire advanceW = advanceT;
wire advanceM2 = advanceW &&
					((m2IsLoad || m2IsStore) ? ack_i : 1'b1) &&
					!StallM2
					;
wire advanceM1 = advanceM2 &
					(m1IsIO ? ack_i : 1'b1) &
					((m1IsLoad & m1IsCacheElement) ? dhit : 1'b1) & 
					!StallM1
					;
wire advanceX = advanceM1 & (
					xIsSqrt ? sqrt_done :
					xIsMult ? mult_done :
					xIsDiv ? div_done :
					xIsFPLoo ? fpLooDone :
					1'b1) &
					!StallX;
wire advanceR = advanceX & !StallR;
wire advanceI = advanceR & ihit;

wire triggerDCacheLoad = (m1IsLoad & m1IsCacheElement & !dhit) &&	// there is a miss
						!(icaccess | dcaccess) && 	// caches are not active
						m2Opcode==`NOPI				// and the pipeline is free of memory-ops
						;
// Since IMM is "sticky" we have to check for it.
wire triggerICacheLoad = !ihit & !triggerDCacheLoad &	// There is a miss
						(dOpcode==`NOPI || dOpcode[6:4]==`IMM) &&			// and the pipeline is flushed
						(xOpcode==`NOPI || xOpcode[6:4]==`IMM) &&
						m1Opcode==`NOPI &&
						m2Opcode==`NOPI
						;
wire EXexception_pending = ovr_error || dbz_error || priv_violation || xOpcode==`TRAPcci || xOpcode==`TRAPcc;
wire M1exception_pending = advanceM1 & (m1IsLoad|m1IsStore) & DTLBMiss;
wire exception_pending = EXexception_pending | M1exception_pending;

wire xWillLoadStore = (xIsLoad||xIsStore) & advanceX;
wire stallCacheLoad = xWillLoadStore;

reg prev_nmi,nmi_edge;


//---------------------------------------------------------
// Register file.
//---------------------------------------------------------

syncRam512x64_1rw3r u5
(
	.wrst(1'b0),
	.wclk(clk),
	.wce(advanceW),
	.we(1'b1),
	.wadr(wRt),
	.i(wData),
	.wo(),
	
	.rrsta(1'b0),
	.rclka(~clk),
	.rcea(advanceR),
	.radra(dRa),
	.roa(rfoa),

	.rrstb(1'b0),
	.rclkb(~clk),
	.rceb(advanceR),
	.radrb(dRb),
	.rob(rfob),

	.rrstc(1'b0),
	.rclkc(~clk),
	.rcec(advanceR),
	.radrc(dRc),
	.roc(rfoc)
);


reg m1clkoff,m2clkoff,m3clkoff,m4clkoff,wclkoff;
reg dFip,xFip,m1Fip,m2Fip,m3Fip,m4Fip,wFip;

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
	dccyc <= 1'b0;
	
	TBA <= 64'd0;
	pc <= `RESET_VECTOR;
	m1Opcode <= `NOPI;
	m2Opcode <= `NOPI;
	m3Opcode <= `NOPI;
	m4Opcode <= `NOPI;
	dIR <= `NOP_INSN;
	dRt <= 9'd0;
	tRt <= 9'd0;
	wRt <= 9'd0;
	m1Rt <= 9'd0;
	m2Rt <= 9'd0;
	m3Rt <= 9'd0;
	m4Rt <= 9'd0;
	tData <= 64'd0;
	wData <= 64'd0;
	m1Data <= 64'd0;
	m2Data <= 64'd0;
	m3Data <= 64'd0;
	m4Data <= 64'd0;
	icaccess <= 1'b0;
	dcaccess <= 1'b0;
	nopI <= 1'b0;
	prev_ihit <= 1'b0;
	dhwxtype <= 2'b00;
	xhwxtype <= 2'b00;
	m1hwxtype <= 2'b00;
	m2hwxtype <= 2'b00;
	m3hwxtype <= 2'b00;
	m4hwxtype <= 2'b00;
	whwxtype <= 2'b00;
	wFip <= 1'b0;
	m4Fip <= 1'b0;
	m3Fip <= 1'b0;
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
	Random <= 4'hF;
	Wired <= 4'd0;
	StatusEXL <= 1'b1;
	StatusHWI <= 1'b0;
	resetA <= 1'b1;
	mutex_gate <= 64'h0;
	gbl_branch_hist <= 3'b000;
	m1IsCacheElement <= 1'b0;
	dtinit <= 1'b1;
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

if (Random==Wired)
	Random <= 4'hF;
else
	Random <= Random - 4'd1;

tick <= tick + 64'd1;

prev_nmi <= nmi_i;
if (!prev_nmi & nmi_i)
	nmi_edge <= 1'b1;


// A store by any device in the system to a reserved address blcok
// clears the reservation.

if (sys_adv && sys_adr[63:5]==resv_address)
	resv_address <= 59'd0;

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
	tRt <= wRt;
	tData <= wData;
	if (wRt!=5'd0)
		$display("Writing regfile[%d:%d] with %h", wRt[8:5],wRt[4:0], wData);
	wRt <= 9'd0;
	wData <= 64'd0;
	if (|whwxtype) begin
		dhwxtype <= 2'b00;
		xhwxtype <= 2'b00;
		m1hwxtype <= 2'b00;
		m2hwxtype <= 2'b00;
		m3hwxtype <= 2'b00;
		m4hwxtype <= 2'b00;
		whwxtype <= 2'b00;
	end
	clk_en <= 1'b1;
	if (wclkoff)
		clk_en <= 1'b0;
	wclkoff <= 1'b0;
	m1clkoff <= 1'b0;
	m2clkoff <= 1'b0;
	m3clkoff <= 1'b0;
	m4clkoff <= 1'b0;
	if (wFip) begin
		wFip <= 1'b0;
		m4Fip <= 1'b0;
		m3Fip <= 1'b0;
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
	wData <= m2Data;
	whwxtype <= m2hwxtype;
	wextype <= m2extype;
	wRt <= m2Rt;
	wpc <= m2pc;
	wpcv <= m2pcv;
	wclkoff <= m2clkoff;
	wFip <= m2Fip;
	
	m2Rt <= 9'd0;
	m2Opcode <= `NOPI;
	m2Func <= 7'd0;
	m2Addr <= 64'd0;
	m2Data <= 64'd0;
	m2clkoff <= 1'b0;
	m2pc <= 64'd0;
	m2extype <= `EX_NON;
	if (m2extype==`EX_NON) begin
		case(m2Opcode)
		`SH,`SC,`SB,`SW,`SWC:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				we_o <= 1'b0;
				sel_o <= 4'h0;
				m3Opcode <= `NOPI;
			end
		`LH:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				m3Data <= sel_o[7] ? {{32{dat_i[63]}},dat_i[63:32]}:{{32{dat_i[31]}},dat_i[31: 0]};
				m3Opcode <= `NOPI;
			end
		`LW,`LWR:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				m3Data <= dat_i;
				m3Opcode <= `NOPI;
			end
		`LHU:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				m3Data <= sel_o[7] ? dat_i[63:32] : dat_i[31: 0];
				m3Opcode <= `NOPI;
			end
		`LC:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				case(sel_o)
				8'b00000011:	m3Data <= {{48{dat_i[15]}},dat_i[15: 0]};
				8'b00001100:	m3Data <= {{48{dat_i[31]}},dat_i[31:16]};
				8'b00110000:	m3Data <= {{48{dat_i[47]}},dat_i[47:32]};
				8'b11000000:	m3Data <= {{48{dat_i[63]}},dat_i[63:48]};
				default:	m3Data <= 64'hDEADDEADDEADDEAD;			
				endcase
				m3Opcode <= `NOPI;
			end
		`LCU:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				case(sel_o)
				8'b00000011:	m3Data <= dat_i[15: 0];
				8'b00001100:	m3Data <= dat_i[31:16];
				8'b00110000:	m3Data <= dat_i[47:32];
				8'b11000000:	m3Data <= dat_i[63:48];
				default:	m3Data <= 64'hDEADDEADDEADDEAD;			
				endcase
				m3Opcode <= `NOPI;
			end
		`LB:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				case(sel_o)
				8'b00000001:	m3Data <= {{56{dat_i[ 7]}},dat_i[ 7: 0]};
				8'b00000010:	m3Data <= {{56{dat_i[15]}},dat_i[15: 8]};
				8'b00000100:	m3Data <= {{56{dat_i[23]}},dat_i[23:16]};
				8'b00001000:	m3Data <= {{56{dat_i[31]}},dat_i[31:24]};
				8'b00010000:	m3Data <= {{56{dat_i[39]}},dat_i[39:32]};
				8'b00100000:	m3Data <= {{56{dat_i[47]}},dat_i[47:40]};
				8'b01000000:	m3Data <= {{56{dat_i[55]}},dat_i[55:48]};
				8'b10000000:	m3Data <= {{56{dat_i[63]}},dat_i[63:56]};
				default:	m3Data <= 64'hDEADDEADDEADDEAD;
				endcase
				m3Opcode = `NOPI;
			end
		`LBU:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				case(sel_o)
				8'b00000001:	m3Data <= dat_i[ 7: 0];
				8'b00000010:	m3Data <= dat_i[15: 8];
				8'b00000100:	m3Data <= dat_i[23:16];
				8'b00001000:	m3Data <= dat_i[31:24];
				8'b00010000:	m3Data <= dat_i[39:32];
				8'b00100000:	m3Data <= dat_i[47:40];
				8'b01000000:	m3Data <= dat_i[55:48];
				8'b10000000:	m3Data <= dat_i[63:56];
				default:	m3Data <= 64'hDEADDEADDEADDEAD;
				endcase
				m3Opcode <= `NOPI;
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
	m2Opcode <= m1Opcode;
	m2Func <= m1Func;
	m2Addr <= pea;
	m2Data <= m1Data;
	m2hwxtype <= m1hwxtype;
	m2extype <= m1extype;
	m2Rt <= m1Rt;
	m2pc <= m1pc;
	m2pcv <= m1pcv;
	m2clkoff <= m1clkoff;
	m2Fip <= m1Fip;

	m1Rt <= 9'd0;
	m1Opcode <= `NOPI;
	m1Func <= 7'd0;
	m1Data <= 64'd0;
	m1clkoff <= 1'b0;
	m1pc <= 64'd0;
	m1IsCacheElement <= 1'b0;
	m1extype <= `EX_NON;

	if (m1extype == `EX_NON) begin
		case(m1Opcode)
		`MISC:
			case(m1Func)
			`TLBP:
				begin
					Index[63] <= ~|DMatch;
				end
			`TLBR:
				begin
					TLBPageMask <= ITLBPageMask[i];
					TLBVirtPage <= ITLBVirtPage[i];
					TLBPhysPage0 <= ITLBPhysPage0[i];
					TLBPhysPage1 <= ITLBPhysPage1[i];
					TLBASID <= ITLBASID[i];
					TLBG <= ITLBG[i];
					TLBD <= ITLBD[i];
					TLBValid <= ITLBValid[i];
				end
			`TLBWI,`TLBWR:
				begin
					ITLBValid[i] <= TLBValid;
					ITLBVirtPage[i] <= TLBVirtPage;
					ITLBPhysPage0[i] <= TLBPhysPage0;
					ITLBPhysPage1[i] <= TLBPhysPage1;
					ITLBPageMask[i] <= TLBPageMask;
					ITLBASID[i] <= TLBASID;
					ITLBD[i] <= TLBD;
					ITLBG[i] <= TLBG;
				end
			endcase
		`INW:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				m2Data <= dat_i;
				m2Opcode <= `NOPI;
			end
		`INH:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				m2Data <= sel_o[7] ? {{32{dat_i[63]}},dat_i[63:32]}:{{32{dat_i[31]}},dat_i[31: 0]};
				m2Opcode <= `NOPI;
			end
		`INHU:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				sel_o <= 8'h00;
				m2Data <= sel_o[7] ? dat_i[63:32] : dat_i[31: 0];
				m2Opcode <= `NOPI;
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
				m2Opcode <= `NOPI;
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
				m2Opcode <= `NOPI;
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
				m2Opcode <= `NOPI;
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
				m2Opcode <= `NOPI;
			end
		`OUTW,`OUTH,`OUTC,`OUTB:
			begin
				cyc_o <= 1'b0;
				stb_o <= 1'b0;
				we_o <= 1'b0;
				sel_o <= 8'h00;
				m2Opcode <= `NOPI;
			end

		`LW:
			if (!m1IsCacheElement) begin
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				sel_o <= 8'hFF;
				adr_o <= {pea[63:3],3'b000};
				m2Addr <= {pea[63:3],3'b000};
			end
			else if (dhit) begin
				m2Opcode <= `NOPI;
				m2Data <= cdat;
			end

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
				m2Opcode <= `NOPI;
				m2Data <= cdat;
				rsv_o <= 1'b1;
				resv_address <= pea[63:5];
			end

		`LH:
			if (!m1IsCacheElement) begin
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				sel_o <= pea[2] ? 8'b11110000 : 8'b00001111;
				adr_o <= {pea[63:2],2'b00};
				m2Addr <= {pea[63:2],2'b00};
			end
			else if (dhit) begin
				m2Opcode <= `NOPI;
				if (pea[1])
					m2Data <= {{32{cdat[31]}},cdat[31:0]};
				else
					m2Data <= {{32{cdat[63]}},cdat[63:32]};
			end

		`LHU:
			if (!m1IsCacheElement) begin
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				sel_o <= pea[2] ? 8'b11110000 : 8'b00001111;
				adr_o <= {pea[63:2],2'b00};
				m2Addr <= {pea[63:2],2'b00};
			end
			else if (dhit) begin
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

		`SW:
			begin
				m2Addr <= {pea[63:3],3'b000};
				wrhit <= dhit;
				if (!m1UnmappedDataArea & !q[4])
					ITLBD[q] <= 1'b1;
				if (resv_address==pea[63:5])
					resv_address <= 59'd0;
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				we_o <= 1'b1;
				sel_o <= 8'hFF;
				adr_o <= {pea[63:3],3'b000};
				dat_o <= m1b;
			end

		`SH:
			begin
				wrhit <= dhit;
				m2Addr <= {pea[63:2],2'b00};
				if (!m1UnmappedDataArea & !q[4])
					ITLBD[q] <= 1'b1;
				if (resv_address==pea[63:5])
					resv_address <= 59'd0;
				cyc_o <= 1'b1;
				stb_o <= 1'b1;
				we_o <= 1'b1;
				sel_o <= pea[2] ? 8'b11110000 : 8'b00001111;
				adr_o <= {pea[63:2],2'b00};
				dat_o <= {2{m1b[31:0]}};
			end

		`SC:
			begin
				$display("Storing char to %h, ea=%h",pea,ea);
				wrhit <= dhit;
				m2Addr <= {pea[63:2],2'b00};
				if (!m1UnmappedDataArea & !q[4])
					ITLBD[q] <= 1'b1;
				if (resv_address==pea[63:5])
					resv_address <= 59'd0;
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
				dat_o <= {4{m1b[15:0]}};
			end

		`SB:
			begin
				wrhit <= dhit;
				m2Addr <= {pea[63:2],2'b00};
				if (resv_address==pea[63:5])
					resv_address <= 59'd0;
				if (!m1UnmappedDataArea & !q[4])
					ITLBD[q] <= 1'b1;
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
				dat_o <= {8{m1b[7:0]}};
			end

		`SWC:
			begin
				rsf <= 1'b0;
				if (resv_address==pea[63:5]) begin
					if (!m1UnmappedDataArea & !q[4])
						ITLBD[q] <= 1'b1;
					wrhit <= dhit;
					m2Addr <= {pea[63:3],3'b00};
					cyc_o <= 1'b1;
					stb_o <= 1'b1;
					we_o <= 1'b1;
					sel_o <= 8'hFF;
					adr_o <= {pea[63:3],3'b000};
					dat_o <= m1b;
					resv_address <= 59'd0;
					rsf <= 1'b1;
				end
				else
					m2Opcode <= `NOPI;
			end
		endcase
	end
end

//---------------------------------------------------------
// EXECUTE:
// - perform datapath operation
// - perform virtual to physical address translation.
//---------------------------------------------------------
if (advanceX) begin
	m1hwxtype <= xhwxtype;
	m1Fip <= xFip;
	m1extype <= xextype;
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
		`WAIT:	m1clkoff <= 1'b1;
		`TLBP:	ea <= TLBVirtPage;
		`TLBR,`TLBWI:
			begin
				i <= Index;
			end
		`TLBWR:
			begin
				i <= Random;
			end
		default:	;
		endcase
	`R:
		case(xFunc)
		`MTSPR:
			case(xIR[12:7])
			`Wired:			Wired <= xData[3:0];
			`ASID:			ASID <= xData[7:0];
			`TLBIndex:		Index <= xData[3:0];
			`TLBVirtPage:	TLBVirtPage <= xData[63:13];
			`TLBPhysPage0:	TLBPhysPage0 <= xData[63:13];
			`TLBPhysPage1:	TLBPhysPage1 <= xData[63:13];
			`TLBPageMask:	TLBPageMask <= xData[24:13];
			`TLBASID:		begin
							TLBValid <= xData[0];
							TLBD <= xData[1];
							TLBG <= xData[2];
							TLBASID <= xData[15:8];
							end
			`PageTableAddr:	PageTableAddr <= xData[63:13];
			`BadVAddr:		BadVAddr <= xData[63:13];
			`EPC:			EPC <= xData;
			`TBA:			TBA <= xData;
			`AXC:			AXC <= xData[3:0];
			default:	;
			endcase
		`MTTBA:	tba <= {xData[63:2],2'b00};
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
	`LB,`LBU,`LC,`LCU,`LH,`LHU,`LW,`LWR,`SW,`SH,`SC,`SB,`SWC:
			begin
			m1b <= b;
			ea <= xData;
			end
	`MEMNDX:
			begin
			m1Opcode <= xFunc;
			m1b <= c;
			ea <= xData;
			end
	`DIVSI,`DIVUI:
		if (b==64'd0) begin
			xextype <= `EX_DBZ;
		end
	default:	;
	endcase
	// Update the branch history
	if (isxBranch) begin
		gbl_branch_hist <= {gbl_branch_hist,takb};
		branch_history_table[bht_wa] <= xbits_new;
	end
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
	xIR <= dIR;
	xpc <= dpc;
	xpcv <= dpcv;
	xbranch_taken <= dbranch_taken;
	dbranch_taken <= 1'b0;
	dextype <= `EX_NON;
	if (dOpcode[6:4]!=`IMM)	// IMM is "sticky"
		dIR <= `NOP_INSN;
	dRa <= 9'd0;
	dRb <= 9'd0;

	// Result forward muxes
	casex(dRa)
	9'bxxxx00000:	a <= 64'd0;
	xRt:	a <= xData;
	m1Rt:	a <= m1Data;
	m2Rt:	a <= m2Data;
	wRt:	a <= wData;
	tRt:	a <= tData;
	default:	a <= rfoa;
	endcase
	casex(dRb)
	9'bxxxx00000:	b <= 64'd0;
	xRt:	b <= disRightShift ? -xData[5:0] : xData;
	m1Rt:	b <= disRightShift ? -m1Data[5:0] : m1Data;
	m2Rt:	b <= disRightShift ? -m2Data[5:0] : m2Data;
	wRt:	b <= disRightShift ? -wData[5:0] : wData;
	tRt:	b <= disRightShift ? -tData[5:0] : tData;
	default:	b <= disRightShift ? -rfob[5:0] : rfob;
	endcase
	if (dOpcode==`SHFTI)
		case(dFunc)
		`RORI:		b <= {58'd0,~dIR[24:19]+6'd1};
		default:	b <= {58'd0,dIR[24:19]};
		endcase
	casex(dRc)
	9'bxxxx00000:	c <= 64'd0;
	xRt:	c <= xData;
	m1Rt:	c <= m1Data;
	m2Rt:	c <= m2Data;
	wRt:	c <= wData;
	tRt:	c <= tData;
	default:	c <= rfoc;
	endcase

	// Set the target register
	casex(dOpcode)
	`SETLO:		xRt <= {dAXC,dIR[36:32]};
	`SETHI:		xRt <= {dAXC,dIR[36:32]};
	`RR:		xRt <= {dAXC,dIR[24:20]};
	`BTRI:		xRt <= 9'd0;
	`BTRR:		xRt <= 9'd0;
	`TRAPcc:	xRt <= 9'd0;
	`TRAPcci:	xRt <= 9'd0;
	`JMP:		xRt <= 9'd00;
	`CALL:		xRt <= {dAXC,5'd31};
	`RET:		xRt <= {dAXC,dIR[24:20]};
	`MEMNDX:
		case(dFunc)
		`SW,`SH,`SC,`SB,`OUTW,`OUTH,`OUTC,`OUTB:
				xRt <= 9'd0;
		default:	xRt <= {dAXC,dIR[24:20]};
		endcase
	`SW,`SH,`SC,`SB,`OUTW,`OUTH,`OUTC,`OUTB:
				xRt <= 9'd0;
	`NOPI:		xRt <= 9'd0;
	`BEQI,`BNEI,`BLTI,`BLEI,`BGTI,`BGEI,`BLTUI,`BLEUI,`BGTUI,`BGEUI:
				xRt <= 9'd0;
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
		`ANDI:	imm <= {39'h7FFFFFFFFF,dIR[24:0]};
		`ORI:	imm <= {39'h0000000000,dIR[24:0]};
		`XORI:	imm <= {39'h0000000000,dIR[24:0]};
		`RET:	imm <= {44'h00000000000,dIR[19:0]};
		`MEMNDX:	imm <= {{51{dIR[19]}},dIR[19:7]};
		default:	imm <= {{39{dIR[24]}},dIR[24:0]};
		endcase
	case(dOpcode)

	`MISC:
		case(dFunc)
		`SEI:	im <= 1'b1;
		`CLI:	im <= 1'b0;
		endcase
	endcase
		
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
	if (nmi_edge & !StatusHWI) begin
		$display("*****************");
		$display("NMI edge detected");
		$display("*****************");
		StatusHWI <= 1'b1;
		nmi_edge <= 1'b0;
		dhwxtype <= 2'b01;
		dIR <= `NOP_INSN;
		dextype <= `EX_NMI;
	end
	else if (irq_i & !im & !StatusHWI) begin
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
	else if (ITLBMiss)
		dIR <= `NOP_INSN;
	else begin
		dIR <= insn;
`include "insn_dumpsc.v"
	end
	nopI <= 1'b0;
	if (dOpcode[6:4]!=`IMM) begin
		dpc <= pc;
		dpcv <= 1'b1;
	end
	casex(iOpcode)
	`SETLO:		dRa <= {AXC,insn[36:32]};
	`SETHI:		dRa <= {AXC,insn[36:32]};
	default:	dRa <= {AXC,insn[34:30]};
	endcase
	dRb <= {AXC,insn[29:25]};
	dRc <= {AXC,insn[24:20]};
	if (ITLBMiss) begin
		$display("TLB miss on instruction fetch.");
		CauseCode <= `EX_TLBI;
		StatusEXL <= 1'b1;
		BadVAddr <= pc[63:13];
		pc <= `ITLB_MissHandler;
		EPC <= pc;
	end
	else begin
		dbranch_taken <= 1'b0;
		pc <= fnIncPC(pc);
		case(iOpcode)
		`MISC:
			case(iFunc)
			`FIP:	dFip <= 1'b1;
			default:	;
			endcase
		`JMP,`CALL:
			begin
				dbranch_taken <= 1'b1;
				pc <= jmp_tgt;
			end
		`BTRR:
			case(insn[4:0])
			`BEQ,`BNE,`BLT,`BLE,`BGT,`BGE,`BLTU,`BLEU,`BGTU,`BGEU,`BAND,`BOR,`BNR:
				if (predict_taken) begin
					$display("Taking predicted branch: %h",{pc[63:4] + {{42{insn[24]}},insn[24:7]},insn[6:5],2'b00});
					dbranch_taken <= 1'b1;
					pc <= {pc[63:4] + {{42{insn[24]}},insn[24:7]},insn[6:5],2'b00};
				end
			default:	;
			endcase
		`BEQI,`BNEI,`BLTI,`BLEI,`BGTI,`BGEI,`BLTUI,`BLEUI,`BGTUI,`BGEUI:
			begin
				if (predict_taken) begin
					dbranch_taken <= 1'b1;
					pc <= {pc[63:4] + {{50{insn[29]}},insn[29:20]},insn[19:18],2'b00};
				end
			end
		`TRAPcc:	if (predict_taken) begin pc <= {TBA[63:13],`GEN_TRAP_OFFSET}; dbranch_taken <= 1'b1; end
		`TRAPcci:	if (predict_taken) begin pc <= {TBA[63:13],`GEN_TRAP_OFFSET}; dbranch_taken <= 1'b1; end
		default:	;
		endcase
	end
end

//`include "RPSTAGE.v"
//---------------------------------------------------------
// EXECUTE - part two:
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
		default:	;
		endcase
	`BTRR:
		case(xIR[4:0])
	// BEQ r1,r2,label
		`BEQ,`BNE,`BLT,`BLE,`BGT,`BGE,`BLTU,`BLEU,`BGTU,`BGEU,`BAND,`BOR,`BNR:
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
	`RET:
		begin
			$display("returning to: %h", {b,2'b00});
			pc[63:2] <= b[63:2];
			dIR <= `NOP_INSN;
			xIR <= `NOP_INSN;
			xRt <= 9'd0;
			xpcv <= 1'b0;
			dpcv <= 1'b0;
		end
	// BEQ r1,#3,r10
	`BTRI:
		if (takb) begin
			pc[63:2] <= b[63:2];
			pc[1:0] <= 2'b00;
			dIR <= `NOP_INSN;
			xIR <= `NOP_INSN;
			xRt <= 9'd0;
			xpcv <= 1'b0;
			dpcv <= 1'b0;
		end
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
			EPC <= xpc;
			if (!xbranch_taken) begin
				pc <= {TBA[63:13],`GEN_TRAP_OFFSET};
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
	default:	;
	endcase
	
	if (dbz_error) begin
		$display("Divide by zero error");
		CauseCode <= `EX_DBZ;
		StatusEXL <= 1'b1;
		EPC <= xpc;
		pc <= {TBA[63:13],`DBZ_TRAP_OFFSET};
		dIR <= `NOP_INSN;
		xIR <= `NOP_INSN;
		xRt <= 9'd0;
		xpcv <= 1'b0;
		dpcv <= 1'b0;
	end
	else if (ovr_error) begin
		$display("Overflow error");
		CauseCode <= `EX_OFL;
		StatusEXL <= 1'b1;
		EPC <= xpc;
		pc <= {TBA[63:13],`OFL_TRAP_OFFSET};
		dIR <= `NOP_INSN;
		xIR <= `NOP_INSN;
		xRt <= 9'd0;
		xpcv <= 1'b0;
		dpcv <= 1'b0;
	end
	else if (priv_violation) begin
		$display("Priviledge violation");
		CauseCode <= `EX_PRIV;
		StatusEXL <= 1'b1;
		EPC <= xpc;
		pc <= {TBA[63:13],`PRIV_OFFSET};
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
	`EX_RST:	begin
				pc <= `RESET_VECTOR;
				case(1'b1)
				wpcv:	IPC <= wpc;
				m2pcv:	IPC <= m2pc;
				m1pcv: 	IPC <= m1pc;
				xpcv:	IPC <= xpc;
				dpcv:	IPC <= dpc;
				default:	IPC <= pc;
				endcase
				end
	`EX_NMI:	begin
				pc <= `NMI_VECTOR;
				case(1'b1)
				wpcv:	IPC <= wpc;
				m2pcv:	IPC <= m2pc;
				m1pcv: 	IPC <= m1pc;
				xpcv:	IPC <= xpc;
				dpcv:	IPC <= dpc;
				default:	IPC <= pc;
				endcase
				end
	`EX_IRQ:	begin
				pc <= `IRQ_VECTOR;
				case(1'b1)
				wpcv:	IPC <= wpc;
				m2pcv:	IPC <= m2pc;
				m1pcv: 	IPC <= m1pc;
				xpcv:	IPC <= xpc;
				dpcv:	IPC <= dpc;
				default:	IPC <= pc;
				endcase
				end
	default:	;
	endcase
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
		bl_o <= 5'd8;
		cyc_o <= 1'b1;
		stb_o <= 1'b1;
		adr_o <= {pea[63:6],6'h00};
		cstate <= DCACT;
	end
	else if (triggerICacheLoad) begin
		icaccess <= 1'b1;
		bte_o <= 2'b00;			// linear burst
		cti_o <= 3'b010;		// burst access
		bl_o <= 5'd8;
		cyc_o <= 1'b1;
		stb_o <= 1'b1;
		adr_o <= {ppc[63:6],6'h00};
		cstate <= ICACT1;
	end
// WISHBONE burst accesses
//
ICACT1:
	if (ack_i) begin
		adr_o[5:3] <= adr_o[5:3] + 3'd1;
		if (adr_o[5:3]==3'h6)
			cti_o <= 3'b111;	// Last cycle ahead
		if (adr_o[5:3]==3'h7) begin
			cti_o <= 3'b000;	// back to non-burst mode
			cyc_o <= 1'b0;
			stb_o <= 1'b0;
			tmem[adr_o[12:6]] <= {1'b1,adr_o[63:13]};	// This will cause ihit to go high
			tvalid[adr_o[12:6]] <= 1'b1;
			icaccess <= 1'b0;
			cstate <= IDLE;
		end
	end
DCACT:
	if (ack_i) begin
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
