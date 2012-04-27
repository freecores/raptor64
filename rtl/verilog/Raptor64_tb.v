module Raptor64_tb();
parameter IDLE = 8'd1;
parameter DOCMD = 8'd2;

reg clk;
reg rst;
wire sys_cyc;
wire sys_stb;
wire sys_we;
wire [7:0] sys_sel;
wire [63:0] sys_adr;
wire [63:0] sys_dbo;
wire [63:0] sys_dbi;
reg [31:0] rd_data;
wire sys_ack;
wire cmd_en;
wire [2:0] cmd_instr;
wire [5:0] cmd_bl;
wire [29:0] cmd_byte_addr;
reg cmd_full;
reg [5:0] tb_cmd_bl;
reg [2:0] tb_cmd_instr;
reg [29:0] tb_cmd_byte_addr;
wire rd_en;
reg rd_empty;
reg [7:0] cnt;

assign sys_ack = sys_stb;

initial begin
	clk = 1;
	rst = 0;
	#100 rst = 1;
	#100 rst = 0;
end

always #10 clk = ~clk;	//  50 MHz

//always @(sys_adr)
//case(sys_adr)
//64'h0:	rd_data <= 64'h00002378 00000000;
//64'h8:	rd_data <= 64'h37800000 00030000;
//64'h10:	rd_data <= 64'h00000378 00000000;
//64'h18:	rd_data <= 64'h37800000 000DE000;
//64'h20:	rd_data <= 64'h700003FF FFFFFFFF;
//64'h28:	rd_data <= 64'h0D83E000 0001200F;
//
//endcase
//assign sys_dbi = rd_data;

reg [7:0] state;
always @(posedge clk)
if (rst) begin
	state <= IDLE;
	cmd_full <= 1'b0;
	rd_empty <= 1'b1;
end
else begin
case(state)
IDLE:
	if (cmd_en) begin
		tb_cmd_instr <= cmd_instr;
		tb_cmd_bl <= cmd_bl;
		tb_cmd_byte_addr <= cmd_byte_addr;
		cmd_full <= 1'b1;
		rd_empty <= 1'b1;
		cnt <= 8'd0;
		state <= DOCMD;
	end
DOCMD:
	case(tb_cmd_instr)
	3'b000:	;
	2'b001:
		begin
			cmd_full <= 1'b0;
			state <= IDLE;
		end
	endcase
default:	state <= IDLE;
endcase
	if (rd_en) begin
		if (cnt>=3) begin
			rd_empty <= 1'b0;
		case(tb_cmd_byte_addr | 64'hFFFF_FFFF_FFFF_0000)
64'hFFFFFFFFFFFFF000:	rd_data <= 32'h020013FD;
64'hFFFFFFFFFFFFF004:	rd_data <= 32'h00006048;
64'hFFFFFFFFFFFFF008:	rd_data <= 32'h01802120;
64'hFFFFFFFFFFFFF00C:	rd_data <= 32'h00848000;
64'hFFFFFFFFFFFFF010:	rd_data <= 32'h0400042B;
64'hFFFFFFFFFFFFF014:	rd_data <= 32'h40006048;
64'hFFFFFFFFFFFFF018:	rd_data <= 32'h00004111;
64'hFFFFFFFFFFFFF01C:	rd_data <= 32'h04803D00;
64'hFFFFFFFFFFFFF020:	rd_data <= 32'h04000032;
64'hFFFFFFFFFFFFF024:	rd_data <= 32'h00193048;
64'hFFFFFFFFFFFFF028:	rd_data <= 32'h00012018;
64'hFFFFFFFFFFFFF02C:	rd_data <= 32'h18844000;
64'hFFFFFFFFFFFFF030:	rd_data <= 32'h42000002;
64'hFFFFFFFFFFFFF034:	rd_data <= 32'h00000420;
64'hFFFFFFFFFFFFF038:	rd_data <= 32'hFFE0A318;
64'hFFFFFFFFFFFFF03C:	rd_data <= 32'h288C1FFF;
64'hFFFFFFFFFFFFF040:	rd_data <= 32'hFFFFFC16;
64'hFFFFFFFFFFFFF044:	rd_data <= 32'h000000C7;
64'hFFFFFFFFFFFFF048:	rd_data <= 32'h000DE000;
64'hFFFFFFFFFFFFF04C:	rd_data <= 32'h37800000;
64'hFFFFFFFFFFFFF050:	rd_data <= 32'h00000000;
64'hFFFFFFFFFFFFF054:	rd_data <= 32'h00000378;
64'hFFFFFFFFFFFFF058:	rd_data <= 32'h000DE000;
64'hFFFFFFFFFFFFF05C:	rd_data <= 32'h04803DC0;
64'hFFFFFFFFFFFFF060:	rd_data <= 32'h44000000;
64'hFFFFFFFFFFFFF064:	rd_data <= 32'h00000100;
64'hFFFFFFFFFFFFF068:	rd_data <= 32'h001AA200;
64'hFFFFFFFFFFFFF06C:	rd_data <= 32'h10040000;
64'hFFFFFFFFFFFFF070:	rd_data <= 32'h3E000000;
64'hFFFFFFFFFFFFF074:	rd_data <= 32'h000024D8;
64'hFFFFFFFFFFFFF078:	rd_data <= 32'hAD504002;
64'hFFFFFFFFFFFFF07C:	rd_data <= 32'h3D5552AA;
64'hFFFFFFFFFFFFF080:	rd_data <= 32'h02AA5555;
64'hFFFFFFFFFFFFF084:	rd_data <= 32'h00000048;
64'hFFFFFFFFFFFFF088:	rd_data <= 32'h00066808;
64'hFFFFFFFFFFFFF08C:	rd_data <= 32'h11A04000;
64'hFFFFFFFFFFFFF090:	rd_data <= 32'h44300006;
64'hFFFFFFFFFFFFF094:	rd_data <= 32'h00002010;
64'hFFFFFFFFFFFFF098:	rd_data <= 32'h00820308;
64'hFFFFFFFFFFFFF09C:	rd_data <= 32'h02210000;
64'hFFFFFFFFFFFFF0A0:	rd_data <= 32'h00000000;
64'hFFFFFFFFFFFFF0A4:	rd_data <= 32'h00000380;
64'hFFFFFFFFFFFFF0A8:	rd_data <= 32'hFF90C81C;
64'hFFFFFFFFFFFFF0AC:	rd_data <= 32'h080C5FFF;
64'hFFFFFFFFFFFFF0B0:	rd_data <= 32'h00A00009;
64'hFFFFFFFFFFFFF0B4:	rd_data <= 32'h00002412;
64'hFFFFFFFFFFFFF0B8:	rd_data <= 32'h00004002;
64'hFFFFFFFFFFFFF0BC:	rd_data <= 32'h11A04000;
64'hFFFFFFFFFFFFF0C0:	rd_data <= 32'h552AAAD5;
64'hFFFFFFFFFFFFF0C4:	rd_data <= 32'hA95557D5;
64'hFFFFFFFFFFFFF0C8:	rd_data <= 32'h0090C21A;
64'hFFFFFFFFFFFFF0CC:	rd_data <= 32'h080C2000;
64'hFFFFFFFFFFFFF0D0:	rd_data <= 32'h10000008;
64'hFFFFFFFFFFFFF0D4:	rd_data <= 32'h00000022;
64'hFFFFFFFFFFFFF0D8:	rd_data <= 32'h000E0000;
64'hFFFFFFFFFFFFF0DC:	rd_data <= 32'h03207000;
64'hFFFFFFFFFFFFF0E0:	rd_data <= 32'hC5FFFFF6;
64'hFFFFFFFFFFFFF0E4:	rd_data <= 32'h00007880;
64'hFFFFFFFFFFFFF0E8:	rd_data <= 32'h009A2850;
64'hFFFFFFFFFFFFF0EC:	rd_data <= 32'h01000800;
64'hFFFFFFFFFFFFF0F0:	rd_data <= 32'hAAD5552A;
64'hFFFFFFFFFFFFF0F4:	rd_data <= 32'h56AAABAA;
64'hFFFFFFFFFFFFF0F8:	rd_data <= 32'h0001200D;
64'hFFFFFFFFFFFFF0FC:	rd_data <= 32'h19A02000;
64'hFFFFFFFFFFFFF100:	rd_data <= 32'h04000000;
64'hFFFFFFFFFFFFF104:	rd_data <= 32'hC000191A;
64'hFFFFFFFFFFFFF108:	rd_data <= 32'h00904110;
64'hFFFFFFFFFFFFF10C:	rd_data <= 32'h080C2000;
64'hFFFFFFFFFFFFF110:	rd_data <= 32'h10000008;
64'hFFFFFFFFFFFFF114:	rd_data <= 32'h00000022;
64'hFFFFFFFFFFFFF118:	rd_data <= 32'h000E0000;
64'hFFFFFFFFFFFFF11C:	rd_data <= 32'h03207000;
64'hFFFFFFFFFFFFF120:	rd_data <= 32'hC5FFFFF6;
64'hFFFFFFFFFFFFF124:	rd_data <= 32'hC0002480;
64'hFFFFFFFFFFFFF128:	rd_data <= 32'h00904802;
64'hFFFFFFFFFFFFF12C:	rd_data <= 32'h01000800;
64'hFFFFFFFFFFFFF130:	rd_data <= 32'h04000000;
64'hFFFFFFFFFFFFF134:	rd_data <= 32'h5554A91A;
64'hFFFFFFFFFFFFF138:	rd_data <= 32'hAAAEAAAB;
64'hFFFFFFFFFFFFF13C:	rd_data <= 32'h0308755A;
64'hFFFFFFFFFFFFF140:	rd_data <= 32'hC2000006;
64'hFFFFFFFFFFFFF144:	rd_data <= 32'h00002080;
64'hFFFFFFFFFFFFF148:	rd_data <= 32'h00008840;
64'hFFFFFFFFFFFFF14C:	rd_data <= 32'h38000000;
64'hFFFFFFFFFFFFF150:	rd_data <= 32'h07000000;
64'hFFFFFFFFFFFFF154:	rd_data <= 32'hFFFFE032;
64'hFFFFFFFFFFFFF158:	rd_data <= 32'h00520317;
64'hFFFFFFFFFFFFF15C:	rd_data <= 32'h28216000;
64'hFFFFFFFFFFFFF160:	rd_data <= 32'h16800014;
64'hFFFFFFFFFFFFF164:	rd_data <= 32'h00001012;
64'hFFFFFFFFFFFFF168:	rd_data <= 32'h014A0850;

64'hFFFFFFFFFFFFFFC0:	rd_data <= 32'h00000000;
64'hFFFFFFFFFFFFFFC4:	rd_data <= 32'h00000378;
64'hFFFFFFFFFFFFFFC8:	rd_data <= 32'h000DE000;
64'hFFFFFFFFFFFFFFCC:	rd_data <= 32'h37800000;

64'hFFFFFFFFFFFFFFD0:	rd_data <= 32'h00000000;
64'hFFFFFFFFFFFFFFD4:	rd_data <= 32'h00000378;
64'hFFFFFFFFFFFFFFD8:	rd_data <= 32'h000DE000;
64'hFFFFFFFFFFFFFFDC:	rd_data <= 32'h37800000;

64'hFFFFFFFFFFFFFFE0:	rd_data <= 32'h00000000;
64'hFFFFFFFFFFFFFFE4:	rd_data <= 32'h00000378;
64'hFFFFFFFFFFFFFFE8:	rd_data <= 32'h000DE000;
64'hFFFFFFFFFFFFFFEC:	rd_data <= 32'h37800000;

64'hFFFFFFFFFFFFFFF0:	rd_data <= 32'hFFFFFC00;
64'hFFFFFFFFFFFFFFF4:	rd_data <= 32'h000000CF;
64'hFFFFFFFFFFFFFFF8:	rd_data <= 32'h00000000;
64'hFFFFFFFFFFFFFFFC:	rd_data <= 32'h00000000;

//		30'h0:	rd_data <= 32'h00000000;
//		30'h4:	rd_data <= 32'h00002378;
//		30'h8:	rd_data <= 32'h00030000;
//		30'hC:	rd_data <= 32'h37800000;
//		30'h10:	rd_data <= 32'h00000000;
//		30'h14:	rd_data <= 32'h00000378;
//		30'h18:	rd_data <= 32'h000DE000;
//		30'h1C:	rd_data <= 32'h37800000;
//		30'h20:	rd_data <= 32'hFFFFFFFF;
//		30'h24:	rd_data <= 32'h700003FF;
//		30'h28:	rd_data <= 32'h0001200F;
//		30'h2C:	rd_data <= 32'h0D83E000;
		endcase
		tb_cmd_byte_addr <= tb_cmd_byte_addr + 30'd4;
		tb_cmd_bl <= tb_cmd_bl - 6'd1;
		if (tb_cmd_bl==6'h0) rd_empty <= 1'b1;
		end
		else
			cnt <= cnt + 1;
	end
end

Raptor64 u1
(
	.rst_i(rst),
	.clk_i(clk),
	.nmi_i(1'b0),
	.irq_i(1'b0),
	.bte_o(),
	.cti_o(),
	.cyc_o(sys_cyc),
	.stb_o(sys_stb),
	.ack_i(sys_ack),
	.we_o(sys_we),
	.sel_o(sys_sel),
	.adr_o(sys_adr),
	.dat_i(sys_dbi),
	.dat_o(sys_dbo),

	.cmd_en(cmd_en),
	.cmd_instr(cmd_instr),
	.cmd_bl(cmd_bl),
	.cmd_byte_addr(cmd_byte_addr),
	.cmd_full(cmd_full),
	
	.rd_en(rd_en),
	.rd_data(rd_data),
	.rd_empty(rd_empty),

	.sys_adv(1'b0),
	.sys_adr(59'd0)
);
endmodule
