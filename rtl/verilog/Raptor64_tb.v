module Raptor64_tb();

reg clk;
reg rst;
wire sys_cyc;
wire sys_stb;
wire sys_we;
wire [7:0] sys_sel;
wire [63:0] sys_adr;
wire [63:0] sys_dbo;
wire [63:0] sys_dbi;
reg [63:0] romout;
wire sys_ack;

assign sys_ack = sys_stb;

initial begin
	clk = 1;
	rst = 0;
	#100 rst = 1;
	#100 rst = 0;
end

always #10 clk = ~clk;	//  50 MHz

always @(sys_adr)
case(sys_adr)
64'h0:	romout <= 64'h0000237800000000;
64'h8:	romout <= 64'h3780000000030000;
64'h10:	romout <= 64'h0000037800000000;
64'h18:	romout <= 64'h37800000000DE000;
64'h20:	romout <= 64'h700003FFFFFFFFFF;
64'h28:	romout <= 64'h0D83E0000001200F;

endcase
assign sys_dbi = romout;

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
	.sys_adv(1'b0),
	.sys_adr(59'd0)
);
endmodule
