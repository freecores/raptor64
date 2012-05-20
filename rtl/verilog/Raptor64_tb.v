module Raptor64_tb();
parameter IDLE = 8'd1;
parameter DOCMD = 8'd2;

reg clk;
reg rst;
reg nmi;
wire sys_cyc;
wire sys_stb;
wire sys_we;
wire [7:0] sys_sel;
wire [63:0] sys_adr;
wire [63:0] sys_dbo;
wire [63:0] sys_dbi;
wire sys_ack;
reg [7:0] cnt;
wire wr_empty = 1'b1;
wire wr_full;
reg [63:0] romout;
wire stk_ack;
wire scr_ack;
wire br_ack;
wire [63:0] br_dato;
wire [63:0] stk_dato;
wire [63:0] scr_dato;
wire [15:0] tc_dato;
wire [15:0] pic_dato;
wire tc_ack;
wire pic_ack;
reg pulse1000Hz;

assign ram_ack = sys_cyc && sys_stb && (sys_adr[63:32]==32'd1);
assign sys_ack = br_ack|stk_ack|scr_ack|tc_ack|pic_ack|ram_ack;

initial begin
	clk = 1;
	pulse1000Hz = 0;
	rst = 0;
	nmi = 0;
	#100 rst = 1;
	#100 rst = 0;
	#800 nmi = 1;
	#100 nmi = 0;
	#500000 pulse1000Hz = 1;
	#10 pulse1000Hz = 0;
end

always #10 clk = ~clk;	//  50 MHz


reg pulse1000HzB;
always @(posedge clk)
if (rst) begin
	pulse1000HzB <= 1'b0;
end
else begin
	if (pulse1000Hz)
		pulse1000HzB <= 1'b1;
	else begin
	if (sys_adr==64'hFFFFFFFF_FFFF0000)
		pulse1000HzB <= 1'b0;
	end
end


rtfTextController tc1
(
	.rst_i(rst),
	.clk_i(clk),
	.cyc_i(sys_cyc),
	.stb_i(sys_stb),
	.ack_o(tc_ack),
	.we_i(sys_we),
	.sel_i(sys_sel[1:0]|sys_sel[3:2]|sys_sel[5:4]|sys_sel[7:6]),
	.adr_i(sys_adr),
	.dat_i(sys_dbo[15:0]),
	.dat_o(tc_dato),
	.lp(),
	.curpos(),
	.vclk(),
	.eol(),
	.eof(),
	.blank(),
	.border(),
	.rgbIn(),
	.rgbOut()
);

scratchmem u_sc
(
	.clk_i(clk),
	.cyc_i(sys_cyc),
	.stb_i(sys_stb),
	.ack_o(scr_ack),
	.we_i(sys_we),
	.sel_i(sys_sel),
	.adr_i(sys_adr),
	.dat_i(sys_dbo),
	.dat_o(scr_dato)
);

stkmem u_stk
(
	.clk_i(clk),
	.cyc_i(sys_cyc),
	.stb_i(sys_stb),
	.ack_o(stk_ack),
	.we_i(sys_we),
	.adr_i(sys_adr),
	.dat_i(sys_dbo),
	.dat_o(stk_dato)
);

bootrom u_br
(
	.clk_i(clk),
	.cyc_i(sys_cyc),
	.stb_i(sys_stb),
	.ack_o(br_ack),
	.adr_i(sys_adr),
	.dat_o(br_dato)
);


RaptorPIC u_pic
(
	.rst_i(rst),		// reset
	.clk_i(clk),	// system clock
	.cyc_i(sys_cyc),	// cycle valid
	.stb_i(sys_stb),	// strobe
	.ack_o(pic_ack),	// transfer acknowledge
	.we_i(sys_we),		// write
	.sel_i(sys_sel[1:0]|sys_sel[3:2]|sys_sel[5:4]|sys_sel[7:6]),			// byte select
	.adr_i(sys_adr),	// address
	.dat_i(sys_dbo[15:0]),
	.dat_o(pic_dato),
	.vol_o(),			// volatile register selected
	.i1(),
	.i2(pulse1000HzB),
	.i3(), .i4(), .i5(), .i6(), .i7(),
	.i8(), .i9(), .i10(), .i11(), .i12(), .i13(), .i14(),
	.i15(),
	.irqo(cpu_irq),	// normally connected to the processor irq
	.nmii(nmi),		// nmi input connected to nmi requester
	.nmio(cpu_nmi),	// normally connected to the nmi of cpu
	.irqenc()
);



reg [63:0] keybdout;
always @(sys_adr)
	if (sys_adr==64'hFFFF_FFFF_FFDC_0000) begin
		$display ("keyboard=FF");
		keybdout <= 64'hFFFF_FFFF_FFFF_FFFF;
	end
	else
		keybdout <= 64'd0;
	
always @(sys_adr)
case(sys_adr)// | 64'hFFFF_FFFF_FFFF_0000)
64'hFFFFFFFFFFFFE800:	romout <= 64'h000030000000000A;
64'hFFFFFFFFFFFFE808:	romout <= 64'h0BEFFFEFFF800000;
64'hFFFFFFFFFFFFE810:	romout <= 64'hFFE920C7FFFFFA66;
64'hFFFFFFFFFFFFE818:	romout <= 64'h0000000004031FFF;
64'hFFFFFFFFFFFFE820:	romout <= 64'h001050A3000000CE;
64'hFFFFFFFFFFFFE828:	romout <= 64'h1080600041462018;
64'hFFFFFFFFFFFFE830:	romout <= 64'h001058A300000020;
64'hFFFFFFFFFFFFE838:	romout <= 64'h1080400041662018;
64'hFFFFFFFFFFFFE840:	romout <= 64'h0002210804000416;
64'hFFFFFFFFFFFFE848:	romout <= 64'h0000000000DBE218;
64'hFFFFFFFFFFFFE850:	romout <= 64'hFFEBD10806000414;
64'hFFFFFFFFFFFFE858:	romout <= 64'h0C7FFFFFAE131FFF;
64'hFFFFFFFFFFFFE860:	romout <= 64'h0010618800000416;
64'hFFFFFFFFFFFFE868:	romout <= 64'h0A1FFFFE90062000;
64'hFFFFFFFFFFFFE870:	romout <= 64'hFFEF94E1FFFFFFFF;
64'hFFFFFFFFFFFFE878:	romout <= 64'h00802000AA831FFF;
64'hFFFFFFFFFFFFE880:	romout <= 64'h50C842F840000129;
64'hFFFFFFFFFFFFE888:	romout <= 64'h008400008A9285D9;
64'hFFFFFFFFFFFFE890:	romout <= 64'h72EA6008400009A9;
64'hFFFFFFFFFFFFE898:	romout <= 64'h00840000929287FB;
64'hFFFFFFFFFFFFE8A0:	romout <= 64'hFFF2440840000A29;
64'hFFFFFFFFFFFFE8A8:	romout <= 64'h2F80000002A33FFF;
64'hFFFFFFFFFFFFE8B0:	romout <= 64'h00000C3FBC000018;
64'hFFFFFFFFFFFFE8B8:	romout <= 64'h0A2FFFFE8F09FD00;
64'hFFFFFFFFFFFFE8C0:	romout <= 64'h0003210082000000;
64'hFFFFFFFFFFFFE8C8:	romout <= 64'h0C7FFFFFA58BE100;
64'hFFFFFFFFFFFFE8D0:	romout <= 64'hFFFE282884000001;
64'hFFFFFFFFFFFFE8D8:	romout <= 64'h27740000003BE007;
64'hFFFFFFFFFFFFE8E0:	romout <= 64'h000000DFBE000018;
64'hFFFFFFFFFFFFE8E8:	romout <= 64'h37800000000DE000;
64'hFFFFFFFFFFFFE8F0:	romout <= 64'h6F57206F6C6C6548;
64'hFFFFFFFFFFFFE8F8:	romout <= 64'h0000000021646C72;
64'hFFFFFFFFFFFFE900:	romout <= 64'h3436726F74706152;
64'hFFFFFFFFFFFFE908:	romout <= 64'h206D657473797320;
64'hFFFFFFFFFFFFE910:	romout <= 64'h676E697472617473;
64'hFFFFFFFFFFFFE918:	romout <= 64'h000000002E2E2E2E;
64'hFFFFFFFFFFFFE920:	romout <= 64'h703FC8A1FFFF8007;
64'hFFFFFFFFFFFFE928:	romout <= 64'h0DFBE0000009200F;
64'hFFFFFFFFFFFFE930:	romout <= 64'h0000003FBC000008;
64'hFFFFFFFFFFFFE938:	romout <= 64'h0A1FFDC0A0067E18;
64'hFFFFFFFFFFFFE940:	romout <= 64'h0000060046000001;
64'hFFFFFFFFFFFFE948:	romout <= 64'h2F8C000000814318;
64'hFFFFFFFFFFFFE950:	romout <= 64'h0000011F86000000;
64'hFFFFFFFFFFFFE958:	romout <= 64'h0DFBE00000880108;
64'hFFFFFFFFFFFFE960:	romout <= 64'h0000203FBC000010;
64'hFFFFFFFFFFFFE968:	romout <= 64'h19F8600000067E10;
64'hFFFFFFFFFFFFE970:	romout <= 64'h000004A3FFDC0A00;
64'hFFFFFFFFFFFFE978:	romout <= 64'h0508400004080310;
64'hFFFFFFFFFFFFE980:	romout <= 64'h000002F881FFFFA8;
64'hFFFFFFFFFFFFE988:	romout <= 64'h11F8600000090308;
64'hFFFFFFFFFFFFE990:	romout <= 64'h0000411F84000008;
64'hFFFFFFFFFFFFE998:	romout <= 64'h1800000045037EF8;
64'hFFFFFFFFFFFFE9A0:	romout <= 64'h0000058000000451;
64'hFFFFFFFFFFFFE9A8:	romout <= 64'h1800200041A28400;
64'hFFFFFFFFFFFFE9B0:	romout <= 64'h000060DFBE000000;
64'hFFFFFFFFFFFFE9B8:	romout <= 64'h27F000000070FEF0;
64'hFFFFFFFFFFFFE9C0:	romout <= 64'h00003D2802000450;
64'hFFFFFFFFFFFFE9C8:	romout <= 64'h0A30000044014108;
64'hFFFFFFFFFFFFE9D0:	romout <= 64'h700000E300000000;
64'hFFFFFFFFFFFFE9D8:	romout <= 64'h24801DC000282017;
64'hFFFFFFFFFFFFE9E0:	romout <= 64'h000005A8C2200010;
64'hFFFFFFFFFFFFE9E8:	romout <= 64'h0504200000F0A108;
64'hFFFFFFFFFFFFE9F0:	romout <= 64'h0011458002000450;
64'hFFFFFFFFFFFFE9F8:	romout <= 64'h2F8440001094A010;
64'hFFFFFFFFFFFFEA00:	romout <= 64'h00003C2884000001;
64'hFFFFFFFFFFFFEA08:	romout <= 64'h1800400045114210;
64'hFFFFFFFFFFFFEA10:	romout <= 64'h0000627700000007;
64'hFFFFFFFFFFFFEA18:	romout <= 64'h0DFBE0000000BEF0;
64'hFFFFFFFFFFFFEA20:	romout <= 64'h000001800200041A;
64'hFFFFFFFFFFFFEA28:	romout <= 64'h03FBC00001037EF8;
64'hFFFFFFFFFFFFEA30:	romout <= 64'h0011467F00000006;
64'hFFFFFFFFFFFFEA38:	romout <= 64'h128020004504A010;
64'hFFFFFFFFFFFFEA40:	romout <= 64'h000002F844000148;
64'hFFFFFFFFFFFFEA48:	romout <= 64'h1A8C410000528C00;
64'hFFFFFFFFFFFFEA50:	romout <= 64'h00003C2884000001;
64'hFFFFFFFFFFFFEA58:	romout <= 64'h1800400045114210;
64'hFFFFFFFFFFFFEA60:	romout <= 64'h0000427700000006;
64'hFFFFFFFFFFFFEA68:	romout <= 64'h0A1FFFFFFFF37EF8;
64'hFFFFFFFFFFFFEA70:	romout <= 64'h0000427700000006;
64'hFFFFFFFFFFFFEA78:	romout <= 64'h1280200045137EF8;
64'hFFFFFFFFFFFFEA80:	romout <= 64'h0002A12804000450;
64'hFFFFFFFFFFFFEA88:	romout <= 64'h0A100000001BE110;
64'hFFFFFFFFFFFFEA90:	romout <= 64'h400028DFBE000000;
64'hFFFFFFFFFFFFEA98:	romout <= 64'h0DFBE00000004108;
64'hFFFFFFFFFFFFEAA0:	romout <= 64'h0000083FBC000010;
64'hFFFFFFFFFFFFEAA8:	romout <= 64'h0A2FFDC00009FD00;
64'hFFFFFFFFFFFFEAB0:	romout <= 64'h0000060082000000;
64'hFFFFFFFFFFFFEAB8:	romout <= 64'h20080000002BE100;
64'hFFFFFFFFFFFFEAC0:	romout <= 64'h00106850420000FF;
64'hFFFFFFFFFFFFEAC8:	romout <= 64'h2F8800000C840010;
64'hFFFFFFFFFFFFEAD0:	romout <= 64'hFFED4AC04514000D;
64'hFFFFFFFFFFFFEAD8:	romout <= 64'h2774000000231FFF;
64'hFFFFFFFFFFFFEAE0:	romout <= 64'h000060DFBE000010;
64'hFFFFFFFFFFFFEAE8:	romout <= 64'h27F400000030FEF0;
64'hFFFFFFFFFFFFEAF0:	romout <= 64'h0000051802000400;
64'hFFFFFFFFFFFFEAF8:	romout <= 64'h1980200040008108;
64'hFFFFFFFFFFFFEB00:	romout <= 64'h000378A2FFD00000;
64'hFFFFFFFFFFFFEB08:	romout <= 64'h0284200000142208;
64'hFFFFFFFFFFFFEB10:	romout <= 64'hFC000188820000DE;
64'hFFFFFFFFFFFFEB18:	romout <= 64'h1180200040042007;
64'hFFFFFFFFFFFFEB20:	romout <= 64'h400100504200007F;
64'hFFFFFFFFFFFFEB28:	romout <= 64'h0C7FFFFFACEB2100;
64'hFFFFFFFFFFFFEB30:	romout <= 64'h0000627740000003;
64'hFFFFFFFFFFFFEB38:	romout <= 64'h03FBC00002037EF8;
64'hFFFFFFFFFFFFEB40:	romout <= 64'hFFED0A7F40000007;
64'hFFFFFFFFFFFFEB48:	romout <= 64'h0284201000031FFF;
64'hFFFFFFFFFFFFEB50:	romout <= 64'h0000050844000000;
64'hFFFFFFFFFFFFEB58:	romout <= 64'h188440000000A210;
64'hFFFFFFFFFFFFEB60:	romout <= 64'h0003211804000408;
64'hFFFFFFFFFFFFEB68:	romout <= 64'h10806000414BE110;
64'hFFFFFFFFFFFFEB70:	romout <= 64'h0010218886000000;
64'hFFFFFFFFFFFFEB78:	romout <= 64'h2774000000766008;
64'hFFFFFFFFFFFFEB80:	romout <= 64'h0000A0DFBE000020;
64'hFFFFFFFFFFFFEB88:	romout <= 64'h27F4000000F0FEF0;
64'hFFFFFFFFFFFFEB90:	romout <= 64'h000C00A100000554;
64'hFFFFFFFFFFFFEB98:	romout <= 64'h0104420001828800;
64'hFFFFFFFFFFFFEBA0:	romout <= 64'hA4A4A41004400009;
64'hFFFFFFFFFFFFEBA8:	romout <= 64'h0A300200000284A4;
64'hFFFFFFFFFFFFEBB0:	romout <= 64'h000000E300000001;
64'hFFFFFFFFFFFFEBB8:	romout <= 64'h028C600000464308;
64'hFFFFFFFFFFFFEBC0:	romout <= 64'h00003EF805FFFFAF;
64'hFFFFFFFFFFFFEBC8:	romout <= 64'h0DFBE0000289DD00;
64'hFFFFFFFFFFFFEBD0:	romout <= 64'h00003C3FBC000028;
64'hFFFFFFFFFFFFEBD8:	romout <= 64'h0A3FFDA00009FD00;
64'hFFFFFFFFFFFFEBE0:	romout <= 64'h00000908C2000000;
64'hFFFFFFFFFFFFEBE8:	romout <= 64'h0104420001842310;
64'hFFFFFFFFFFFFEBF0:	romout <= 64'h001050A100000020;
64'hFFFFFFFFFFFFEBF8:	romout <= 64'h0C7FFFFFB3242020;
64'hFFFFFFFFFFFFEC00:	romout <= 64'h000000A3FFD00000;
64'hFFFFFFFFFFFFEC08:	romout <= 64'h188C801000062308;
64'hFFFFFFFFFFFFEC10:	romout <= 64'hFFFEBC28C6000002;
64'hFFFFFFFFFFFFEC18:	romout <= 64'h2774000000FBE017;
64'hFFFFFFFFFFFFEC20:	romout <= 64'h0000A0DFBE000028;
64'hFFFFFFFFFFFFEC28:	romout <= 64'h27F4000000F0FEF0;
64'hFFFFFFFFFFFFEC30:	romout <= 64'h000000A3FFDA0000;
64'hFFFFFFFFFFFFEC38:	romout <= 64'h108C400000242308;
64'hFFFFFFFFFFFFEC40:	romout <= 64'h8000141044200018;
64'hFFFFFFFFFFFFEC48:	romout <= 64'h0A3FFD0000004208;
64'hFFFFFFFFFFFFEC50:	romout <= 64'h000001A8C2400001;
64'hFFFFFFFFFFFFEC58:	romout <= 64'h028C600000262320;
64'hFFFFFFFFFFFFEC60:	romout <= 64'h680002F805FFFF8F;
64'hFFFFFFFFFFFFEC68:	romout <= 64'h108C200000228FFF;
64'hFFFFFFFFFFFFEC70:	romout <= 64'hFFEC843842000001;
64'hFFFFFFFFFFFFEC78:	romout <= 64'h2774000000F31FFF;
64'hFFFFFFFFFFFFEC80:	romout <= 64'h000060DFBE000028;
64'hFFFFFFFFFFFFEC88:	romout <= 64'h27F000000070FEF0;
64'hFFFFFFFFFFFFEC90:	romout <= 64'h000000A3FFDA0000;
64'hFFFFFFFFFFFFEC98:	romout <= 64'h0108230001842310;
64'hFFFFFFFFFFFFECA0:	romout <= 64'h40000018C6080000;
64'hFFFFFFFFFFFFECA8:	romout <= 64'h0A1000000200A31F;
64'hFFFFFFFFFFFFECB0:	romout <= 64'h00000988C2000000;
64'hFFFFFFFFFFFFECB8:	romout <= 64'h2F80400000F0A318;
64'hFFFFFFFFFFFFECC0:	romout <= 64'h0000627700000007;
64'hFFFFFFFFFFFFECC8:	romout <= 64'h050420000FF37EF8;
64'hFFFFFFFFFFFFECD0:	romout <= 64'h60016AA040180041;
64'hFFFFFFFFFFFFECD8:	romout <= 64'h2B84018007AAC100;
64'hFFFFFFFFFFFFECE0:	romout <= 64'h000182A040080061;
64'hFFFFFFFFFFFFECE8:	romout <= 64'h058420001000C108;
64'hFFFFFFFFFFFFECF0:	romout <= 64'h0003FCDFBE000000;
64'hFFFFFFFFFFFFECF8:	romout <= 64'h2B84014001A14108;
64'hFFFFFFFFFFFFED00:	romout <= 64'h000000284200003C;
64'hFFFFFFFFFFFFED08:	romout <= 64'h1080200041637EF8;
64'hFFFFFFFFFFFFED10:	romout <= 64'h680000504200007F;
64'hFFFFFFFFFFFFED18:	romout <= 64'h208C400000028FFF;
64'hFFFFFFFFFFFFED20:	romout <= 64'h0010601082200018;
64'hFFFFFFFFFFFFED28:	romout <= 64'h0504200007F42008;
64'hFFFFFFFFFFFFED30:	romout <= 64'h0000581082200003;
64'hFFFFFFFFFFFFED38:	romout <= 64'h0188408000092310;
64'hFFFFFFFFFFFFED40:	romout <= 64'h0000002883D00000;
64'hFFFFFFFFFFFFED48:	romout <= 64'h2C84018000D37EF8;
64'hFFFFFFFFFFFFED50:	romout <= 64'h0000018800000418;
64'hFFFFFFFFFFFFED58:	romout <= 64'h2C84038009137EF8;
64'hFFFFFFFFFFFFED60:	romout <= 64'h0000003FBC000008;
64'hFFFFFFFFFFFFED68:	romout <= 64'h1080400041867E10;
64'hFFFFFFFFFFFFED70:	romout <= 64'h000006C080100038;
64'hFFFFFFFFFFFFED78:	romout <= 64'h188040004180A210;
64'hFFFFFFFFFFFFED80:	romout <= 64'h0000211F84000000;
64'hFFFFFFFFFFFFED88:	romout <= 64'h2C84034009037EF8;
64'hFFFFFFFFFFFFED90:	romout <= 64'h0000003FBC000008;
64'hFFFFFFFFFFFFED98:	romout <= 64'h1080400041667E10;
64'hFFFFFFFFFFFFEDA0:	romout <= 64'h000006C0BFE00000;
64'hFFFFFFFFFFFFEDA8:	romout <= 64'h188040004160E210;
64'hFFFFFFFFFFFFEDB0:	romout <= 64'hC0024EF801FFFE8A;
64'hFFFFFFFFFFFFEDB8:	romout <= 64'h03FBC000008B2100;
64'hFFFFFFFFFFFFEDC0:	romout <= 64'h0010619F84000000;
64'hFFFFFFFFFFFFEDC8:	romout <= 64'h2C0BFC0000042010;
64'hFFFFFFFFFFFFEDD0:	romout <= 64'h0010603884000001;
64'hFFFFFFFFFFFFEDD8:	romout <= 64'h2F801FFFD8A62010;
64'hFFFFFFFFFFFFEDE0:	romout <= 64'h000022C840280092;
64'hFFFFFFFFFFFFEDE8:	romout <= 64'h19F840000000FEF0;
64'hFFFFFFFFFFFFEDF0:	romout <= 64'h4000790804000416;
64'hFFFFFFFFFFFFEDF8:	romout <= 64'h02884000001B02FE;
64'hFFFFFFFFFFFFEE00:	romout <= 64'hFFF0298804000416;
64'hFFFFFFFFFFFFEE08:	romout <= 64'h2C840380094BE007;
64'hFFFFFFFFFFFFEE10:	romout <= 64'h0000003FBC000008;
64'hFFFFFFFFFFFFEE18:	romout <= 64'h1080400041867E10;
64'hFFFFFFFFFFFFEE20:	romout <= 64'h001062F880000088;
64'hFFFFFFFFFFFFEE28:	romout <= 64'h2F801FFFB0A62000;
64'hFFFFFFFFFFFFEE30:	romout <= 64'hFFEA298800000416;
64'hFFFFFFFFFFFFEE38:	romout <= 64'h03FBC000030BE007;
64'hFFFFFFFFFFFFEE40:	romout <= 64'h8002667F4000001F;
64'hFFFFFFFFFFFFEE48:	romout <= 64'h0C7FFFFFB42B2100;
64'hFFFFFFFFFFFFEE50:	romout <= 64'h0010601002300009;
64'hFFFFFFFFFFFFEE58:	romout <= 64'h2F8000001CA42008;
64'hFFFFFFFFFFFFEE60:	romout <= 64'h001062C840600008;
64'hFFFFFFFFFFFFEE68:	romout <= 64'h2F8800004C842010;
64'hFFFFFFFFFFFFEE70:	romout <= 64'h0010603884000001;
64'hFFFFFFFFFFFFEE78:	romout <= 64'h0C7FFFFFB4262010;
64'hFFFFFFFFFFFFEE80:	romout <= 64'h0010601002300009;
64'hFFFFFFFFFFFFEE88:	romout <= 64'h108C400000242008;
64'hFFFFFFFFFFFFEE90:	romout <= 64'h00000988C4000000;
64'hFFFFFFFFFFFFEE98:	romout <= 64'h028420000010A318;
64'hFFFFFFFFFFFFEEA0:	romout <= 64'h000000A4FFDA0000;
64'hFFFFFFFFFFFFEEA8:	romout <= 64'h2F84BFFFF4482428;
64'hFFFFFFFFFFFFEEB0:	romout <= 64'hFFFFF8A200000020;
64'hFFFFFFFFFFFFEEB8:	romout <= 64'h2F80000024A62317;
64'hFFFFFFFFFFFFEEC0:	romout <= 64'h000026C04034000A;
64'hFFFFFFFFFFFFEEC8:	romout <= 64'h0C7FFFFFB4204009;
64'hFFFFFFFFFFFFEED0:	romout <= 64'h4000241002300009;
64'hFFFFFFFFFFFFEED8:	romout <= 64'h0C7FFFFFB3204020;
64'hFFFFFFFFFFFFEEE0:	romout <= 64'hFFEF058803D00000;
64'hFFFFFFFFFFFFEEE8:	romout <= 64'h2774000001F31FFF;
64'hFFFFFFFFFFFFEEF0:	romout <= 64'hFFEF44DFBE000030;
64'hFFFFFFFFFFFFEEF8:	romout <= 64'h2774000001F31FFF;
64'hFFFFFFFFFFFFEF00:	romout <= 64'h000080DFBE000030;
64'hFFFFFFFFFFFFEF08:	romout <= 64'h27F400000070FEF0;
64'hFFFFFFFFFFFFEF10:	romout <= 64'h000058A3FFDA0000;
64'hFFFFFFFFFFFFEF18:	romout <= 64'h0284200000182308;
64'hFFFFFFFFFFFFEF20:	romout <= 64'h00106248C2000016;
64'hFFFFFFFFFFFFEF28:	romout <= 64'h0284200000142008;
64'hFFFFFFFFFFFFEF30:	romout <= 64'h0000018802000418;
64'hFFFFFFFFFFFFEF38:	romout <= 64'h2F8440002C682310;
64'hFFFFFFFFFFFFEF40:	romout <= 64'h0010598800000418;
64'hFFFFFFFFFFFFEF48:	romout <= 64'h0284200000142008;
64'hFFFFFFFFFFFFEF50:	romout <= 64'h6800018802000416;
64'hFFFFFFFFFFFFEF58:	romout <= 64'h208C400000228FFF;
64'hFFFFFFFFFFFFEF60:	romout <= 64'h000006F844000146;
64'hFFFFFFFFFFFFEF68:	romout <= 64'h188040004160E210;
64'hFFFFFFFFFFFFEF70:	romout <= 64'h00005908C4000000;
64'hFFFFFFFFFFFFEF78:	romout <= 64'h0104410000542308;
64'hFFFFFFFFFFFFEF80:	romout <= 64'hFFEC2588C2000016;
64'hFFFFFFFFFFFFEF88:	romout <= 64'h2774000000731FFF;
64'hFFFFFFFFFFFFEF90:	romout <= 64'h000060DFBE000020;
64'hFFFFFFFFFFFFEF98:	romout <= 64'h27F400000030DEF0;
64'hFFFFFFFFFFFFEFA0:	romout <= 64'h0000001040200009;
64'hFFFFFFFFFFFFEFA8:	romout <= 64'h028840000014A208;
64'hFFFFFFFFFFFFEFB0:	romout <= 64'hFFED4AF840000088;
64'hFFFFFFFFFFFFEFB8:	romout <= 64'h2F801FFFFAA31FFF;
64'hFFFFFFFFFFFFEFC0:	romout <= 64'h0000627740000003;
64'hFFFFFFFFFFFFEFC8:	romout <= 64'h03FBC00000837EF8;
64'hFFFFFFFFFFFFEFD0:	romout <= 64'hFFEF959FBE000000;
64'hFFFFFFFFFFFFEFD8:	romout <= 64'h11FBE00000031FFF;
64'hFFFFFFFFFFFFEFE0:	romout <= 64'h0000402FBC000008;
64'hFFFFFFFFFFFFEFE8:	romout <= 64'h19F820000000DEF0;
64'hFFFFFFFFFFFFEFF0:	romout <= 64'h0000359FBE000008;
64'hFFFFFFFFFFFFEFF8:	romout <= 64'h0C7FFFFFB5228400;
64'hFFFFFFFFFFFFF000:	romout <= 64'hFFED48A10000000A;
64'hFFFFFFFFFFFFF008:	romout <= 64'h11F8200000031FFF;
64'hFFFFFFFFFFFFF010:	romout <= 64'h0000411FBE000008;
64'hFFFFFFFFFFFFF018:	romout <= 64'h037BC00001037EF8;
64'hFFFFFFFFFFFFF020:	romout <= 64'h0000019FBE000008;
64'hFFFFFFFFFFFFF028:	romout <= 64'h0504200000F67E08;
64'hFFFFFFFFFFFFF030:	romout <= 64'h4000E42042000030;
64'hFFFFFFFFFFFFF038:	romout <= 64'h02042000007AC100;
64'hFFFFFFFFFFFFF040:	romout <= 64'h000000C7FFFFFB52;
64'hFFFFFFFFFFFFF048:	romout <= 64'h11FBE00000847E08;
64'hFFFFFFFFFFFFF050:	romout <= 64'h000040DFBE000010;
64'hFFFFFFFFFFFFF058:	romout <= 64'h27F400000010FEF0;
64'hFFFFFFFFFFFFF060:	romout <= 64'hFFF0181842200004;
64'hFFFFFFFFFFFFF068:	romout <= 64'h0184220000231FFF;
64'hFFFFFFFFFFFFF070:	romout <= 64'h000004C7FFFFFC06;
64'hFFFFFFFFFFFFF078:	romout <= 64'h0DFBE0000109DD00;
64'hFFFFFFFFFFFFF080:	romout <= 64'h0000143FBC000018;
64'hFFFFFFFFFFFFF088:	romout <= 64'h0A3000000079FD00;
64'hFFFFFFFFFFFFF090:	romout <= 64'hFFF0541842400002;
64'hFFFFFFFFFFFFF098:	romout <= 64'h2F80600000F31FFF;
64'hFFFFFFFFFFFFF0A0:	romout <= 64'h0000627740000005;
64'hFFFFFFFFFFFFF0A8:	romout <= 64'h0A10000003A37EF8;
64'hFFFFFFFFFFFFF0B0:	romout <= 64'h400024C7FFFFFB52;
64'hFFFFFFFFFFFFF0B8:	romout <= 64'h0C7FFFFFC2004200;
64'hFFFFFFFFFFFFF0C0:	romout <= 64'h000080A300000007;
64'hFFFFFFFFFFFFF0C8:	romout <= 64'h0C7FFFFFB5228400;
64'hFFFFFFFFFFFFF0D0:	romout <= 64'hFFF0550082000000;
64'hFFFFFFFFFFFFF0D8:	romout <= 64'h0288400000131FFF;
64'hFFFFFFFFFFFFF0E0:	romout <= 64'hFFEFE6F807FFFF2F;
64'hFFFFFFFFFFFFF0E8:	romout <= 64'h03FBC00003033FFF;
64'hFFFFFFFFFFFFF0F0:	romout <= 64'h00002A7F000000FC;
64'hFFFFFFFFFFFFF0F8:	romout <= 64'h0A80000001328800;
64'hFFFFFFFFFFFFF100:	romout <= 64'h800000104430001C;
64'hFFFFFFFFFFFFF108:	romout <= 64'h0194FE000000631F;
64'hFFFFFFFFFFFFF110:	romout <= 64'h0000241908200001;
64'hFFFFFFFFFFFFF118:	romout <= 64'h0194A20000104439;
64'hFFFFFFFFFFFFF120:	romout <= 64'h0000001146500009;
64'hFFFFFFFFFFFFF128:	romout <= 64'h2F811FFFF0F1C108;
64'hFFFFFFFFFFFFF130:	romout <= 64'h0000001909800001;
64'hFFFFFFFFFFFFF138:	romout <= 64'h0110C40000906532;
64'hFFFFFFFFFFFFF140:	romout <= 64'h400024194B800001;
64'hFFFFFFFFFFFFF148:	romout <= 64'h0100A20000904020;
64'hFFFFFFFFFFFFF150:	romout <= 64'h0000C277000000FC;
64'hFFFFFFFFFFFFF158:	romout <= 64'h03FBC00002037EF8;
64'hFFFFFFFFFFFFF160:	romout <= 64'h00003E7F0000009C;
64'hFFFFFFFFFFFFF168:	romout <= 64'h0504400000F2A000;
64'hFFFFFFFFFFFFF170:	romout <= 64'h0000005884000030;
64'hFFFFFFFFFFFFF178:	romout <= 64'h0190840000106217;
64'hFFFFFFFFFFFFF180:	romout <= 64'h00002418CBC00000;
64'hFFFFFFFFFFFFF188:	romout <= 64'h018C640000104429;
64'hFFFFFFFFFFFFF190:	romout <= 64'h80000410C4300009;
64'hFFFFFFFFFFFFF198:	romout <= 64'h2F811FFFECF06108;
64'hFFFFFFFFFFFFF1A0:	romout <= 64'h8000241008100009;
64'hFFFFFFFFFFFFF1A8:	romout <= 64'h2770000009C04018;
64'hFFFFFFFFFFFFF1B0:	romout <= 64'h0000E0DFBE000020;
64'hFFFFFFFFFFFFF1B8:	romout <= 64'h27F400007C40FEF0;
64'hFFFFFFFFFFFFF1C0:	romout <= 64'hFFF0E81004B00009;
64'hFFFFFFFFFFFFF1C8:	romout <= 64'h01004A0000931FFF;
64'hFFFFFFFFFFFFF1D0:	romout <= 64'h000004C7FFFFFC56;
64'hFFFFFFFFFFFFF1D8:	romout <= 64'h0A8000000072A400;
64'hFFFFFFFFFFFFF1E0:	romout <= 64'h0000001A4E180000;
64'hFFFFFFFFFFFFF1E8:	romout <= 64'h029CE0000040A738;
64'hFFFFFFFFFFFFF1F0:	romout <= 64'hC0004050460000FF;
64'hFFFFFFFFFFFFF1F8:	romout <= 64'h018424000016A758;
64'hFFFFFFFFFFFFF200:	romout <= 64'h400026F811FFFF0F;
64'hFFFFFFFFFFFFF208:	romout <= 64'h2F813FFFECF04010;
64'hFFFFFFFFFFFFF210:	romout <= 64'hFFF1581014100009;
64'hFFFFFFFFFFFFF218:	romout <= 64'h0A80000000331FFF;
64'hFFFFFFFFFFFFF220:	romout <= 64'hC0004050460000FF;
64'hFFFFFFFFFFFFF228:	romout <= 64'h018424000016A858;
64'hFFFFFFFFFFFFF230:	romout <= 64'h000052F811FFFF8F;
64'hFFFFFFFFFFFFF238:	romout <= 64'h277400007C460B00;
64'hFFFFFFFFFFFFF240:	romout <= 64'hFFBFE0DFBE000038;
64'hFFFFFFFFFFFFF248:	romout <= 64'h1800000041A2FBFF;
64'hFFFFFFFFFFFFF250:	romout <= 64'h000090C7FFFFFBF9;
64'hFFFFFFFFFFFFF258:	romout <= 64'h0C7FFFFFB5228400;
64'hFFFFFFFFFFFFF260:	romout <= 64'h0FFFFCC7FFFFFA8A;
64'hFFFFFFFFFFFFF268:	romout <= 64'h2C04018000DB0100;
64'hFFFFFFFFFFFFF270:	romout <= 64'hFFFE28C7FFFFFB52;
64'hFFFFFFFFFFFFF278:	romout <= 64'h18800000418BE007;
64'hFFFFFFFFFFFFF280:	romout <= 64'hC00024C7FFFFFB42;
64'hFFFFFFFFFFFFF288:	romout <= 64'h108C200000004100;
64'hFFFFFFFFFFFFF290:	romout <= 64'hFFECF428C6000002;
64'hFFFFFFFFFFFFF298:	romout <= 64'h2C84020002431FFF;
64'hFFFFFFFFFFFFF2A0:	romout <= 64'h00000908C2000000;
64'hFFFFFFFFFFFFF2A8:	romout <= 64'h0C7FFFFFB3D0A318;
64'hFFFFFFFFFFFFF2B0:	romout <= 64'h000112C04130003A;
64'hFFFFFFFFFFFFF2B8:	romout <= 64'h2C075500042B0106;
64'hFFFFFFFFFFFFF2C0:	romout <= 64'h800132C04154004A;
64'hFFFFFFFFFFFFF2C8:	romout <= 64'h2C04050003FB0109;
64'hFFFFFFFFFFFFF2D0:	romout <= 64'hFFEEAAC040080043;
64'hFFFFFFFFFFFFF2D8:	romout <= 64'h108C2000000BE007;
64'hFFFFFFFFFFFFF2E0:	romout <= 64'hFFECF428C6000002;
64'hFFFFFFFFFFFFF2E8:	romout <= 64'h2C87F64004C31FFF;
64'hFFFFFFFFFFFFF2F0:	romout <= 64'h00000908C2000000;
64'hFFFFFFFFFFFFF2F8:	romout <= 64'h0C7FFFFFB3D0A318;
64'hFFFFFFFFFFFFF300:	romout <= 64'hFFEBD2C87F440053;
64'hFFFFFFFFFFFFF308:	romout <= 64'h2F801FFFA2A31FFF;
64'hFFFFFFFFFFFFF310:	romout <= 64'hFFEF94A1FFFFF320;
64'hFFFFFFFFFFFFF318:	romout <= 64'h2F801FFF9AA31FFF;
64'hFFFFFFFFFFFFF320:	romout <= 64'h70736944203D203F;
64'hFFFFFFFFFFFFF328:	romout <= 64'h706C65682079616C;
64'hFFFFFFFFFFFFF330:	romout <= 64'h203D20534C430A0D;
64'hFFFFFFFFFFFFF338:	romout <= 64'h6373207261656C63;
64'hFFFFFFFFFFFFF340:	romout <= 64'h203A0A0D6E656572;
64'hFFFFFFFFFFFFF348:	romout <= 64'h6D2074696445203D;
64'hFFFFFFFFFFFFF350:	romout <= 64'h79622079726F6D65;
64'hFFFFFFFFFFFFF358:	romout <= 64'h3D204C0A0D736574;
64'hFFFFFFFFFFFFF360:	romout <= 64'h31532064616F4C20;
64'hFFFFFFFFFFFFF368:	romout <= 64'h0A0D656C69662039;
64'hFFFFFFFFFFFFF370:	romout <= 64'h706D7544203D2044;
64'hFFFFFFFFFFFFF378:	romout <= 64'h0D79726F6D656D20;
64'hFFFFFFFFFFFFF380:	romout <= 64'h617473203D20420A;
64'hFFFFFFFFFFFFF388:	romout <= 64'h20796E6974207472;
64'hFFFFFFFFFFFFF390:	romout <= 64'h4A0A0D6369736162;
64'hFFFFFFFFFFFFF398:	romout <= 64'h20706D754A203D20;
64'hFFFFFFFFFFFFF3A0:	romout <= 64'h0D65646F63206F74;
64'hFFFFFFFFFFFFF3A8:	romout <= 64'hFFFFFFFFFFFF000A;
64'hFFFFFFFFFFFFF3B0:	romout <= 64'h0000003FBC000008;
64'hFFFFFFFFFFFFF3B8:	romout <= 64'h108C200000067EF8;
64'hFFFFFFFFFFFFF3C0:	romout <= 64'hFFECF428C6000002;
64'hFFFFFFFFFFFFF3C8:	romout <= 64'h2C07FF0002031FFF;
64'hFFFFFFFFFFFFF3D0:	romout <= 64'h00000038C6000002;
64'hFFFFFFFFFFFFF3D8:	romout <= 64'h0DFBE00000847EF8;
64'hFFFFFFFFFFFFF3E0:	romout <= 64'hFFF474C7FFFFFCEC;
64'hFFFFFFFFFFFFF3E8:	romout <= 64'h0104050000931FFF;
64'hFFFFFFFFFFFFF3F0:	romout <= 64'hFFF3B0A400000007;
64'hFFFFFFFFFFFFF3F8:	romout <= 64'h0C7FFFFFD1D31FFF;
64'hFFFFFFFFFFFFF400:	romout <= 64'h0000058142000000;
64'hFFFFFFFFFFFFF408:	romout <= 64'h2F809FFFFAF0A528;
64'hFFFFFFFFFFFFF410:	romout <= 64'hFFF3B2F801FFF1AA;
64'hFFFFFFFFFFFFF418:	romout <= 64'h0C7FFFFFD1D31FFF;
64'hFFFFFFFFFFFFF420:	romout <= 64'h0000001040300009;
64'hFFFFFFFFFFFFF428:	romout <= 64'h2F801FFF12A343F8;
64'hFFFFFFFFFFFFF430:	romout <= 64'hFFF474C7FFFFFCEC;
64'hFFFFFFFFFFFFF438:	romout <= 64'h0104020000931FFF;
64'hFFFFFFFFFFFFF440:	romout <= 64'hFFF0A8C7FFFFFBF9;
64'hFFFFFFFFFFFFF448:	romout <= 64'h0C7FFFFFC2A31FFF;
64'hFFFFFFFFFFFFF450:	romout <= 64'hFFF0A8C7FFFFFC2A;
64'hFFFFFFFFFFFFF458:	romout <= 64'h0C7FFFFFC2A31FFF;
64'hFFFFFFFFFFFFF460:	romout <= 64'hFFF0A8C7FFFFFC2A;
64'hFFFFFFFFFFFFF468:	romout <= 64'h0C7FFFFFC2A31FFF;
64'hFFFFFFFFFFFFF470:	romout <= 64'h000062F801FFEEAA;
64'hFFFFFFFFFFFFF478:	romout <= 64'h27F4000000A0FEF0;
64'hFFFFFFFFFFFFF480:	romout <= 64'h00003CA200000000;
64'hFFFFFFFFFFFFF488:	romout <= 64'h108C200000029000;
64'hFFFFFFFFFFFFF490:	romout <= 64'hFFECF428C6000002;
64'hFFFFFFFFFFFFF498:	romout <= 64'h0C7FFFFFD3231FFF;
64'hFFFFFFFFFFFFF4A0:	romout <= 64'h800002C0401BFFFF;
64'hFFFFFFFFFFFFF4A8:	romout <= 64'h0504200000F06210;
64'hFFFFFFFFFFFFF4B0:	romout <= 64'hFFFB3C1082200009;
64'hFFFFFFFFFFFFF4B8:	romout <= 64'h01080100009BE027;
64'hFFFFFFFFFFFFF4C0:	romout <= 64'h000062774000000A;
64'hFFFFFFFFFFFFF4C8:	romout <= 64'h2A04054003037EF8;
64'hFFFFFFFFFFFFF4D0:	romout <= 64'h0000C2B840100039;
64'hFFFFFFFFFFFFF4D8:	romout <= 64'h0DFBE0000000E108;
64'hFFFFFFFFFFFFF4E0:	romout <= 64'h60011AA040340041;
64'hFFFFFFFFFFFFF4E8:	romout <= 64'h03842000041AE100;
64'hFFFFFFFFFFFFF4F0:	romout <= 64'h000000284200000A;
64'hFFFFFFFFFFFFF4F8:	romout <= 64'h2A04024006137EF8;
64'hFFFFFFFFFFFFF500:	romout <= 64'h000186B840140066;
64'hFFFFFFFFFFFFF508:	romout <= 64'h0284200000A0E108;
64'hFFFFFFFFFFFFF510:	romout <= 64'hFFFFFCDFBE000000;
64'hFFFFFFFFFFFFF518:	romout <= 64'h0DFBE000000287FF;
64'hFFFFFFFFFFFFF520:	romout <= 64'hFFF782F80000008A;
64'hFFFFFFFFFFFFF528:	romout <= 64'h2C84004000A31FFF;
64'hFFFFFFFFFFFFF530:	romout <= 64'h500068C7FFFFFDE0;
64'hFFFFFFFFFFFFF538:	romout <= 64'h2C87FF40053B01F4;
64'hFFFFFFFFFFFFF540:	romout <= 64'h9000C0C7FFFFFDE0;
64'hFFFFFFFFFFFFF548:	romout <= 64'h2987FE40039A01FF;
64'hFFFFFFFFFFFFF550:	romout <= 64'hFFF7801040400009;
64'hFFFFFFFFFFFFF558:	romout <= 64'h0C7FFFFFD3231FFF;
64'hFFFFFFFFFFFFF560:	romout <= 64'hFFF7801040200009;
64'hFFFFFFFFFFFFF568:	romout <= 64'h0C7FFFFFD3231FFF;
64'hFFFFFFFFFFFFF570:	romout <= 64'h8000241884200000;
64'hFFFFFFFFFFFFF578:	romout <= 64'h0108230000904208;
64'hFFFFFFFFFFFFF580:	romout <= 64'h9000C6C13FA40030;
64'hFFFFFFFFFFFFF588:	romout <= 64'h2C100B00032B0402;
64'hFFFFFFFFFFFFF590:	romout <= 64'h5000D6C100A80033;
64'hFFFFFFFFFFFFF598:	romout <= 64'h2C100B40037B04FE;
64'hFFFFFFFFFFFFF5A0:	romout <= 64'h1000E6C100B40038;
64'hFFFFFFFFFFFFF5A8:	romout <= 64'h2F801FFFC2AB0403;
64'hFFFFFFFFFFFFF5B0:	romout <= 64'h00000450C60000FF;
64'hFFFFFFFFFFFFF5B8:	romout <= 64'h0C7FFFFFDE00E318;
64'hFFFFFFFFFFFFF5C0:	romout <= 64'h800000C7FFFFFD32;
64'hFFFFFFFFFFFFF5C8:	romout <= 64'h0108220000906210;
64'hFFFFFFFFFFFFF5D0:	romout <= 64'hFFF4C8C7FFFFFDE0;
64'hFFFFFFFFFFFFF5D8:	romout <= 64'h0188420000031FFF;
64'hFFFFFFFFFFFFF5E0:	romout <= 64'h0000001082200009;
64'hFFFFFFFFFFFFF5E8:	romout <= 64'h0294A00000160510;
64'hFFFFFFFFFFFFF5F0:	romout <= 64'hFFF782F807FFFE4F;
64'hFFFFFFFFFFFFF5F8:	romout <= 64'h0C7FFFFFD3231FFF;
64'hFFFFFFFFFFFFF600:	romout <= 64'h8000241884200000;
64'hFFFFFFFFFFFFF608:	romout <= 64'h0C7FFFFFDE004208;
64'hFFFFFFFFFFFFF610:	romout <= 64'h800000C7FFFFFD32;
64'hFFFFFFFFFFFFF618:	romout <= 64'h0108220000906210;
64'hFFFFFFFFFFFFF620:	romout <= 64'hFFF676F801FFF82A;
64'hFFFFFFFFFFFFF628:	romout <= 64'h2F801FFFC8A31FFF;
64'hFFFFFFFFFFFFF630:	romout <= 64'hFFF028C7FFFFFDA5;
64'hFFFFFFFFFFFFF638:	romout <= 64'h0C7FFFFFDADBE007;
64'hFFFFFFFFFFFFF640:	romout <= 64'hFFF6B6F801FFFB8A;
64'hFFFFFFFFFFFFF648:	romout <= 64'h1980A00000031FFF;
64'hFFFFFFFFFFFFF650:	romout <= 64'hFFF696F801FFDFAA;
64'hFFFFFFFFFFFFF658:	romout <= 64'h1980A00000031FFF;
64'hFFFFFFFFFFFFF660:	romout <= 64'hFFF676F801FFDF2A;
64'hFFFFFFFFFFFFF668:	romout <= 64'h1980A00000031FFF;
64'hFFFFFFFFFFFFF670:	romout <= 64'h000022F801FFDEAA;
64'hFFFFFFFFFFFFF678:	romout <= 64'h19FBE0000000FEF0;
64'hFFFFFFFFFFFFF680:	romout <= 64'hFFF4C8C7FFFFFDE0;
64'hFFFFFFFFFFFFF688:	romout <= 64'h0104020000931FFF;
64'hFFFFFFFFFFFFF690:	romout <= 64'h000022F8000004AA;
64'hFFFFFFFFFFFFF698:	romout <= 64'h19FBE0000000FEF0;
64'hFFFFFFFFFFFFF6A0:	romout <= 64'hFFF4C8C7FFFFFDE0;
64'hFFFFFFFFFFFFF6A8:	romout <= 64'h0104020000931FFF;
64'hFFFFFFFFFFFFF6B0:	romout <= 64'h000022F80000024A;
64'hFFFFFFFFFFFFF6B8:	romout <= 64'h19FBE0000000FEF0;
64'hFFFFFFFFFFFFF6C0:	romout <= 64'hFFF4C8C7FFFFFDE0;
64'hFFFFFFFFFFFFF6C8:	romout <= 64'h0104020000931FFF;
64'hFFFFFFFFFFFFF6D0:	romout <= 64'hFFF4C8C7FFFFFDE0;
64'hFFFFFFFFFFFFF6D8:	romout <= 64'h0188420000031FFF;
64'hFFFFFFFFFFFFF6E0:	romout <= 64'hFFF7801044200009;
64'hFFFFFFFFFFFFF6E8:	romout <= 64'h0C7FFFFFD3231FFF;
64'hFFFFFFFFFFFFF6F0:	romout <= 64'h8000241884200000;
64'hFFFFFFFFFFFFF6F8:	romout <= 64'h0C7FFFFFDE004208;
64'hFFFFFFFFFFFFF700:	romout <= 64'h800000C7FFFFFD32;
64'hFFFFFFFFFFFFF708:	romout <= 64'h0108220000906210;
64'hFFFFFFFFFFFFF710:	romout <= 64'hFFF4C8C7FFFFFDE0;
64'hFFFFFFFFFFFFF718:	romout <= 64'h0188420000031FFF;
64'hFFFFFFFFFFFFF720:	romout <= 64'hFFF7801082200009;
64'hFFFFFFFFFFFFF728:	romout <= 64'h0C7FFFFFD3231FFF;
64'hFFFFFFFFFFFFF730:	romout <= 64'h8000241884200000;
64'hFFFFFFFFFFFFF738:	romout <= 64'h0C7FFFFFDE004208;
64'hFFFFFFFFFFFFF740:	romout <= 64'h800000C7FFFFFD32;
64'hFFFFFFFFFFFFF748:	romout <= 64'h0108220000906210;
64'hFFFFFFFFFFFFF750:	romout <= 64'hFFF4C8C7FFFFFDE0;
64'hFFFFFFFFFFFFF758:	romout <= 64'h0188420000031FFF;
64'hFFFFFFFFFFFFF760:	romout <= 64'h0000281082200009;
64'hFFFFFFFFFFFFF768:	romout <= 64'h0108050000904421;
64'hFFFFFFFFFFFFF770:	romout <= 64'h0000211FBE000000;
64'hFFFFFFFFFFFFF778:	romout <= 64'h0DFBE0000000BEF0;
64'hFFFFFFFFFFFFF780:	romout <= 64'h0000003FBC000008;
64'hFFFFFFFFFFFFF788:	romout <= 64'h0C7FFFFFA9E67EF8;
64'hFFFFFFFFFFFFF790:	romout <= 64'hFFEA2AF841FF8D88;
64'hFFFFFFFFFFFFF798:	romout <= 64'h2C07AB4000031FFF;
64'hFFFFFFFFFFFFF7A0:	romout <= 64'hFFFD20C7FFFFFDEE;
64'hFFFFFFFFFFFFF7A8:	romout <= 64'h11FBE000000BE107;
64'hFFFFFFFFFFFFF7B0:	romout <= 64'h0000002FBC000008;
64'hFFFFFFFFFFFFF7B8:	romout <= 64'h20003DC0A0137EF8;
64'hFFFFFFFFFFFFF7C0:	romout <= 64'h0002A05042000001;
64'hFFFFFFFFFFFFF7C8:	romout <= 64'h20003DC0A00BE100;
64'hFFFFFFFFFFFFF7D0:	romout <= 64'h000000504200007F;
64'hFFFFFFFFFFFFF7D8:	romout <= 64'h0CFFFFFFC9137EF8;
64'hFFFFFFFFFFFFF7E0:	romout <= 64'h726F747061520A0D;
64'hFFFFFFFFFFFFF7E8:	romout <= 64'h20796E6954203436;
64'hFFFFFFFFFFFFF7F0:	romout <= 64'h3176204349534142;
64'hFFFFFFFFFFFFF7F8:	romout <= 64'h202943280A0D302E;
64'hFFFFFFFFFFFFF800:	romout <= 64'h6F52202032313032;
64'hFFFFFFFFFFFFF808:	romout <= 64'h6E69462074726562;
64'hFFFFFFFFFFFFF810:	romout <= 64'h0A0D000A0A0D6863;
64'hFFFFFFFFFFFFF818:	romout <= 64'h616857000A0D4B4F;
64'hFFFFFFFFFFFFF820:	romout <= 64'h726F53000A0D3F74;
64'hFFFFFFFFFFFFF828:	romout <= 64'h6F43000A0D2E7972;
64'hFFFFFFFFFFFFF830:	romout <= 64'h4C4620746361706D;
64'hFFFFFFFFFFFFF838:	romout <= 64'h6461657220485341;
64'hFFFFFFFFFFFFF840:	romout <= 64'h0A0D726F72726520;
64'hFFFFFFFFFFFFF848:	romout <= 64'h207265626D754E00;
64'hFFFFFFFFFFFFF850:	romout <= 64'h62206F6F74207369;
64'hFFFFFFFFFFFFF858:	romout <= 64'h766944000A0D6769;
64'hFFFFFFFFFFFFF860:	romout <= 64'h7962206E6F697369;
64'hFFFFFFFFFFFFF868:	romout <= 64'h000A0D6F72657A20;
64'hFFFFFFFFFFFFF870:	romout <= 64'h7620666F2074754F;
64'hFFFFFFFFFFFFF878:	romout <= 64'h20656C6261697261;
64'hFFFFFFFFFFFFF880:	romout <= 64'h000A0D6563617073;
64'hFFFFFFFFFFFFF888:	romout <= 64'h6620736574796220;
64'hFFFFFFFFFFFFF890:	romout <= 64'h0A0D000A0D656572;
64'hFFFFFFFFFFFFF898:	romout <= 64'h000A0D7964616552;
64'hFFFFFFFFFFFFF8A0:	romout <= 64'h6E69746365707845;
64'hFFFFFFFFFFFFF8A8:	romout <= 64'h6D6D6F6320612067;
64'hFFFFFFFFFFFFF8B0:	romout <= 64'h656E694C000A0D61;
64'hFFFFFFFFFFFFF8B8:	romout <= 64'h207265626D756E20;
64'hFFFFFFFFFFFFF8C0:	romout <= 64'h0D676962206F6F74;
64'hFFFFFFFFFFFFF8C8:	romout <= 64'h746365707845000A;
64'hFFFFFFFFFFFFF8D0:	romout <= 64'h6176206120676E69;
64'hFFFFFFFFFFFFF8D8:	romout <= 64'h0A0D656C62616972;
64'hFFFFFFFFFFFFF8E0:	romout <= 64'h64616220444E5200;
64'hFFFFFFFFFFFFF8E8:	romout <= 64'h74656D6172617020;
64'hFFFFFFFFFFFFF8F0:	romout <= 64'h535953000A0D7265;
64'hFFFFFFFFFFFFF8F8:	romout <= 64'h6464612064616220;
64'hFFFFFFFFFFFFF900:	romout <= 64'h49000A0D73736572;
64'hFFFFFFFFFFFFF908:	romout <= 64'h707865205455504E;
64'hFFFFFFFFFFFFF910:	romout <= 64'h6120676E69746365;
64'hFFFFFFFFFFFFF918:	romout <= 64'h6C62616972617620;
64'hFFFFFFFFFFFFF920:	romout <= 64'h5458454E000A0D65;
64'hFFFFFFFFFFFFF928:	romout <= 64'h74756F6874697720;
64'hFFFFFFFFFFFFF930:	romout <= 64'h4E000A0D524F4620;
64'hFFFFFFFFFFFFF938:	romout <= 64'h6570786520545845;
64'hFFFFFFFFFFFFF940:	romout <= 64'h206120676E697463;
64'hFFFFFFFFFFFFF948:	romout <= 64'h2064656E69666564;
64'hFFFFFFFFFFFFF950:	romout <= 64'h656C626169726176;
64'hFFFFFFFFFFFFF958:	romout <= 64'h2F4F544F47000A0D;
64'hFFFFFFFFFFFFF960:	romout <= 64'h6162204255534F47;
64'hFFFFFFFFFFFFF968:	romout <= 64'h6E20656E696C2064;
64'hFFFFFFFFFFFFF970:	romout <= 64'h000A0D7265626D75;
64'hFFFFFFFFFFFFF978:	romout <= 64'h77204E5255544552;
64'hFFFFFFFFFFFFF980:	romout <= 64'h472074756F687469;
64'hFFFFFFFFFFFFF988:	romout <= 64'h50000A0D4255534F;
64'hFFFFFFFFFFFFF990:	romout <= 64'h69206D6172676F72;
64'hFFFFFFFFFFFFF998:	romout <= 64'h6962206F6F742073;
64'hFFFFFFFFFFFFF9A0:	romout <= 64'h72747845000A0D67;
64'hFFFFFFFFFFFFF9A8:	romout <= 64'h6361726168632061;
64'hFFFFFFFFFFFFF9B0:	romout <= 64'h206E6F2073726574;
64'hFFFFFFFFFFFFF9B8:	romout <= 64'h6E676920656E696C;
64'hFFFFFFFFFFFFF9C0:	romout <= 64'h0D000A0D6465726F;
64'hFFFFFFFFFFFFF9C8:	romout <= 64'h0D000A0A0D00520A;
64'hFFFFFFFFFFFFF9D0:	romout <= 64'h0048000A0D004F0A;
64'hFFFFFFFFFFFFF9D8:	romout <= 64'h000A0D0057000A0D;
64'hFFFFFFFFFFFFF9E0:	romout <= 64'hFFFFFF000A0D0053;
64'hFFFFFFFFFFFFF9E8:	romout <= 64'hFFFFFFFFFFFFFFFF;
64'hFFFFFFFFFFFFF9F0:	romout <= 64'hAAAB541000800009;
64'hFFFFFFFFFFFFF9F8:	romout <= 64'h05802AA5555F5554;
64'hFFFFFFFFFFFFFA00:	romout <= 64'h0000019A02000000;
64'hFFFFFFFFFFFFFA08:	romout <= 64'h0104430000646810;
64'hFFFFFFFFFFFFFA10:	romout <= 64'h000022F8C00000A9;
64'hFFFFFFFFFFFFFA18:	romout <= 64'h042060000000A840;
64'hFFFFFFFFFFFFFA20:	romout <= 64'h800026F8C1FFFF00;
64'hFFFFFFFFFFFFFA28:	romout <= 64'h0100080000904802;
64'hFFFFFFFFFFFFFA30:	romout <= 64'hA955551A04000000;
64'hFFFFFFFFFFFFFA38:	romout <= 64'h2F8C00001091021A;
64'hFFFFFFFFFFFFFA40:	romout <= 64'h0000002210000008;
64'hFFFFFFFFFFFFFA48:	romout <= 64'h2F8C1FFFF801081C;
64'hFFFFFFFFFFFFFA50:	romout <= 64'h000026FA14000329;
64'hFFFFFFFFFFFFFA58:	romout <= 64'h3AAAAD5552A04002;
64'hFFFFFFFFFFFFFA60:	romout <= 64'h000000580355AAAA;
64'hFFFFFFFFFFFFFA68:	romout <= 64'h11A0400000066808;
64'hFFFFFFFFFFFFFA70:	romout <= 64'h0003241044300006;
64'hFFFFFFFFFFFFFA78:	romout <= 64'h02210000008BE300;
64'hFFFFFFFFFFFFFA80:	romout <= 64'hFFFC804207000000;
64'hFFFFFFFFFFFFFA88:	romout <= 64'h01200B00009BE307;
64'hFFFFFFFFFFFFFA90:	romout <= 64'h0000001000800009;
64'hFFFFFFFFFFFFFA98:	romout <= 64'h0408755AAAA46810;
64'hFFFFFFFFFFFFFAA0:	romout <= 64'h000022F8C00000A9;
64'hFFFFFFFFFFFFFAA8:	romout <= 64'h0420700000008840;
64'hFFFFFFFFFFFFFAB0:	romout <= 64'h000222F8C1FFFF20;
64'hFFFFFFFFFFFFFAB8:	romout <= 64'h01216800014BE858;
64'hFFFFFFFFFFFFFAC0:	romout <= 64'h000052FA14000048;
64'hFFFFFFFFFFFFFAC8:	romout <= 64'h1981000040004852;
64'hFFFFFFFFFFFFFAD0:	romout <= 64'h000020DFBE000000;
64'hFFFFFFFFFFFFFAD8:	romout <= 64'h19F820000000FEF0;
64'hFFFFFFFFFFFFFAE0:	romout <= 64'h8000060803DC0FF0;
64'hFFFFFFFFFFFFFAE8:	romout <= 64'h2C840180002B01B4;
64'hFFFFFFFFFFFFFAF0:	romout <= 64'h0002A8C7FFFFFAB9;
64'hFFFFFFFFFFFFFAF8:	romout <= 64'h2C84014000FBE000;
64'hFFFFFFFFFFFFFB00:	romout <= 64'h000000C7FFFFFA6D;
64'hFFFFFFFFFFFFFB08:	romout <= 64'h02FBC00000847E08;
64'hFFFFFFFFFFFFFB10:	romout <= 64'h0000800000000020;
64'hFFFFFFFFFFFFFB18:	romout <= 64'h19803FF000000000;
64'hFFFFFFFFFFFFFB20:	romout <= 64'h0000D19805FF0008;
64'hFFFFFFFFFFFFFB28:	romout <= 64'h2F84000002902008;
64'hFFFFFFFFFFFFFB30:	romout <= 64'h0010A00802000228;
64'hFFFFFFFFFFFFFB38:	romout <= 64'h008800005A902010;
64'hFFFFFFFFFFFFFB40:	romout <= 64'h40000C1884680001;
64'hFFFFFFFFFFFFFB48:	romout <= 64'h1184400000004110;
64'hFFFFFFFFFFFFFB50:	romout <= 64'h0014A45084000000;
64'hFFFFFFFFFFFFFB58:	romout <= 64'h1184400000802200;
64'hFFFFFFFFFFFFFB60:	romout <= 64'h0016A45084000000;
64'hFFFFFFFFFFFFFB68:	romout <= 64'h0080000003402200;
64'hFFFFFFFFFFFFFB70:	romout <= 64'hFC00000800000035;
64'hFFFFFFFFFFFFFB78:	romout <= 64'h11805FF00084600F;
64'hFFFFFFFFFFFFFB80:	romout <= 64'h0000000000000020;
64'hFFFFFFFFFFFFFB88:	romout <= 64'h37800000000DE000;
64'hFFFFFFFFFFFFFFB0:	romout <= 64'h000000CFFFFFFEC6;
64'hFFFFFFFFFFFFFFB8:	romout <= 64'h37800000000DE000;
64'hFFFFFFFFFFFFFFC0:	romout <= 64'h000000CFFFFFFEC6;
64'hFFFFFFFFFFFFFFC8:	romout <= 64'h37800000000DE000;
64'hFFFFFFFFFFFFFFD0:	romout <= 64'h000000CFFFFFFEB5;
64'hFFFFFFFFFFFFFFD8:	romout <= 64'h37800000000DE000;
64'hFFFFFFFFFFFFFFE0:	romout <= 64'h000000CFFFFFFEC5;
64'hFFFFFFFFFFFFFFE8:	romout <= 64'h37800000000DE000;
64'hFFFFFFFFFFFFFFF0:	romout <= 64'h000000CFFFFFFA00;
64'hFFFFFFFFFFFFFFF8:	romout <= 64'h37800000000DE000;
default:	romout <= 64'd0;
endcase
assign sys_dbi = br_dato|keybdout|stk_dato|scr_dato| {4{tc_dato}} | {4{pic_dato}};


Raptor64sc u1
(
	.rst_i(rst),
	.clk_i(clk),
	.nmi_i(cpu_nmi),
	.irq_i(cpu_irq),
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
