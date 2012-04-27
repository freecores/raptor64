		$display("Fetched AXC=%h pc=%h insn: %h", AXC, pc_axc, insn);
		casex(insn[41:35])
		`MISC:
			begin
			$display("MISC");
			case(insn[6:0])
			`BRK:	$display("BRK");
			`WAIT:	$display("WAIT");
			`IRET:	$display("IRET");
			`CLI:	$display("CLI");
			`SEI:	$display("SEI");
			`TLBR:	$display("TLBR");
			`TLBWI:	$display("TLBWI");
			`TLBWR:	$display("TLBWR");
			default:	;
			endcase
			end
		`R:	
			case(insn[6:0])
//			`SGN:	$display("SGN");
			`NEG:	$display("NEG r%d,r%d",insn[34:30],insn[29:25]);
			`COM:	$display("COM r%d,r%d",insn[34:30],insn[29:25]);
			`ABS:	$display("ABS r%d,r%d",insn[34:30],insn[29:25]);
			`SQRT:	$display("SQRT r%d,r%d",insn[34:30],insn[29:25]);
			default:	;
			endcase
		`RR:
			case(insn[6:0])
			`ADD:	$display("ADD r%d,r%d,r%d",insn[34:30],insn[29:25],insn[24:20]);
			`SUB:	$display("SUB r%d,r%d,r%d",insn[34:30],insn[29:25],insn[24:20]);
			`CMP:	$display("CMP r%d,r%d,r%d",insn[34:30],insn[29:25],insn[24:20]);
			`AND:	$display("AND r%d,r%d,r%d",insn[34:30],insn[29:25],insn[24:20]);
			`OR:	$display("OR  r%d,r%d,r%d",insn[34:30],insn[29:25],insn[24:20]);
			`XOR:	$display("XOR r%d,r%d,r%d",insn[34:30],insn[29:25],insn[24:20]);
			`NAND:	$display("NAND r%d,r%d,r%d",insn[34:30],insn[29:25],insn[24:20]);
			`NOR:	$display("NOR  r%d,r%d,r%d",insn[34:30],insn[29:25],insn[24:20]);
			`XNOR:	$display("XNOR r%d,r%d,r%d",insn[34:30],insn[29:25],insn[24:20]);
			`MULU:	$display("MULU r%d,r%d,r%d",insn[34:30],insn[29:25],insn[24:20]);
			default:	;
			endcase
		`ADDI:	$display("ADDI r%d,r%d,#%d",insn[34:30],insn[29:25],{{39{insn[24]}},insn[24:0]});
		`SUBI:	$display("SUBI r%d,r%d,#%d",insn[34:30],insn[29:25],{{39{insn[24]}},insn[24:0]});
		`ANDI:	$display("ANDI r%d,r%d,#%d",insn[34:30],insn[29:25],{39'h7FFFFFFFFF,insn[24:0]});
		`ORI:	$display("ORI  r%d,r%d,#%d",insn[34:30],insn[29:25],{39'd0,insn[24:0]});
		`XORI:	$display("XORI r%d,r%d,#%d",insn[34:30],insn[29:25],{39'd0,insn[24:0]});
		`JMP:	$display("JMP  %h.%d",{insn[34:2],4'b0000},insn[1:0]);
		`CALL:	$display("CALL %h",{insn[34:0],2'b00});
		`JAL:	$display("JAL");
		`RET:	$display("RET R%d,R%d,#%h",insn[34:30],insn[29:25],{{39{insn[24]}},insn[24:0]});
		`NOPI:	$display("NOP");
		`SC:	$display("SC r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		7'b111xxxx:	$display("IMM %h", insn[38:0]);
		default:	;
		endcase
