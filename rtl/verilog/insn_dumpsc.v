		$display("Fetched pc=%h.%h insn: %h", {pc[63:4],4'h0},pc[3:2], insn);
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
			`ICACHE_ON:	$display("ICACHE_ON");
			`ICACHE_OFF:	$display("ICACHE_OFF");
			`DCACHE_ON:	$display("DCACHE_ON");
			`DCACHE_OFF:	$display("DCACHE_OFF");
			default:	;
			endcase
			end
		`R:	
			case(insn[6:0])
//			`SGN:	$display("SGN");
			`NEG:	$display("NEG r%d,r%d",insn[29:25],insn[34:30]);
			`COM:	$display("COM r%d,r%d",insn[29:25],insn[34:30]);
			`ABS:	$display("ABS r%d,r%d",insn[29:25],insn[34:30]);
			`SQRT:	$display("SQRT r%d,r%d",insn[29:25],insn[34:30]);
			`OMGI:	$display("OMG r%d,#%d",insn[29:25],insn[12:7]);
			default:	;
			endcase
		`RR:
			case(insn[6:0])
			`ADD:	$display("ADD r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`ADDU:	$display("ADDU r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`SUB:	$display("SUB r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`SUBU:	$display("SUBU r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`CMP:	$display("CMP r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`AND:	$display("AND r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`ANDC:	$display("ANDC r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`OR:	$display("OR  r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`ORC:	$display("ORC r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`XOR:	$display("XOR r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`NAND:	$display("NAND r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`NOR:	$display("NOR  r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`XNOR:	$display("XNOR r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			`MULU:	$display("MULU r%d,r%d,r%d",insn[24:20],insn[34:30],insn[29:25]);
			default:	;
			endcase
		`SHFTI:
			case(insn[4:0])
			`SHLI:	$display("SHLI r%d,r%d,#%d",insn[29:25],insn[34:30],insn[24:19]);
			`SHRUI:	$display("SHRUI r%d,r%d,#%d",insn[29:25],insn[34:30],insn[24:19]);
			`SHRI:	$display("SHRI r%d,r%d,#%d",insn[29:25],insn[34:30],insn[24:19]);
			`ROLI:	$display("ROLI r%d,r%d,#%d",insn[29:25],insn[34:30],insn[24:19]);
			`RORI:	$display("RORI r%d,r%d,#%d",insn[29:25],insn[34:30],insn[24:19]);
			`ROLAMI:	$display("ROLAMI r%d,r%d,#%d",insn[29:25],insn[34:30],insn[24:19]);
			endcase
		`BTRR:
			case(insn[4:0])
			`BEQ:	$display("BEQ r%d,r%d,%h.%h)",insn[34:30],insn[29:25],{{44{insn[24]}},insn[24:7]},insn[6:5]);
			`BNE:	$display("BNE r%d,r%d,%h.%h)",insn[34:30],insn[29:25],{{44{insn[24]}},insn[24:7]},insn[6:5]);
			`BLT:	$display("BLT r%d,r%d,%h.%h)",insn[34:30],insn[29:25],{{44{insn[24]}},insn[24:7]},insn[6:5]);
			`BLE:	$display("BLE r%d,r%d,%h.%h)",insn[34:30],insn[29:25],{{44{insn[24]}},insn[24:7]},insn[6:5]);
			`BGT:	$display("BGT r%d,r%d,%h.%h)",insn[34:30],insn[29:25],{{44{insn[24]}},insn[24:7]},insn[6:5]);
			`BGE:	$display("BGE r%d,r%d,%h.%h)",insn[34:30],insn[29:25],{{44{insn[24]}},insn[24:7]},insn[6:5]);
			`BLTU:	$display("BLTU r%d,r%d,%h.%h)",insn[34:30],insn[29:25],{{44{insn[24]}},insn[24:7]},insn[6:5]);
			`BLEU:	$display("BLEU r%d,r%d,%h.%h)",insn[34:30],insn[29:25],{{44{insn[24]}},insn[24:7]},insn[6:5]);
			`BGTU:	$display("BGTU r%d,r%d,%h.%h)",insn[34:30],insn[29:25],{{44{insn[24]}},insn[24:7]},insn[6:5]);
			`BGEU:	$display("BGEU r%d,r%d,%h.%h)",insn[34:30],insn[29:25],{{44{insn[24]}},insn[24:7]},insn[6:5]);
			`BRA:	$display("BRA %h.%h)",{{44{insn[24]}},insn[24:7]},insn[6:5]);
			`LOOP:	$display("LOOP r%d,%h.%h)",insn[29:25],{{44{insn[24]}},insn[24:7]},insn[6:5]);
			default:	;
			endcase
		`SETLO:	$display("SETLO r%d,#%h", insn[36:32],{{32{insn[31]}},insn[31:0]});
		`ADDI:	$display("ADDI r%d,r%d,#%d",insn[29:25],insn[34:30],{{39{insn[24]}},insn[24:0]});
		`ADDUI:	$display("ADDUI r%d,r%d,#%d",insn[29:25],insn[34:30],{{39{insn[24]}},insn[24:0]});
		`SUBI:	$display("SUBI r%d,r%d,#%d",insn[29:25],insn[34:30],{{39{insn[24]}},insn[24:0]});
		`SUBUI:	$display("SUBUI r%d,r%d,#%d",insn[29:25],insn[34:30],{{39{insn[24]}},insn[24:0]});
		`ANDI:	$display("ANDI r%d,r%d,#%d",insn[29:25],insn[34:30],{39'h7FFFFFFFFF,insn[24:0]});
		`ORI:	$display("ORI  r%d,r%d,#%d",insn[29:25],insn[34:30],{39'd0,insn[24:0]});
		`XORI:	$display("XORI r%d,r%d,#%d",insn[29:25],insn[34:30],{39'd0,insn[24:0]});
		`JMP:	$display("JMP  %h.%d",{insn[34:2],4'b0000},insn[1:0]);
		`CALL:	$display("CALL %h.%d",{insn[34:2],4'b0000},insn[1:0]);
		`JAL:	$display("JAL");
		`RET:	$display("RET R%d,R%d,#%h",insn[34:30],insn[29:25],{{39{insn[24]}},insn[24:0]});
		`BEQI:	$display("BEQI r%d,#%d,%h.%h)",insn[34:30],insn[17:0],{{50{insn[29]}},insn[29:20]},insn[19:18]);
		`BNEI:	$display("BNEI r%d,#%d,%h.%h)",insn[34:30],insn[17:0],{{50{insn[29]}},insn[29:20]},insn[19:18]);
		`BLTI:	$display("BLTI r%d,#%d,%h.%h)",insn[34:30],insn[17:0],{{50{insn[29]}},insn[29:20]},insn[19:18]);
		`BLEI:	$display("BLEI r%d,#%d,%h.%h)",insn[34:30],insn[17:0],{{50{insn[29]}},insn[29:20]},insn[19:18]);
		`BGTI:	$display("BGTI r%d,#%d,%h.%h)",insn[34:30],insn[17:0],{{50{insn[29]}},insn[29:20]},insn[19:18]);
		`BGEI:	$display("BGEI r%d,#%d,%h.%h)",insn[34:30],insn[17:0],{{50{insn[29]}},insn[29:20]},insn[19:18]);
		`BLTUI:	$display("BLTUI r%d,#%d,%h.%h)",insn[34:30],insn[17:0],{{50{insn[29]}},insn[29:20]},insn[19:18]);
		`BLEUI:	$display("BLEUI r%d,#%d,%h.%h)",insn[34:30],insn[17:0],{{50{insn[29]}},insn[29:20]},insn[19:18]);
		`BGTUI:	$display("BGTUI r%d,#%d,%h.%h)",insn[34:30],insn[17:0],{{50{insn[29]}},insn[29:20]},insn[19:18]);
		`BGEUI:	$display("BGEUI r%d,#%d,%h.%h)",insn[34:30],insn[17:0],{{50{insn[29]}},insn[29:20]},insn[19:18]);
		`NOPI:	$display("NOP");
		`SB:	$display("SB r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		`SC:	$display("SC r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		`SH:	$display("SH r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		`SW:	$display("SW r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		`LEA:	$display("LEA r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		`LB:	$display("LB r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		`LBU:	$display("LBU r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		`LC:	$display("LC r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		`LCU:	$display("LCU r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		`LH:	$display("LH r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		`LHU:	$display("LHU r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		`LW:	$display("LW r%d,%d[r%d]",insn[29:25],{{39{insn[24]}},insn[24:0]},insn[34:30]);
		7'b111xxxx:	$display("IMM %h", {insn[38:0],25'd0});
		default:	;
		endcase
