; ============================================================================
; (C) 2012 Robert Finch
; All Rights Reserved.
; robfinch<remove>@opencores.org
;
; This source file is free software: you can redistribute it and/or modify 
; it under the terms of the GNU Lesser General Public License as published 
; by the Free Software Foundation, either version 3 of the License, or     
; (at your option) any later version.                                      
;                                                                          
; This source file is distributed in the hope that it will be useful,      
; but WITHOUT ANY WARRANTY; without even the implied warranty of           
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            
; GNU General Public License for more details.                             
;                                                                          
; You should have received a copy of the GNU General Public License        
; along with this program.  If not, see <http://www.gnu.org/licenses/>.    
;                                                                          
; ============================================================================
;
CR	EQU	0x0D		;ASCII equates
LF	EQU	0x0A
TAB	EQU	0x09
CTRLC	EQU	0x03
CTRLH	EQU	0x08
CTRLS	EQU	0x13
CTRLX	EQU	0x18

STACKTOP	EQU		0xFFFF_FFFF_FFFE_FFF8
Milliseconds	EQU		0x400
Lastloc			EQU		0x408
ScreenColor	EQU		0x414
CursorRow	EQU		0x416
CursorCol	EQU		0x418
KeybdEcho	EQU		0x41A
KeybdBuffer	EQU		0x440
KeybdHead	EQU		0x450
KeybdTail	EQU		0x451
TEXTSCR		EQU		0xFFFF_FFFF_FFD0_0000
COLORSCR	EQU		0xFFFF_FFFF_FFD1_0000
TEXTREG		EQU		0xFFFFFFFF_FFDA0000
TEXT_COLS	EQU		0x0
TEXT_ROWS	EQU		0x2
TEXT_CURPOS	EQU		0x16
KEYBD		EQU		0xFFFF_FFFF_FFDC_0000
KEYBDCLR	EQU		0xFFFF_FFFF_FFDC_0002
UART	EQU		0xFFFF_FFFF_FFDC_0A00
UART_LS		EQU	0xFFFF_FFFF_FFDC_0A01
PIC		EQU		0xFFFF_FFFF_FFDC_0FF0
PSG			EQU		0xFFFF_FFFF_FFD5_0000
AC97		EQU		0xFFFF_FFFF_FFDC_1000
BOOT_STACK	EQU		0xFFFF_FFFF_FFFE_FFF8
BITMAPSCR	EQU		0x00000001_00200000

txempty	EQU		0x40
rxfull	EQU		0x01

;	org 0x070
;	iret
;	nop
;	nop
;	nop
;	nop
;	nop
;	nop
;	nop
;
	code
	org 0xFFFF_FFFF_FFFF_E800

; jump table
;
	jmp		SerialGetChar
	jmp		SerialPutChar
	jmp		SetKeyboardEcho
	jmp		KeybdCheckForKey
	jmp		KeybdGetChar
	jmp		DisplayChar
	jmp		DisplayString

start:
;	lea		MSGRAM,a1
;	jsr		DisplayString

ColdStart:
	icache_on				; turn on the ICache
	dcache_on				; turn on the DCache
	setlo	sp,#STACKTOP	; top of stack
	call	KeybdInit
	call	PICInit
	cli						; enable interrupts
	setlo	r3,#0xCE		; blue on blue
	sc		r3,ScreenColor
	lc		r3,0x414
	setlo	r3,#32
	sc		r3,0x416		; we do a store, then a load through the dcache
	lc		r2,0x416		;
	lc		r2,0x416		;
	beq		r2,r3,dcokay
	dcache_off				; data cache failed
dcokay:
	lc		r3,ScreenColor
	call	ClearScreen
	call	ClearBmpScreen
	sc		r0,CursorRow
	sc		r0,CursorCol
	setlo	r1,#<MSGSTART
	sethi	r1,#>MSGSTART
	call	DisplayString
;	call	SetupAC97		; and Beep

; Allow some other contexts to start up
; equal processing time for sixteen contexts
;
	mfspr	r1,AXC			; which context am I
	bne		r1,r0,j4
	setlo	r1,#0x76543210		
	mtspr	EP0,r1
	mtspr	EP2,r1
	setlo	r1,#0xFEDCBA98
	mtspr	EP1,r1
	mtspr	EP3,r1
j4:
	jmp		Monitor
	bra		j4

;	call	ramtest

;-----------------------------------------
; Hello World!
;-----------------------------------------
HelloWorld:
	subui	r30,r30,#24
	sm		[r30],r1/r2/r31
	setlo	r2,#MSG
j3:
	lb		r1,[r2]
	beq		r1,r0,j2
	call	SerialPutChar
	addui	r2,r2,#1
	bra		j3
j2:
	lm		[r30],r1/r2/r31
	ret		#24


	align	16
MSG:	
	DB	"Hello World!",0,0,0,0
	align	16
MSGSTART:
	db	"Raptor64 system starting....",CR,LF,0,0

	align 16

;----------------------------------------------------------
; Initialize programmable interrupt controller (PIC)
;  0 = nmi
;  1 = keyboard reset
;  2 = 1000Hz pulse (cursor flash)
; 15 = keyboard char
;----------------------------------------------------------
PICInit:
	setlo	r1,#0x8007	; enable nmi,kbd_rst,and kbd_irq
	outc	r1,PIC+2
	ret

;-----------------------------------------
; Get character from serial port
;-----------------------------------------
SerialGetChar:
	subui	r30,r30,#8
	sw		r3,[r30]
	setlo	r1,#UART
sgc1:
	inb		r3,1[r1]		; uart status
	andi	r3,r3,#rxfull	; is there a char available ?
	beq		r3,r0,sgc1
	lw		r3,[r30]
	inb		r1,[r1]
	ret		#8

;-----------------------------------------
; Put character to serial port
;-----------------------------------------
SerialPutChar:
	subui	r30,r30,#16
	sw		r2,8[r30]
	sw		r3,[r30]
	setlo	r3,#UART
spc1:
	inb		r2,1[r3]		; uart status
	andi	r2,r2,#txempty	; is there a char available ?
	beq		r2,r0,spc1
	outb	r1,[r3]
	lw		r3,[r30]
	lw		r2,8[r30]
	ret		#16

;==============================================================================
; Keyboard
;==============================================================================
;------------------------------------------------------------------------------
; Initialize keyboard
;------------------------------------------------------------------------------
KeybdInit:
	sb		r0,KeybdHead
	sb		r0,KeybdTail
	setlo	r1,#1			; turn on keyboard echo
	sb		r1,KeybdEcho
	ret

;------------------------------------------------------------------------------
; Normal keyboard interrupt, the lowest priority interrupt in the system.
; Grab the character from the keyboard device and store it in a buffer.
;------------------------------------------------------------------------------
;
KeybdIRQ:
	subui	sp,sp,#24
	sm		[sp],r1/r2/r3
	lbu		r1,KeybdHead
	andi	r1,r1,#0x0f				; r1 = index into buffer
	setlo	r3,#<KeybdBuffer
	sethi	r3,#>KeybdBuffer
KeybdIRQa:
	inch	r2,KEYBD				; get keyboard character
	outc	r0,KEYBD+2				; clear keyboard strobe (turns off the IRQ)
	sb		r2,[r3+r1]				; store character in buffer
	addui	r1,r1,#1				; increment head index
	andi	r1,r1,#0x0f
	sb		r1,KeybdHead
KeybdIRQb:
	lbu		r2,KeybdTail			; check to see if we've collided
	bne		r1,r2,KeybdIRQc			; with the tail
	addui	r2,r2,#1				; if so, increment the tail index
	andi	r2,r2,#0x0f				; the oldest character will be lost
	sb		r2,KeybdTail
KeybdIRQc:
	lm		[sp],r1/r2/r3
	addui	sp,sp,#24
	ret

;------------------------------------------------------------------------------
; r1 0=echo off, non-zero = echo on
;------------------------------------------------------------------------------
SetKeyboardEcho:
	sb		r1,KeybdEcho
	ret

;-----------------------------------------
; Get character from keyboard buffer
;-----------------------------------------
KeybdGetChar:
	subui	sp,sp,#16
	sm		[sp],r2/r3
	lbu		r2,KeybdTail
	lbu		r1,KeybdHead
	beq		r1,r2,nochar
	setlo	r3,#KeybdBuffer
	lbu		r1,[r3+r2]
	addui	r2,r2,#1
	andi	r2,r2,#0x0f
	sb		r2,KeybdTail
	lm		[sp],r2/r3
	ret		#16
nochar:
	setlo	r1,#-1
	lm		[sp],r2/r3
	ret		#16

;------------------------------------------------------------------------------
; Check if there is a keyboard character available in the keyboard buffer.
;------------------------------------------------------------------------------
;
KeybdCheckForKey:
	lbu		r1,KeybdTail
	lbu		r2,KeybdHead
	beq		r1,r2,kck1
	setlo	r1,#1
	ret
kck1:
	xor		r1,r1,r1		; return zero
	ret

;------------------------------------------------------------------------------
; Check if there is a keyboard character available. If so return true (1)
; otherwise return false (0) in r1.
;------------------------------------------------------------------------------
;
KeybdCheckForKeyDirect:
	inch	r1,KEYBD
	bge		r1,r0,cfkd1
	setlo	r1,#1
	ret
cfkd1:
	xor		r1,r1,r1	; return 0 in r1
	ret

;------------------------------------------------------------------------------
; Get character directly from keyboard. This routine blocks until a key is
; available.
;------------------------------------------------------------------------------
;
KeybdGetCharDirect:
	subui	sp,sp,#16
	sm		[sp],r2/r31
	setlo	r2,KEYBD
kgc1:
	inch	r1,KEYBD
	bge		r1,r0,kgc1
	outc	r0,KEYBD+2		; clear keyboard strobe
	andi	r1,r1,#0xff		; remove strobe bit
	lb		r2,KeybdEcho	; is keyboard echo on ?
	beq		r2,r0,gk1
	bnei	r1,#'\r',gk2	; convert CR keystroke into CRLF
	call	CRLF
	bra		gk1
gk2:
	call	DisplayChar
gk1:
	lm		[sp],r2/r31
	ret		#16

;==============================================================================
;==============================================================================
;------------------------------------------------------------------------------
; 1000 Hz interrupt
; - takes care of "flashing" the cursor
;------------------------------------------------------------------------------
;
Pulse1000:
	subui	sp,sp,#24
	sm		[sp],r1/r2/lr
	lw		r1,Milliseconds
	addui	r1,r1,#1
	sw		r1,Milliseconds
	setlo	r2,TEXTSCR
	lc		r1,222[r2]
	addui	r1,r1,#1
	sc		r1,222[r2]
	lc		r0,0xFFFF_FFFF_FFFF_0000	; clear interrupt
	lw		r1,Milliseconds
	andi	r1,r1,#0x7f
	bnei	r1,#64,p10001
	call	FlashCursor
p10001:
	lm		[sp],r1/r2/lr
	ret		#24

;------------------------------------------------------------------------------
; Flash Cursor
;------------------------------------------------------------------------------
;
FlashCursor:
	subui	sp,sp,#32
	sm		[sp],r1/r2/r3/r31
	call	CalcScreenLoc
	addui	r1,r1,#0x10000
	; causes screen colors to flip around
	lc		r2,[r1]
	addui	r2,r2,#1
	sc		r2,[r1]
	lw		r2,Lastloc
	beq		r1,r2,flshcrsr1
	; restore the screen colors of the previous cursor location
	lc		r3,ScreenColor
	sc		r3,[r2]
	sw		r1,Lastloc
flshcrsr1:
	lm		[sp],r1/r2/r3/r31
	ret		#32

;------------------------------------------------------------------------------
;------------------------------------------------------------------------------
ClearBmpScreen:
	subui	sp,sp,#40
	sm		[sp],r1/r2/r3/r4/r31
	setlo	r1,#1364			; calc number to clear
	setlo	r2,#768
	mulu	r2,r1,r2			; r2 = # pixels to clear
	or		r4,r0,r2			; r4 = # pixels to clear
	setlo	r1,#0x29292929		;
	setlo	r3,#<BITMAPSCR		; screen address
	sethi	r3,#>BITMAPSCR
csj4:
	sh		r1,[r3]
	addui	r3,r3,#4
	loop	r2,csj4
	lm		[sp],r1/r2/r3/r4/r31
	ret		#40

;------------------------------------------------------------------------------
; Clear the screen and the screen color memory
; We clear the screen to give a visual indication that the system
; is working at all.
;------------------------------------------------------------------------------
;
ClearScreen:
	subui	sp,sp,#40
	sm		[sp],r1/r2/r3/r4/r31
	setlo	r3,#TEXTREG
	lc		r1,TEXT_COLS[r3]	; calc number to clear
	lc		r2,TEXT_ROWS[r3]
	mulu	r2,r1,r2			; r2 = # chars to clear
	setlo	r1,#32			; space char
	lc		r4,ScreenColor
	call	AsciiToScreen
	setlo	r3,#TEXTSCR		; text screen address
csj4:
	sc		r1,[r3]
	sc		r4,0x10000[r3]	; color screen is 0x10000 higher
	addu	r3,r3,#2
	loop	r2,csj4
	lm		[sp],r1/r2/r3/r4/r31
	ret		#40

;------------------------------------------------------------------------------
; Scroll text on the screen upwards
;------------------------------------------------------------------------------
;
ScrollUp:
	subui	sp,sp,#40
	sm		[sp],r1/r2/r3/r4/r31
	setlo	r3,#TEXTREG
	lc		r1,TEXT_COLS[r3]	; r1 = # text columns
	lc		r2,TEXT_ROWS[r3]
	mulu	r2,r1,r2			; calc number of chars to scroll
	subu	r2,r2,r1			; one less row
	setlo	r3,#TEXTSCR
scrup1:
	lc		r4,[r3+r1]			; indexed addressing example
	sc		r4,[r3]
	addui	r3,r3,#2
	loop	r2,scrup1

	setlo	r3,#TEXTREG
	lc		r1,TEXT_ROWS[r3]
	subui	r1,r1,#1
	call	BlankLine
	lm		[sp],r1/r2/r3/r4/r31
	ret		#40

;------------------------------------------------------------------------------
; Blank out a line on the display
; line number to blank is in r1
;------------------------------------------------------------------------------
;
BlankLine:
	subui	sp,sp,#24
	sm		[sp],r1/r2/r3
	setlo	r3,TEXTREG			; r3 = text register address
	lc		r2,TEXT_COLS[r3]	; r2 = # chars to blank out
	mulu	r3,r2,r1
	shli	r3,r3,#1
	addui	r3,r3,#TEXTSCR		; r3 = screen address
	setlo	r1,#' '
blnkln1:
	sc		r1,[r3]
	addui	r3,r3,#2
	loop	r2,blnkln1
	lm		[sp],r1/r2/r3
	ret		#24

;------------------------------------------------------------------------------
; Convert ASCII character to screen display character.
;------------------------------------------------------------------------------
;
AsciiToScreen:
	andi	r1,r1,#0x00ff
	bltui	r1,#'A',atoscr1
	bleui	r1,#'Z',atoscr1
	bgtui   r1,#'z',atoscr1
	bltui	r1,#'a',atoscr1
	subi	r1,r1,#0x60
atoscr1:
	ori		r1,r1,#0x100
	ret

;------------------------------------------------------------------------------
; Convert screen character to ascii character
;------------------------------------------------------------------------------
;
ScreenToAscii:
	andi	r1,r1,#0xff
	bgtui	r1,#26,stasc1
	addui	r1,r1,#60
stasc1:
	ret

;------------------------------------------------------------------------------
; Calculate screen memory location from CursorRow,CursorCol.
; Also refreshes the cursor location.
; Destroys r1,r2,r3
; r1 = screen location
;------------------------------------------------------------------------------
;
CalcScreenLoc:
	lc		r1,CursorRow
	andi	r1,r1,#0x7f
	setlo	r3,TEXTREG
	inch	r2,TEXT_COLS[r3]
	mulu	r2,r2,r1
	lc		r1,CursorCol
	andi	r1,r1,#0x7f
	addu	r2,r2,r1
	outc	r2,TEXT_CURPOS[r3]
	shli	r2,r2,#1
	addui	r1,r2,#TEXTSCR			; r1 = screen location
	ret

;------------------------------------------------------------------------------
; Display a character on the screen
; d1.b = char to display
;------------------------------------------------------------------------------
;
DisplayChar:
	bnei	r1,#'\r',dccr		; carriage return ?
	subui	sp,sp,#32
	sm		[sp],r1/r2/r3/lr
	sc		r0,CursorCol		; just set cursor column to zero on a CR
	bra		dcx7
dccr:
	bnei	r1,#0x91,dcx6		; cursor right ?
	subui	sp,sp,#32
	sm		[sp],r1/r2/r3/lr
	lc		r2,CursorCol
	beqi	r2,#56,dcx7
	addui	r2,r2,#1
	sc		r2,CursorCol
dcx7:
	call	CalcScreenLoc
	lm		[sp],r1/r2/r3/lr
	ret		#32
dcx6:
	bnei	r1,#0x90,dcx8		; cursor up ?
	subui	sp,sp,#32
	sm		[sp],r1/r2/r3/lr
	lc		r2,CursorRow
	beqi	r2,#0,dcx7
	subui	r2,r2,#1
	sc		r2,CursorRow
	bra		dcx7
dcx8:
	bnei	r1,#0x93,dcx9		; cursor left ?
	subui	sp,sp,#32
	sm		[sp],r1/r2/r3/lr
	lc		r2,CursorCol
	beqi	r2,#0,dcx7
	subui	r2,r2,#1
	sc		r2,CursorCol
	bra		dcx7
dcx9:
	bnei	r1,#0x92,dcx10		; cursor down ?
	subui	sp,sp,#32
	sm		[sp],r1/r2/r3/lr
	lc		r2,CursorRow
	beqi	r2,#30,dcx7
	addui	r2,r2,#1
	sc		r2,CursorRow
	bra		dcx7
dcx10:
	bnei	r1,#0x94,dcx11			; cursor home ?
	subui	sp,sp,#32
	sm		[sp],r1/r2/r3/lr
	lc		r2,CursorCol
	beq		r2,r0,dcx12
	sc		r0,CursorCol
	bra		dcx7
dcx12:
	sc		r0,CursorRow
	bra		dcx7
dcx11:
	subui	sp,sp,#48
	sm		[sp],r1/r2/r3/r4/r5/r31
	bnei	r1,#0x99,dcx13		; delete ?
	call	CalcScreenLoc
	or		r3,r0,r1			; r3 = screen location
	lc		r1,CursorCol		; r1 = cursor column
	bra		dcx5
dcx13:
	bnei	r1,#CTRLH,dcx3		; backspace ?
	lc		r2,CursorCol
	beq		r2,r0,dcx4
	subui	r2,r2,#1
	sc		r2,CursorCol
	call	CalcScreenLoc		; a0 = screen location
	or		r3,r0,r1			; r3 = screen location
	lc		r1,CursorCol
dcx5:
	lc		r2,2[r3]
	sc		r2,[r3]
	addui	r3,r3,#2
	addui	r1,r1,#1
	setlo	r4,#TEXTREG
	inch	r5,TEXT_COLS[r4]
	bltu	r1,r5,dcx5
	setlo	r2,#' '
	sc		r2,-2[r3]
	bra		dcx4
dcx3:
	beqi	r1,#'\n',dclf	; linefeed ?
	or		r4,r0,r1		; save r1 in r4
	call	CalcScreenLoc	; r1 = screen location
	or		r3,r0,r1		; r3 = screen location
	or		r1,r0,r4		; restore r1
	call	AsciiToScreen	; convert ascii char to screen char
	sc		r1,[r3]
	call	IncCursorPos
	lm		[sp],r1/r2/r3/r4/r5/r31
	ret		#48
dclf:
	call	IncCursorRow
dcx4:
	lm		[sp],r1/r2/r3/r4/r5/r31
	ret		#48


;------------------------------------------------------------------------------
; Increment the cursor position, scroll the screen if needed.
;------------------------------------------------------------------------------
;
IncCursorPos:
	subui	sp,sp,#32
	sm		[r30],r1/r2/r3/r31
	lc		r1,CursorCol
	addui	r1,r1,#1
	sc		r1,CursorCol
	inch	r2,TEXTREG+TEXT_COLS
	bleu	r1,r2,icc1
	sc		r0,CursorCol		; column = 0
	bra		icr1
IncCursorRow:
	subui	sp,sp,#32
	sm		[sp],r1/r2/r3/r31
icr1:
	lc		r1,CursorRow
	addui	r1,r1,#1
	sc		r1,CursorRow
	inch	r2,TEXTREG+TEXT_ROWS
	bleu	r1,r2,icc1
	subui	r2,r2,#1			; backup the cursor row, we are scrolling up
	sc		r2,CursorRow
	call	ScrollUp
icc1:
	call	CalcScreenLoc
	lm		[sp],r1/r2/r3/r31
	ret		#32

;------------------------------------------------------------------------------
; Display a string on the screen.
;------------------------------------------------------------------------------
;
DisplayString:
	subi	sp,sp,#24
	sm		[sp],r1/r2/r31
	or		r2,r1,r0		; r2 = pointer to string
dspj1:
	lbu		r1,[r2]			; move string char into r1
	addui	r2,r2,#1		; increment pointer
	beq		r1,r0,dsret		; is it end of string ?
	call	DisplayChar		; display character
	bra		dspj1			; go back for next character
dsret:
	lm		[r30],r1/r2/r31
	ret		#24

DisplayStringCRLF:
	subui	r30,r30,#8
	sw		r31,[r30]
	call	DisplayString
	lw		r31,[r30]
	addui	r30,r30,#8

CRLF:
	subui	r30,r30,#16
	sw		r1,[r30]
	sw		r31,8[r30]
	setlo	r1,#'\r'
	call	DisplayChar
	setlo	r1,#'\n'
	call	DisplayChar
	lw		r1,[r30]
	lw		r31,8[r30]
	ret		#16

;------------------------------------------------------------------------------
; Display nybble in r1
;------------------------------------------------------------------------------
;
DisplayNybble:
	subui	r30,r30,#16
	sw		r31,8[r30]
	sw		r1,[r30]
	andi	r1,r1,#0x0F
	addui	r1,r1,#'0'
	bleui	r1,#'9',dispnyb1
	addui	r1,r1,#7
dispnyb1:
	call	DisplayChar
	lw		r1,[r30]
	lw		r31,8[r30]
	ret		#16

;------------------------------------------------------------------------------
; Display the byte in r1
;------------------------------------------------------------------------------
;
DisplayByte:
	subui	sp,sp,#16
	sm		[sp],r1/r31
	rori	r1,r1,#4	
	call	DisplayNybble
	roli	r1,r1,#4
	call	DisplayNybble
	lm		[sp],r1/r31
	ret		#16

;------------------------------------------------------------------------------
; Display the 64 bit word in r1
;------------------------------------------------------------------------------
;
DisplayWord:
	subui	sp,sp,#24
	sm		[sp],r1/r3/r31
	setlo	r3,#7
dspwd1:
	roli	r1,r1,#8
	call	DisplayByte
	loop	r3,dspwd1
	lm		[sp],r1/r3/r31
	ret		#24

;------------------------------------------------------------------------------
; Display memory pointed to by r2.
; destroys r1,r3
;------------------------------------------------------------------------------
;
DisplayMem:
	subui	sp,sp,#8
	sw		lr,[sp]
	setlo	r1,#':'
	call	DisplayChar
	or		r1,r2,r0
	call	DisplayWord
	setlo	r3,#7
dspmem1:
	setlo	r1,#' '
	call	DisplayChar
	lb		r1,[r2]
	call	DisplayByte
	addui	r2,r2,#1
	loop	r3,dspmem1
	call	CRLF
	lw		lr,[sp]
	ret		#8

;------------------------------------------------------------------------------
; Converts binary number in r1 into BCD number in r2 and r1.
;------------------------------------------------------------------------------
;
BinToBCD:
	subui	sp,sp,#48
	sm		[sp],r3/r4/r5/r6/r7/r8
	setlo	r2,#10
	setlo	r8,#19		; number of digits to produce - 1
bta1:
	mod		r3,r1,r2
	shli	r3,r3,#60	; shift result to uppermost bits
	shli	r7,r5,#60	; copy low order nybble of r5 to r4 topmost nybble
	shrui	r4,r4,#4
	or		r4,r4,r7
	shrui	r5,r5,#4
	or		r5,r5,r3	; copy new bcd digit into uppermost bits of r5
	divui	r1,r1,r2	; r1=r1/10
	loop	r8,bta1
	shrui	r4,r4,#48	; right align number in register
	shli	r6,r5,#16
	or		r4,r4,r6	; copy bits into r4
	shrui	r5,r5,#48
	or		r1,r0,r4
	or		r2,r0,r5
	lm		[sp],r3/r4/r5/r6/r7/r8
	ret		#48

;------------------------------------------------------------------------------
; Converts BCD number in r1 into Ascii number in r2 and r1.
;------------------------------------------------------------------------------
;
BCDToAscii:
	subui	sp,sp,#32
	sm		[sp],r3/r4/r5/r8
	setlo	r8,#15
bta2:
	andi	r2,r1,#0x0F
	ori		r2,r2,#0x30
	shli	r2,r2,#56
	shrui	r4,r4,#8
	shli	r5,r3,#56
	or		r4,r4,r5
	shrui	r3,r3,#8
	or		r3,r3,r2
	shrui	r1,r1,#4
	loop	r8,bta2
	or		r1,r0,r4
	or		r2,r0,r3
	lm		[sp],r3/r4/r5/r8
	ret		#32

;------------------------------------------------------------------------------
; Convert a binary number into a 20 character ascii string.
; r1 = number to convert
; r2 = address of string buffer
;------------------------------------------------------------------------------
;
BinToStr:
	subui	sp,sp,#56
	sm		[sp],r3/r7/r8/r9/r10/r11/r31
	or		r11,r0,r2
	call	BinToBCD
	or		r10,r0,r2	; save off r2
	call	BCDToAscii
	setlo	r9,#1
btos3:
	setlo	r8,#7
btos1:
	shli	r7,r9,#3
	addui	r7,r7,r8
	addui	r7,r7,#4
	andi	r3,r1,#0xff
	sb		r3,[r7+r11]
	shrui	r1,r1,#8
	loop	r8,btos1
	or		r1,r0,r2
	loop	r9,btos3
; the last four digits
	or		r1,r0,r10	; get back r2
	call	BCDToAscii
	setlo	r8,#3
btos2:
	andi	r3,r1,#0xff
	sb		r3,[r8+r11]
	shrui	r1,r1,#8
	loop	r8,btos2
	sb		r0,20[r11]	; null terminate
	lm		[sp],r3/r7/r8/r9/r10/r11/r31
	ret		#56


;==============================================================================
;==============================================================================
Monitor:
	setlo	sp,#STACKTOP	; top of stack; reset the stack pointer
	sb		r0,KeybdEcho	; turn off keyboard echo
PromptLn:
	call	CRLF
	setlo	r1,#'$'
	call	DisplayChar

; Get characters until a CR is keyed
;
Prompt3:
	call	KeybdGetChar
	beqi	r1,#-1,Prompt3	; wait for a character
	beqi	r1,#CR,Prompt1
	call	DisplayChar
	bra		Prompt3

; Process the screen line that the CR was keyed on
;
Prompt1:
	sc		r0,CursorCol	; go back to the start of the line
	call	CalcScreenLoc	; r1 = screen memory location
	or		r3,r1,r0
	lc		r1,[r3]
	addui	r3,r3,#2
	call	ScreenToAscii
	bnei	r1,#'$',Prompt2	; skip over '$' prompt character
	lc		r1,[r3]
	addui	r3,r3,#2
	call	ScreenToAscii

; Dispatch based on command character
;
Prompt2:
	beqi	r1,#':',Editmem		; $: - edit memory
	beqi	r1,#'D',Dumpmem		; $D - dump memory
	beqi	r1,#'B',START		; $B - start tiny basic
	beqi	r1,#'J',ExecuteCode	; $J - execute code
	beqi	r1,#'L',LoadS19		; $L - load S19 file
	beqi	r1,#'?',DisplayHelp	; $? - display help
	beqi	r1,#'C',TestCLS		; $C - clear screen
	bra		Monitor

TestCLS:
	lc		r1,[r3]
	addui	r3,r3,#2
	call	ScreenToAscii
	bnei	r1,#'L',Monitor
	lc		r1,[r3]
	addui	r3,r3,#2
	call	ScreenToAscii
	bnei	r1,#'S',Monitor
	call	ClearScreen
	sb		r0,CursorCol
	sb		r0,CursorRow
	call	CalcScreenLoc
	bra		Monitor
	
DisplayHelp:
	setlo	r1,HelpMsg
	call	DisplayString
	bra		Monitor

	align	16
HelpMsg:
	db	"? = Display help",CR,LF
	db	"CLS = clear screen",CR,LF
	db	": = Edit memory bytes",CR,LF
	db	"L = Load S19 file",CR,LF
	db	"D = Dump memory",CR,LF
	db	"B = start tiny basic",CR,LF
	db	"J = Jump to code",CR,LF,0
	align	16

;------------------------------------------------------------------------------
; Ignore blanks in the input
; r3 = text pointer
; r1 destroyed
;------------------------------------------------------------------------------
;
ignBlanks:
	subui	sp,sp,#8
	sw		r31,[sp]
ignBlanks1:
	lc		r1,[r3]
	addui	r3,r3,#2
	call	ScreenToAscii
	beqi	r1,#' ',ignBlanks1
	subui	r3,r3,#2
	lw		r31,[sp]
	ret		#8

;------------------------------------------------------------------------------
; Edit memory byte(s).
;------------------------------------------------------------------------------
;
EditMem:
	call	ignBlanks
	call	GetHexNumber
	or		r5,r1,r0
	setlo	r4,#7
edtmem1:
	call	ignBlanks
	call	GetHexNumber
	sb		r1,[r5]
	addui	r5,r5,#1
	loop	r4,edtmem1
	bra		Monitor

;------------------------------------------------------------------------------
; Execute code at the specified address.
;------------------------------------------------------------------------------
;
ExecuteCode:
	call	ignBlanks
	call	GetHexNumber
	or		r3,r1,r0
	jal		r31,[r3]
	bra     Monitor

;------------------------------------------------------------------------------
; Do a memory dump of the requested location.
;------------------------------------------------------------------------------
;
DumpMem:
	call	ignBlanks
	call	GetHexNumber
	or		r2,r1,r0
	call	CRLF
	call	DisplayMem
	call	DisplayMem
	call	DisplayMem
	call	DisplayMem
	call	DisplayMem
	call	DisplayMem
	call	DisplayMem
	call	DisplayMem
	bra		Monitor

;------------------------------------------------------------------------------
; Get a hexidecimal number. Maximum of sixteen digits.
; R3 = text pointer (updated)
;------------------------------------------------------------------------------
;
GetHexNumber:
	subui	sp,sp,#24
	sm		[sp],r2/r4/r31
	setlo	r2,#0
	setlo	r4,#15
gthxn2:
	lc		r1,[r3]
	addui	r3,r3,#2
	call	ScreenToAscii
	call	AsciiToHexNybble
	beqi	r1,#-1,gthxn1
	shli	r2,r2,#4
	andi	r1,r1,#0x0f
	or		r2,r2,r1
	loop	r4,gthxn2
gthxn1:
	or		r1,r2,r0
	lm		[sp],r2/r4/r31
	ret		#24

;------------------------------------------------------------------------------
; Convert ASCII character in the range '0' to '9', 'a' to 'f' or 'A' to 'F'
; to a hex nybble.
;------------------------------------------------------------------------------
;
AsciiToHexNybble:
	bltui	r1,#'0',gthx3
	bgtui	r1,#'9',gthx5
	subui	r1,r1,#'0'
	ret
gthx5:
	bltui	r1,#'A',gthx3
	bgtui	r1,#'F',gthx6
	subui	r1,r1,#'A'
	addui	r1,r1,#10
	ret
gthx6:
	bltui	r1,#'a',gthx3
	bgtui	r1,#'f',gthx3
	subui	r1,r1,#'a'
	addui	r1,r1,#10
	ret
gthx3:
	setlo	r1,#-1		; not a hex number
	ret

;==============================================================================
; Load an S19 format file
;==============================================================================
;
LoadS19:
	bra		ProcessRec
NextRec:
	call	sGetChar
	bne		r1,#LF,NextRec
ProcessRec:
	call	sGetChar
	beqi	r1,#26,Monitor	; CTRL-Z ?
	bnei	r1,#'S',NextRec
	call	sGetChar
	blt		r1,#'0',NextRec
	bgt		r1,#'9',NextRec
	or		r4,r1,r0		; r4 = record type
	call	sGetChar
	call	AsciiToHexNybble
	or		r2,r1,r0
	call	sGetChar
	call	AsciiToHexNybble
	shli	r2,r2,#4
	or		r2,r2,r1		; r2 = byte count
	or		r3,r2,r1		; r3 = byte count
	beqi	r4,#'0',NextRec	; manufacturer ID record, ignore
	beqi	r4,#'1',ProcessS1
	beqi	r4,#'2',ProcessS2
	beqi	r4,#'3',ProcessS3
	beqi	r4,#'5',NextRec	; record count record, ignore
	beqi	r4,#'7',ProcessS7
	beqi	r4,#'8',ProcessS8
	beqi	r4,#'9',ProcessS9
	bra		NextRec

pcssxa:
	andi	r3,r3,#0xff
	subui	r3,r3,#1		; one less for loop
pcss1a:
	call	sGetChar
	call	AsciiToHexNybble
	shli	r2,r2,#4
	or		r2,r2,r1
	call	sGetChar
	call	AsciiToHexNybble
	shli	r2,r2,#4
	or		r2,r2,r1
	sb		r2,[r5]
	addui	r5,r5,#1
	loop	r3,pcss1a
; Get the checksum byte
	call	sGetChar
	call	AsciiToHexNybble
	shli	r2,r2,#4
	or		r2,r2,r1
	call	sGetChar
	call	AsciiToHexNybble
	shli	r2,r2,#4
	or		r2,r2,r1
	bra		NextRec

ProcessS1:
	call	S19Get16BitAddress
	bra		pcssxa
ProcessS2:
	call	S19Get24BitAddress
	bra		pcssxa
ProcessS3:
	call	S19Get32BitAddress
	bra		pcssxa
ProcessS7:
	call	S19Get32BitAddress
	sw		r5,S19StartAddress
	bra		Monitor
ProcessS8:
	call	S19Get24BitAddress
	sw		r5,S19StartAddress
	bra		Monitor
ProcessS9:
	call	S19Get16BitAddress
	sw		r5,S19StartAddress
	bra		Monitor

S19Get16BitAddress:
	subui	sp,sp,#8
	sw		r31,[sp]
	call	sGetChar
	call	AsciiToHexNybble
	or		r2,r1,r0
	bra		S1932b

S19Get24BitAddress:
	subui	sp,sp,#8
	sw		r31,[sp]
	call	sGetChar
	call	AsciiToHexNybble
	or		r2,r1,r0
	bra		S1932a

S19Get32BitAddress:
	subui	sp,sp,#8
	sw		r31,[sp]
	call	sGetChar
	call	AsciiToHexNybble
	or		r2,r1,r0
	call	sGetChar
	call	AsciiToHexNybble
	shli	r2,r2,#4
	or		r2,r1,r2
	call	sGetChar
	call	AsciiToHexNybble
	shli	r2,r2,#4
	or		r2,r2,r1
S1932a:
	call	sGetChar
	call	AsciiToHexNybble
	shli	r2,r2,#4
	or		r2,r2,r1
	call	sGetChar
	call	AsciiToHexNybble
	shli	r2,r2,#4
	or		r2,r2,r1
S1932b:
	call	sGetChar
	call	AsciiToHexNybble
	shli	r2,r2,#4
	or		r2,r2,r1
	call	sGetChar
	call	AsciiToHexNybble
	shli	r2,r2,#4
	or		r2,r2,r1
	call	sGetChar
	call	AsciiToHexNybble
	shli	r2,r2,#4
	or		r2,r2,r1
	xor		r4,r4,r4
	or		r5,r2,r0
	lw		r31,[sp]
	addui	sp,sp,#8
	ret

;------------------------------------------------------------------------------
; Get a character from auxillary input, checking the keyboard status for a
; CTRL-C
;------------------------------------------------------------------------------
;
sGetChar:
	subui	sp,sp,#8
	sw		r31,[sp]
sgc2:
	call	KeybdCheckForKey
	beq		r1,r0,sgc1
	call	KeybdGetchar
	beqi	r1,#CRTLC,Monitor
sgc1:
	call	AUXIN
	beq		r1,r0,sgc2
	lw		r31,[sp]
	addui	sp,sp,#8
	ret

;--------------------------------------------------------------------------
; Sound a 800 Hz beep
;--------------------------------------------------------------------------
;
SetupAC97:
	ori		r1,r0,#0		; trigger a read of register 26
	sc		r1,AC97+0x26
sac971:						; wait for status to register 0xF (all ready)
	lc		r1,AC97+0x26
	bnei	r1,#0x0F,sac971
	ori		r1,r0,#0		; master volume, 0db attenuation, mute off
	sc		r1,AC97+2
	sc		r1,AC97+4		; headphone volume, 0db attenuation, mute off
	ori		r1,r0,#8000		; wait a while for the settings to take effect
sac972:					
	loop	r1,sac972

Beep:
	ori		r1,r0,#15		; master volume to max
	sc		r1,PSG+128
	ori		r1,r0,#13422	; 800Hz
	sc		r1,PSG
	ori		r1,r0,#32		; attack (8.192 ms)
	sc		r1,PSG+8
	ori		r1,r0,#64		; decay  (16.384 ms)
	sc		r1,PSG+10
	ori		r1,r0,#0xC0		; sustain level
	sc		r1,PSG+12
	ori		r1,r0,#4000		; release (1.024 s)
	sc		r1,PSG+14
	ori		r1,r0,#0x1104	; gate, output enable, triangle waveform
	sc		r1,PSG+4
	ori		r1,r0,#25000000	; delay about 1s
beep1:
	loop	r1,beep1
	ori		r1,r0,#0x0000	; gate off, output enable off, no waveform
	ret

;*
;* ===== Input a character from the host into register D0 (or
;*	return Zero status if there's no character available).
;*
AUXIN:
	inb		r1,UART_LS		; is character ready ?
	andi	r1,r1,#rxfull
	beq		r1,r0,AXIRET	;if not, return Zero status
	inb		r1,UART			; else get the character
	andi	r1,r1,#0x7f		;zero out the high bit
AXIRET:
	ret

;*
;* ===== Return to the resident monitor, operating system, etc.
;*
BYEBYE:
	jmp		Monitor
;    MOVE.B	#228,D7 	;return to Tutor
;	TRAP	#14

	align 16
msgInit db	CR,LF,"Raptor64 Tiny BASIC v1.0",CR,LF,"(C) 2012  Robert Finch",CR,LF,LF,0
OKMSG	db	CR,LF,"OK",CR,LF,0
msgWhat	db	"What?",CR,LF,0
SRYMSG	db	"Sorry."
CLMSG	db	CR,LF,0
msgReadError	db	"Compact FLASH read error",CR,LF,0
msgNumTooBig	db	"Number is too big",CR,LF,0
msgDivZero		db	"Division by zero",CR,LF,0
msgVarSpace     db  "Out of variable space",CR,LF,0
msgBytesFree	db	" bytes free",CR,LF,0
msgReady		db	CR,LF,"Ready",CR,LF,0
msgComma		db	"Expecting a comma",CR,LF,0
msgLineRange	db	"Line number too big",CR,LF,0
msgVar			db	"Expecting a variable",CR,LF,0
msgRNDBad		db	"RND bad parameter",CR,LF,0
msgSYSBad		db	"SYS bad address",CR,LF,0
msgInputVar		db	"INPUT expecting a variable",CR,LF,0
msgNextFor		db	"NEXT without FOR",CR,LF,0
msgNextVar		db	"NEXT expecting a defined variable",CR,LF,0
msgBadGotoGosub	db	"GOTO/GOSUB bad line number",CR,LF,0
msgRetWoGosub   db	"RETURN without GOSUB",CR,LF,0
msgTooBig		db	"Program is too big",CR,LF,0
msgExtraChars	db	"Extra characters on line ignored",CR,LF,0

INITMSG:
	db		CR,LF,'Raptor64 Tiny BASIC, v1.0',CR,LF,LF,0
OKMSG:
	db		CR,LF,'OK',CR,LF,0
HOWMSG:
	db		'How?',CR,LF,0
WHTMSG:
	db		'What?',CR,LF,0
SRYMSG:
	db		'Sorry.'
CLMSG:
	db		CR,LF,0
;	DC.B	0	;<- for aligning on a word boundary
	align	16
	
LSTROM	EQU		$
	;	end of possible ROM area

	bss
	align	16

		org		0x0080
typef   db      0   ; variable / expression type
        align   8
OSSP	dw	1	; OS value of sp
CURRNT	dw	1	;	Current line pointer
STKGOS	dw	1	;	Saves stack pointer in 'GOSUB'
STKINP	dw	1	;	Saves stack pointer during 'INPUT'
LOPVAR	dw	1	;	'FOR' loop save area
LOPINC	dw	1	;	increment
LOPLMT	dw	1	;	limit
LOPLN	dw	1	;	line number
LOPPT	dw	1	;	text pointer
TXTUNF	dw	1	;	points to unfilled text area
VARBGN	dw	1	;	points to variable area
IVARBGN dw  1   ;   points to integer variable area
SVARBGN dw  1   ;   points to string variable area
FVARBGN dw  1   ;   points to float variable area
STKBOT	dw	1	;	holds lower limit for stack growth
NUMWKA	fill.b	12,0			; numeric work area
BUFFER	fill.b	BUFLEN,0x00		;		Keyboard input buffer


;==============================================================================
; Checkerboard RAM tester
;==============================================================================
;
	code
	align	16
ramtest:
	or		r8,r0,r0		; r8 = 0
	ori		r1,r0,#0xAAAA5555AAAA5555	; checkerboard pattern
ramtest2:
	sw		r1,[r8]			; save the checkerboard to memory
	lw		r2,[r8]			; read it back
	cmp		r3,r1,r2		; is it the same ?
	bne 	r3,r0,ramtest1
	addui	r8,r8,#8		; increment RAM pointer
	cmpi	r3,r8,#0x0000_0000_0400_0000
	blt		r3,r0,ramtest2
ramtest1:
	or		r10,r8,r0		; r10 = max ram address
	; readback the checkerboard pattern
	or		r8,r0,r0		; r8 = 0
ramtest4:
	lw		r2,[r8]
	cmpi	r3,r2,#0xAAAA5555AAAA5555
	bne		r3,r0,ramtest3
	addi	r8,r8,#8
	cmpi	r3,r8,#0x0000_0000_0100_0000
	blt 	r3,r0,ramtest4
ramtest3:
	bne		r8,r10,ramtest8	; check for equal maximum address

	; perform ramtest again with inverted checkerboard
	or		r8,r0,r0		; r8 = 0
	ori		r1,r0,#0x5555AAAA5555AAAA
ramtest5:
	sw		r1,[r8]
	lw		r2,[r8]
	cmp		r3,r1,r2
	bne		r3,r0,ramtest6
	addi	r8,r8,#8
	cmpi	r3,r8,#0x0000_0000_0100_0000
	blt		r3,r0,ramtest5
ramtest6:
	or		r11,r8,r0		; r11 = max ram address
	; readback checkerboard
	or		r8,r0,r0
ramtest7:
	lw		r2,[r8]
	cmpi	r3,r2,#0x5555AAAA5555AAAA
	bne		r3,r0,ramtest8
	addi	r8,r8,#8
	cmpi	r3,r8,#0x0000_0000_0100_0000
	blt		r3,r0,ramtest7
ramtest8:
	beq		r8,r11,ramtest9
	min		r8,r8,r11
ramtest9:
	beq		r8,r10,ramtest10
	min		r8,r8,r10
ramtest10:
	sw		r8,0x00000400	;memend
	ret

;-------------------------------------------
; IRQ routine
;-------------------------------------------
irqrout:
	subui	sp,sp,#16
	sm		[sp],r1/lr
	inch	r1,PIC
	beqi	r1,#1,ColdStart
irqrout3:
	bnei	r1,#2,irqrout2
	call	Pulse1000
	bra		irqrout1
irqrout2:
	bnei	r1,#15,irqrout1
	call	KeybdIRQ
irqrout1:
	lm		[sp],r1/lr
	addui	sp,sp,#16
	iret

;-------------------------------------------
; NMI routine
;-------------------------------------------
nmirout:
	iret

;-------------------------------------------
; Handle miss on Data TLB
;-------------------------------------------
DTLBHandler:
	sw		r1,0xFFFF_FFFF_FFFF_0000
	sw		r2,0xFFFF_FFFF_FFFF_0008
dh1:
	omgi	r1,#0		; try open mutex gate #0 (TLB protector)
	bne		r1,r0,dh1	; spinlock if gate is closed
	mfspr	r1,PTA		; get the page table address
	mfspr	r2,BadVAddr	; get the bad virtual address
	mtspr	TLBVirtPage,r2	; which virtual address to update
	shrui	r2,r2,#13	; turn va into index
	addu	r1,r1,r2
	lw		r2,[r1]		; get the physical address from the table
	and		r2,r2,#FFFF_FFFF_FFFF_E000	; mask off lower bits
	mtspr	TLBPhysPage0,r2	;
	lw		r2,8[r1]	; get the physical address from the table
	and		r2,r2,#FFFF_FFFF_FFFF_E000	; mask off lower bits
	mtspr	TLBPhysPage1,r2	;
	tlbwr				; update a random entry in the TLB
	cmgi	#0			; close the mutex gate
	lw		r1,0xFFFF_FFFF_FFFF_0000
	lw		r2,0xFFFF_FFFF_FFFF_0008
	iret
	nop
	nop
	
	org		0xFFFF_FFFF_FFFF_FFB0
	jmp		DTLBHandler
	nop
	nop
	org		0xFFFF_FFFF_FFFF_FFC0
	jmp		DTLBHandler
	nop
	nop
	org     0xFFFF_FFFF_FFFF_FFD0
	jmp		irqrout
	nop
	nop
	org     0xFFFF_FFFF_FFFF_FFE0
	jmp		nmirout
	nop
	nop
	org		0xFFFF_FFFF_FFFF_FFF0
	jmp		start
	nop
	nop


