
	org 0xFFFF_FFFF_FFFF_F000
start:
	ori		r1,r0,#5117		; test data
	sqrt	r4,r1
	sqrt	r4,r1
	ori		r2,r0,#1067
	mulu	r5,r1,r2
	ori		r1,r0,#0xFFFF_FFFF_FFD0_0000
	ori 	r2,r0,#0x32
	ori 	r3,r0,#1612
j1:
	sc		r2,[r1]
	addi	r1,r1,#2
	subi	r3,r3,#1
	bne		r3,r0,j1
	call	Getkey
	nop
	nop
	nop
	nop
	
Getkey:
	ori		r1,r0,#0xFFFF_FFFF_FFDC_0000
sr1:
	lb		r2,[r1]
	bge		r2,r0,sr1
	lb		r0,1[r1]		; clear keyboard strobe
	ret

;==============================================================================
; Checkerboard RAM tester
;==============================================================================
;
ramtest:
	or		r8,r0,r0		; r8 = 0
	ori		r1,r0,#0xAAAA5555AAAA5555	; checkerboard pattern
ramtest2:
	sw		r1,[r8]			; save the checkerboard to memory
	lw		r2,[r8]			; read it back
	cmp		r3,r1,r2		; is it the same ?
	bnez 	r3,r0,ramtest1
	addi	r8,r8,#8		; increment RAM pointer
	cmpi	r3,r8,#0x0000_0000_0100_0000
	bltz	r3,r0,ramtest2
ramtest1:
	or		r10,r8,r0		; r10 = max ram address
	; readback the checkerboard pattern
	or		r8,r0,r0		; r8 = 0
ramtest4:
	lw		r2,[r8]
	cmpi	r3,r2,#0xAAAA5555AAAA5555
	bnez	r3,r0,ramtest3
	addi	r8,r8,#8
	cmpi	r3,r8,#0x0000_0000_0100_0000
	bltz	r3,r0,ramtest4
ramtest3:
	bne		r8,r10,ramtest8	; check for equal maximum address

	; perform ramtest again with inverted checkerboard
	or		r8,r0,r0		; r8 = 0
	ori		r1,r0,#0x5555AAAA5555AAAA
ramtest5:
	sw		r1,[r8]
	lw		r2,[r8]
	cmp		r3,r1,r2
	bnez	r3,r0,ramtest6
	addi	r8,r8,#8
	cmpi	r3,r8,#0x0000_0000_0100_0000
	bltz	r3,r0,ramtest5
ramtest6:
	or		r11,r8,r0		; r11 = max ram address
	; readback checkerboard
	or		r8,r0,r0
ramtest7:
	lw		r2,[r8]
	cmpi	r3,r2,#0x5555AAAA5555AAAA
	bnez	r3,r0,ramtest8
	addi	r8,r8,#8
	cmpi	r3,r8,#0x0000_0000_0100_0000
	bltz	r3,r0,ramtest7
ramtest8:
	beq		r8,r11,ramtest9
	min		r8,r8,r11
ramtest9:
	beq		r8,r10,ramtest10
	min		r8,r8,r10
ramtest10:
	sw		r8,0x00000400	;memend
	
	org		0xFFFF_FFFF_FFFF_FFC0
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	org		0xFFFF_FFFF_FFFF_FFF0
	jmp		start
