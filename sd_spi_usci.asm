; MSP430 host device, sd card slave device
; bare-metal assembly for interacting w/ the SD cards internal microcontroller
; MSP430 sends out commands over USCI and waits for a response
; Currently the response isn't processed, just confirm there are no errors
; Also will verify the communication over sigrok

	.include "msp430g2553.inc" ; include device header file

	.sect ".data", "wa"
CmdBytes: .space 6, 0    ; reserve 6 bytes and fill w/ 0s
;RespBytes: .space 148, 0    ; reserve x bytes and fill w/ 0s

	.text

	.equ SP,      R1
	.equ MOSI,    BIT7
	.equ MISO,    BIT6
	.equ SPI_CLK, BIT5
	.equ CS,      BIT4
	.equ SUB_CLK_SEL_BIT, UCSSEL1

	.equ LED1,    BIT0  ; debug LED
	.equ DELAYLOOPS, 100
	.equ STARTUP_DELAY, 500

	.equ INIT_TX_COUNT, 12  ; total number of clocks is INIT_TX_COUNT * 8
	.equ SPI_DUMMY_BYTE, 0xFF

	; commands
	.equ CMD_BYTES, 6
	.equ CMD_MASK, 0b111111
	.equ CMD_TRANS_BIT,  0x40

	.equ CRC7_MSB_OUT_MASK, 0x40 ; CRC7 is 7 bits wide hence the MSB being 0x40 not 0x80
	.equ CRC7_BIT2_MASK,    0x04 ; BIT2 mask before the left-shift into BIT3
	.equ CRC7_MSB_IN_MASK,  0x80
	; iteration length constants
	.equ CRC7_CMD_ILENGTH, 40
	.equ CRC7_CSD_ILENGTH, 120
	.equ CRC7_CID_ILENGTH, 120

	.global Reset  ; expose to linker


 ;
 ; Crc7Calc subroutine performs a CRC7 check on either a 40-bit or
 ; 120-bit input (fed byte by byte via R6)
 ; R8 is used for the address of the input byte (later stored in R6)
 ; R7 is used to for the CRC7 result
 ; R14 is used for the total number of iterations/bits (either 40 or 120)
 ; R12-R15 are used for intermediate calculations
 ;
 ; Since a command is 6 bytes long we write the resulting CRC7 result back into RAM
 ; so you can later send a command over USCI w/o calculating the CRC7 whilst transmitting
 ;
 ; The result returned is a concat of a 7-bit CRC plus an extra stop bit on the LSB
 ; making the output 8-bits wide
 ;
Crc7Calc:
	; use register r6 and r7 as our working register, save context onto stack
	push.w   r6  ; used for input byte for CRC7
	push.w   r7  ; used for CRC7 result
	push.w   r8  ; used for array address/index
	mov.w    #CmdBytes, r8  ; move address of CmdBytes to r8
	mov.b    #0, r7  ; CRC init
	; now process r7 (r6 into CRC7 stream)
GetNewByte:
	mov.b    #8, r15  ; reset byte counter, when reaches zero we increase the array index
	mov.b    @r8+, r6
AlgoLoop:
	; temp registers for intermediate results are r12-r15
	bit.b    #CRC7_MSB_IN_MASK, r6
	jnz      MsbIsSet  ; if r6 (input) MSB is set then set r12 to 1, otherwise clear below
	mov.b    #0, r12
	jmp      AlgoLoopCont1
MsbIsSet:
	mov.b    #1, r12  ; move MSB to LSB of r12
AlgoLoopCont1:
	bit.b    #CRC7_MSB_OUT_MASK, r7
	jnz      CrcMsbIsSet  ; if r7 (CRC) MSB is set then set r13 to 1, otherwise clear below
	xor.b    #0, r12  ; xor 0 w/ CRC input r12: xor crc_msb w/ input bit
	jmp      AlgoLoopCont2
CrcMsbIsSet:
	xor.b    #1, r12  ; xor 1 w/ CRC input r12: xor crc_msb w/ input bit
AlgoLoopCont2:
	; re-use r12 (our intermediate result) in the next xor
	; r13 stores a 1 or 0 depending on whether the BIT2 is set or unset in the CRC
	bit.b    #CRC7_BIT2_MASK, r7
	jnz      CrcBitIsSet  ; if r7 mask (CRC bit) is set then set r13 to 1, otherwise clear below
	mov.b    #0, r13
	jmp      AlgoLoopCont3
CrcBitIsSet:
	mov.b    #1, r13  ; set LSB of r13 (used for a quick XOR later rather than shifting)
AlgoLoopCont3:
	xor.b    r12, r13  ; 2nd xor result (r13), 1st xor result (r12) xor'd w/ mask of BIT2 (r13)

	; r12 and r13 now store the 1st and 2nd xor results
	; shift out CRC (r7) and input (r6) to the left
	rla.b    r7
	rla.b    r6   ; get new MSB
.ifdef DEBUG
	bic.b    #128, r7  ; clear MSB of CRC (useful for debugging w/ GDB e.g. info reg)
.endif
	cmp.b    #1, r12
	jz       SetCrcLsb
CheckCrcBit3:
	bic.b    #BIT3, r7 ; clear BIT2 of CRC7
	cmp.b    #1, r13
	jz       SetCrcBit3
	jmp      CheckLoop  ; don't un-intentionally set CRC LSB (equiv to jnz ans jz isn't taken)
SetCrcLsb:
	bis.b    #1, r7
	jmp      CheckCrcBit3 ; unconditional jump as we always need to check BIT2
SetCrcBit3:
	bis.b    #BIT3, r7  ; BIT2 --> BIT3, update based on r13 (2nd xor result)
CheckLoop:
	; need to go for either 40 or 120 iterations
	; also need to change r6 every 8 iterations (sending a new byte)
	dec.b    r14
	jz       AlgoDone
	dec.b    r15
	jz       GetNewByte
	jnz      AlgoLoop
AlgoDone:
	rla.b    r7      ; make space for LSB (stop bit)
	bis.b    #1, r7  ; set stop bit
	mov.b    r7, &CmdBytes + 5  ; move CRC7 result to end of array

	; restore prev pushes to registers
	pop.w    r8
	pop.w    r7
	pop.w    r6

	ret  ; crc7


 ;
 ; InitSdSpiBus performs the basic SD card SPI startup sequence
 ; (sending 74 clocks whilst CS line is held high)
 ; clock must be betweeb 100-400 kHz during this sequence
 ;
InitSdSpiBus:
	; drive clock for 74 clocks (minimum) whilst CS is held high
	bis.b    #CS, &P1OUT ; pre-drive cs output high (active low)
	bis.b    #CS, &P1DIR ; set CS to an output
	mov.b    #INIT_TX_COUNT, r5
TxFlagPollInitFunc:
	; if we can write to the TxBuffer do
	bit.b    #UCB0TXIFG, &IFG2
	jz       TxFlagPollInitFunc
DataTxRxInitFunc:
	mov.b    #SPI_DUMMY_BYTE, &UCB0TXBUF  ; ignored while CS is held high
RxFlagPollInitFunc:
	bit.b    #UCB0RXIFG, &IFG2  ; check if transfer is complete
	jz       RxFlagPollInitFunc
	cmp.b    #0, &UCB0RXBUF    ; clear RXFIG
	dec.b    r5
	jnz      TxFlagPollInitFunc    ; make sure TXBUF is free (it should be)
	; enable TX RX interrupts?

	ret  ; InitSdSpiBus

 ;
 ; CmdHighestByteToRam transfers the highest CMD byte to RAM
 ; Highest byte of a SD CMD command is: 0k xxxxxx,
 ; where 1) k = direction of transmission (1 is master to slave),
 ;       2) 6-bit xx field is the binary representation of the command
 ;       3) start bit (MSB) is always 0
 ; e.g. CMD 17 the 6-bit field is 010001, max val is 111111 --> 63
 ; R14 is used for the CMD index (which should be less than 63)
 ;
CmdHighestByteToRam:
	; assuming we will always send a 1 for transmission direction
	and.b    #CMD_MASK, r14  ; in the unlikely case we pass a arg larger than 63 to r14, also clears MSB as a side effect
	bis.b    #CMD_TRANS_BIT, r14
	mov.b    r14, &CmdBytes  ; send byte to RAM

	ret  ; CMDHighestByte


 ; SendCmdSpi sends a specific SD SPI CMD to the SD card
 ; reads from a fixed location (array) in RAM
 ; R8 is used for the array index of the CMD bytes
 ; R6 is used to store the current CMD byte to be transmitted
 ; R14 is used for the number of iterations (total number of CMD bytes to send)
SendCmdSpi:
	mov.w    #CmdBytes, r8  ; move address of CmdBytes to r8
	mov.b    #CMD_BYTES, r14
	bic.b    #CS, &P1OUT ; assert CS line low to communicate w/ SD slave device
SendNewByte:
	mov.b    @r8+, r6   ; move value pointed to by r8 into r6
TxFlagPollCmdSpi:
	; if we can write to the TxBuffer
	bit.b    #UCB0TXIFG, &IFG2
	jz       TxFlagPollCmdSpi
DataTxRxCmdSpi:
	mov.b    r6, &UCB0TXBUF
RxFlagPollCmdSpi:
	bit.b    #UCB0RXIFG, &IFG2  ; check if transfer is complete
	jz       RxFlagPollCmdSpi
	cmp.b    #0, &UCB0RXBUF    ; clear RXFIG
	dec.b    r14            ; decrement counter
	jnz      SendNewByte

	ret ; SendCmdSpi

 ; WaitForResponse will keep sending clocks to the SD card until it
 ; reads a valid response (by continually sending 0xFF over the MOSI line
 ; and then probing the RXBUF input)
 ; R14 is used for the number of iterations (NCr)
WaitForResponse:
	mov.b    #8, r14   ; maximum number of bytes to wait (Ncr)
TxFlagPollResp:
	; if we can write to the TxBuffer do
	bit.b    #UCB0TXIFG, &IFG2
	jz       TxFlagPollResp
DataTxRxResp:
	mov.b    #SPI_DUMMY_BYTE, &UCB0TXBUF  ; send 0xFF to recieve a response
RxFlagPollResp:
	bit.b    #UCB0RXIFG, &IFG2  ; check if transfer is complete
	jz       RxFlagPollResp
RxCheck:
	cmp.b    #0x01, &UCB0RXBUF  ; check for response
	jz       RecvResp
ReLoop:
	dec.b    r14
	jnz      TxFlagPollResp
	; drive CS high after exhausting all attempts
	bis.b    #CS, &P1OUT ; drive cs output high (active low) (re-add this line)
	jmp      EarlyRet
RecvResp:
	bis.b    #CS, &P1OUT ; drive cs output high (active low) (re-add this line)
	bis.b    #LED1, &P1OUT  ; recieved a valid response (continue coding)
EarlyRet:
	ret  ; WaitForResponse



Reset:
	mov.w    #WDTPW | WDTHOLD, &WDTCTL ; stop watchdog timer
	mov.w    #__stack, SP

	; USCI setup
	bis.b    #UCSWRST, &UCB0CTL1 ; reset USCI_B
	; MSP430 is SPI master, SPI mode 0 (CPHA 0 (UCCKPH 1), CPOL 0), MSB out (first), 3 pin SPI
	bis.b    #UCCKPH | UCMST | UCMSB | UCMODE_0, &UCB0CTL0
	bis.b    #SUB_CLK_SEL_BIT, &UCB0CTL1 ; clock is SMCLK (~1MHz upon reset)

	; pin setup for USCI_B0 (check your device specific datasheet, there are 2 PxSEL registers for each port)
	bis.b    #MOSI | MISO | SPI_CLK, &P1SEL ; configure pins for USCI
	bis.b    #MOSI | MISO | SPI_CLK, &P1SEL2 ; configure pins for USCI
	;bis.b    #MISO, &P1REN ; use internal pull-down res for MISO

	mov.b    #4, &UCB0BR0 ; SMCLK / 4 ---> ~250kHz (see: InitSdSpiBus subroutine comments)
	mov.b    #0, &UCB0BR1

	bic.b    #UCSWRST, &UCB0CTL1 ; enable USCI_B after configuration
	; enable interrupts if necessary
	xor.b    r11, r11  ; clear counter, 1 cycle clear vs mov #0, r11 (2 cycles)
	mov.w    #DELAYLOOPS, r7

	; LED setup
	bic.b    #LED1, &P1OUT
	bis.b    #LED1, &P1DIR

	; wait 1ms for SD card (each clock is 1us) need to wait at least 1000 clocks
	; only needed if MSP430 is powered on at the same time as the SD card
	; clock divider is /4 (so we are now delaying for ~4ms)
	mov.b    #STARTUP_DELAY, r14
StartupDelay:
	dec.b    r14          ; 1 cycle
	jnz      StartupDelay ; 2 cycles, 3 cycles per loop

SpiComm:
	call   #InitSdSpiBus

	mov.b    #0, r14  ; CMD_index <= 63
	call     #CmdHighestByteToRam
	mov.b    #0x00, &CmdBytes + 1
	mov.b    #0x00, &CmdBytes + 2
	mov.b    #0x00, &CmdBytes + 3
	mov.b    #0x00, &CmdBytes + 4
	mov.b    #CRC7_CMD_ILENGTH, r14
	call     #Crc7Calc

	call     #SendCmdSpi  ; sending command 0
	call     #WaitForResponse
	jmp      InfLoop  ; nothing to do here so goto InfLoop

InfLoop:
	jmp      InfLoop

;----------------
	.sect "__reset_vector", "a"  ; Reset vector, a flag is necessary for ELF output
	.word  Reset   ; Address of execution
	.end