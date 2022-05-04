; ASM style MSP430 for GNU AS
; basic code for setting up USCI_B0 in SPI mode (3-pin)

	.include "msp430g2553.inc" ; include device header file

	.text

	.equ MOSI,    BIT7
	.equ MISO,    BIT6
	.equ SPI_CLK, BIT5
	.equ CS,      BIT4
	.equ SUB_CLK_SEL_BIT, UCSSEL1

	.equ LED1,    BIT0  ; debug LED, set if TXBUF = RXBUF
	.equ DELAYLOOPS, 100

	.global Reset  ; expose to linker

Reset:
	mov.w    #WDTPW | WDTHOLD, &WDTCTL ; stop watchdog timer
	mov.w    #__stack, R1  ; R1 = SP

	bis.b    #UCSWRST, &UCB0CTL1 ; reset USCI_B
	bis.b    #UCMST | UCMSB | UCMODE_0, &UCB0CTL0 ; MSP430 is SPI master, 3 pin SPI, MSB out first
	bis.b    #SUB_CLK_SEL_BIT, &UCB0CTL1 ; clock is SMCLK (~1MHz upon reset)

	bis.b    #UCLISTEN, &UCB0STAT  ; internal loopback
	; pin setup for USCI_B0 (check your device specific datasheet, there are 2 PxSEL registers for each port)
	bis.b    #MOSI | MISO | SPI_CLK, &P1SEL ; configure pins for USCI
	bis.b    #MOSI | MISO | SPI_CLK, &P1SEL2 ; configure pins for USCI
	bis.b    #CS, &P1OUT ; pre-drive cs output high (active low)
	bis.b    #CS, &P1DIR ; set CS to an output
	;bis.b    #MISO, &P1REN ; use pull-down res for MISO
	mov.b    #1, &UCB0BR0 ; SMCLK / 1
	mov.b    #0, &UCB0BR1

	bic.b    #UCSWRST, &UCB0CTL1 ; enable USCI_B after configuration
	; enable interrupts if necessary
	mov.b    #0, r11   ; our counter
	mov.w    #DELAYLOOPS, r7

	; LED setup
	bic.b    #LED1, &P1OUT
	bis.b    #LED1, &P1DIR
TxFlagPoll:
	; if we can write to the TxBuffer
	bit.b    #UCB0TXIFG, &IFG2
	jz       TxFlagPoll
DataTxRx:
	bic.b    #CS, &P1OUT ; drive cs output low (active low)
	bic.b    #LED1, &P1OUT ; clear LED on TxRx
	mov.b    r11, &UCB0TXBUF
RxFlagPoll:
	bit.b    #UCB0RXIFG, &IFG2  ; check if transfer is complete
	jz       RxFlagPoll
	bis.b    #CS, &P1OUT ; drive cs output high (active low)
	cmp.b    r11, &UCB0RXBUF
	jnz      DelayTx
	bis.b    #LED1, &P1OUT  ; if r11 (TXBUF) == RXBUF set LED1

	; delay between transfers
DelayTx:
	dec.w    r7
	jnz      DelayTx

	mov.w    #DELAYLOOPS, r7  ; reset loop counter
	inc.b    r11
	bit.b    #8, r11
	jz       TxFlagPoll  ; restart SPI transfer
InfLoop:
	jmp      InfLoop

;----------------
	.sect "__reset_vector", "a"  ; Reset vector, a flag is necessary for ELF output
	.word  Reset   ; Address of execution
	.end
