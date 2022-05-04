;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;;  msp430g2553 include file
;;;
;;;  hacked from the msp430-gcc header file with a series of regex replaces
;;;  and a little bit of hand tweaking.
;;;
;;;
;;;  adapted from source: https://github.com/gbhug5a/SD-Card-Bootloader-for-MSP430/blob/master/G2553/msp430g2553.inc
;;;  in vim to match GNU AS syntax
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;
;;; STANDARD BITS
;;;

.equ BIT0,      0x0001
.equ BIT1,      0x0002
.equ BIT2,      0x0004
.equ BIT3,      0x0008
.equ BIT4,      0x0010
.equ BIT5,      0x0020
.equ BIT6,      0x0040
.equ BIT7,      0x0080
.equ BIT8,      0x0100
.equ BIT9,      0x0200
.equ BITA,      0x0400
.equ BITB,      0x0800
.equ BITC,      0x1000
.equ BITD,      0x2000
.equ BITE,      0x4000
.equ BITF,      0x8000

;;;
;;; STATUS REGISTER BITS
;;;

.equ C,           0x0001
.equ Z,           0x0002
.equ N,           0x0004
.equ V,           0x0100
.equ GIE,         0x0008
.equ CPUOFF,      0x0010
.equ OSCOFF,      0x0020
.equ SCG0,        0x0040
.equ SCG1,        0x0080

;;;
;;; Low Power Modes coded with Bits 4-7 in SR
;;;

.equ LPM0,      CPUOFF
.equ LPM1,      SCG0+CPUOFF
.equ LPM2,      SCG1+CPUOFF
.equ LPM3,      SCG1+SCG0+CPUOFF
.equ LPM4,      SCG1+SCG0+OSCOFF+CPUOFF


;;;
;;; SPECIAL FUNCTION REGISTER ADDRESSES + CONTROL BITS
;;;

.equ IE1,         0x0000    ; Interrupt Enable 1

.equ WDTIE,       0x01      ; Watchdog Interrupt Enable
.equ OFIE,        0x02      ; Osc. Fault  Interrupt Enable
.equ NMIIE,       0x10      ; NMI Interrupt Enable
.equ ACCVIE,      0x20      ; Flash Access Violation Interrupt Enable

.equ IFG1,        0x0002    ; Interrupt Flag 1

.equ WDTIFG,      0x01      ; Watchdog Interrupt Flag
.equ OFIFG,       0x02      ; Osc. Fault Interrupt Flag
.equ PORIFG,      0x04      ; Power On Interrupt Flag
.equ RSTIFG,      0x08      ; Reset Interrupt Flag
.equ NMIIFG,      0x10      ; NMI Interrupt Flag

.equ IE2,         0x0001    ; Interrupt Enable 2
.equ UC0IE,       IE2
.equ UCA0RXIE,    0x01
.equ UCA0TXIE,    0x02
.equ UCB0RXIE,    0x04
.equ UCB0TXIE,    0x08

.equ IFG2,        0x0003    ; Interrupt Flag 2
.equ UC0IFG,      IFG2
.equ UCA0RXIFG,   0x01
.equ UCA0TXIFG,   0x02
.equ UCB0RXIFG,   0x04
.equ UCB0TXIFG,   0x08


;;;
;;; ADC10
;;;

#define __MSP430_HAS_ADC10__    ; Definition to show that Module is available

.equ ADC10DTC0,    0x0048    ; ADC10 Data Transfer Control 0
.equ ADC10DTC1,    0x0049    ; ADC10 Data Transfer Control 1
.equ ADC10AE0,     0x004A    ; ADC10 Analog Enable 0
.equ ADC10CTL0,    0x01B0    ; ADC10 Control 0
.equ ADC10CTL1,    0x01B2    ; ADC10 Control 1
.equ ADC10MEM,     0x01B4    ; ADC10 Memory
.equ ADC10SA,      0x01BC    ; ADC10 Data Transfer Start Address

; ADC10CTL0
.equ ADC10SC,      0x001     ; ADC10 Start Conversion
.equ ENC,          0x002     ; ADC10 Enable Conversion
.equ ADC10IFG,     0x004     ; ADC10 Interrupt Flag
.equ ADC10IE,      0x008     ; ADC10 Interrupt Enalbe
.equ ADC10ON,      0x010     ; ADC10 On/Enable
.equ REFON,        0x020     ; ADC10 Reference on
.equ REF2_5V,      0x040     ; ADC10 Ref 0:1.5V / 1:2.5V
.equ MSC,          0x080     ; ADC10 Multiple SampleConversion
.equ REFBURST,     0x100     ; ADC10 Reference Burst Mode
.equ REFOUT,       0x200     ; ADC10 Enalbe output of Ref.
.equ ADC10SR,      0x400     ; ADC10 Sampling Rate 0:200ksps / 1:50ksps
.equ ADC10SHT0,    0x800     ; ADC10 Sample Hold Select Bit: 0
.equ ADC10SHT1,    0x1000    ; ADC10 Sample Hold Select Bit: 1
.equ SREF0,        0x2000    ; ADC10 Reference Select Bit: 0
.equ SREF1,        0x4000    ; ADC10 Reference Select Bit: 1
.equ SREF2,        0x8000    ; ADC10 Reference Select Bit: 2
.equ ADC10SHT_0,   0x0000    ; 4 x ADC10CLKs
.equ ADC10SHT_1,   0x0800    ; 8 x ADC10CLKs
.equ ADC10SHT_2,   0x1000    ; 16 x ADC10CLKs
.equ ADC10SHT_3,   0x1800    ; 64 x ADC10CLKs

.equ SREF_0,      0x0000    ; VR+ = AVCC and VR- = AVSS
.equ SREF_1,      0x2000    ; VR+ = VREF+ and VR- = AVSS
.equ SREF_2,      0x4000    ; VR+ = VEREF+ and VR- = AVSS
.equ SREF_3,      0x6000    ; VR+ = VEREF+ and VR- = AVSS
.equ SREF_4,      0x8000    ; VR+ = AVCC and VR- = VREF-/VEREF-
.equ SREF_5,      0xA000    ; VR+ = VREF+ and VR- = VREF-/VEREF-
.equ SREF_6,      0xC000    ; VR+ = VEREF+ and VR- = VREF-/VEREF-
.equ SREF_7,      0xE000    ; VR+ = VEREF+ and VR- = VREF-/VEREF-

; ADC10CTL1
.equ ADC10BUSY,    0x0001    ; ADC10 BUSY
.equ CONSEQ0,      0x0002    ; ADC10 Conversion Sence Select 0
.equ CONSEQ1,      0x0004    ; ADC10 Conversion Sence Select 1
.equ ADC10SSEL0,   0x0008    ; ADC10 Clock Source Select Bit: 0
.equ ADC10SSEL1,   0x0010    ; ADC10 Clock Source Select Bit: 1
.equ ADC10DIV0,    0x0020    ; ADC10 Clock Divider Select Bit: 0
.equ ADC10DIV1,    0x0040    ; ADC10 Clock Divider Select Bit: 1
.equ ADC10DIV2,    0x0080    ; ADC10 Clock Divider Select Bit: 2
.equ ISSH,         0x0100    ; ADC10 Invert Sample Hold Signal
.equ ADC10DF,      0x0200    ; ADC10 Data Format 0:binary 1:2's complement
.equ SHS0,         0x0400    ; ADC10 Sample/Hold Source Bit: 0
.equ SHS1,         0x0800    ; ADC10 Sample/Hold Source Bit: 1
.equ INCH0,        0x1000    ; ADC10 Input Channel Select Bit: 0
.equ INCH1,        0x2000    ; ADC10 Input Channel Select Bit: 1
.equ INCH2,        0x4000    ; ADC10 Input Channel Select Bit: 2
.equ INCH3,        0x8000    ; ADC10 Input Channel Select Bit: 3
.equ CONSEQ_0,     0x0000    ; Single channel single conversion
.equ CONSEQ_1,     0x0002    ; Sence of channels
.equ CONSEQ_2,     0x0004    ; Repeat single channel
.equ CONSEQ_3,     0x0006    ; Repeat sence of channels

.equ ADC10SSEL_0,    0x0000    ; ADC10OSC
.equ ADC10SSEL_1,    0x0008    ; ACLK
.equ ADC10SSEL_2,    0x0010    ; MCLK
.equ ADC10SSEL_3,    0x0018    ; SMCLK

.equ ADC10DIV_0,    0x0000    ; ADC10 Clock Divider Select 0
.equ ADC10DIV_1,    0x0020    ; ADC10 Clock Divider Select 1
.equ ADC10DIV_2,    0x0040    ; ADC10 Clock Divider Select 2
.equ ADC10DIV_3,    0x0060    ; ADC10 Clock Divider Select 3
.equ ADC10DIV_4,    0x0080    ; ADC10 Clock Divider Select 4
.equ ADC10DIV_5,    0x00A0    ; ADC10 Clock Divider Select 5
.equ ADC10DIV_6,    0x00C0    ; ADC10 Clock Divider Select 6
.equ ADC10DIV_7,    0x00E0    ; ADC10 Clock Divider Select 7

.equ SHS_0,      0x0000    ; ADC10SC
.equ SHS_1,      0x0400    ; TA3 OUT1
.equ SHS_2,      0x0800    ; TA3 OUT0
.equ SHS_3,      0x0C00    ; TA3 OUT2

.equ INCH_0,       0x0000    ; Selects Channel 0
.equ INCH_1,       0x1000    ; Selects Channel 1
.equ INCH_2,       0x2000    ; Selects Channel 2
.equ INCH_3,       0x3000    ; Selects Channel 3
.equ INCH_4,       0x4000    ; Selects Channel 4
.equ INCH_5,       0x5000    ; Selects Channel 5
.equ INCH_6,       0x6000    ; Selects Channel 6
.equ INCH_7,       0x7000    ; Selects Channel 7
.equ INCH_8,       0x8000    ; Selects Channel 8
.equ INCH_9,       0x9000    ; Selects Channel 9
.equ INCH_10,      0xA000    ; Selects Channel 10
.equ INCH_11,      0xB000    ; Selects Channel 11
.equ INCH_12,      0xC000    ; Selects Channel 12
.equ INCH_13,      0xD000    ; Selects Channel 13
.equ INCH_14,      0xE000    ; Selects Channel 14
.equ INCH_15,      0xF000    ; Selects Channel 15

; ADC10DTC0
.equ ADC10FETCH,    0x001    ; This bit should normally be reset
.equ ADC10B1,       0x002    ; ADC10 block one
.equ ADC10CT,       0x004    ; ADC10 continuous transfer
.equ ADC10TB,       0x008    ; ADC10 two-block mode
.equ ADC10DISABLE,  0x000    ; ADC10DTC1

;;;
;;; Basic Clock Module
;;;

#define __MSP430_HAS_BC2__    ; Definition to show that Module is available

.equ DCOCTL,       0x0056    ; DCO Clock Frency Control
.equ BCSCTL1,      0x0057    ; Basic Clock System Control 1
.equ BCSCTL2,      0x0058    ; Basic Clock System Control 2
.equ BCSCTL3,      0x0053    ; Basic Clock System Control 3

.equ MOD0,      0x01    ; Modulation Bit 0
.equ MOD1,      0x02    ; Modulation Bit 1
.equ MOD2,      0x04    ; Modulation Bit 2
.equ MOD3,      0x08    ; Modulation Bit 3
.equ MOD4,      0x10    ; Modulation Bit 4
.equ DCO0,      0x20    ; DCO Select Bit 0
.equ DCO1,      0x40    ; DCO Select Bit 1
.equ DCO2,      0x80    ; DCO Select Bit 2

.equ RSEL0,      0x01    ; Range Select Bit 0
.equ RSEL1,      0x02    ; Range Select Bit 1
.equ RSEL2,      0x04    ; Range Select Bit 2
.equ RSEL3,      0x08    ; Range Select Bit 3
.equ DIVA0,      0x10    ; ACLK Divider 0
.equ DIVA1,      0x20    ; ACLK Divider 1
.equ XTS,        0x40    ; LFXTCLK 0:Low Freq. / 1: High Freq.
.equ XT2OFF,     0x80    ; Enable XT2CLK

.equ DIVA_0,     0x00    ; ACLK Divider 0: /1
.equ DIVA_1,     0x10    ; ACLK Divider 1: /2
.equ DIVA_2,     0x20    ; ACLK Divider 2: /4
.equ DIVA_3,     0x30    ; ACLK Divider 3: /8

.equ DIVS0,      0x02    ; SMCLK Divider 0
.equ DIVS1,      0x04    ; SMCLK Divider 1
.equ SELS,       0x08    ; SMCLK Source Select 0:DCOCLK / 1:XT2CLK/LFXTCLK
.equ DIVM0,      0x10    ; MCLK Divider 0
.equ DIVM1,      0x20    ; MCLK Divider 1
.equ SELM0,      0x40    ; MCLK Source Select 0
.equ SELM1,      0x80    ; MCLK Source Select 1

.equ DIVS_0,     0x00    ; SMCLK Divider 0: /1
.equ DIVS_1,     0x02    ; SMCLK Divider 1: /2
.equ DIVS_2,     0x04    ; SMCLK Divider 2: /4
.equ DIVS_3,     0x06    ; SMCLK Divider 3: /8

.equ DIVM_0,     0x00    ; MCLK Divider 0: /1
.equ DIVM_1,     0x10    ; MCLK Divider 1: /2
.equ DIVM_2,     0x20    ; MCLK Divider 2: /4
.equ DIVM_3,     0x30    ; MCLK Divider 3: /8

.equ SELM_0,     0x00    ; MCLK Source Select 0: DCOCLK
.equ SELM_1,     0x40    ; MCLK Source Select 1: DCOCLK
.equ SELM_2,     0x80    ; MCLK Source Select 2: XT2CLK/LFXTCLK
.equ SELM_3,     0xC0    ; MCLK Source Select 3: LFXTCLK

.equ LFXT1OF,    0x01    ; Low/high Frency Oscillator Fault Flag
.equ XT2OF,      0x02    ; High frency oscillator 2 fault flag
.equ XCAP0,      0x04    ; XIN/XOUT Cap 0
.equ XCAP1,      0x08    ; XIN/XOUT Cap 1
.equ LFXT1S0,    0x10    ; Mode 0 for LFXT1 XTS = 0
.equ LFXT1S1,    0x20    ; Mode 1 for LFXT1 XTS = 0
.equ XT2S0,      0x40    ; Mode 0 for XT2
.equ XT2S1,      0x80    ; Mode 1 for XT2

.equ XCAP_0,     0x00    ; XIN/XOUT Cap : 0 pF
.equ XCAP_1,     0x04    ; XIN/XOUT Cap : 6 pF
.equ XCAP_2,     0x08    ; XIN/XOUT Cap : 10 pF
.equ XCAP_3,     0x0C    ; XIN/XOUT Cap : 12.5 pF

.equ LFXT1S_0,   0x00    ; Mode 0 for LFXT1 : Normal operation
.equ LFXT1S_1,   0x10    ; Mode 1 for LFXT1 : Reserved
.equ LFXT1S_2,   0x20    ; Mode 2 for LFXT1 : VLO
.equ LFXT1S_3,   0x30    ; Mode 3 for LFXT1 : Digital input signal

.equ XT2S_0,     0x00    ; Mode 0 for XT2 : 0.4 - 1 MHz
.equ XT2S_1,     0x40    ; Mode 1 for XT2 : 1 - 4 MHz
.equ XT2S_2,     0x80    ; Mode 2 for XT2 : 2 - 16 MHz
.equ XT2S_3,     0xC0    ; Mode 3 for XT2 : Digital input signal

;;;
;;; Comparator A
;;;

#define __MSP430_HAS_CAPLUS__    ; Definition to show that Module is available

.equ CACTL1,     0x0059    ; Comparator A Control 1
.equ CACTL2,     0x005A    ; Comparator A Control 2
.equ CAPD  ,     0x005B    ; Comparator A Port Disable

.equ CAIFG ,     0x01    ; Comp. A Interrupt Flag
.equ CAIE  ,     0x02    ; Comp. A Interrupt Enable
.equ CAIES ,     0x04    ; Comp. A Int. Edge Select: 0:rising / 1:falling
.equ CAON  ,     0x08    ; Comp. A enable
.equ CAREF0,     0x10    ; Comp. A Internal Reference Select 0
.equ CAREF1,     0x20    ; Comp. A Internal Reference Select 1
.equ CARSEL,     0x40    ; Comp. A Internal Reference Enable
.equ CAEX  ,     0x80    ; Comp. A Exchange Inputs

.equ CAREF_0,    0x00    ; Comp. A Int. Ref. Select 0 : Off
.equ CAREF_1,    0x10    ; Comp. A Int. Ref. Select 1 : 0.25*Vcc
.equ CAREF_2,    0x20    ; Comp. A Int. Ref. Select 2 : 0.5*Vcc
.equ CAREF_3,    0x30    ; Comp. A Int. Ref. Select 3 : Vt

.equ CAOUT,      0x01    ; Comp. A Output
.equ CAF  ,      0x02    ; Comp. A Enable Output Filter
.equ P2CA0,      0x04    ; Comp. A +Terminal Multiplexer
.equ P2CA1,      0x08    ; Comp. A -Terminal Multiplexer
.equ P2CA2,      0x10    ; Comp. A -Terminal Multiplexer
.equ P2CA3,      0x20    ; Comp. A -Terminal Multiplexer
.equ P2CA4,      0x40    ; Comp. A +Terminal Multiplexer
.equ CASHORT,    0x80    ; Comp. A Short + and - Terminals

.equ CAPD0,      0x01    ; Comp. A Disable Input Buffer of Port Register .0
.equ CAPD1,      0x02    ; Comp. A Disable Input Buffer of Port Register .1
.equ CAPD2,      0x04    ; Comp. A Disable Input Buffer of Port Register .2
.equ CAPD3,      0x08    ; Comp. A Disable Input Buffer of Port Register .3
.equ CAPD4,      0x10    ; Comp. A Disable Input Buffer of Port Register .4
.equ CAPD5,      0x20    ; Comp. A Disable Input Buffer of Port Register .5
.equ CAPD6,      0x40    ; Comp. A Disable Input Buffer of Port Register .6
.equ CAPD7,      0x80    ; Comp. A Disable Input Buffer of Port Register .7

;;;
;;; Flash Memory
;;;

#define __MSP430_HAS_FLASH2__    ; Definition to show that Module is available

.equ FCTL1,      0x0128    ; FLASH Control 1
.equ FCTL2,      0x012A    ; FLASH Control 2
.equ FCTL3,      0x012C    ; FLASH Control 3

.equ FRKEY,      0x9600    ; Flash key returned by read
.equ FWKEY,      0xA500    ; Flash key for write
.equ FXKEY,      0x3300    ; for use with XOR instruction

.equ ERASE,      0x0002    ; Enable bit for Flash segment erase
.equ MERAS,      0x0004    ; Enable bit for Flash mass erase
.equ WRT,        0x0040    ; Enable bit for Flash write
.equ BLKWRT,     0x0080    ; Enable bit for Flash segment write
.equ SEGWRT,     0x0080    ; old definition -- Enable bit for Flash segment write

.equ FN0   ,     0x0001    ; Divide Flash clock by 1 to 64 using FN0 to FN5 according to:
.equ FN1   ,     0x0002    ;  32*FN5 + 16*FN4 + 8*FN3 + 4*FN2 + 2*FN1 + FN0 + 1
.equ FN2   ,     0x0004
.equ FN3   ,     0x0008
.equ FN4   ,     0x0010
.equ FN5   ,     0x0020
.equ FSSEL0,     0x0040    ; Flash clock select 0 -- to distinguish from USART SSELx
.equ FSSEL1,     0x0080    ; Flash clock select 1

.equ FSSEL_0,    0x0000    ; Flash clock select: 0 - ACLK
.equ FSSEL_1,    0x0040    ; Flash clock select: 1 - MCLK
.equ FSSEL_2,    0x0080    ; Flash clock select: 2 - SMCLK
.equ FSSEL_3,    0x00C0    ; Flash clock select: 3 - SMCLK

.equ BUSY   ,    0x0001    ; Flash busy: 1
.equ KEYV   ,    0x0002    ; Flash Key violation flag
.equ ACCVIFG,    0x0004    ; Flash Access violation flag
.equ WAIT   ,    0x0008    ; Wait flag for segment write
.equ LOCK   ,    0x0010    ; Lock bit: 1 - Flash is locked read only
.equ EMEX   ,    0x0020    ; Flash Emergency Exit
.equ LOCKA  ,    0x0040    ; Segment A Lock bit: read = 1 - Segment is locked read only
.equ FAIL   ,    0x0080    ; Last Program or Erase failed

;;;
;;; DIGITAL I/O Port1/2 Pull up / Pull down Resistors
;;;

#define __MSP430_HAS_PORT1_R__    ; Definition to show that Module is available
#define __MSP430_HAS_PORT2_R__    ; Definition to show that Module is available

.equ P1IN  ,     0x0020    ; Port 1 Input
.equ P1OUT ,     0x0021    ; Port 1 Output
.equ P1DIR ,     0x0022    ; Port 1 Direction
.equ P1IFG ,     0x0023    ; Port 1 Interrupt Flag
.equ P1IES ,     0x0024    ; Port 1 Interrupt Edge Select
.equ P1IE  ,     0x0025    ; Port 1 Interrupt Enable
.equ P1SEL ,     0x0026    ; Port 1 Selection
.equ P1SEL2,     0x0041    ; Port 1 Selection 2
.equ P1REN ,     0x0027    ; Port 1 Resistor Enable

.equ P2IN  ,     0x0028    ; Port 2 Input
.equ P2OUT ,     0x0029    ; Port 2 Output
.equ P2DIR ,     0x002A    ; Port 2 Direction
.equ P2IFG ,     0x002B    ; Port 2 Interrupt Flag
.equ P2IES ,     0x002C    ; Port 2 Interrupt Edge Select
.equ P2IE  ,     0x002D    ; Port 2 Interrupt Enable
.equ P2SEL ,     0x002E    ; Port 2 Selection
.equ P2SEL2,     0x0042    ; Port 2 Selection 2
.equ P2REN ,     0x002F    ; Port 2 Resistor Enable

;;;
;;; DIGITAL I/O Port3 Pull up / Pull down Resistors
;;;

#define __MSP430_HAS_PORT3_R__    ; Definition to show that Module is available

.equ P3IN  ,    0x0018    ; Port 3 Input
.equ P3OUT ,    0x0019    ; Port 3 Output
.equ P3DIR ,    0x001A    ; Port 3 Direction
.equ P3SEL ,    0x001B    ; Port 3 Selection
.equ P3SEL2,    0x0043    ; Port 3 Selection 2
.equ P3REN ,    0x0010    ; Port 3 Resistor Enable

;;;
;;; Timer0_A3
;;;

#define __MSP430_HAS_TA3__    ; Definition to show that Module is available

.equ TA0IV   ,  0x012E    ; Timer0_A3 Interrupt Vector Word
.equ TA0CTL  ,  0x0160    ; Timer0_A3 Control
.equ TA0CCTL0,  0x0162    ; Timer0_A3 Capture/Compare Control 0
.equ TA0CCTL1,  0x0164    ; Timer0_A3 Capture/Compare Control 1
.equ TA0CCTL2,  0x0166    ; Timer0_A3 Capture/Compare Control 2
.equ TA0R    ,  0x0170    ; Timer0_A3
.equ TA0CCR0 ,  0x0172    ; Timer0_A3 Capture/Compare 0
.equ TA0CCR1 ,  0x0174    ; Timer0_A3 Capture/Compare 1
.equ TA0CCR2 ,  0x0176    ; Timer0_A3 Capture/Compare 2

; Alternate register names
.equ TAIV   ,   TA0IV    ; Timer A Interrupt Vector Word
.equ TACTL  ,   TA0CTL    ; Timer A Control
.equ TACCTL0,   TA0CCTL0  ; Timer A Capture/Compare Control 0
.equ TACCTL1,   TA0CCTL1  ; Timer A Capture/Compare Control 1
.equ TACCTL2,   TA0CCTL2  ; Timer A Capture/Compare Control 2
.equ TAR    ,   TA0R      ; Timer A
.equ TACCR0 ,   TA0CCR0   ; Timer A Capture/Compare 0
.equ TACCR1 ,   TA0CCR1   ; Timer A Capture/Compare 1
.equ TACCR2 ,   TA0CCR2   ; Timer A Capture/Compare 2

; Alternate register names 2
.equ CCTL0,       TACCTL0    ; Timer A Capture/Compare Control 0
.equ CCTL1,       TACCTL1    ; Timer A Capture/Compare Control 1
.equ CCTL2,       TACCTL2    ; Timer A Capture/Compare Control 2
.equ CCR0 ,       TACCR0     ; Timer A Capture/Compare 0
.equ CCR1 ,       TACCR1     ; Timer A Capture/Compare 1
.equ CCR2 ,       TACCR2     ; Timer A Capture/Compare 2

.equ TASSEL1,      0x0200    ; Timer A clock source select 0
.equ TASSEL0,      0x0100    ; Timer A clock source select 1
.equ ID1    ,      0x0080    ; Timer A clock input divider 1
.equ ID0    ,      0x0040    ; Timer A clock input divider 0
.equ MC1    ,      0x0020    ; Timer A mode control 1
.equ MC0    ,      0x0010    ; Timer A mode control 0
.equ TACLR  ,      0x0004    ; Timer A counter clear
.equ TAIE   ,      0x0002    ; Timer A counter interrupt enable
.equ TAIFG  ,      0x0001    ; Timer A counter interrupt flag

.equ MC_0    ,  0x0000    ; Timer A mode control: 0 - Stop
.equ MC_1    ,  0x0010    ; Timer A mode control: 1 - Up to CCR0
.equ MC_2    ,  0x0020    ; Timer A mode control: 2 - Continous up
.equ MC_3    ,  0x0030    ; Timer A mode control: 3 - Up/Down
.equ ID_0    ,  0x0000    ; Timer A input divider: 0 - /1
.equ ID_1    ,  0x0040    ; Timer A input divider: 1 - /2
.equ ID_2    ,  0x0080    ; Timer A input divider: 2 - /4
.equ ID_3    ,  0x00C0    ; Timer A input divider: 3 - /8
.equ TASSEL_0,  0x0000    ; Timer A clock source select: 0 - TACLK
.equ TASSEL_1,  0x0100    ; Timer A clock source select: 1 - ACLK
.equ TASSEL_2,  0x0200    ; Timer A clock source select: 2 - SMCLK
.equ TASSEL_3,  0x0300    ; Timer A clock source select: 3 - INCLK

.equ CM1    ,   0x8000    ; Capture mode 1
.equ CM0    ,   0x4000    ; Capture mode 0
.equ CCIS1  ,   0x2000    ; Capture input select 1
.equ CCIS0  ,   0x1000    ; Capture input select 0
.equ SCS    ,   0x0800    ; Capture sychronize
.equ SCCI   ,   0x0400    ; Latched capture signal read
.equ CAP    ,   0x0100    ; Capture mode: 1 /Compare mode : 0
.equ OUTMOD2,   0x0080    ; Output mode 2
.equ OUTMOD1,   0x0040    ; Output mode 1
.equ OUTMOD0,   0x0020    ; Output mode 0
.equ CCIE   ,   0x0010    ; Capture/compare interrupt enable
.equ CCI    ,   0x0008    ; Capture input signal read
.equ OUT    ,   0x0004    ; PWM Output signal if output mode 0
.equ COV    ,   0x0002    ; Capture/compare overflow flag
.equ CCIFG  ,   0x0001    ; Capture/compare interrupt flag

.equ OUTMOD_0,    0x0000    ; PWM output mode: 0 - output only
.equ OUTMOD_1,    0x0020    ; PWM output mode: 1 - set
.equ OUTMOD_2,    0x0040    ; PWM output mode: 2 - PWM toggle/reset
.equ OUTMOD_3,    0x0060    ; PWM output mode: 3 - PWM set/reset
.equ OUTMOD_4,    0x0080    ; PWM output mode: 4 - toggle
.equ OUTMOD_5,    0x00A0    ; PWM output mode: 5 - Reset
.equ OUTMOD_6,    0x00C0    ; PWM output mode: 6 - PWM toggle/set
.equ OUTMOD_7,    0x00E0    ; PWM output mode: 7 - PWM reset/set
.equ CCIS_0,      0x0000    ; Capture input select: 0 - CCIxA
.equ CCIS_1,      0x1000    ; Capture input select: 1 - CCIxB
.equ CCIS_2,      0x2000    ; Capture input select: 2 - GND
.equ CCIS_3,      0x3000    ; Capture input select: 3 - Vcc
.equ CM_0,        0x0000    ; Capture mode: 0 - disabled
.equ CM_1,        0x4000    ; Capture mode: 1 - pos. edge
.equ CM_2,        0x8000    ; Capture mode: 1 - neg. edge
.equ CM_3,        0xC000    ; Capture mode: 1 - both edges

; T0_A3IV Definitions
.equ TA0IV_NONE,      0x0000    ; No Interrupt pending
.equ TA0IV_TACCR1,    0x0002    ; TA0CCR1_CCIFG
.equ TA0IV_TACCR2,    0x0004    ; TA0CCR2_CCIFG
.equ TA0IV_6,         0x0006    ; Reserved
.equ TA0IV_8,         0x0008    ; Reserved
.equ TA0IV_TAIFG,     0x000A    ; TA0IFG

;;;
;;; Timer1_A3
;;;

#define __MSP430_HAS_T1A3__    ; Definition to show that Module is available

.equ TA1IV,       0x011E    ; Timer1_A3 Interrupt Vector Word
.equ TA1CTL,      0x0180    ; Timer1_A3 Control
.equ TA1CCTL0,    0x0182    ; Timer1_A3 Capture/Compare Control 0
.equ TA1CCTL1,    0x0184    ; Timer1_A3 Capture/Compare Control 1
.equ TA1CCTL2,    0x0186    ; Timer1_A3 Capture/Compare Control 2
.equ TA1R,        0x0190    ; Timer1_A3
.equ TA1CCR0,     0x0192    ; Timer1_A3 Capture/Compare 0
.equ TA1CCR1,     0x0194    ; Timer1_A3 Capture/Compare 1
.equ TA1CCR2,     0x0196    ; Timer1_A3 Capture/Compare 2

; Bits are already defined within the Timer0_Ax

; T1_A3IV Definitions
.equ TA1IV_NONE,    0x0000    ; No Interrupt pending
.equ TA1IV_TACCR1,  0x0002    ; TA1CCR1_CCIFG
.equ TA1IV_TACCR2,  0x0004    ; TA1CCR2_CCIFG
.equ TA1IV_TAIFG,   0x000A    ; TA1IFG

;;;
;;; USCI
;;;

#define __MSP430_HAS_USCI__    ; Definition to show that Module is available

.equ UCA0CTL0,    0x0060    ; USCI A0 Control Register 0
.equ UCA0CTL1,    0x0061    ; USCI A0 Control Register 1
.equ UCA0BR0,     0x0062    ; USCI A0 Baud Rate 0
.equ UCA0BR1,     0x0063    ; USCI A0 Baud Rate 1
.equ UCA0MCTL,    0x0064    ; USCI A0 Modulation Control
.equ UCA0STAT,    0x0065    ; USCI A0 Status Register
.equ UCA0RXBUF,   0x0066    ; USCI A0 Receive Buffer
.equ UCA0TXBUF,   0x0067    ; USCI A0 Transmit Buffer
.equ UCA0ABCTL,   0x005D    ; USCI A0 LIN Control
.equ UCA0IRTCTL,  0x005E    ; USCI A0 IrDA Transmit Control
.equ UCA0IRRCTL,  0x005F    ; USCI A0 IrDA Receive Control


.equ UCB0CTL0,    0x0068    ; USCI B0 Control Register 0
.equ UCB0CTL1,    0x0069    ; USCI B0 Control Register 1
.equ UCB0BR0,     0x006A    ; USCI B0 Baud Rate 0
.equ UCB0BR1,     0x006B    ; USCI B0 Baud Rate 1
.equ UCB0I2CIE,   0x006C    ; USCI B0 I2C Interrupt Enable Register
.equ UCB0STAT,    0x006D    ; USCI B0 Status Register
.equ UCB0RXBUF,   0x006E    ; USCI B0 Receive Buffer
.equ UCB0TXBUF,   0x006F    ; USCI B0 Transmit Buffer
.equ UCB0I2COA,   0x0118    ; USCI B0 I2C Own Address
.equ UCB0I2CSA,   0x011A    ; USCI B0 I2C Slave Address


; UART-Mode Bits
.equ UCPEN,       0x80    ; Async. Mode: Parity enable
.equ UCPAR,       0x40    ; Async. Mode: Parity    0:odd / 1:even
.equ UCMSB,       0x20    ; Async. Mode: MSB first  0:LSB / 1:MSB
.equ UC7BIT,      0x10    ; Async. Mode: Data Bits  0:8-bits / 1:7-bits
.equ UCSPB,       0x08    ; Async. Mode: Stop Bits  0:one / 1: two
.equ UCMODE1,     0x04    ; Async. Mode: USCI Mode 1
.equ UCMODE0,     0x02    ; Async. Mode: USCI Mode 0
.equ UCSYNC,      0x01    ; Sync-Mode  0:UART-Mode / 1:SPI-Mode

; SPI-Mode Bits
.equ UCCKPH,      0x80    ; Sync. Mode: Clock Phase
.equ UCCKPL,      0x40    ; Sync. Mode: Clock Polarity
.equ UCMST,       0x08    ; Sync. Mode: Master Select

; I2C-Mode Bits
.equ UCA10,       0x80    ; 10-bit Address Mode
.equ UCSLA10,     0x40    ; 10-bit Slave Address Mode
.equ UCMM,        0x20    ; Multi-Master Environment
;                 0x10    ; reserved
.equ UCMODE_0,    0x00    ; Sync. Mode: USCI Mode: 0
.equ UCMODE_1,    0x02    ; Sync. Mode: USCI Mode: 1
.equ UCMODE_2,    0x04    ; Sync. Mode: USCI Mode: 2
.equ UCMODE_3,    0x06    ; Sync. Mode: USCI Mode: 3

; UART-Mode Bits
.equ UCSSEL1,      0x80    ; USCI 0 Clock Source Select 1
.equ UCSSEL0,      0x40    ; USCI 0 Clock Source Select 0
.equ UCRXEIE,      0x20    ; RX Error interrupt enable
.equ UCBRKIE,      0x10    ; Break interrupt enable
.equ UCDORM,       0x08    ; Dormant Sleep Mode
.equ UCTXADDR,     0x04    ; Send next Data as Address
.equ UCTXBRK,      0x02    ; Send next Data as Break
.equ UCSWRST,      0x01    ; USCI Software Reset

; SPI-Mode Bits
;                 0x20    ; reserved
;                 0x10    ; reserved
;                 0x08    ; reserved
;                 0x04    ; reserved
;                 0x02    ; reserved

; I2C-Mode Bits
;                 0x20    ; reserved
.equ UCTR,        0x10    ; Transmit/Receive Select/Flag
.equ UCTXNACK,    0x08    ; Transmit NACK
.equ UCTXSTP,     0x04    ; Transmit STOP
.equ UCTXSTT,     0x02    ; Transmit START
.equ UCSSEL_0,    0x00    ; USCI 0 Clock Source: 0
.equ UCSSEL_1,    0x40    ; USCI 0 Clock Source: 1
.equ UCSSEL_2,    0x80    ; USCI 0 Clock Source: 2
.equ UCSSEL_3,    0xC0    ; USCI 0 Clock Source: 3

.equ UCBRF3,      0x80    ; USCI First Stage Modulation Select 3
.equ UCBRF2,      0x40    ; USCI First Stage Modulation Select 2
.equ UCBRF1,      0x20    ; USCI First Stage Modulation Select 1
.equ UCBRF0,      0x10    ; USCI First Stage Modulation Select 0
.equ UCBRS2,      0x08    ; USCI Second Stage Modulation Select 2
.equ UCBRS1,      0x04    ; USCI Second Stage Modulation Select 1
.equ UCBRS0,      0x02    ; USCI Second Stage Modulation Select 0
.equ UCOS16,      0x01    ; USCI 16-times Oversampling enable

.equ UCBRF_0,      0x00    ; USCI First Stage Modulation: 0
.equ UCBRF_1,      0x10    ; USCI First Stage Modulation: 1
.equ UCBRF_2,      0x20    ; USCI First Stage Modulation: 2
.equ UCBRF_3,      0x30    ; USCI First Stage Modulation: 3
.equ UCBRF_4,      0x40    ; USCI First Stage Modulation: 4
.equ UCBRF_5,      0x50    ; USCI First Stage Modulation: 5
.equ UCBRF_6,      0x60    ; USCI First Stage Modulation: 6
.equ UCBRF_7,      0x70    ; USCI First Stage Modulation: 7
.equ UCBRF_8,      0x80    ; USCI First Stage Modulation: 8
.equ UCBRF_9,      0x90    ; USCI First Stage Modulation: 9
.equ UCBRF_10,     0xA0    ; USCI First Stage Modulation: A
.equ UCBRF_11,     0xB0    ; USCI First Stage Modulation: B
.equ UCBRF_12,     0xC0    ; USCI First Stage Modulation: C
.equ UCBRF_13,     0xD0    ; USCI First Stage Modulation: D
.equ UCBRF_14,     0xE0    ; USCI First Stage Modulation: E
.equ UCBRF_15,     0xF0    ; USCI First Stage Modulation: F

.equ UCBRS_0,      0x00    ; USCI Second Stage Modulation: 0
.equ UCBRS_1,      0x02    ; USCI Second Stage Modulation: 1
.equ UCBRS_2,      0x04    ; USCI Second Stage Modulation: 2
.equ UCBRS_3,      0x06    ; USCI Second Stage Modulation: 3
.equ UCBRS_4,      0x08    ; USCI Second Stage Modulation: 4
.equ UCBRS_5,      0x0A    ; USCI Second Stage Modulation: 5
.equ UCBRS_6,      0x0C    ; USCI Second Stage Modulation: 6
.equ UCBRS_7,      0x0E    ; USCI Second Stage Modulation: 7

.equ UCLISTEN,     0x80    ; USCI Listen mode
.equ UCFE,         0x40    ; USCI Frame Error Flag
.equ UCOE,         0x20    ; USCI Overrun Error Flag
.equ UCPE,         0x10    ; USCI Parity Error Flag
.equ UCBRK,        0x08    ; USCI Break received
.equ UCRXERR,      0x04    ; USCI RX Error Flag
.equ UCADDR,       0x02    ; USCI Address received Flag
.equ UCBUSY,       0x01    ; USCI Busy Flag
.equ UCIDLE,       0x02    ; USCI Idle line detected Flag

;                  0x80    ; reserved
;                  0x40    ; reserved
;                  0x20    ; reserved
;                  0x10    ; reserved
.equ UCNACKIE,     0x08    ; NACK Condition interrupt enable
.equ UCSTPIE,      0x04    ; STOP Condition interrupt enable
.equ UCSTTIE,      0x02    ; START Condition interrupt enable
.equ UCALIE,       0x01    ; Arbitration Lost interrupt enable

.equ UCSCLLOW,     0x40    ; SCL low
.equ UCGC,         0x20    ; General Call address received Flag
.equ UCBBUSY,      0x10    ; Bus Busy Flag
.equ UCNACKIFG,    0x08    ; NAK Condition interrupt Flag
.equ UCSTPIFG,     0x04    ; STOP Condition interrupt Flag
.equ UCSTTIFG,     0x02    ; START Condition interrupt Flag
.equ UCALIFG,      0x01    ; Arbitration Lost interrupt Flag

.equ UCIRTXPL5,    0x80    ; IRDA Transmit Pulse Length 5
.equ UCIRTXPL4,    0x40    ; IRDA Transmit Pulse Length 4
.equ UCIRTXPL3,    0x20    ; IRDA Transmit Pulse Length 3
.equ UCIRTXPL2,    0x10    ; IRDA Transmit Pulse Length 2
.equ UCIRTXPL1,    0x08    ; IRDA Transmit Pulse Length 1
.equ UCIRTXPL0,    0x04    ; IRDA Transmit Pulse Length 0
.equ UCIRTXCLK,    0x02    ; IRDA Transmit Pulse Clock Select
.equ UCIREN,       0x01    ; IRDA Encoder/Decoder enable

.equ UCIRRXFL5,    0x80    ; IRDA Receive Filter Length 5
.equ UCIRRXFL4,    0x40    ; IRDA Receive Filter Length 4
.equ UCIRRXFL3,    0x20    ; IRDA Receive Filter Length 3
.equ UCIRRXFL2,    0x10    ; IRDA Receive Filter Length 2
.equ UCIRRXFL1,    0x08    ; IRDA Receive Filter Length 1
.equ UCIRRXFL0,    0x04    ; IRDA Receive Filter Length 0
.equ UCIRRXPL,     0x02    ; IRDA Receive Input Polarity
.equ UCIRRXFE,     0x01    ; IRDA Receive Filter enable

;                 0x80    ; reserved
;                 0x40    ; reserved
.equ UCDELIM1,    0x20    ; Break Sync Delimiter 1
.equ UCDELIM0,    0x10    ; Break Sync Delimiter 0
.equ UCSTOE,      0x08    ; Sync-Field Timeout error
.equ UCBTOE,      0x04    ; Break Timeout error
;                 0x02    ; reserved
.equ UCABDEN,     0x01    ; Auto Baud Rate detect enable

.equ UCGCEN,     0x8000    ; I2C General Call enable
.equ UCOA9,      0x0200    ; I2C Own Address 9
.equ UCOA8,      0x0100    ; I2C Own Address 8
.equ UCOA7,      0x0080    ; I2C Own Address 7
.equ UCOA6,      0x0040    ; I2C Own Address 6
.equ UCOA5,      0x0020    ; I2C Own Address 5
.equ UCOA4,      0x0010    ; I2C Own Address 4
.equ UCOA3,      0x0008    ; I2C Own Address 3
.equ UCOA2,      0x0004    ; I2C Own Address 2
.equ UCOA1,      0x0002    ; I2C Own Address 1
.equ UCOA0,      0x0001    ; I2C Own Address 0

.equ UCSA9,      0x0200    ; I2C Slave Address 9
.equ UCSA8,      0x0100    ; I2C Slave Address 8
.equ UCSA7,      0x0080    ; I2C Slave Address 7
.equ UCSA6,      0x0040    ; I2C Slave Address 6
.equ UCSA5,      0x0020    ; I2C Slave Address 5
.equ UCSA4,      0x0010    ; I2C Slave Address 4
.equ UCSA3,      0x0008    ; I2C Slave Address 3
.equ UCSA2,      0x0004    ; I2C Slave Address 2
.equ UCSA1,      0x0002    ; I2C Slave Address 1
.equ UCSA0,      0x0001    ; I2C Slave Address 0

;;;
;;; WATCHDOG TIMER
;;;

#define __MSP430_HAS_WDT__    ; Definition to show that Module is available

.equ WDTCTL,      0x0120    ; Watchdog Timer Control

; The bit names have been prefixed with "WDT"
.equ WDTIS0,      0x0001
.equ WDTIS1,      0x0002
.equ WDTSSEL,     0x0004
.equ WDTCNTCL,    0x0008
.equ WDTTMSEL,    0x0010
.equ WDTNMI,      0x0020
.equ WDTNMIES,    0x0040
.equ WDTHOLD,     0x0080

.equ WDTPW,       0x5A00

; WDT-interval times [1ms] coded with Bits 0-2
; WDT is clocked by fSMCLK assumed 1MHz
.equ WDT_MDLY_32,      WDTPW+WDTTMSEL+WDTCNTCL               ; 32ms interval default
.equ WDT_MDLY_8,       WDTPW+WDTTMSEL+WDTCNTCL+WDTIS0        ; 8ms"
.equ WDT_MDLY_0_5,     WDTPW+WDTTMSEL+WDTCNTCL+WDTIS1        ; 0.5ms"
.equ WDT_MDLY_0_064,   WDTPW+WDTTMSEL+WDTCNTCL+WDTIS1+WDTIS0 ; 0.064ms"

; WDT is clocked by fACLK assumed 32KHz
.equ WDT_ADLY_1000,    WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL         ; 1000ms"
.equ WDT_ADLY_250,     WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS0  ; 250ms"
.equ WDT_ADLY_16,      WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1  ; 16ms"
.equ WDT_ADLY_1_9,     WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1+WDTIS0  ; 1.9ms    "

; Watchdog mode -> reset after expired time
; WDT is clocked by fSMCLK assumed 1MHz
.equ WDT_MRST_32,      WDTPW+WDTCNTCL                ; 32ms interval default
.equ WDT_MRST_8,       WDTPW+WDTCNTCL+WDTIS0         ; 8ms"
.equ WDT_MRST_0_5,     WDTPW+WDTCNTCL+WDTIS1         ; 0.5ms"
.equ WDT_MRST_0_064,   WDTPW+WDTCNTCL+WDTIS1+WDTIS0  ; 0.064ms"

; WDT is clocked by fACLK assumed 32KHz
.equ WDT_ARST_1000,    WDTPW+WDTCNTCL+WDTSSEL                ; 1000ms"
.equ WDT_ARST_250,     WDTPW+WDTCNTCL+WDTSSEL+WDTIS0         ; 250ms"
.equ WDT_ARST_16,      WDTPW+WDTCNTCL+WDTSSEL+WDTIS1         ; 16ms"
.equ WDT_ARST_1_9,     WDTPW+WDTCNTCL+WDTSSEL+WDTIS1+WDTIS0  ; 1.9ms"

; INTERRUPT CONTROL
; These two bits are defined in the Special Function Registers
; WDTIE   0x01
; WDTIFG  0x01

;;;
;;; Calibration Data in Info Mem
;;;

.equ CALDCO_16MHZ,    0x10F8    ; DCOCTL  Calibration Data for 16MHz
.equ CALBC1_16MHZ,    0x10F9    ; BCSCTL1 Calibration Data for 16MHz
.equ CALDCO_12MHZ,    0x10FA    ; DCOCTL  Calibration Data for 12MHz
.equ CALBC1_12MHZ,    0x10FB    ; BCSCTL1 Calibration Data for 12MHz
.equ CALDCO_8MHZ,     0x10FC    ; DCOCTL  Calibration Data for 8MHz
.equ CALBC1_8MHZ,     0x10FD    ; BCSCTL1 Calibration Data for 8MHz
.equ CALDCO_1MHZ,     0x10FE    ; DCOCTL  Calibration Data for 1MHz
.equ CALBC1_1MHZ,     0x10FF    ; BCSCTL1 Calibration Data for 1MHz


;;;
;;; Interrupt Vectors offset from 0xFFE0
;;;

.equ PORT1_VECTOR,        0x0004    ; 0xFFE4 Port 1
.equ PORT2_VECTOR,        0x0006    ; 0xFFE6 Port 2
.equ ADC10_VECTOR,        0x000A    ; 0xFFEA ADC10
.equ USCIAB0TX_VECTOR,    0x000C    ; 0xFFEC USCI A0/B0 Transmit
.equ USCIAB0RX_VECTOR,    0x000E    ; 0xFFEE USCI A0/B0 Receive
.equ TIMER0_A1_VECTOR,    0x0010    ; 0xFFF0 Timer0A CC1, TA0
.equ TIMER0_A0_VECTOR,    0x0012    ; 0xFFF2 Timer0_A CC0
.equ WDT_VECTOR,          0x0014    ; 0xFFF4 Watchdog Timer
.equ COMPARATORA_VECTOR,  0x0016    ; 0xFFF6 Comparator A
.equ TIMER1_A1_VECTOR,    0x0018    ; 0xFFF8 Timer1_A CC1-4, TA1
.equ TIMER1_A0_VECTOR,    0x001A    ; 0xFFFA Timer1_A CC0
.equ NMI_VECTOR,          0x001C    ; 0xFFFC Non-maskable
.equ RESET_VECTOR,        0x001E    ; 0xFFFE Reset [Highest Priority]

;;;
;;; End of Modules
;;;
