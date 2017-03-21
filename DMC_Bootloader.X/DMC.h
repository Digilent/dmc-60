/************************************************************************/
/*									*/
/*	DMC.h -- Board Level Declarations for DMC1 / DMC2               */
/*									*/
/************************************************************************/
/*	Author: Michael T. Alexander                                    */
/*	Copyright 2016, Digilent Inc.					*/
/************************************************************************/
/*  File Description:							*/
/*									*/
/*  This file contains board level declarations for the DMC1 and DMC2.	*/
/*  This includes pinout of the onboard peripherals and various other	*/
/*  symbols related to the hardware configuration of the board.         */
/*									*/
/************************************************************************/
/*  Revision History:							*/
/*      								*/
/*  09/13/2016(MichaelA): created					*/
/*                                                                      */
/************************************************************************/

#if !defined(_DMC_INC)
#define	     _DMC_INC

/* ------------------------------------------------------------ */
/*		Miscellaneous Declarations			*/
/* ------------------------------------------------------------ */

/* LEDs on the circuit board.
*/
#define trsLedR1                        TRISB
#define prtLedR1                        PORTB
#define latLedR1                        LATB
#define bnLedR1                         4

#define trsLedR2                        TRISC
#define prtLedR2                        PORTC
#define latLedR2                        LATC
#define bnLedR2                         1

#define trsLedR3                        TRISC
#define prtLedR3                        PORTC
#define latLedR3                        LATC
#define bnLedR3                         2

#define trsLedR4                        TRISC
#define prtLedR4                        PORTC
#define latLedR4                        LATC
#define bnLedR4                         3

#define trsLedG1                        TRISC
#define prtLedG1                        PORTC
#define latLedG1                        LATC
#define bnLedG1                         4

#define trsLedG2                        TRISC
#define prtLedG2                        PORTC
#define latLedG2                        LATC
#define bnLedG2                         5

#define trsLedG3                        TRISC
#define prtLedG3                        PORTC
#define latLedG3                        LATC
#define bnLedG3                         6

#define trsLedG4                        TRISC
#define prtLedG4                        PORTC
#define latLedG4                        LATC
#define bnLedG4                         7

#define trsLedB1                        TRISA
#define prtLedB1                        PORTA
#define latLedB1                        LATA
#define bnLedB1                         10

#define trsLedB2                        TRISA
#define prtLedB2                        PORTA
#define latLedB2                        LATA
#define bnLedB2                         7

#define trsLedB3                        TRISA
#define prtLedB3                        PORTA
#define latLedB3                        LATA
#define bnLedB3                         8

#define trsLedB4                        TRISA
#define prtLedB4                        PORTA
#define latLedB4                        LATA
#define bnLedB4                         9

#define trsLedBC                        TRISB
#define prtLedBC                        PORTB
#define latLedBC                        LATB
#define bnLedBC                         11


#define trsPwmR                         TRISB
#define prtPwmR                         PORTB
#define latPwmR                         LATB
#define ppsPwmR                         RPOR1bits.RP37R
#define bnPwmR                          5

#define trsPwmG                         TRISB
#define prtPwmG                         PORTB
#define latPwmG                         LATB
#define ppsPwmG                         RPOR2bits.RP38R
#define bnPwmG                          6

#define trsPwmB                         TRISB
#define prtPwmB                         PORTB
#define latPwmB                         LATB
#define ppsPwmB                         RPOR2bits.RP39R
#define bnPwmB                          7

/* Buttons on the circuit board.
*/
#define trsBtnCal                       TRISA
#define prtBtnCal                       PORTA
#define latBtnCal                       LATA
#define ppsBtnCal                       0b0010100 // RP20, Port A Pin 4
#define bnBtnCal                        4

/* ------------------------------------------------------------ */
/*		ICSP Header Declarations			*/
/* ------------------------------------------------------------ */

#define trsIcspDat                      TRISB
#define prtIcspDat                      PORTB
#define latIcspDat                      LATB
#define bnIcspDat                       3

/* ------------------------------------------------------------ */
/*		PWM PIN Declarations                    	*/
/* ------------------------------------------------------------ */

#define trsPwm1L                        TRISB
#define prtPwm1L                        PORTB
#define latPwm1L                        LATB
#define bnPwm1L                         15

#define trsPwm1H                        TRISB
#define prtPwm1H                        PORTB
#define latPwm1H                        LATB
#define bnPwm1H                         14

#define trsPwm2L                        TRISB
#define prtPwm2L                        PORTB
#define latPwm2L                        LATB
#define bnPwm2L                         13

#define trsPwm2H                        TRISB
#define prtPwm2H                        PORTB
#define latPwm2H                        LATB
#define bnPwm2H                         12

/* ------------------------------------------------------------ */
/*		Servo Input Declarations			*/
/* ------------------------------------------------------------ */

#define trsServoIn                      TRISB
#define prtServoIn                      PORTB
#define latServoIn                      LATB
#define ppsServoIn                      0b0101010 // RP42, Port B Pin 10
#define bnServoIn                       10

/* ------------------------------------------------------------ */
/*		CAN Interface Declarations			*/
/* ------------------------------------------------------------ */

#define trsCanRx                        TRISB
#define prtCanRx                        PORTB
#define latCanRx                        LATB
#define ppsCanRx                        0b0101010 // RP42, Port B Pin 10
#define bnCanRx                         10

#define trsCanTx                        TRISB
#define prtCanTx                        PORTB
#define latCanTx                        LATB
#define ppsCanTx                        RPOR3bits.RP40R
#define bnCanTx                         8

/* ------------------------------------------------------------ */
/*		Analog Input Declarations			*/
/* ------------------------------------------------------------ */

#define adcVmon1                        0b00000
#define trsVmon1                        TRISA
#define prtVmon1                        PORTA
#define latVmon1                        LATA
#define ansVmon1                        ANSELA
#define bnVmon1                         0

#define adcImon1                        0b00001
#define trsImon1                        TRISA
#define prtImon1                        PORTA
#define latImon1                        LATA
#define ansImon1                        ANSELA
#define bnImon1                         1

#define adcTmon1                        0b00010
#define trsTmon1                        TRISB
#define prtTmon1                        PORTB
#define latTmon1                        LATB
#define ansTmon1                        ANSELB
#define bnTmon1                         0

#define adcUserAN1                      0b00110
#define trsUserAN1                      TRISC
#define prtUserAN1                      PORTC
#define latUserAN1                      LATC
#define ansUserAN1                      ANSELC
#define bnUserAN1                       0

/* ------------------------------------------------------------ */
/*		Current Amplifier Declarations			*/
/* ------------------------------------------------------------ */

#define trsGainSel                      TRISA
#define prtGainSel                      PORTA
#define latGainSel                      LATA
#define bnGainSel                       3

#endif

/************************************************************************/
