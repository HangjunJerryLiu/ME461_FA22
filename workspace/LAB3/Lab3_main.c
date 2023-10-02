//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
//#include "song.h" //commented out to play song
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define FUDGEFACTORNOTE 1
#define C4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/261.63))
#define D4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/293.66))
#define E4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/329.63))
#define F4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/349.23))
#define G4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/392.00))
#define A4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/440.00))
#define B4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/493.88))
#define C5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/523.25))
#define D5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/587.33))
#define E5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/659.25))
#define F5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/698.46))
#define G5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/783.99))
#define A5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/880.00))
#define B5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/987.77))
#define F4SHARPNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/369.99))
#define G4SHARPNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/415.3))
#define A4FLATNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/415.3))
#define C5SHARPNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/554.37))
#define A5FLATNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/830.61))

#define SONG_LENGTH2 81
uint16_t songarray2[SONG_LENGTH2] = {
C4NOTE,
C4NOTE,
C4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
F4NOTE,
F4NOTE,
F4NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
F4NOTE,
F4NOTE,
F4NOTE,
F4NOTE,
F4NOTE,
F4NOTE,
D4NOTE,
D4NOTE,
D4NOTE,
G4NOTE,
G4NOTE,
G4NOTE,
G4NOTE,
G4NOTE,
G4NOTE,
G4NOTE,
G4NOTE,
G4NOTE,
G4NOTE,
G4NOTE,
G4NOTE,
F4NOTE,
F4NOTE,
F4NOTE,
G4NOTE,
G4NOTE,
G4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
D5NOTE,
E5NOTE,
E5NOTE,
E5NOTE,
F5NOTE,
F5NOTE,
F5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,
C5NOTE,

};


#define OFFNOTE 1

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200



// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

uint16_t updown = 1; // up count mode (1) or down count mode (0) indicator
float controleffort = -10;
uint16_t UD = 1;          //  up count mode (1) or down count mode (0) indicator for the controleffort

uint32_t songindex = 0;    // current song index

void setPWM2A (float controleffort)        // relate the input "controleffort" to the duty cycle
{
    if (controleffort > 10) {
        controleffort = 10;
    }
    if (controleffort < -10) {
        controleffort = -10;
    }
    EPwm2Regs.CMPA.bit.CMPA = 125.0*controleffort + 1250.0; // make -10 to 10 proportional to 0 to 2500, values of CMPA and CMPB should have opposite signs for the motor to spin in the same direction
}


void setPWM2B (float controleffort)        // relate the input "controleffort" to the duty cycle
{
    if (controleffort > 10) {
        controleffort = 10;
    }
    if (controleffort < -10) {
        controleffort = -10;
    }
    EPwm2Regs.CMPB.bit.CMPB = 125.0*controleffort + 1250.0;
}

// angle input for testing
uint16_t test_updown = 1;
float angle = 0.0;

// ex3.6 saturate in case a value outside of the range is passed

void setEPWM8A_RCServo(float angle)
{
    if (angle > 90) {
            angle = 90;
        }
    if (angle < -90) {
            angle = -90;
        }
    EPwm8Regs.CMPA.bit.CMPA = (125.0/18.0)*angle + 1250.0;
}

void setEPWM8B_RCServo(float angle)
{
    if (angle > 90) {
            angle = 90;
        }
    if (angle < -90) {
            angle = -90;
        }
    EPwm8Regs.CMPB.bit.CMPB = (125.0/18.0)*angle + 1250.0;
}
void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();
	
	// Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5);        // set this as the EPWMA12A output pin
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	
	// LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LED7	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	// LED11	
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

	// LED12	
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

	// LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

	// LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
	
	// LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

	// LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

	//SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
	
    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
	
	//WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;
	
    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);
	
	//Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
	PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
	PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 125000); // called every 125ms
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    init_serialSCID(&SerialD,115200);

    // Lab3 ex1
    EPwm12Regs.TBCTL.bit.CTRMODE = 0;
    EPwm12Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm12Regs.TBCTL.bit.PHSEN = 0;
    EPwm12Regs.TBCTL.bit.CLKDIV = 0;          // set to count up mode, free soft emulation mode to free run, disable the phase loading and clock divide by 1

    EPwm12Regs.TBCTR = 0;            // start the timer at 0

    EPwm12Regs.TBPRD = 2500;    // set the period (carrier frequency) of he PWM signal to 20 kHZ.

    EPwm12Regs.CMPA.bit.CMPA = 0;        // initialize the duty cycle to be 0%

    EPwm12Regs.AQCTLA.bit.CAU = 1;
    EPwm12Regs.AQCTLA.bit.ZRO = 2;       // clear the signal pin when TBCTR register reaches the value in CMPA, set the pin when TBCTR register is zero

    EPwm12Regs.TBPHS.bit.TBPHS =0;       // set the phase to zero

// set up for EPWM2A

    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;          // set to count up mode, free soft emulation mode to free run, disable the phase loading and clock divide by 1

    EPwm2Regs.TBCTR = 0;            // start the timer at 0

    EPwm2Regs.TBPRD = 2500;    // set the period (carrier frequency) of he PWM signal to 20 kHZ.

    EPwm2Regs.CMPA.bit.CMPA = 0;        // initialize the duty cycle to be 0%

    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;       // clear the signal pin when TBCTR register reaches the value in CMPA, set the pin when TBCTR register is zero
    EPwm2Regs.TBPHS.bit.TBPHS =0;       // set the phase to zero
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); // change pin mux for EPWM2A

    // set up for EPWM2B
    EPwm2Regs.CMPB.bit.CMPB = 0;        // initialize the duty cycle to be 0%
    EPwm2Regs.AQCTLB.bit.CBU = 1;
    EPwm2Regs.AQCTLB.bit.ZRO = 2;       // clear the signal pin when TBCTR register reaches the value in CMPA, set the pin when TBCTR register is zero

    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); // change pin mux for EPWM2B


    // set up for EPWM8A

    EPwm8Regs.TBCTL.bit.CTRMODE = 0;
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm8Regs.TBCTL.bit.PHSEN = 0;
    EPwm8Regs.TBCTL.bit.CLKDIV = 6;          // set to count up mode, free soft emulation mode to free run, disable the phase loading and clock divide by 64

    EPwm8Regs.TBCTR = 0;            // start the timer at 0

    EPwm8Regs.TBPRD = 15625;    // set the period (carrier frequency) of he PWM signal to 50HZ.

    EPwm8Regs.CMPA.bit.CMPA = 1250;        // initialize the duty cycle to be 8% (CMPB = 8% * TBPRD)

    EPwm8Regs.AQCTLA.bit.CAU = 1;
    EPwm8Regs.AQCTLA.bit.ZRO = 2;       // clear the signal pin when TBCTR register reaches the value in CMPA, set the pin when TBCTR register is zero

    EPwm8Regs.TBPHS.bit.TBPHS = 0;       // set the phase to zero
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1); // change pin mux for EPWM8A

    // set up for EPWM8B
    EPwm8Regs.CMPB.bit.CMPB = 1250;        // initialize the duty cycle to be 8% (CMPB = 8% * TBPRD)

    EPwm8Regs.AQCTLB.bit.CBU = 1;
    EPwm8Regs.AQCTLB.bit.ZRO = 2;       // clear the signal pin when TBCTR register reaches the value in CMPB, set the pin when TBCTR register is zero
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1); // change pin mux for EPWM8B

// set up for EPWM9A

    EPwm9Regs.TBCTL.bit.CTRMODE = 0;
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm9Regs.TBCTL.bit.PHSEN = 0;
    EPwm9Regs.TBCTL.bit.CLKDIV = 1;          // set to count up mode, free soft emulation mode to free run, disable the phase loading and clock divide by 2

    EPwm9Regs.TBCTR = 0;            // start the timer at 0

    EPwm9Regs.TBPRD = 2500;    // set the period (carrier frequency) of he PWM signal to 20 kHZ.

    //EPwm9Regs.CMPA.bit.CMPA = 0;        // initialize the duty cycle to be 0% (commented out for ex4)

    EPwm9Regs.AQCTLA.bit.CAU = 0;        // disable CMPA
    EPwm9Regs.AQCTLA.bit.ZRO = 3;       // Set to toggle to produce square wave

    EPwm9Regs.TBPHS.bit.TBPHS =0;       // set the phase to zero
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5); // change pin mux for EPWM9A


    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1; // For EPWM9A
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
	
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
				serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.  
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
	
	
	
    // Insert SWI ISR Code here.......
	
	
    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%250) == 0) {
        //displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

	// Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
	
	
    CpuTimer1.InterruptCount++;
    EPwm9Regs.TBPRD = songarray2[songindex];    // set EPWM9A's TBPRD to the value stored at the current index in the song array.
    songindex ++;   // increment the song index
    if (songindex == SONG_LENGTH2){
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0); // change pin mux for GPIO16
        GpioDataRegs.GPACLEAR.bit.GPIO16 = 1; // set GPIO 16 to low
    }

}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	
	
	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
	
	if ((CpuTimer2.InterruptCount % 50) == 0) {
		UARTPrint = 1;
	}

	if (EPwm12Regs.CMPA.bit.CMPA == 0 ){
	    updown = 1;         // set to up count mode when CMPA = 0
	}
	if (EPwm12Regs.CMPA.bit.CMPA == EPwm12Regs.TBPRD){
	    updown = 0;        // set to down count mode when CMPA = TBPRD
	}

	if (updown == 1){
	    EPwm12Regs.CMPA.bit.CMPA ++;    // increment CMPA every 1ms in up count mode

	}
	if (updown == 0){
	    EPwm12Regs.CMPA.bit.CMPA = EPwm12Regs.CMPA.bit.CMPA - 1;   // decrement CMPA every 1ms in down count mode
	}

// Exercise two
	if (controleffort <= -10.0 ){
	    UD = 1.0;         // set to up count mode when controleffort = -10
	}
	if (controleffort >= 10.0){
	    UD = 0.0;        // set to down count mode when controleffort = 10
	}

	if (UD == 1.0){
	    controleffort = controleffort + 0.001;    // increment controleffort every 100ms in up count mode

	}
	if (UD == 0.0){
	    controleffort = controleffort - 0.001;   // decrement controleffort every 100ms in up count mode

    }
	setPWM2A (-controleffort);
	setPWM2B (controleffort);
// ex3.6 for testing: add 0.001 each time it is called when its up counting
	if (angle >= 90){
	    test_updown = 0.0;
	}
	if (angle <= -90){
	    test_updown = 1.0;
	}
	if (test_updown == 1.0){
	    angle = angle + 0.1;
	}
	if (test_updown == 0.0){
	    angle = angle - 0.1;
	}
	setEPWM8A_RCServo(angle);
	setEPWM8B_RCServo(angle);
}
