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
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

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

__interrupt void ADCA_ISR (void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// count variable for ADCA1 interrupt function
uint16_t adcaCount = 0;

// global variables for the converted voltage
float adcV0 = 0.0;
// global variable to store ADCAINA1, ADCAINA2, ADCAINA3, and ADCAINA4 raw reading

int16_t adca2result = 0;              // left IR sensor
int16_t adca3result = 0;              // right IR sensor
int16_t adca4result = 0;              // ultrasonic sensor

float LeftWheel = 0.0;
float RightWheel = 0.0;
// distance travelled
float dis_left = 0.0;
float dis_right = 0.0;
//for lab6 ex2
float uLeft = 0.0; //control effort variables
float uRight = 0.0;

float PosLeft_K = 0.0; //current left position
float PosRight_K = 0.0; // current right position
float PosLeft_K_1 = 0.0; // previous left position
float PosRight_K_1 = 0.0; //previous right position
float VLeftK = 0.0; //velocity left in ft
float VRightK = 0.0; //velocity right in ft
//for lab6 ex3
float Vref = 0.0;
float Kp = 3.0;
float Ki = 25.0;

float ThLK = 0.0; // theta left current
float ThLK_1 = 0.0; // theta left past
float ILK = 0.0;
float ILK_1 = 0.0;
float errLK = 0.0;
float errLK_1 = 0.0;

float ThRK = 0.0; // theta right current
float ThRK_1 = 0.0; // theta right past
float IRK = 0.0;
float IRK_1 = 0.0;
float errRK = 0.0;
float errRK_1 = 0.0;
//for ex4
float turn = 0.0;
float errturn = 0.0;
float KP_turn = 3.0;
//for ex5
float ninetyturn = 0.56759*PI/4.0; //arclength of a ninety degree trun
float PL_i = 0.0; // record current location for left wheel
float PR_i = 0.0; // .... for right wheel

float wr = 0.56759;     // robot width in ft
float rwh = 0.1946;     // wheel radius in ft
float bear_r = 0.0;         // robot pose bearing in radians
float bear_d = 0.0;        // robot pose bearing in degrees
float bear_mod = 0.0;            // absolute bearing
float theta_r = 0.0;   // right wheel rotation angle in radians
float theta_l = 0.0;
float angVL = 0.0;
float angVR = 0.0;      // angular velocity of wheels in rad/s

float theta_avg_dot = 0.0; // average wheel speed
float xr = 0.0;
float yr = 0.0;           // XY coordinates of robot
float xr_dot = 0.0;
float yr_dot = 0.0;       // derivative of xr and yr
float xr_dot_1 = 0.0;
float yr_dot_1 = 0.0;       // previous value of xr and yr

// final project variables
int statevar = 0;                 // hardcoded to test
int statevar_1 = 0;       // previous value of the state
long timeint = 0;
long timeout = 2000;       // stay in a state for 1.5 seconds whenever the robot enters that state
int cam_out = 0;        // integer output from RasberryPi camera (legal states)-
float voltout = 0.0;        // for the honk
int songstate = 50;
int songflag = 0;
int playingsound= 0;
uint16_t playtime = 0;
float wave1 = 0.0;
float wave2 = 0.0;
float amplitude = 0.0;
float amp1 = 1.5;
float amp2 = .75;
float frequency = 750.0;
float frequency2 = 1500.0;
float frequency3 = 0.0;
float frequency4 = 1000.0;
float frequency5 = 1250.0;
float frequency6 = 500.0;
float ultra = 0.0;   // ultrasonic voltage
float LIR = 0.0;
float RIR = 0.0;    // left and right IR sensor voltage
float aLIR_1 = 0.0;
float aRIR_1 = 0.0;       // old values
float FIR = 0.0;          // front infra-red voltage
// variables for average filter
float L1 = 0.0;
float L2 = 0.0;
float L3 = 0.0;
float L4 = 0.0;
float R1 = 0.0;
float R2 = 0.0;
float R3 = 0.0;
float R4 = 0.0;
float F1 = 0.0;
float F2 = 0.0;
float F3 = 0.0;
float F4 = 0.0;     // left, right, and front

float sumLIR = 0.0;
float sumRIR = 0.0;
float sumFIR = 0.0;
int avgCount = 0;
float aLIR = 0.0;
float aRIR = 0.0;
float aFIR = 0.0;     // filtered average readings
// wall follow variables
float leftref = 1.65;
float rightref = 1.7;      // want to be away from wall
float FrontDist = 1.7;       // front distance from sensor
float thresh1 = 6;       // lower front threshold
float thresh2 = 10;       // higher front threshold [in]
float Kpl = 0.4;      // Kp gain to turn left
float Kpr = 0.4;      // Kp gain to turn right
float Kpf = 1.5;        // Kp gain for front reading
int turnCount = 3000;
int startCount = 1500;  // keep in a state for some time
int initialCount = 10000;       // only used in very beginning

// turn timer used to turn at corners
float turnSat = 0.085;       // max allowable magnitude of "turn", between 0.08 and 0.09 is good

// RasberryPi variables
extern char crossing_flag[1];

void setDACA(float dacouta0) {
    if (dacouta0 > 3.0) dacouta0 = 3.0;
    if (dacouta0 < 0.0) dacouta0 = 0.0;
    DacaRegs.DACVALS.bit.DACVALS = (4095.0/3.0)*dacouta0; // perform scaling of 0-3 to 0-4095
}

// ADCA Interrupt service routine
__interrupt void ADCA_ISR (void)         // called every 1ms
{
    adca2result = AdcaResultRegs.ADCRESULT0;
    adca3result = AdcaResultRegs.ADCRESULT1;
    adca4result = AdcaResultRegs.ADCRESULT2;
    // Here covert ADCINA2, ADCINA3 to volts

    LIR = (3.0/4095.0)*adca2result;       // left IR
    RIR = (3.0/4095.0)*adca3result;       // right IR
    FIR = (3.0/4095.0)*adca4result;       // fromt IR
    LIR = 3.0 - LIR;
    RIR = 3.0 - RIR;       // invert the values
    FIR = 3.0 - FIR;

    sumLIR += LIR;
    sumRIR += RIR;
    sumFIR += FIR;
    avgCount ++;
    if (avgCount == 5){
        aLIR = sumLIR/5.0;
        aRIR = sumRIR/5.0;
        aFIR = sumFIR/5.0;
        sumLIR = 0;
        sumRIR = 0;
        sumFIR = 0;
        avgCount = 0;          // 5 sample average filter
    }

    aRIR_1 = aRIR;
    aLIR_1 = aLIR;        // update IR readings
    adcaCount ++;
    voltout = 1.5+amplitude*sin(2*PI*wave1*adcaCount*.0001)+amplitude*sin(2*PI*wave2*adcaCount*.0001);
    setDACA(voltout);
    if (playingsound == 1) {

        switch(songstate){
        case 11:
            adcaCount ++;
            wave1 = frequency;
            wave2 = frequency2;
            amplitude= amp1;
            if (adcaCount > 5000){
                adcaCount = 0;
                songstate = 12;
            }

            break;
        case 12:
            adcaCount++;
            wave1 = frequency3;
            wave2 = frequency3;
            amplitude= amp1;
            if (adcaCount > 5000){
                adcaCount = 0;
                songstate = 13;
            }
            break;
        case 13:
            adcaCount++;
            wave1 = frequency5;
            wave2 = frequency5;
            amplitude= amp1;
            if (adcaCount > 9500){
                adcaCount = 0;
                songstate = 14;
            }
            break;
        case 14:
            adcaCount++;
            wave1 = frequency3;
            wave2 = frequency3;
            amplitude= amp1;
            if (adcaCount > 500){
                adcaCount = 0;
                playingsound = 0;
            }
            break;
        case 21:
            adcaCount ++;
            wave1 = frequency4;
            wave2 = frequency6;
            amplitude= amp2;
            if (adcaCount > 5000){
                adcaCount = 0;
                songstate = 22;
            }

            break;
        case 22:
            adcaCount++;
            wave1 = frequency3;
            wave2 = frequency3;
            amplitude= amp2;
            if (adcaCount > 2500){
                adcaCount = 0;
                songstate = 23;
            }
            break;
        case 23:
            adcaCount++;
            wave1 = frequency2;
            wave2 = frequency5;
            amplitude= amp2;
            if (adcaCount > 5000){
                adcaCount = 0;
                songstate = 24;
            }
            break;
        case 24:
            adcaCount++;
            wave1 = frequency3;
            wave2 = frequency3;
            amplitude= amp2;
            if (adcaCount > 2500){
                adcaCount = 0;
                songstate = 25;
            }
            break;
        case 25:
            adcaCount++;
            wave1 = frequency;
            wave2 = frequency4;
            amplitude= amp2;
            if (adcaCount > 4500){
                adcaCount = 0;
                songstate = 26;
            }
            break;
        case 26:
            adcaCount++;
            wave1 = frequency3;
            wave2 = frequency3;
            amplitude= amp2;
            if (adcaCount > 500){
                adcaCount = 0;
                playingsound = 0;
            }
            break;
        case 31:
            adcaCount ++;
            wave1 = frequency4;
            wave2 = frequency;
            amplitude= amp2;
            if (adcaCount > 2500){
                adcaCount = 0;
                songstate = 32;
            }
        case 32:
            adcaCount ++;
            wave1 = frequency3;
            wave2 = frequency3;
            amplitude= amp2;
            if (adcaCount > 1250){
                adcaCount = 0;
                songstate = 33;
            }
        case 33:
            adcaCount ++;
            wave1 = frequency6;
            wave2 = frequency5;
            amplitude= amp2;
            if (adcaCount > 2500){
                adcaCount = 0;
                songstate = 34;
            }
        case 34:
            adcaCount ++;
            wave1 = frequency3;
            wave2 = frequency3;
            amplitude= amp2;
            if (adcaCount > 250){
                adcaCount = 0;
                playingsound = 0;
            }
        }

    } else {
        if (songflag == 1) { // horn sound for state 20 when the robot sees obstacle for first time
            playingsound = 1;
            songstate = 11;
            songflag = 0;
        } else if (songflag == 2) { // beeping for when the robot finds line again (state 40)
            playingsound = 1;
            songstate = 21;
            songflag = 0;
        } else if (songflag == 3) { // short turn signal sound during wall-following state (probably will not be used)
            songstate = 31;
            songflag = 0;
        }
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void PeriodicFunction (void) {
    timeint ++;
    switch(statevar) {
    case 0:
        Vref = 0.0;
        turn = 0.0;
        if (initialCount != 0) {
            initialCount --;
        }
        if (initialCount == 0) {
            statevar = 10;
        }
        break;

    case 10:
        if (startCount == 0) {
            Vref = 0.1;
            turn = 0.0;// operations to do in this state
            if (aFIR <= FrontDist) {    // sonar detects obstacle
                // RIR around 1.76, LIR around 1.76, 5 average filter
                statevar = 20;
                timeout = 2000;
                startCount = 1000;
                Vref = 0.0;
                turn = 0.0;
                songflag = 1;
            } else {
                statevar = 10;      // remains in line follow mode if no obstacle detected
            }
        }
        if (startCount != 0) {
            startCount --;
        }
        break;

    case 20:   // pending state entered when obstacle is detected

        statevar_1 = statevar;      // update the previous state variable to decide turn direction
        // operations to do in this state
        if (startCount != 0) {
            startCount --;
        }
        if (startCount == 0) {
            if (cam_out == 1) {
                statevar = 25;           // turn left for right wall following
                timeout = 2000;
            } else if (cam_out == 2) {
                statevar = 26;            // turn right for left wall following
                timeout = 2000;
            } else if (cam_out == 3) {
                statevar = 50;
                songflag = 3;
                // stop any operation when encountering two solid lines (stay still and honk)
            }
        }
        break;

    case 25:
        if (timeout != 0) {
            turn = 0.075;
            timeout --;
        }
        if (timeout == 0) {
            statevar = 30;
            timeout = 2000;
            turnCount = 4800;
        }
        break;

    case 26:
        if (timeout != 0) {
            turn = -turnSat;
            timeout --;
        }
        if (timeout == 0) {
            statevar = 31;
            timeout = 1500;
            turnCount = 3800;
        }
        break;

    case 30:        // turn left for right wall following
        turn = Kpr*(rightref - aRIR);
        Vref = 0.15;
        if ((aRIR - aRIR_1) > 0.6) {
            Vref = 0.15;
            turn = -0.07;
        }
        statevar_1 = statevar;      // update the previous state variable to decide turn direction
        if (turn <= -turnSat) {
            turn = -turnSat;
        }
        if (turn >= turnSat) {
            turn = turnSat;        // saturate the turn value
        }
        // operations to do in this state
        if (timeout != 0) {
            timeout --;
        }
        if (turnCount == 0) {
            statevar = 40;
            timeout = 1800;
            turnCount = 3100;
            crossing_flag[0] = 'X';
        } else {
            turnCount --;           // hard coded
        }

        //        if (timeout == 0) {
        //            if (cam_out == 4) {
        //                statevar = 40;             // go to the "back to line" state
        //            } else {
        //                statevar = 30;
        //            }
        //        } else {
        //            timeout --;        // continue time delay if timeout =! 0
        //        }
        break;

    case 31:        // turn left for right wall following
        turn = -Kpl*(leftref - aLIR);
        Vref = 0.15;
        if ((aLIR - aLIR_1) > 0.6) {
            Vref = 0.15;
            turn = turnSat;          // hard coded
        }
        statevar_1 = statevar;      // update the previous state variable to decide turn direction
        if (turn <= -turnSat) {
            turn = -turnSat;
        }
        if (turn >= turnSat) {
            turn = turnSat;        // saturate the turn value
        }
        // operations to do in this state
        if (timeout != 0) {
            timeout --;
        }
        if (timeout == 0) {
            if (turnCount == 0) {
                statevar = 40;
                timeout = 2300;
                turnCount = 3100;
                crossing_flag[0] = 'X';
            } else {
                turnCount --;
            }
        }
        //        if (timeout == 0) {
        //            if (cam_out == 4) {                 // conditions to switch states
        //                statevar = 40;             // go to the "back to line" state
        //            } else {
        //                statevar = 31;
        //            }
        //        } else {
        //            timeout --;        // continue time delay if timeout =! 0
        //        }
        break;

    case 40:           // "back to line" state
        songflag = 2;
        // operations to do in this state
        if (statevar_1 == 30) {
            if (timeout != 0) {
                turn = turnSat;
                Vref = 0.0;
                timeout --;
            }
            if (timeout == 0) {
                statevar = 10;
                startCount = 300;
                crossing_flag[0] = 'Z';
            }
        }
        if (statevar_1 == 31) {
            if (timeout != 0) {
                turn = -turnSat;
                Vref = 0;
                timeout --;
            }
            if (timeout == 0) {
                statevar = 10;
                crossing_flag[0] = 'Z';
            }
        }
        //        if (cam_out != 4) {               // conditions to switch states
        //            statevar = 10;
        //        }
        break;

    case 50:
        Vref = 0.0;
        turn = 0.0;            // "finish" state
        break;
        }
    }

    //set up eQEP1 for lab6
    void init_eQEPs(void) {
        // setup eQEP1 pins for input
        EALLOW;
        //Disable internal pull-up for the selected output pins for reduced power consumption
        GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
        GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
        GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
        GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
        EDIS;
        // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
        // Comment out other unwanted lines.
        GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
        GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
        EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
        EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
        EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
        EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
        EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
        EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
        EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
        EQep1Regs.QPOSCNT = 0;
        EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

        // setup QEP2 pins for input
        EALLOW;
        //Disable internal pull-up for the selected output pinsfor reduced power consumption
        GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
        GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
        GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
        GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
        EDIS;
        GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
        GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
        EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
        EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
        EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
        EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
        EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
        EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
        EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
        EQep2Regs.QPOSCNT = 0;
        EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    }

    float readEncLeft(void) {
        int32_t raw = 0;
        uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
        raw = EQep1Regs.QPOSCNT;
        if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
        // 100 slits in the encoder disk so 100 square waves per one revolution of the
        // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
        // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
        // 400*30 = 12000 count per rev >>>> 12000/(2pi)  count/rad
        return (-raw/12000.0*(2*PI));
    }
    float readEncRight(void) {
        int32_t raw = 0;
        uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
        raw = EQep2Regs.QPOSCNT;
        if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
        // 100 slits in the encoder disk so 100 square waves per one revolution of the
        // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
        // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
        // 400*30 = 12000 count per rev >>>> 12000/(2pi)  count/rad
        return (raw/12000.0*(2*PI)); // angular displacement
    }

    // command ePWM2A and ePWM2B with a command between -10 and 10
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

    void main(void) {


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
        GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
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

        //set up this GPIO pin as output to fire the ultrasonic wave
        GPIO_SetupPinMux(67, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);
        //    GpioDataRegs.GPCSET.bit.GPIO67 = 1;

        // ultrasoinic output
        GPIO_SetupPinMux(69, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(69, GPIO_INPUT, GPIO_PULLUP);

        //Left Infrared distance sensor
        GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(29, GPIO_INPUT, GPIO_PULLUP);

        //Right Infrared distance sensor
        GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(26, GPIO_INPUT, GPIO_PULLUP);

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

        // This is how you tell the F28379D processor to call your defined functions when certain interrupt events occur.
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
        PieVectTable.ADCA1_INT = &ADCA_ISR;

        PieVectTable.EMIF_ERROR_INT = &SWI_isr;
        EDIS;    // This is needed to disable write to EALLOW protected registers


        // Initialize the CpuTimers Device Peripheral. This function can be
        // found in F2837xD_CpuTimers.c
        InitCpuTimers();

        // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
        // 200MHz CPU Freq,                       Period (in uSeconds)
        ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000);
        ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
        ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

        // Enable CpuTimer Interrupt bit TIE
        CpuTimer0Regs.TCR.all = 0x4000;
        CpuTimer1Regs.TCR.all = 0x4000;
        CpuTimer2Regs.TCR.all = 0x4000;

        init_serialSCIA(&SerialA,115200);
        //    init_serialSCIB(&SerialB,115200);
        //    init_serialSCIC(&SerialC,115200);
        init_serialSCID(&SerialD,115200);

        //ex1.1 o command the ADCD peripheral to sample ADCIND0 and ADCIND1 every 1ms
        EALLOW;
        EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
        EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
        EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
        EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)

        EPwm5Regs.TBCTR = 0x0; // Clear counter
        EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
        EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
        EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
        EPwm5Regs.TBPRD = 5000; // Set Period to 10000Hz. Input clock is 50MHz.
        // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
        EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
        EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
        EDIS;



        // ex1.2
        //setup ADCD so that it uses 2 of its 16 SOCs (Start of Conversions)
        // We will assign channel ADCIND0 to SOC0 and channel ADCIND1 to SOC1. These below initializations also setup ADCD to flag
        // an interrupt when SOC1 is finished converting.

        EALLOW;
        //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
        AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
        AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
        AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
        AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings

        //Set pulse positions to late
        AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

        //power up the ADCs
        AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        //delay for 1ms to allow ADC time to power up

        DELAY_US(1000);
        //Select the channels to convert and end of conversion flag

        //Many statements commented out, To be used when using ADCA or ADCB
        //ADCA
        AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;      //SOC0 will convert Channel ADCINA2
        AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
        AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;     //SOC1 will convert Channel ADCAIN3
        AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
        AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
        AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4;       //SOC2 will convert Channel ADCAIN4
        AdcaRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
        AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

        EDIS;


        // ex1.3
        // Enable DACA and DACB outputs
        // Note: In this lab, we will only be using DACA but we will setup DACB just in case you need it for your final project. See Pinmux Table
        // for pin location of DACA and DACB.
        EALLOW;
        DacaRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacA output-->uses ADCINA0
        DacaRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
        DacaRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
        DacbRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacB output-->uses ADCINA1
        DacbRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
        DacbRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
        EDIS;

        init_eQEPs();

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


        // set GPIO
        GPIO_SetupPinMux(52,GPIO_MUX_CPU1,0);
        GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);

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
        // Enable TINT1 in the PIE: Group 1 interrupt 1, set PIE channel map to ADCA
        PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

        // Enable global Interrupts and higher priority real-time debug events
        EINT;  // Enable Global interrupt INTM
        ERTM;  // Enable Global realtime interrupt DBGM


        // IDLE loop. Just sit and loop forever (optional):
        while(1)
        {
            if (UARTPrint == 1 ) {
                serial_printf(&SerialA,"left IR (V): %.3f  right IR (V): %.3f  Front IR: %.3f bearing: %.3f \r\n ",LIR,RIR,FIR,bear_mod);
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

        if ((numTimer0calls%250) == 0) {
            displayLEDletter(LEDdisplaynum);
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
    }

    // cpu_timer2_isr CPU Timer2 ISR
    __interrupt void cpu_timer2_isr(void)
    {
        PeriodicFunction();
        LeftWheel = readEncLeft();
        RightWheel  =  readEncRight();

        dis_left = LeftWheel/2/PI*1.225; // distance traveled in ft
        dis_right = RightWheel/2/PI*1.225;

        theta_r = RightWheel;
        theta_l = LeftWheel;
        //calculate velocity
        PosLeft_K = dis_left;
        PosRight_K = dis_right;

        VLeftK = (PosLeft_K-PosLeft_K_1)/0.004;
        VRightK = (PosRight_K-PosRight_K_1)/0.004;    // wheel linear speed (ft/s)
        angVL = VLeftK/(PI*2*rwh)*(2*PI);
        angVR = VRightK/(PI*2*rwh)*(2*PI);   // angular velocity (rad/s)

        theta_avg_dot = 0.5*(angVR + angVL);
        bear_r = rwh/wr*(theta_r-theta_l);          // robot bearing in radians
        bear_d = bear_r*180.0/PI;           // robot bearing in degrees
        bear_mod = fmod(bear_d,360);
        if (bear_mod > 180.0) {
            bear_mod = bear_mod - 360.0;   // invert the bearing for -1 ~ -180 degrees
        } else {
            bear_mod = bear_mod;
        }
        xr_dot = rwh*theta_avg_dot*cos(bear_r);
        yr_dot = rwh*theta_avg_dot*sin(bear_r);

        xr = xr + (xr_dot + xr_dot_1)/2.0*0.004;
        yr = yr + (yr_dot + yr_dot_1)/2.0*0.004;     //calculating x y coordinate

        xr_dot_1 = xr_dot;
        yr_dot_1 = yr_dot;
        PosLeft_K_1 = PosLeft_K;
        PosRight_K_1 = PosRight_K;

        errturn = turn + (VLeftK-VRightK);

        //left
        errLK = Vref - VLeftK - KP_turn*errturn;
        ILK = ILK_1 + (errLK+errLK_1)/2.0*0.004;
        uLeft = Kp*errLK + Ki*ILK;

        //right
        errRK = Vref - VRightK + KP_turn*errturn;
        IRK = IRK_1 + (errRK+errRK_1)/2.0*0.004;
        uRight = Kp*errRK + Ki*IRK;

        if (uLeft >= 6){
            uLeft = 6;
            ILK = ILK_1;
        }
        if (uLeft <= -6){
            uLeft = -6;
            ILK = ILK_1;
        }
        if (uRight >= 6){
            uRight = 6;
            IRK  = IRK_1;
        }
        if (uRight <= -6){
            uRight = -6;
            IRK = IRK_1;
        }

        setPWM2A(uRight);
        setPWM2B(-uLeft);

        errLK_1 = errLK;
        ILK_1 = ILK;
        errRK_1 = errRK;
        IRK_1 = IRK;

        // Blink LaunchPad Blue LED
        GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
        CpuTimer2.InterruptCount++;
        if ((CpuTimer2.InterruptCount % 400) == 0) {          // print every 500ms
            UARTPrint = 1;
        }
    }
