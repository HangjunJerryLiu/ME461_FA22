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

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// proj 3 variables initialization
uint16_t RUN = 0;
uint16_t updown = 0;

uint16_t RC1 = 0;
uint16_t RC2 = 0;
uint16_t ADC1 = 0;
uint16_t ADC2 = 0;
uint16_t second = 0;
uint16_t minute = 0;
uint16_t hour = 0;
uint16_t day = 0;
uint16_t date = 0;
uint16_t month = 0;
uint16_t year = 0;

// creating text message for day. First line index 0 corresponds to nothing, second line index 1 corresponds to Monday
char daytext[8][9] = {{0,0,0,0,0,0,0,0,0},
                      {'M','o','n','d','a','y',0,0,0},
                      {'T','u','e','s','d','a','y',0,0},
                      {'W','e','d','n','e','s','d','a','y'},
                      {'T','h','u','r','s','d','a','y',0},
                      {'F','r','i','d','a','y',0,0,0}

};


// project 2 I2C initialization function
void I2CB_Init(void)
{
    EALLOW;
    /* Enable internal pull-up for the selected I2C pins */
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1;
    /* Set qualification for the selected I2C pins */
    GpioCtrlRegs.GPBQSEL1.bit.GPIO40 = 3;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO41 = 3;
    /* Configure which of the possible GPIO pins will be I2C_B pins using GPIO regs*/
    GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 2;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO41 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 2;
    EDIS;
    // Initialize I2C
    I2cbRegs.I2CMDR.bit.IRS = 0;
    // 200MHz / 20 = 10MHz
    I2cbRegs.I2CPSC.all = 19;
    // 10MHz/40 = 250KHz
    I2cbRegs.I2CCLKL = 15; //psc > 2 so d = 5 See Usersguide
    I2cbRegs.I2CCLKH = 15; //psc > 2 so d = 5 See Usersguide
    I2cbRegs.I2CIER.all = 0x00;
    I2cbRegs.I2CMDR.bit.IRS = 1;
    DELAY_US(5000);
}

// project 2 I2C write function
//Write 2 16bit commands (LSB then MSB) to I2C Slave DAN777 starting at DAN777's register 4,
uint16_t WriteDAN777RCServo(uint16_t RC16bit_1, uint16_t RC16bit_2) {
    uint16_t RC1LSB = 0;
    uint16_t RC1MSB = 0;
    uint16_t RC2LSB = 0;
    uint16_t RC2MSB = 0;
    RC1LSB = RC16bit_1 & 0xFF; //Bottom 8 bits of RC
    RC1MSB = (RC16bit_1 >> 8) & 0xFF; //Top 8 bits of RC
    RC2LSB = RC16bit_2 & 0xFF; //Bottom 8 bits of RC
    RC2MSB = (RC16bit_2 >> 8) & 0xFF; //Top 8 bits of RC
    DELAY_US(200); // Allow time for I2C to finish up previous RC.
    if (I2cbRegs.I2CSTR.bit.BB == 1) { // Check if I2C busy, if it is better
        return 2; // to Exit and try again next sample
    } // This should not happen too often
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CSAR.all = 0x25; // I2C address of DAN777
    I2cbRegs.I2CCNT = 5; //Num Values plus Start Register 4 + 1
    I2cbRegs.I2CDXR.all = 4; // First need to transfer the Register value
    // to start writing data
    // I2C in master mode (MST), I2C is in transmit mode (TRX) with start and stop
    I2cbRegs.I2CMDR.all = 0x6E20;
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = RC1LSB; // Write RC 1 LSB (byte 2)
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = RC1MSB; // Write Command 1 MSB (byte 3)
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = RC2LSB; // Write Command 2 LSB (byte4)
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = RC2MSB; // Write Command 2 MSB (byte5)
    // After this write since I2CCNT = 0
    // A Stop condition will be issued
    if (I2cbRegs.I2CSTR.bit.NACK == 1){
        return 3;
    }
    return 0;
}

//Write 7 16bit commands (LSB then MSB) to I2C Slave DS1388Z starting at DS1388Z's register 1,
uint16_t WriteDS1388Z(uint16_t second,uint16_t minute,uint16_t hour,uint16_t day,uint16_t date,uint16_t month,uint16_t year) {
    uint16_t second_reg = 0; //second
    //uint16_t second_tens = 0;
    uint16_t minute_reg = 0;  //minute
    //uint16_t minute_tens = 0;
    uint16_t hour_reg = 0;  //hour
    //uint16_t hour_tens = 0;
    uint16_t day_reg = 0;  //day
    uint16_t date_reg = 0;  //Date
    //uint16_t date_tens = 0;
    uint16_t month_reg = 0; //month
    //uint16_t month_tens = 0;
    uint16_t year_reg = 0; //year
    //uint16_t year_tens = 0;
    second_reg = ((second/10) << 4)|(second % 10); //send second to register
    minute_reg = ((minute/10) << 4)|(minute % 10); // send minute to register
    hour_reg = ((hour/10) << 4)|(hour % 10); //send hour to register
    day_reg = day % 10; //send day to register
    date_reg = ((date/10) << 4)|(date % 10); //send date to register
    month_reg = ((month/10) << 4)|(month % 10);//send month to register
    year_reg = ((year/10) << 4)|(year % 10); //send year to register
    DELAY_US(200); // Allow time for I2C to finish up previous RC.
    if (I2cbRegs.I2CSTR.bit.BB == 1) { // Check if I2C busy, if it is better
        return 2; // to Exit and try again next sample
    } // This should not happen too often
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CSAR.all = 0x68; // I2C address of DS1388 (dec = 104)
    I2cbRegs.I2CCNT = 8; //Num Values plus Start Register 7 + 1
    I2cbRegs.I2CDXR.all = 1; // First need to transfer the Register value
    // to start writing data
    // I2C in master mode (MST), I2C is in transmit mode (TRX) with start and stop
    I2cbRegs.I2CMDR.all = 0x6E20;
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = second_reg; // Write
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = minute_reg; // Write
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = hour_reg; // Write
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = day_reg; // Write
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = date_reg; // Write
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = month_reg; // Write
    if (I2cbRegs.I2CSTR.bit.NACK == 1){ // Check for No Acknowledgement
        return 3; // This should not happen
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CDXR.all = year_reg; // Write
    // After this write since I2CCNT = 0
    // A Stop condition will be issued
    if (I2cbRegs.I2CSTR.bit.NACK == 1){
        return 3;
    }
    return 0;
}
//
//// project 2 I2C read function
////Read Two 16 Bit values from I2C Slave DAN777 starting at DAN777's register 0
////Notice the Rvalue1 and Rvalue2 passed as pointers (passed by reference)
////So pass address of uint16_t variable when using this function for example:
//// uint16_t Rval1 = 0;
//// uint16_t Rval2 = 0;
//// err = ReadTwo16BitValuesFromCHIPXYZ(&Rval1,&Rval2);
//// This allows Rval1 and Rval2 to be changed inside the function and return
//// the values read inside the function.
uint16_t ReadDAN777ADC(uint16_t *Rvalue1,uint16_t *Rvalue2) {
    uint16_t adc1LSB = 0;
    uint16_t adc1MSB = 0;
    uint16_t adc2LSB = 0;
    uint16_t adc2MSB = 0;
    DELAY_US(200); // Allow time for I2C to finish up previous commands.
    if (I2cbRegs.I2CSTR.bit.BB == 1) {
        return 2;
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CSAR.all = 0x25; // I2C address of DAN777
    I2cbRegs.I2CCNT = 1; // Just Sending Address to start reading from
    I2cbRegs.I2CDXR.all = 0; // Start Reading at this Register location
    // I2C in master mode (MST), I2C is in transmit mode (TRX) with start
    I2cbRegs.I2CMDR.all = 0x6620;
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CSAR.all = 0x25; // I2C address of ChipXYZ
    I2cbRegs.I2CCNT = 4;
    // I2C in master mode (MST), TRX=0, receive mode start stop
    I2cbRegs.I2CMDR.all = 0x6C20; // Reissuing another Start with Read
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    adc1LSB = I2cbRegs.I2CDRR.all; // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    adc1MSB = I2cbRegs.I2CDRR.all; // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    adc2LSB = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    adc2MSB = I2cbRegs.I2CDRR.all; // Read CHIPXYZ
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    // After this read since I2CCNT = 0
    // A Stop condition will be issued
    *Rvalue1 = (adc1MSB << 8) | (adc1LSB & 0xFF);
    *Rvalue2 = (adc2MSB << 8) | (adc2LSB & 0xFF);
    return 0;
}

////project 2 I2C read function for DS1388Z
uint16_t ReadDS1388Z(uint16_t *second,uint16_t *minute,uint16_t *hour,uint16_t *day,uint16_t *date,uint16_t *month,uint16_t *year) {
    uint16_t second_dec = 0; //second
    //uint16_t second_tens = 0;
    uint16_t minute_dec = 0;  //minute
    //uint16_t minute_tens = 0;
    uint16_t hour_dec = 0;  //hour
    //uint16_t hour_tens = 0;
    uint16_t day_dec = 0;  //day
    uint16_t date_dec = 0;  //Date
    //uint16_t date_tens = 0;
    uint16_t month_dec = 0; //month
    //uint16_t month_tens = 0;
    uint16_t year_dec = 0; //year
    //uint16_t year_tens = 0;
    DELAY_US(200); // Allow time for I2C to finish up previous commands.
    if (I2cbRegs.I2CSTR.bit.BB == 1) {
        return 2;
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CSAR.all = 0x68; // I2C address of DS1388Z
    I2cbRegs.I2CCNT = 1; // Just Sending Address to start reading from
    I2cbRegs.I2CDXR.all = 1; // Start Reading at this Register location
    // I2C in master mode (MST), I2C is in transmit mode (TRX) with start
    I2cbRegs.I2CMDR.all = 0x6620;
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.XRDY); //Poll until I2C ready to Transmit
    I2cbRegs.I2CSAR.all = 0x68; // I2C address of DS1388Z
    I2cbRegs.I2CCNT = 7; // read 7 bytes in total
    //I2C in master mode (MST), TRX=0, receive mode start stop
    I2cbRegs.I2CMDR.all = 0x6C20; // Reissuing another Start with Read
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    second_dec = I2cbRegs.I2CDRR.all; // Read DS1388Z second
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    minute_dec = I2cbRegs.I2CDRR.all; // Read DS1388Z minute
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    hour_dec = I2cbRegs.I2CDRR.all; // Read DS1388Z hour
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    day_dec = I2cbRegs.I2CDRR.all; // Read DS1388Z day
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    date_dec = I2cbRegs.I2CDRR.all; // Read DS1388Z date
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    month_dec = I2cbRegs.I2CDRR.all; // Read DS1388Z month
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    while(!I2cbRegs.I2CSTR.bit.RRDY); //Poll until I2C has Recieved 8 bit value
    year_dec = I2cbRegs.I2CDRR.all; // Read DS1388Z year
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    // After this read since I2CCNT = 0
    // A Stop condition will be issued
    *second = (second_dec >> 4)*10 + (second_dec & 0xF);
    *minute = (minute_dec >> 4)*10 + (minute_dec & 0xF);
    *hour = (hour_dec >> 4)*10 + (hour_dec & 0xF);
    *day = day_dec & 0xF;
    *date = (date_dec >> 4)*10 + (date_dec & 0xF);
    *month = (month_dec >> 4)*10 + (month_dec & 0xF);
    *year = (year_dec >> 4)*10 + (year_dec & 0xF);
    return 0;
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
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 20000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);
    // call I2C initialization function for project 2
    I2CB_Init();

    // write time
    second = 0;
    minute = 41;
    hour = 13;
    day = 2;
    date = 1;
    month = 11;
    year = 22;
    WriteDS1388Z(second,minute,hour,day,date,month,year);


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

    WriteDS1388Z(second,minute,hour,day,date,month,year);
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (RUN == 1){
            WriteDAN777RCServo(RC1,RC2);
            ReadDAN777ADC(&ADC1, &ADC2);

            ReadDS1388Z(&second,&minute,&hour,&day,&date,&month,&year);

            if (RC1 >= 5200){
                updown = 0.0;
            }
            if (RC1 <= 1200){
                updown = 1.0;
            }
            if (updown == 1.0){
                RC1 = RC1 + 20;
                RC2 = RC2 + 20;
            }
            if (updown == 0.0){
                RC1 = RC1 - 20;
                RC2 = RC2 - 20;
            }

            RUN = 0;
        }

        if (UARTPrint == 1 ){
            serial_printf(&SerialA,"ADC1:%d ADC2:%d \r\n Time: %s %d/%d/%d %d:%d:%d\r\n",ADC1,ADC2,daytext[day],month,date,year,hour,minute,second);
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


    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
    // make it print every 100 ms
    if ((CpuTimer2.InterruptCount % 5) == 0) {
        UARTPrint = 1;
    }
    // make it operate every 20 ms
    if ((CpuTimer2.InterruptCount % 1) == 0) {
        RUN = 1;

    }
}

