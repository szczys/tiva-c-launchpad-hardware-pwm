/*
 *************************************************************************************************************
 * TO RUN THIS FILE, YOU NEED TO HAVE THESE RTOS PRODUCTS ENABLED
 *
 * SWI:
 * {{ Buffer_SWI | switchBuffers | 15 | 0x0 }}
 *
 * TIMER:
 * {{ Light_Timer | Timer3IntHandler | 2 | 10000 | timer starts automatically | periodic and continuous }}
 *
 * CLK: period = 50000 (us) | ANY | Timer Interrupt Every Period | SWI priority = 14
 * {{ Buffer_Clk | Timer1IntHandler | 1 | 40 | DO NOT start at boot time }}
 * {{ PID_Clk | Timer2IntHandler | 1 | 1 | start at boot time }}
 *
 *
 *************************************************************************************************************
 */

/*
 *************************************************************************************************************
 * BIOS HEADER FILES
 *************************************************************************************************************
 */
#include <xdc/std.h>                        //mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h>                //mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>                //needed for any Log_info() call
#include <xdc/cfg/global.h>                 //header file for statically defined objects/handles
#include <xdc/runtime/Timestamp.h>          // used for Timestamp() calls
/*
 *************************************************************************************************************
 * C HEADER FILES
 *************************************************************************************************************
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
/*
 *************************************************************************************************************
 * TIVA C HEADER FILES
 *************************************************************************************************************
 */
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.c"
#include "driverlib/uart.c"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.c"
#include "driverlib/pwm.c"
#include "inc/hw_ints.h"
#include "driverlib/timer.c"
#include "driverlib/udma.h"
#include "inc/hw_udma.h"
#include "inc/hw_uart.h"
#include "driverlib/systick.h"

/*
 *************************************************************************************************************
 * DEFINING CONSTANTS
 *************************************************************************************************************
 */
#define PWM_FREQ            10000
#define PWM_ADJUST          83
#define ADC_MAX_VALUE       4096
#define TARGET_VALUE        (ADC_MAX_VALUE / 2) //2000
#define SEQ1                1
#define SEQ2                2
#define SEQ3                3
#define SEQ4                4
#define PRI_0               0
#define PRI_1               1
#define STEP_0              0
#define BUFFER_SIZE         20

/*
 *************************************************************************************************************
 * GLOBAL VARIABLES
 *************************************************************************************************************
 */
char command[2] = "  "; //2-char command array
uint32_t ForwardSensor[1];

/*
 *************************************************************************************************************
 * ADC VALUE
 *************************************************************************************************************
 */
uint32_t rightSensorValue = 0;
uint32_t frontSensorValue = 0;

/*
 *************************************************************************************************************
 * PWM VALUES
 *************************************************************************************************************
 */
volatile uint32_t PWM_CLOCK, PWM_LOAD;
volatile uint32_t TWO_SECONDS = 80000000;

/*
 *************************************************************************************************************
 * PID VALUES
 *************************************************************************************************************
 */
volatile float proportionalRight;
volatile float lastProportionalRight = 250;
volatile float integralRight = 0;
volatile float derivativeRight;
volatile float pidRight;

/*
 *************************************************************************************************************
 * LIGHT SENSOR VALUES
 *************************************************************************************************************
 */
int blkCounter = 0;
int readData = 1;

/*
 *************************************************************************************************************
 * PING PONG BUFFER VALUES
 *************************************************************************************************************
 */
volatile int buffer[BUFFER_SIZE];
volatile int buffer_2[BUFFER_SIZE];
volatile int temp_buffer[BUFFER_SIZE];
signed int error = 0;
int i = 0;
int j = 0;
int k = 0;
int m = 0;
int swap = 0;
int error_count = 1;
int bufferCt = 0;

/*
 *************************************************************************************************************
 * DECLARING FUNCTIONS
 *************************************************************************************************************
 */
void ConfigurePeripherals(void);
void ConfigureUART(void);
void ConfigureADC(void);
void ConfigurePWM(void);
void PID(int RightValue, int FrontValue);
void Timer2IntHandler(void);
void Timer1IntHandler(void);
void Timer3IntHandler(void);
void switchBuffers(void);

/*
 *************************************************************************************************************
 * INITIALIZE EVERYTHING
 *************************************************************************************************************
 */
void ConfigurePeripherals(void) {

    FPULazyStackingEnable();

    /* Set system clock */
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    ConfigureUART();
    ConfigurePWM();
    ConfigureADC();
}

/*
 *************************************************************************************************************
 * UART CONFIG
 *************************************************************************************************************
 */
void ConfigureUART(void)
{
    /* Enable the clocks to PortB and UART1 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    /* Configure PortB pins 0 & 1 for UART1 RX & TX, respectively */
    GPIOPinConfigure(GPIO_PB0_U1RX); //goes to TX pin
    GPIOPinConfigure(GPIO_PB1_U1TX); //goes to RX pin
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Set the UART1 module's clock source */
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));

    /* Configure UART1 */
    // UART module: 1
    // Baud rate: 115200
    // UART clock speed: 16 [MHz]
    UARTStdioConfig(1, 115200, 16000000);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_TX);
}

/*
 *************************************************************************************************************
 * ADC CONFIG
 *************************************************************************************************************
 */
void ConfigureADC(void) {

    /* Enable the clock for ADC0 and PortE */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    /* Configure PortE pins 2 & 3 for ADC usage */
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2); //pin3 = right sensor, pin2 = front sensor

    /* Disable sequencers to ensure safe reconfiguration of them */
    ADCSequenceDisable(ADC0_BASE, 1); //disable sequence 1
    ADCSequenceDisable(ADC0_BASE, 2); //disable sequence 2
    ADCSequenceDisable(ADC0_BASE, 3); //disable sequence 3
    ADCSequenceDisable(ADC0_BASE, 4); //disable sequence 4

    /* Configure sequence priorities and triggers */
    // SS0: unused
    // SS1 - to sample right sensor: trigger = timer, priority = 0
    // SS2 - to sample front sensor: trigger = timer, priority = 1
    // SS3: unused
    ADCSequenceConfigure(ADC0_BASE, SEQ1, ADC_TRIGGER_PROCESSOR,
                         PRI_0);
    ADCSequenceConfigure(ADC0_BASE, SEQ2, ADC_TRIGGER_PROCESSOR,
                         PRI_1);

    /* Configuring sequence steps for sequence 1 */
    // Step 0: sample right sensor, end of sequence
    ADCSequenceStepConfigure(ADC0_BASE, SEQ1, STEP_0,
                             (ADC_CTL_CH0 | ADC_CTL_END));

    /* Configuring sequence steps for sequence 2 */
    // Step 0: sample front sensor, end of sequence
    ADCSequenceStepConfigure(ADC0_BASE, SEQ2, STEP_0,
                             (ADC_CTL_CH1 | ADC_CTL_END));

    /* Re-enable our now newly configured sequences */
    ADCSequenceEnable(ADC0_BASE, SEQ1);
    ADCSequenceEnable(ADC0_BASE, SEQ2);
}

/*
 *************************************************************************************************************
 * PWM CONFIG
 *************************************************************************************************************
 */
void ConfigurePWM(void)
{
    /* Set the PWM module's clock divider */
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    /* Enable the clock for PWM1 and PortA, PortF, PortB, PortE */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    /* LEDs for reading data (not related to PWM) */
    //configuring red, blue, and green LEDs
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    //Phase pins and mode
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6|GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //L motor set forward
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //R motor set forward
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7); //PHASE/ENABLE Mode

    /* Configure PortA pins 6 & 7 for PWM module 1, generator 1 usage */
    GPIOPinConfigure(GPIO_PA6_M1PWM2);
    GPIOPinConfigure(GPIO_PA7_M1PWM3);
    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    /* Configure M1PWM0 for count down mode */
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);

    /* Calculate the PWM clock and load values */
    PWM_CLOCK = SysCtlClockGet() / 64;
    PWM_LOAD = (PWM_CLOCK / PWM_FREQ) - 1;

    /* Set the period of the PWM generator */
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWM_LOAD);
    //PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 10000); //for manual motor control

    /* Specify the duty cycle for the PWM signal */
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWM_ADJUST * PWM_LOAD / 100); //left motor
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, PWM_ADJUST * PWM_LOAD / 100); //right motor

    /* Enable PWM output */
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);

    /* Enable the timer/counter for M1PWM0 */
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);

}

/*
 *************************************************************************************************************
 * TIMER1 INTERRUPT FUNCTION - OUTPUT BUFFER TO TERMINAL
 *************************************************************************************************************
 */
void Timer1IntHandler(void) {


    if (swap == 0) {

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0); //reset LEDs
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //constant green LED to signal that it's transmitting to PC

        UARTprintf(": ");
        for (j = 0; j < BUFFER_SIZE; ++j) {
            UARTprintf("%X, ", buffer[j]);

            if(UARTBusy(UART1_BASE)) {
                buffer_2[j] = temp_buffer[j];
            }
        }
        UARTprintf("\r\n\n");

        Swi_post(Buffer_SWI);

    }

    else if (swap == 1) {

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0); //reset
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //constant green LED to signal that it's transmitting to PC

        UARTprintf(": ");
        for (j = 0; j < BUFFER_SIZE; ++j) {
            UARTprintf("%X, ", buffer_2[j]);

            if(UARTBusy(UART1_BASE)) {
                buffer[j] = temp_buffer[j];
            }
        }
        UARTprintf("\r\n\n");

        Swi_post(Buffer_SWI);

    }

    i = 0;
}

/*
 *************************************************************************************************************
 * SWI INTERRUPT FUNCTION - SWITCH BUFFERS
 *************************************************************************************************************
 */
void switchBuffers(void) {
    if (swap == 0) {
        swap = 1;
    }
    else if (swap == 1) {
        swap = 0;
    }
}

/*
 *************************************************************************************************************
 * TIMER2 INTERRUPT FUNCTION - RUN PID
 *************************************************************************************************************
 */
void Timer2IntHandler(void) {

    /* Trigger the sample sequence 1 */
    ADCProcessorTrigger(ADC0_BASE, SEQ1);

    /* Get results from sample sequence 1 */
    ADCSequenceDataGet(ADC0_BASE, SEQ1, &rightSensorValue);

    /* Trigger sample sequence 2 */
    ADCProcessorTrigger(ADC0_BASE, SEQ2);

    /* Get results from sample sequence 2 */
    ADCSequenceDataGet(ADC0_BASE, SEQ2, &frontSensorValue);

    /* Perform PID on the sensor values */
    PID(rightSensorValue, frontSensorValue);
}

/*
 *************************************************************************************************************
 * PID (set PWM frequency to PWM_LOAD)
 *************************************************************************************************************
 */
void PID(int RightValue, int FrontValue) {

    proportionalRight = (RightValue - TARGET_VALUE) / 20;

    /* Calculate integral terms */
    integralRight = (RightValue - TARGET_VALUE);

    /* Calculate derivative terms */
    derivativeRight = ((RightValue - TARGET_VALUE) - lastProportionalRight)
                                                * (3 / 2);

    /* Calculate PID result */
    pidRight = proportionalRight + (integralRight / 10000) + derivativeRight;

    /* Update some values for proper calculations of the next PID update*/
    lastProportionalRight = (RightValue - TARGET_VALUE);

    if ((pidRight < 25) && (FrontValue > 2000)) // check if dead end & U-Turn
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0); //left backward
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //right forward

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 99 * PWM_LOAD / 100);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 99 * PWM_LOAD / 100);
        //UARTprintf("U-Turn\n");

    }
    else if ((pidRight > 27) && (FrontValue < 1000))  // Turn Left
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //forward
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //forward

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 70 * PWM_LOAD / 100); //left slow
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 80 * PWM_LOAD / 100); //right
        //UARTprintf("Slight Left\n");
    }
    else if ((pidRight > -80) && (pidRight < -20) && (FrontValue < 1000)) // Turn Right //-25
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //forward
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //forward

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 75 * PWM_LOAD / 100); //left
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 65 * PWM_LOAD / 100); //right slow
        //UARTprintf("Slight Right\n");
    }
    else if (pidRight < -100 && FrontValue < 1400) // SHARP RIGHT //-70
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //forward
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //forward

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 99 * PWM_LOAD / 100); //left
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 17 * PWM_LOAD / 100); //right slow
        //UARTprintf("************************RIGHT************************\n");
        SysCtlDelay(500);
    }
    else if ((pidRight > -20) && (pidRight < 20) && (FrontValue < 1800))// Go straight
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //forward
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //forward

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 70 * PWM_LOAD / 100);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 70 * PWM_LOAD / 100);
        //UARTprintf("Straight\n");
    }
    else if ((pidRight > -50) && (pidRight < 0) && (FrontValue > 1000) && (FrontValue < 1500))// Special straight
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //forward
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //forward

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 80 * PWM_LOAD / 100);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 80 * PWM_LOAD / 100);
        //UARTprintf("=================Straight================\n");
    }

    /*calculates error and stores in buffer */
    if ((error_count % 2) == 0) { //only runs every 100ms

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0); //reset LEDs
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //constant blue LED to signal that it's collecting data

        error = RightValue - 1900; //error = measured distance - desired distance
        //makes sure error is positive
        if (error < 0) {
            error = error * (-1);
        }
        //fills the first buffer manually once
        if (bufferCt < 20) {
            buffer[k] = error;
            ++k;
        }
        //fills the second buffer manually once
        if ((bufferCt >= 20) && (bufferCt < 40)) {
            buffer_2[m] = error;
            ++m;
        }
        if (i < BUFFER_SIZE) {
            temp_buffer[i] = error;
        }
        ++i;
        ++bufferCt;
        error_count = 0;

    }
    error_count += 1;
}

/*
 *************************************************************************************************************
 * TIMER3 INTERRUPT FUNCTION - LIGHT SENSOR
 *************************************************************************************************************
 */
void Timer3IntHandler(void) {
    /* clear timer2A interrupt flag */
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    /* light sensor config */
    uint32_t lightSensorValue = 0;
    uint32_t lightCounter = 0;
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
    SysCtlDelay(100);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    while (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) != 0) {
        lightCounter++;
    }
    lightSensorValue = lightCounter;
    /* Determine White or Black Surface */
    if (lightSensorValue < 2000) {
        //white
        /* debugging
        if (blkCounter > 1) {
            UARTprintf("\n\n====================Black count = %d====================\n\n", blkCounter);
            blkCounter = 0;
        }
        */
        if ((blkCounter > 1) && (blkCounter < 10) && (readData == 1)) {
            Clock_start(Buffer_Clk);
            readData = 0;
            blkCounter = 0;
            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); //set to output to display LEDs
            UARTprintf("\n\n*********READING DATA*********\n\n");
        }
        else if ((blkCounter > 1) && (blkCounter < 10) && (readData == 0)) {
            /*stops Buffer clock*/
            Clock_stop(Buffer_Clk);

            /*Outputs Partially filled buffer*/
            if (swap == 0) {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0); //reset LEDs
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //constant green LED to signal that it's transmitting to PC
                UARTprintf("Partial Buffer: ");
                for (j = 0; j < BUFFER_SIZE; ++j) {
                    UARTprintf("%X, ", buffer[j]);
                    if(UARTBusy(UART1_BASE)) {
                        buffer_2[j] = temp_buffer[j];
                    }
                }
                UARTprintf("\r\n\n");
            }

            else if (swap == 1) {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0); //reset
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //constant green LED to signal that it's transmitting to PC
                UARTprintf("Partial Buffer: ");
                for (j = 0; j < BUFFER_SIZE; ++j) {
                    UARTprintf("%X, ", buffer_2[j]);

                    if(UARTBusy(UART1_BASE)) {
                        buffer[j] = temp_buffer[j];
                    }
                }
                UARTprintf("\r\n\n");
            }
            i = 0;
            /*Buffer has out for the last time*/

            /*Reset Counter*/
            blkCounter = 0;

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0); //reset LEDs
            GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); //set to input to prevent LED from turning back on

            UARTprintf("\n\n!!!!!!!!!!!!!STOPPED READING!!!!!!!!!!!!!\n\n");
        }
        else if (blkCounter > 10) {
            PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, false);
            Clock_stop(PID_Clk);
            Clock_stop(Buffer_Clk);

            /*Reset Counter*/
            blkCounter = 0;

            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); //set to output to display LEDs
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0); //reset LEDs
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); //turn on red LED to signal the robot has stopped

            UARTprintf("\n\n===========RUN COMPLETED===========\n\n");
        }
    }
    else {
        //black
        ++blkCounter;
    }
    lightCounter = 0;
}

/*
 *************************************************************************************************************
 * MAIN
 *************************************************************************************************************
 */
 int main(void) {

    /* Initialize everything */
    ConfigurePeripherals();

    /* Turn off motor to prevent robot going rogue */
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, false);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0); //reset LEDs

    /* Menu */
    while(true) {

        UARTprintf("\n");
        UARTprintf("Version 10.5\n");
        UARTprintf("The following is a list of commands:\n"
                "GO - Run Maze\n");
        UARTgets(command, strlen(command) + 1);
        UARTprintf("\n");
        if (!strcmp(command, "GO"))    // Start
        {
            PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
            /* Initialize RTOS */
            BIOS_start();
        }
    }
}
