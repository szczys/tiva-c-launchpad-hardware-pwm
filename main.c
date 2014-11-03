/*################################################
# Hardware PWM proof of concept using
# the Tiva C Launchpad
#
# Started with example code by
# lawrence_jeff found here:
# http://forum.stellarisiti.com/topic/707-using-hardware-pwm-on-tiva-launchpad/
# 
# Altered to use code found on section
# 22.3 of the TivaWare Peripheral Driver
# Library User's Guide found here:
# http://www.ti.com/lit/ug/spmu298a/spmu298a.pdf
#
#
# This example pulses three on-board LEDs
#
#################################################*/


#include "driverlib/pin_map.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

void delayMS(int ms) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}

struct Servo {                      //Examples:
    uint32_t gpioPeripheral;        //SYSCTL_PERIPH_GPIOF
    uint32_t pwmModulePeripheral;   //SYSCTL_PERIPH_PWM1
    uint32_t pinConfig;             //GPIO_PF1_M1PWM5
    uint32_t portBase;              //GPIO_PORTF_BASE
    uint8_t gpioPin;                //GPIO_PIN_1
    uint32_t pwmModuleBase;         //PWM1_BASE
    uint32_t pwmGenerator;          //PWM_GEN_1 (notice "M1" in pinConfig)
    uint32_t pwmMode;               //PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC
    uint32_t period;                //period (SystemClock/PWM_Divider/#_Periods_per_second)
    uint32_t duty;                  //duty cycle (Period / (100/percent_pin_high_time))
    uint32_t pwmOut;                //PWM_OUT_5
    uint32_t pwmOutBit;             //PWM_OUT_5_BIT (notice 'PWM5' in pinConfig)
};

void pwmClockSetup(uint32_t pwmClockDivider) {
    SysCtlPWMClockSet(pwmClockDivider);
}

struct Servo setupServo(uint32_t gpioPeripheral, uint32_t pwmModulePeripheral, uint32_t pinConfig,
                    uint32_t portBase, uint8_t gpioPin, uint32_t pwmModuleBase, uint32_t pwmGenerator,
                    uint32_t pwmMode, uint32_t period, uint32_t duty, uint32_t pwmOut, uint32_t pwmOutBit) {
    
    //setup the new struct
    struct Servo s = { gpioPeripheral, pwmModulePeripheral, pinConfig, portBase, gpioPin, pwmModuleBase, pwmGenerator, pwmMode, period, duty, pwmOut, pwmOutBit };

    // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(gpioPeripheral);
    SysCtlPeripheralEnable(pwmModulePeripheral);

    //Configure pins PWM
    GPIOPinConfigure(pinConfig);
    //Configure PWM type
    GPIOPinTypePWM(portBase, gpioPin);

    //Configure PWM Options
    PWMGenConfigure(pwmModuleBase, pwmGenerator, pwmMode);

    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(pwmModuleBase, pwmGenerator, period);

    //Set PWM duty
    PWMPulseWidthSet(pwmModuleBase, pwmOut, duty);

    // Enable the PWM generator
    PWMGenEnable(pwmModuleBase, pwmGenerator);

    // Turn on the Output pins
    PWMOutputState(pwmModuleBase, pwmOutBit, true);

    //return the struct so the calling code has a reference to it
    return s;
}

void setDuty(struct Servo s, uint32_t duty) {
    s.duty = duty;
    PWMPulseWidthSet(s.pwmModuleBase, s.pwmOut, duty);
}

int main(void)
{
/*
need to know the pwm divider

each object:
    -gpio register
    -module and pwm output configuration declaration
    -port base
    -gpio pin
    -pwmbase
    -pwm generator
    -pwm mode
    -period
    -duty
    -pwmoutbit
*/
    
    //Set the clock
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //Configure PWM Clock to divide system by 64
    pwmClockSetup(SYSCTL_PWMDIV_64);

    uint32_t period = 5000;
    uint32_t duty = 5000;

    //Setup a servo on PF1
    struct Servo sPF1 = setupServo(    SYSCTL_PERIPH_GPIOF,
                                SYSCTL_PERIPH_PWM1,
                                GPIO_PF1_M1PWM5,
                                GPIO_PORTF_BASE,
                                GPIO_PIN_1,
                                PWM1_BASE,
                                PWM_GEN_2,
                                PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC,
                                period,
                                duty,
                                PWM_OUT_5,
                                PWM_OUT_5_BIT
                            );


    /*
    // Unlock the Pin PF0 and Set the Commit Bit
    // See datasheet table 10-1 for explanation of
    // why this pin needs unlocking
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR)   |= 0x01;
    */


    //Fade
    bool fadeUp = true;
    uint32_t increment = 10;


    while(1)
    {   
        delayMS(2);
        if (fadeUp) {
            duty += increment;
            if (duty >= period) { fadeUp = false; }
        }
        else {
            duty -= increment;
            if (duty <= increment) { fadeUp = true; }
        }    
        
        setDuty(sPF1,duty);

    }

}
