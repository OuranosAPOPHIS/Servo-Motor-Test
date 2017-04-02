/*
 * initializations.c
 *
 *  Created on: Jan 20, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Function definitions for all initializations of all peripherals.
 *      This will aid in simplicity for the main function.
 */

//*****************************************************************************
//
// Includes
//
//*****************************************************************************
#include "APOPHIS_pin_map.h"
#include "initializations.h"

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#define APOPHIS false
#define CLOCK_PIOSC 16000000

//*****************************************************************************
//
// External Function Declarations for IntRegister() functions.
//
//*****************************************************************************
extern void SysTickIntHandler(void);
extern void ConsoleIntHandler(void);

/*
 * LED Initialization function.
 */
//*****************************************************************************
//
// This function will initialize the 4 User LED pins. They are pins PN0, PN1
// PF0, and PF4.
//
//*****************************************************************************
void InitLED(uint32_t SysClockSpeed) {
	//
	// Initialize the GPIO port for the LEDs.
	SysCtlPeripheralEnable(LED_GPIO_PERIPH1);
	SysCtlPeripheralEnable(LED_GPIO_PERIPH2);

	//
	// Configure the pins as output pins.
	GPIOPinTypeGPIOOutput(LED_PORT1, LED1_PIN | LED2_PIN);
	GPIOPinTypeGPIOOutput(LED_PORT2, LED3_PIN | LED4_PIN);

	//
	// Initialize a 1 second SysTick for blinking the LED pin 4 to indicate
	// program running.
	SysTickPeriodSet(SysClockSpeed);

	//
	// Register the interrupt handler for blinking the LED and enable it.
	SysTickIntRegister(SysTickIntHandler);
	SysTickIntEnable();
}

/*
 * UART Initialization Functions
 */
//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the program is running.
//
//*****************************************************************************
void InitConsole(void) {
	//
	// Enable GPIO port A which is used for UART0 pins.
	SysCtlPeripheralEnable(CONSOLE_GPIO_PERIPH);

	//
	// Configure the pin muxing for UART0 functions on port A0 and A1.
	GPIOPinConfigure(CONSOLE_CONFIG_PINRX);
	GPIOPinConfigure(CONSOLE_CONFIG_PINTX);

	//
	// Enable UART0 so that we can configure the clock.
	SysCtlPeripheralEnable(CONSOLE_PERIPH);

	//
	// Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(CONSOLE_UART, UART_CLOCK_PIOSC);

	//
	// Select the alternate (UART) function for these pins.
	GPIOPinTypeUART(CONSOLE_PORT, CONSOLE_PINRX | CONSOLE_PINTX);

	//
	// Initialize the UART for console I/O.
	UARTStdioConfig(0, 115200, CLOCK_PIOSC);

	//
	// Enable the UART interrupt.
	IntEnable(CONSOLE_INT);
	UARTIntEnable(CONSOLE_UART, UART_INT_RX | UART_INT_RT);
	UARTIntRegister(CONSOLE_UART, ConsoleIntHandler);
}

/*
 * Initialization for the PWM module 0 for the
 * servo motors. PWM M0 pins 5-6 will be used on pins PG1 and PK4.
 * Returns: the speed of the PWM pulse in clock cycles.
 */
uint32_t InitServoMtrs(uint32_t sysClockSpeed) {
	float speed = 0.0f;

	UARTprintf("Initializing servo motors...\n\r");

	SysCtlPeripheralEnable(SERVO_GPIO_PERIPH);

	//
	// Wait for the Peripheral to be ready for programming
	while (!SysCtlPeripheralReady(SERVO_GPIO_PERIPH))
		;

	//
	// Turn on the peripherals for the PWM.
	SysCtlPeripheralEnable(SERVO_PERIPHERAL);

	//
	// Configure the GPIO pins.
	GPIOPinConfigure(GPIO_PK4_M0PWM6);
	GPIOPinConfigure(GPIO_PK5_M0PWM7);

	GPIOPinTypePWM(SERVO_GPIO_PORT1, SERVO_GPIO_1 | SERVO_GPIO_2);

	PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_64);

	//
	// Frequency of PWM.
	speed = (sysClockSpeed / 64 / SERVO_FREQUENCY);

	//
	// Configure the PWM generator for modules 6 and 7.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, speed);

	uint32_t stuff = PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3);

	UARTprintf("PWM generator period: %d\r\n", stuff);

	//
	// Initialize pulse to 10%
	PWMPulseWidthSet(PWM0_BASE, SERVO_1, (uint32_t)(speed * 0.025));
	PWMPulseWidthSet(PWM0_BASE, SERVO_2, (uint32_t)(speed * 0.025));

	//
	// Set the output PWM modules.
	PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);

	UARTprintf("Done!\n\r");

	return speed;
}
