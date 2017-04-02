/*
 * Project: Aerial Platform for Overland Haul and Import System (APOPHIS)
 *
 *  Created On: Mar 28, 2017
 *  Last Updated:
 *      Author(s): Brandon Klefman
 *
 *      Purpose: Software for testing servo actuation.
 *
 */

//*****************************************************************************
//
// Includes
//
//*****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "inc/hw_memmap.h"

#include "driverlib/adc.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "misc/buttons.h"

#include "utils/uartstdio.h"

#include "initializations.h"
#include "APOPHIS_pin_map.h"

//*****************************************************************************
//
// Defines
//
//*****************************************************************************

#define SPEEDIS120MHZ true

//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
void SysTickIntHandler(void);
void ConsoleIntHandler(void);
void TurnOnLED(uint32_t LEDNum);
void TurnOffLED(uint32_t LEDNum);
void Menu(char CharReceived);
void WaitForButtonPress(uint8_t ButtonState);

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************

bool g_LEDON = false;

//
// Counter for Systick printing at 2 Hz instead of 12 Hz.
uint32_t g_SysTickCount = 0;

//
// Variable to store when USER LED 4 is on.
bool g_LED4On = false;

//
// Variable to store the system clock speed.
uint32_t g_SysClockSpeed;

//
// Variable to store characters received from the PC.
char g_CharConsole;

//
// Variable to trigger evaluation of received character from PC in the main
// function.
bool g_ConsoleFlag = false;

//
// Variable to indicate end of program.
bool g_Quit = false;

//
// Variable used to track actual update rate of radio.
uint32_t g_RadioCount = 0;

/*
 * Throttle Values
 */
uint32_t g_StartPosition = 0;
uint32_t g_EndPosition = 0;
uint32_t g_ui32AngleIncrement = 0;

//
// Actual angle of the servo.
uint32_t g_ui32ServoAngle = 0;

//*****************************************************************************
//
// Start of program.
//
//*****************************************************************************
int main(void) {
	float speed = 0.0f;

	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	FPUEnable();
	FPULazyStackingEnable();

	//
	// Set the clocking to run at 120 MHz.
#if SPEEDIS120MHZ
	g_SysClockSpeed = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
	SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);
#else
	g_SysClockSpeed = SysCtlClockFreqSet(SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ, 16000000);
#endif

	//
	// Disable interrupts during initialization period.
	IntMasterDisable();

	//
	// Before doing anything, initialize the LED.
	InitLED(g_SysClockSpeed);

	//
	// Turn off all LEDs, in case one was left on.
	TurnOffLED(5);

	//
	// Initialization has begun. Turn on LED 1.
	TurnOnLED(1);

	//
	// Initialize the buttons.
	ButtonsInit();

	//
	// Initialize the Console.
	InitConsole();
	UARTprintf("Clock speed: %d\r\n", g_SysClockSpeed);

	//
	// Initialize the air motors.
	speed = InitServoMtrs(g_SysClockSpeed);

    //
    // Calculate zero position corresponding to a 2.5% duty cycle.
    g_StartPosition = (uint32_t)(speed * 0.025);

    //
    // Calculate end position corresponding to a 12.5% duty cycle.
    g_EndPosition = g_StartPosition * 5;
    g_ui32AngleIncrement = (g_EndPosition - g_StartPosition) / 100;

	//
	// Initialize the throttle of the system.
	g_ui32ServoAngle = g_StartPosition;

	//
	// Before starting program, wait for a button press on either switch.
	UARTprintf("Initialization Complete!\r\nPress left button to start.\r\n");

	TurnOnLED(5);

	WaitForButtonPress(LEFT_BUTTON);

	TurnOffLED(5);

	//
	// Turn off LED1, and enable the systick at 12 Hz to
	// blink LED 4, signifying regular operation.
	// The Systick cannot handle any value larger than 16MHz.
	TurnOffLED(1);
	SysTickPeriodSet(g_SysClockSpeed / 12);
	SysTickEnable();

	//
	// Activate the motors for APOPHIS.
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);

	//
	// Print menu.
	Menu('M');

	//
	// Initialization complete. Enable interrupts.
	IntMasterEnable();

	//
	// Program start.
	while (!g_Quit) {
		//
		// First check for commands from Console.
		if (g_ConsoleFlag)
			Menu(g_CharConsole);
	}

	//
	// Kill the motor.
	Menu('x');

	//
	// Program ending. Do any clean up that's needed.
	UARTprintf("Dave, I'm scared. Will I dream?\r\n");

	TurnOffLED(5);

	IntMasterDisable();

	return 0;
}

/*
 * Interrupt handlers go here:
 */
//*****************************************************************************
//
// Interrupt handler for the console communication with the PC.
//
//*****************************************************************************
void SysTickIntHandler(void) {

	if (g_SysTickCount >= 5) {
		if (g_LED4On) {
			//
			// Turn off LED 4 if it is on.
			TurnOffLED(4);

			g_LED4On = false;
		} else {
			//
			// Otherwise turn it on.
			TurnOnLED(4);

			g_LED4On = true;
		}

		//
		// Reset SysTick Count.
		g_SysTickCount = 0;
	} else
		g_SysTickCount++;
}

//*****************************************************************************
//
// Interrupt handler for the console communication with the PC.
//
//*****************************************************************************
void ConsoleIntHandler(void) {
	//
	// First get the interrupt status. Then clear the associated interrupt flag.
	uint32_t ui32Status = UARTIntStatus(CONSOLE_UART, true);
	UARTIntClear(CONSOLE_UART, ui32Status);

	//
	// Get the character sent from the PC.
	g_CharConsole = UARTCharGetNonBlocking(CONSOLE_UART);

	//
	// Echo back to Radio.
	UARTCharPutNonBlocking(CONSOLE_UART, g_CharConsole);

	//
	// Call the menu function to handle the character received.
	Menu(g_CharConsole);
}

/*
 * Other functions used by main.
 */
//*****************************************************************************
//
// This function will turn on the associated LED.
// Parameter: LEDNum - the desired LED number to turn on.
//
//*****************************************************************************
void TurnOnLED(uint32_t LEDNum) {
	//
	// Turn on the associated LED number.
	switch (LEDNum) {
	case 1: // Turn on User LED 1
	{
		GPIOPinWrite(LED_PORT1, LED1_PIN, LED1_PIN);
		return;
	}
	case 2: // Turn on User LED 2
	{
		GPIOPinWrite(LED_PORT1, LED2_PIN, LED2_PIN);
		return;
	}
	case 3: // Turn on User LED 3
	{
		GPIOPinWrite(LED_PORT2, LED3_PIN, LED3_PIN);
		return;
	}
	case 4: // Turn on User LED 4
	{
		GPIOPinWrite(LED_PORT2, LED4_PIN, LED4_PIN);
		return;
	}
	default: // Turn on all LEDs.
	{
		GPIOPinWrite(LED_PORT1, LED1_PIN | LED2_PIN, LED1_PIN | LED2_PIN);
		GPIOPinWrite(LED_PORT2, LED3_PIN | LED4_PIN, LED3_PIN | LED4_PIN);
		return;
	}
	}
}

//*****************************************************************************
//
// This function will turn off the associated LED.
// Parameter: LEDNum - the desired LED number to turn off.
//
//*****************************************************************************
void TurnOffLED(uint32_t LEDNum) {
	//
	// Turn on the associated LED number.
	switch (LEDNum) {
	case 1: // Turn on User LED 1
	{
		GPIOPinWrite(LED_PORT1, LED1_PIN, 0x00);
		return;
	}
	case 2: // Turn on User LED 2
	{
		GPIOPinWrite(LED_PORT1, LED2_PIN, 0x00);
		return;
	}
	case 3: // Turn on User LED 3
	{
		GPIOPinWrite(LED_PORT2, LED3_PIN, 0x00);
		return;
	}
	case 4: // Turn on User LED 4
	{
		GPIOPinWrite(LED_PORT2, LED4_PIN, 0x00);
		return;
	}
	default: // Turn off all LEDs.
	{
		GPIOPinWrite(LED_PORT1, LED1_PIN | LED2_PIN, 0x00);
		GPIOPinWrite(LED_PORT2, LED3_PIN | LED4_PIN, 0x00);
		return;
	}
	}
}

//*****************************************************************************
//
// This function will stop the current program run in order to wait for a
// button press.
//
// desiredButtonState: One of three values, LEFT_BUTTON, RIGHT_BUTTON or ALL_BUTTONS.
//
//*****************************************************************************
void WaitForButtonPress(uint8_t desiredButtonState) {
	uint8_t actualButtonState;
	uint8_t rawButtonState;
	uint8_t *pRawButtonState = &rawButtonState;
	uint8_t delta;
	uint8_t *pDelta = &delta;

	//
	// Get the state of the buttons.
	actualButtonState = ButtonsPoll(pDelta, pRawButtonState);

	if (desiredButtonState == LEFT_BUTTON) {
		while (actualButtonState != LEFT_BUTTON) {
			actualButtonState = ButtonsPoll(pDelta,
					pRawButtonState) & LEFT_BUTTON;
		}
		return;
	} else if (desiredButtonState == RIGHT_BUTTON) {
		while (actualButtonState != RIGHT_BUTTON) {
			actualButtonState = ButtonsPoll(pDelta,
					pRawButtonState) & RIGHT_BUTTON;
		}
		return;
	} else if (desiredButtonState == ALL_BUTTONS) {
		while (actualButtonState != ALL_BUTTONS) {
			actualButtonState = ButtonsPoll(pDelta,
					pRawButtonState) & ALL_BUTTONS;
		}
		return;
	}
}

//*****************************************************************************
//
// This function will handle analysis of characters received from the console.
//
//*****************************************************************************
void Menu(char charReceived) {
	//
	// Check the character received.
	switch (charReceived) {
	case 'Q': // Quit the program
	{
		g_Quit = true;
		break;
	}
	case 'M': // Print Menu.
	{
		UARTprintf("Menu:\r\nM - Print this menu.\r\n");
		UARTprintf("Q - Quit this program.\r\n");
		UARTprintf("w - Increase servo angle (CW).\r\n");
		UARTprintf("s - Decrease servo angle (CCW).\r\n");
        UARTprintf("x - Reset the servo angle.\r\n");
		UARTprintf("e - Set servo angle furthest CCW.\r\n");
		break;
	}
	case 'w': // Increase angle of servo motors.
	{
	    g_ui32ServoAngle += g_ui32AngleIncrement;
		PWMPulseWidthSet(PWM0_BASE, SERVO_1, g_ui32ServoAngle);
		PWMPulseWidthSet(PWM0_BASE, SERVO_2, g_ui32ServoAngle);

		UARTprintf("Angle Increase: %d\r\n", g_ui32ServoAngle);
		break;
	}
	case 's': // Decrease angle of servo motors.
	{
	    g_ui32ServoAngle -= g_ui32AngleIncrement;

		PWMPulseWidthSet(PWM0_BASE, SERVO_1, g_ui32ServoAngle);
		PWMPulseWidthSet(PWM0_BASE, SERVO_2, g_ui32ServoAngle);

		UARTprintf("Angle Decrease: %d\r\n", g_ui32ServoAngle);
		break;
	}
	case 'x': // return to starting position.
	{
	    g_ui32ServoAngle = g_StartPosition;

		PWMPulseWidthSet(PWM0_BASE, SERVO_1, g_ui32ServoAngle);
		PWMPulseWidthSet(PWM0_BASE, SERVO_2, g_ui32ServoAngle);

		UARTprintf("Start Position: %d\r\n", g_ui32ServoAngle);
		break;
	}
	case 'e': // send to end position.
	{
	    g_ui32ServoAngle = g_EndPosition;

		PWMPulseWidthSet(PWM0_BASE, SERVO_1, g_ui32ServoAngle);
		PWMPulseWidthSet(PWM0_BASE, SERVO_2, g_ui32ServoAngle);

		UARTprintf("End Position: %d\r\n", g_ui32ServoAngle);
		break;
	}
	}

	//
	// Reset the flag.
	g_ConsoleFlag = false;
}
