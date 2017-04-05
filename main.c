/**
 * Unnecessarily Redundant Care Unit for The Elderly (URCUTE)
 * 2017 (C) JUSTIN NG, LIM HONG WEI [NUS CEG2]
 * MIT License
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_rtc.h"

#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "light.h"
#include "temp.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "uart2.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/** TIMER **/
volatile uint32_t msTicks = 0;

void SysTick_Handler(void) {
   msTicks++;
}

uint32_t getTicks(void) {
   return msTicks;
}

int timesUpOrNot(uint32_t startTicks, int delayInMs) {
   return (getTicks() - startTicks) >= delayInMs;
}
/* ==================== END TIMER */

/** SEVEN SEGMENT DISPLAY **/
const uint8_t sevenSegChars[] = {
	'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'
};

/** ACCELEROMETER **/
int8_t x = 0, y = 0, z = 0, xoff = 0, yoff = 0, zoff = 0, xLast = 0, yLast = 0, zLast = 0;

/** SYS_MODE **/
typedef enum {
    MODE_STABLE, MODE_MONITOR
} system_mode_t;
volatile system_mode_t mode;

/** SAMPLE_MODE **/
typedef enum {
	PRINT, SEND
} sample_mode_t;
volatile sample_mode_t sample_mode;

/** MOVEMENT **/
typedef enum {
	MOVEMENT_NO, MOVEMENT_DETECTED
} movemen_t;
volatile movemen_t movement;

/** LIGHT **/
typedef enum {
	LIGHT_NORMAL, LIGHT_LOW
} ligh_t;
volatile ligh_t light_flag;

/** FALL **/
typedef enum {
	FALL_NO, FALL_DETECTED
} fall_t;
volatile fall_t fall;

/** EMERGENCY **/
typedef enum {
	EMER_NO, EMER_RAISED, EMER_WAIT, EMER_RESOLVED
} emergency_t;
volatile emergency_t emergency_flag;


/** MISC FLAGS **/
uint8_t cems_message_counter = 0, blink_blue = 0, blink_red = 0, monitor_firstrun = 0, state = 0, mode_button = 1;
int pageIndex = 0;

/** INIT PROTOCOLS **/
static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

 static void init_GPIO(void) {
	//Initialize button sw4
	PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 0;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 31;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(1, 1<<31, 0);

	//Initialize button sw3
	PinCfg.Funcnum = 1; // Using EINT0
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1 << 10, 0);
 }
 /* ==================== END INIT PROTOCOLS */

 /** LIGHT SENSOR CONFIG **/
const uint32_t lightLoLimit = 50, lightHiLimit = 4000;

static void config_light(void) {
	// Setup light limit for triggering interrupt
	light_setRange(LIGHT_RANGE_4000);
	light_setLoThreshold(lightLoLimit);
	light_setHiThreshold(lightHiLimit);
	light_setIrqInCycles(LIGHT_CYCLE_16); // Used 16 for 4 sec activation time.
	light_clearIrqStatus();
	LPC_GPIOINT->IO2IntClr = 1 << 5;
	LPC_GPIOINT->IO2IntEnF |= 1 << 5; // light sensor
}
/* ==================== END LIGHT SENSOR CONFIG */

/** INIT UART **/
void pinsel_uart3(void){
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
}

void init_uart(void){
    UART_CFG_Type uartCfg;
    uartCfg.Baud_rate = 115200;
    uartCfg.Databits = UART_DATABIT_8;
    uartCfg.Parity = UART_PARITY_NONE;
    uartCfg.Stopbits = UART_STOPBIT_1;
    //pin select for uart3;
    pinsel_uart3();
    //supply power & setup working parameters for uart3
    UART_Init(LPC_UART3, &uartCfg);
    //enable transmit for uart3
    UART_TxCmd(LPC_UART3, ENABLE);
}

/* ==================== END INIT UART */

/**
 * @brief Send a string to CEMS (UART).
 */
static void sendToCems(unsigned char *string) {
	// Send to Cems
//	uart2_sendString("Opening connection...");
//	uart2_sendString(string);
	UART_Send(LPC_UART3, (uint8_t *) string, strlen(string), BLOCKING);
	printf(string);
}

/** INTERRUPT HANDLERS **/

void eint_init(void) {
	// Enable EINT3 interrupt
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_SetPriority(EINT3_IRQn, 2);
	NVIC_EnableIRQ(EINT3_IRQn);

	// Enable EINT0 interrupt with SW3
	LPC_SC->EXTINT = 1; // Clear existing interrupts
	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_SetPriority(EINT0_IRQn, 1);
	NVIC_EnableIRQ(EINT0_IRQn);
}

// EINT3 Interrupt Handler
void EINT3_IRQHandler(void) {
	// Determine whether GPIO Interrupt P2.5 has occurred (LIGHT SENSOR)
	if ((LPC_GPIOINT->IO2IntStatF>>5)& 0x1) {
		light_flag = LIGHT_LOW;
		LPC_GPIOINT->IO2IntClr = (1<<5);
	}
}

// EINT0 Interrupt Handler (for SW3)
void EINT0_IRQHandler(void)
{
	if (mode == MODE_MONITOR && emergency_flag == EMER_NO) {
		emergency_flag = EMER_RAISED;
	}
    // Clear EINT0
    LPC_SC->EXTINT = (1<<0);
}

/* ==================== END INTERRUPT HANDLERS */

/** MOVEMENT LOGIC **/
void isMovementInDarkDetected(uint8_t threshold) {
	if (sqrt(x*x+y*y+z*z) >= threshold && light_flag == LIGHT_LOW) {
		blink_blue = 1;
	}
}

/** FALL LOGIC **/
void isFallDetected(uint8_t threshold) {
	if (abs(y - yLast) >= threshold) {
		fall = FALL_DETECTED;
	}
}

static void sampleAcc() {
	acc_read(&x, &y, &z);
	x = x + xoff;
	y = y + yoff;
	z = z + zoff;
	isMovementInDarkDetected(10);
	isFallDetected(30);
}
/* ==================== END MOVEMENT LOGIC */

/**
 * @brief Samples the environment for temp, lux and acc, then prints to OLED. If SEND is specified, send to UART.
 */
static void sampleEnvironmentAnd(sample_mode_t sample_mode) {
	unsigned char temp[100] = "", lux[100] = "", acc[100] = "", warning[100] = "";

	float temperature  = temp_read()/10.0;
	if (temperature >= 25)
		blink_red = 1;

	int light = light_read();

	if (sample_mode == SEND) {
		// NNN_-_T*****_L*****_AX*****_AY*****_AZ*****\r\n
		unsigned char string[100] = "";
		sprintf(string, "%03d_-_T%05.1f_L%05d_AX%05d_AY%05d_AZ%05d\r\n",cems_message_counter++,temperature,light,x,y,z);
		if (blink_red)
			sendToCems("Fire was Detected.\r\n");
		if (blink_blue)
			sendToCems("Movement in darkness was Detected.\r\n");
		if (fall == FALL_DETECTED)
			sendToCems("Serious fall was Detected.\r\n");
		sendToCems(string);
	}

	// Only print values if page is at 0.
	if (pageIndex == 0) {
		sprintf(temp, "%.1f degrees C\r", temperature);
		sprintf(lux, "%d lux", light);
		sprintf(acc, "%dx, %dy, %dz\r", x,y,z);
		sprintf(warning, "%s%c%s", blink_red?"FIRE!":"", blink_red&&blink_blue?"":"", blink_blue?"DARK!":"");
		oled_clearScreen(OLED_COLOR_BLACK);
		oled_putString(0,0, "MONITOR", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0,10, warning, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0,20, temp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0,30, lux, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0,40, acc, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		if (emergency_flag == EMER_RESOLVED) {
			oled_putString(0,55, "HELP IS COMING.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		} else if (emergency_flag == EMER_WAIT) {
			oled_putString(0,55, "HELP REQUESTED.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		}
	}
}

static void changePage(uint8_t joyState)
{
    int maxIndex = 3;
	oled_clearScreen(OLED_COLOR_BLACK);
    // Reset
    if ((joyState & JOYSTICK_CENTER) != 0) {
    	pageIndex = 0;
    } else if ((joyState & JOYSTICK_RIGHT) != 0) {
        pageIndex = (pageIndex+1)%3;
    } else if ((joyState & JOYSTICK_LEFT) != 0) {
    	pageIndex = (pageIndex+5)%3;
    }

    if (pageIndex == 0) {
    	sampleEnvironmentAnd(PRINT);
    } else if (pageIndex == 1) {
    	oled_putString(0,10,"URCUTE v0.1", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    	oled_putString(0,30,"For licensing", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    	oled_putString(0,40,"opportunities", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    	oled_putString(0,50,"contact Arcana.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    } else if (pageIndex == 2) {
    	char time[20] = "";
    	sprintf(time, "Time: %d:%d:%d", RTC_GetTime(LPC_RTC, RTC_TIMETYPE_HOUR),
    			RTC_GetTime(LPC_RTC, RTC_TIMETYPE_MINUTE),
				RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND));
    	oled_putString(0,30,time, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    }
}

int main (void) {
    SysTick_Config(SystemCoreClock / 1000);
    init_i2c();
    init_ssp();
	init_GPIO();

	acc_init();
	acc_read(&x, &y, &z);
	xoff = 0 - x; // Calculate offsets
	yoff = 0 - y;
	zoff = 0 - z;

	light_enable();
	config_light();

	temp_init(getTicks);

    pca9532_init();
    joystick_init();
    rgb_init(); // Since green LED interferes with OLED, do not use.
    oled_init();
    led7seg_init();
    init_uart();
    RTC_Init(LPC_RTC);
    eint_init();

    uint8_t train = 1, data = 0;
    uint32_t startTicks = getTicks();
    uint32_t trainTime = startTicks, redLightTime= startTicks, rgbTime= startTicks, blueLightTime= startTicks, sevenSegTime= startTicks, emergencyTime = startTicks;

    oled_clearScreen(OLED_COLOR_BLACK); // Initial clear screen

    while (1) {
    	// Local mode switching by polling SW4
    	mode_button = GPIO_ReadValue(1) >> 31 & 0x01;
    	if (mode_button == 0) {
        	if (mode == MODE_STABLE) {
        		monitor_firstrun = 1;
        		mode = MODE_MONITOR;
        	} else if (mode == MODE_MONITOR) {
        		printf("Entering STABLE mode.\n");
        		mode = MODE_STABLE;
        	}
        	mode_button = 1;
    	}

	    // Wireless mode switching
	    UART_Receive(LPC_UART3, &data, 1, NONE_BLOCKING);
    	if (data == 's') {
	    	mode = MODE_STABLE;
	    	data = 0;
	    	sendToCems("\r\nEntering STABLE mode by CEMS.\r\n");
	    } else if (data == 'm') {
	    	mode = MODE_MONITOR;
	    	data = 0;
	    	sendToCems("\r\nEntering MONITOR mode by CEMS.\r\n");
	    } else if (data == 'e' && mode == MODE_MONITOR && emergency_flag == EMER_WAIT) {
	    	emergency_flag = EMER_RESOLVED;
	    	data = 0;
	    	sendToCems("\r\nCEMS responding to emergency...\r\n");
	    }

    	switch (mode) {
    	case MODE_STABLE:
    		led7seg_setChar('{',FALSE); // Clear 7SEG display
    		blink_red = 0;	// Clear blink red flag
    		blink_blue = 0;	// Clear blink blue flag
    		light_flag = LIGHT_NORMAL; // Absolve light warning
    		movement = MOVEMENT_NO; // Absolve movement
    		emergency_flag = EMER_NO; // Clear emergency
    		pca9532_setLeds(0, 0xffff); // Clear floodlights
            GPIO_ClearValue( 2, 1); // Clear red
            GPIO_ClearValue( 0, (1<<26) ); // Clear blue
    		oled_clearScreen(OLED_COLOR_BLACK); // cls
    		break;

    	/**
    	 * MONITOR MODE: In monitor mode,
    	 * 1) Time is tracked and output in BASE 16 to the seven segment display.
    	 * 2) Acceleration is sampled every second
    	 * 3) Environment vars (Temperature, Light, Movement) is sampled and sent to UART every 5 seconds.
    	 * 4) Monitors for movement in darkness (interrupt) or fire (through sample).
    	 *
    	 * # Assumes that the time tracked on SSD is reflective of real time.
    	 *
    	 * Enhancements:
    	 * 1) Emergency override
    	 */
    	case MODE_MONITOR:
    		if (monitor_firstrun) {
        		oled_putString(0,0, "MONITOR", OLED_COLOR_WHITE, OLED_COLOR_BLACK); // Set MONITOR on graphics display
    			sendToCems("Entering MONITOR mode.\r\n");
    			monitor_firstrun = 0;
    		}

    		if (emergency_flag == EMER_RAISED) {
    			sendToCems("[MANUAL OVERRIDE] EMERGENCY ASSISTANCE REQUESTED!\r\n");
    			emergency_flag = EMER_WAIT;
    		} else if (emergency_flag == EMER_WAIT) {
    			oled_putString(0,55, "HELP REQUESTED.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    		}

    		// PRINT PAGE NUMBER
            unsigned char index[1] = "";
            sprintf(index, "%d", pageIndex);
            oled_putString(50,0,index, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    		if (timesUpOrNot(sevenSegTime, 1000)) { // 1s period
            	// Sample acceleration
            	sampleAcc();
            	// SSEG display increments by 1 per second (SSP/SSI)
            	static int cycler = 16;
                led7seg_setChar(sevenSegChars[++cycler%16], FALSE);
                // Sample the environment every 5 seconds.
            	if (cycler%16 == 5 || cycler%16 == 10) {
            		sampleEnvironmentAnd(PRINT);
            	}
            	if (cycler%16 == 15) {
            		// Send samples to SEMS
            		sampleEnvironmentAnd(SEND);
            	}
                sevenSegTime = getTicks();
            }

            // Blink both LEDs for fire detection and movement in the dark (synchronous).
            if (blink_red == 1 && blink_blue == 1) {
                if (timesUpOrNot(rgbTime, 333)) {
                	pca9532_setLeds(0xffff, 0); // Flash floodlights on
                	GPIO_SetValue( 2, 1); // Red first
                	GPIO_ClearValue( 0, (1<<26)); // Clear blue
                }
                if (timesUpOrNot(rgbTime, 666)) {
                	pca9532_setLeds(0, 0xffff); // Flash floodlights off
                	GPIO_ClearValue( 2, 1); // Clear red
                    GPIO_SetValue( 0, (1<<26)); // Set blue
                    rgbTime = getTicks();
            	}
            } else if (blink_blue == 1) { // Blink BLUE LED for movement in the dark.
            	pca9532_setLeds(0xffff, 0); // On floodlights
            	if (timesUpOrNot(rgbTime, 333))
                	GPIO_SetValue( 0, (1<<26));
                if (timesUpOrNot(rgbTime, 666)) {
                	GPIO_ClearValue( 0, (1<<26));
                	rgbTime = getTicks();
                }
            } else if (blink_red == 1) { // Blink RED LED for fire detection.
                if (timesUpOrNot(rgbTime, 333))
                	GPIO_SetValue( 2, 1);
                if (timesUpOrNot(rgbTime, 666)) {
                	GPIO_ClearValue( 2, 1);
                	rgbTime = getTicks();
                }
            }

            // Joystick page control
            if (timesUpOrNot(trainTime, 250)) {
            	state = joystick_read();
            	if (state != 0) {
            		changePage(state);
            	}
            	trainTime = getTicks();
            }

//	    if (timesUpOrNot(emergencyTime, 1000)) {
//            emergency_button = GPIO_ReadValue(1) >> 31 & 0x01;
//            if (emergency_button == 0 && emergency_flag == EMER_NO) {
//            	pca9532_setBlink0Period(151);
//            	pca9532_setBlink0Duty(100);
//            	pca9532_setBlink0Leds(0xffff); // Check this function
//            	sendToCems("[MANUAL OVERRIDE] EMERGENCY ASSISTANCE REQUESTED!\r\n");
//        		oled_putString(0,55, "HELP REQUESTED.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
//            	emergency_flag = EMER_RAISED;
//            	emergency_button = 1;
//            }
//	    }

//        // 4. Allow the speaker (GPIO) to toggle between making a continuous buzzing sound, and no buzzing sound, when SW4 (GPIO) is pressed.
//        // Note: Avoid loud and frustrating sounds
//        // Includes debouncing
//        if (timesUpOrNot(buttonTime, 500)) {
//        	btn1 = (GPIO_ReadValue(1) >> 31) & 0x01;
//        	if (btn1 == 0) {
//        		buzzState = !buzzState;
//        	}
//        	buttonTime = getTicks();
//        }
//    	if (buzzState) {
//    		buzz();
//    	}
//
//    	// 4. Read the accelerometer (I2C) in polling mode
//    	if (timesUpOrNot(accTime, 1000)) {
//    		acc_read(&x, &y, &z);
//    		x = x+xoff;
//    		y = y+yoff;
//    		z = z+zoff;
//    		uint8_t string[] = {x,'x',y,'y',z,'z'};
//    		oled_putString(0,10, string, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
//    		accTime = getTicks();
//    	}
//
//    	// 5. Read the light sensor (I2C) in polling mode
//    	if (timesUpOrNot(lightTime, 1000)) {
//    	    		int light = light_read();
//    	    		oled_putString(0,20,light, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
//    	    		lightTime = getTicks();
//    	}
//
//    	// 6. Read the temperature sensor (GPIO) in polling mode
//    	if (timesUpOrNot(tempTime, 100000)) {
//    	    		oled_putString(0,30,temp_read(), OLED_COLOR_WHITE, OLED_COLOR_BLACK);
//    	    		tempTime = getTicks();
//    	}
//
//    	// 7. Display required words on the 96x64 White OLED (SPI/SSP)
//    	if (timesUpOrNot(guiTime, 100)) {
//    		oled_putString(0,0, "MONITOR", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
//    		guiTime = getTicks();
//    	}
        break;
    	}
    }
}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}

