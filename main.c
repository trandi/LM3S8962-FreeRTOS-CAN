// optimized string manipulation
#include "utils/ustdlib.h"
#include <stdlib.h>

// include the StellarisWare directory on the compiler path (Build / GNU Compiler / Directories)
#include "inc/hw_memmap.h" // for the base address of the memories and peripherals.
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"

#include "drivers/rit128x96x4.h"     // OLED functions

// include the StellarisWare/third_party/FreeRTOS/Source directory on the compiler path (Build / GNU Compiler / Directories)
#include "include/FreeRTOS.h"
#include "include/task.h"
#include "include/queue.h"
#include "include/semphr.h"


#define LED GPIO_PIN_0
// Not the same as CAN ID, which is inside the message
#define CAN_MSG_OBJ_ID_RX 1
#define CAN_MSG_OBJ_ID_TX 2
//CANStatusGet() returns values under 0x00000080 (see can.h), so use stuff outside that range
#define CAN_UNKNOWN_INT 0x00000160


volatile unsigned long _ulCANStatusFlag = CAN_STATUS_LEC_NONE;
volatile unsigned char _ucCANNewMsgRX = 0; // tried to use _ulCANMsgRXCount to also indicate new msg, but caused many issues (maybe longs are not updated atomically ?)
volatile unsigned long _ulCANMsgRXCount = 0;
volatile unsigned long _ulCANMsgTXCount = 0;
volatile unsigned char _ucUARTNewCharRX = 0;


// guarding the RIT128x96x4 OLED display
xSemaphoreHandle _xSemDisplay = NULL;
char _cBuff[21]; // Buffer for display. 1 row has max 21 chars
void display(unsigned long ulX, unsigned long ulY, unsigned char ucLevel, const char *pcStr, ...) {
	if(xSemaphoreTake(_xSemDisplay, 1000000)) {
		va_list vaArgP;
		// Start the varargs processing.
		va_start(vaArgP, pcStr);

		uvsnprintf(_cBuff, 0xffff, pcStr, vaArgP);
		RIT128x96x4StringDraw((const char *)&_cBuff, ulX, ulY, ucLevel);

		// End the varargs processing.
		va_end(vaArgP);

		if( ! xSemaphoreGive(_xSemDisplay) ) {
			// We would not expect this call to fail because we must have obtained the semaphore to get here.
			RIT128x96x4StringDraw("ERR GIVE Sem Disp", 0, 0, 15);
			exit(1);
		}
	} else {
		RIT128x96x4StringDraw("ERR TAKE Sem Disp", 0, 0, 15);
		exit(1);
	}
}


void sendUART(const char *msg) {
	while(*msg != '\0') {
		UARTCharPut(UART0_BASE, *msg++);
	}
}

void initUART(void) {
    // The UART itself needs to be enabled, as well as the GPIO port containing the pins that will be used.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable processor interrupts.
    //IntMasterEnable();

    // Set GPIO A0 and A1 as UART pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 38400,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Enable the UART interrupt.
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

// guarding the shared CAN Msg object
xSemaphoreHandle _xSemCAN = NULL;
tCANMsgObject _sCANMsgTX;
unsigned char _ucCANMsgData[8];
void initCAN(void) {
	// Config relevant pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeCAN(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Enable the CAN controller
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    // Reset the state of all message object and of the CAN module to a known state
    CANInit(CAN0_BASE);
    // Clock rate to the CAN controller is fixed at 8MHz for this class of device and the bit rate is set to 250KHz.
    CANBitRateSet(CAN0_BASE, 8000000, 500000);
    // Enable interrupts from CAN controller.
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    // Enable interrupts for the CAN in the NVIC.
    IntEnable(INT_CAN0);
    // Take the CAN0 device out of INIT state.
    CANEnable(CAN0_BASE);


    //*** CAN RX config - Initialize a message object to be used for receiving CAN messages
    tCANMsgObject sCANMsgRXInit;
    // To receive any CAN ID, the ID and mask must both be set to 0, and the ID filter enabled.
    sCANMsgRXInit.ulMsgID = 0;                        // CAN msg ID - 0 for any
    sCANMsgRXInit.ulMsgIDMask = 0;                    // mask is 0 for any ID
    sCANMsgRXInit.ulFlags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sCANMsgRXInit.ulMsgLen = 8;                       // allow up to 8 bytes

    // Load the message object into the CAN peripheral.
    // Once loaded the CAN will receive any message on the bus, and an interrupt will occur.
    // Use message object objectID for receiving messages (this is not the same as the CAN ID which can be any value in this example).
    CANMessageSet(CAN0_BASE, CAN_MSG_OBJ_ID_RX, &sCANMsgRXInit, MSG_OBJ_TYPE_RX);


	/* Initialize shared CAN resources */
    _sCANMsgTX.pucMsgData = _ucCANMsgData;

	_xSemCAN = xSemaphoreCreateRecursiveMutex();
	if(_xSemCAN == NULL){
		RIT128x96x4StringDraw("ERR CR Sem CAN", 0, 0, 15);
		exit(1);
	}
}



void sendCANShared(void) {
	if(xSemaphoreTakeRecursive(_xSemCAN, 1000000)) {
	    _sCANMsgTX.ulMsgIDMask = 0;                    // no mask needed for TX
	    _sCANMsgTX.ulFlags = MSG_OBJ_TX_INT_ENABLE;    // enable interrupt on TX

		CANMessageSet(CAN0_BASE, CAN_MSG_OBJ_ID_TX, &_sCANMsgTX, MSG_OBJ_TYPE_TX);

	    display(0, 70, 10, "%3x #%3x%3x%3x%3x", _sCANMsgTX.ulMsgID, _sCANMsgTX.pucMsgData[0], _sCANMsgTX.pucMsgData[1], _sCANMsgTX.pucMsgData[2], _sCANMsgTX.pucMsgData[3]);
	    display(0, 80, 10, "     %3x%3x%3x%3x", _sCANMsgTX.pucMsgData[4], _sCANMsgTX.pucMsgData[5], _sCANMsgTX.pucMsgData[6], _sCANMsgTX.pucMsgData[7]);

	    xSemaphoreGiveRecursive(_xSemCAN);
	}
}


void sendCAN(unsigned long ulID, unsigned char ucMsgLength, unsigned char ucData0, unsigned char ucData1, unsigned char ucData2,
		unsigned char ucData3, unsigned char ucData4, unsigned char ucData5, unsigned char ucData6, unsigned char ucData7)
{
	if(xSemaphoreTakeRecursive(_xSemCAN, 1000000)) {
		_sCANMsgTX.ulMsgID = ulID;
		_sCANMsgTX.ulMsgLen = ucMsgLength;
		_ucCANMsgData[0] = ucData0;
		_ucCANMsgData[1] = ucData1;
		_ucCANMsgData[2] = ucData2;
		_ucCANMsgData[3] = ucData3;
		_ucCANMsgData[4] = ucData4;
		_ucCANMsgData[5] = ucData5;
		_ucCANMsgData[6] = ucData6;
		_ucCANMsgData[7] = ucData7;

		sendCANShared();

		xSemaphoreGiveRecursive(_xSemCAN);
	}
}


void initBoard(void) {
	/* Set the clocking to run from PLL at 50 MHz */
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);

	/* Configure Status LED */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED);

	/* Initialize OLED Screen */
	RIT128x96x4Init(1000000);
	RIT128x96x4Clear();
	_xSemDisplay = xSemaphoreCreateMutex();
	if(_xSemDisplay == NULL){
		RIT128x96x4StringDraw("ERR CR Sem Disp", 0, 0, 15);
		exit(1);
	}
}


// this should be protected by a semaphore, however only the UART task writes it and the CAN one reads it
// we can afford if the 1st message is sent with an out of sync message
unsigned long _ulCANMsg_UART[10];
void taskUART(void* pdata) {
	char c;
	char ucMsgData[5];
	unsigned long ulCANData[10];
	unsigned char ucMsgPos = 0;
	unsigned char ucCANPos = 0;

	sendUART("\n\rNew CAN Data, start with ID, separate with commas: \n\r");

	while(1) {
		if(_ucUARTNewCharRX) {
			while(UARTCharsAvail(UART0_BASE)) {
				c = UARTCharGet(UART0_BASE);
				UARTCharPut(UART0_BASE, c); // echo

				if(c == ','){
					// chars between 2 commas represent an unsigned long in hex format for the CAN
					ucMsgData[ucMsgPos] = '\0'; //such that the below knows when to stop
					ulCANData[ucCANPos++] = ustrtoul(ucMsgData, NULL, 16);
					ucMsgPos = 0;
				} else {
					ucMsgData[ucMsgPos++] = c;
				}



				if(ucMsgPos >= 5) {
					sendUART("Too many chars for 1 CAN element.\n\r");
					ucMsgPos = 0;
				}
				if(ucCANPos == 10) {
					// this will be picked up by the CAN sending task and used
					for(unsigned char i=0; i<10; i++) _ulCANMsg_UART[i] = ulCANData[i];
					ucCANPos = 0;
					sendUART("\n\rNew CAN Data, start with ID, separate with commas: \n\r");
				}

			}

			// just read everything there was, wait for another interrupt to tell us to start reading
			_ucUARTNewCharRX = 0;
		}

		vTaskDelay(1 / portTICK_RATE_MS); //give the CPU some time to breathe...
	}
}



void taskBlink(void* pdata) {
	while (1) {
		GPIOPinWrite(GPIO_PORTF_BASE, LED, 0);
		vTaskDelay(500 / portTICK_RATE_MS); // see configTICK_RATE_HZ in FreeRTOSConfig.h how this number translates into time
		GPIOPinWrite(GPIO_PORTF_BASE, LED, 1);
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

void taskCANDisplay(void* pdata) {
	tCANMsgObject sCANMsgRX;
	unsigned char ucMsgData[8]; // Buffer for storing the received data must be provided
	sCANMsgRX.pucMsgData = ucMsgData;
	char cBuff[21]; // Buffer for display
	char cTempBuff[5];

	while (1) {
		if(_ulCANStatusFlag != CAN_STATUS_LEC_NONE) {
			display(50, 0, 15, "STS=0x%04X", _ulCANStatusFlag);
		}

		if(_ucCANNewMsgRX) {
			// Interrupt clearing flag is not set because this interrupt was already cleared in the interrupt handler.
			CANMessageGet(CAN0_BASE, CAN_MSG_OBJ_ID_RX, &sCANMsgRX, 0);
			_ucCANNewMsgRX = 0;

			if(sCANMsgRX.ulFlags & MSG_OBJ_DATA_LOST) {
				display(0, 0, 10, "MLOSS");
			}

			display(0, 10, 6, "RX#=%u", _ulCANMsgRXCount);

			display(0, 20, 10, "ID=0x%08X l=%u", sCANMsgRX.ulMsgID, sCANMsgRX.ulMsgLen);

			// display received data, eventually on several rows
			const unsigned char BYTES_PER_ROW = 4;
			for(unsigned char ucRow = 0; ucRow * BYTES_PER_ROW < sCANMsgRX.ulMsgLen; ucRow ++) {
				unsigned char i = 0;
				for(; i < BYTES_PER_ROW && i + ucRow * BYTES_PER_ROW < sCANMsgRX.ulMsgLen; i++) {
					usprintf(cTempBuff, "%02X ", ucMsgData[i + ucRow * BYTES_PER_ROW]);
					ustrncpy(cBuff + i*3, cTempBuff, ustrlen(cTempBuff)); // using a constant number instead of strlen() seems to crash the program !!
				}
				cBuff[i * 3] = '\0';	// end of string
				display(0, 30 + 10 * ucRow, 15, (const char *)&cBuff);
			}

		}

		vTaskDelay(1 / portTICK_RATE_MS); //give the CPU some time to breathe...
	}
}


unsigned char lo8(unsigned int input) {
	return input & 0xFF;
}

unsigned char hi8(unsigned int input) {
	return (input >> 8) & 0xFF;
}

void setRPM(unsigned int uiRPM) {
	unsigned int tempRPM = uiRPM << 2;
	sendCAN(0x280, 8, 0, 0, lo8(tempRPM), hi8(tempRPM), 0, 0, 0, 0);
}

// has to be called periodically otherwise it will turn on again
void turnOffAirbagLight() {
	sendCAN(0x050, 8, 0, 0x80, 0, 0, 0, 0, 0, 0);
}

// has to be called periodically otherwise it will turn on again
void turnOffABSAnd2OtherLights() {
	sendCAN(0x1A0, 8, 0x18, 0, 0, 0, 0, 0, 0, 0);
}

// has to be called periodically otherwise it will turn on again
void engineOnAndESPEnabled() {
	sendCAN(0xDA0, 8, 0x01, 0x80, 0, 0, 0, 0, 0, 0);
}

// has to be called periodically otherwise it will turn on again
void immobilizer() {
	sendCAN(0x3D0, 8, 0, 0x80, 0, 0, 0, 0, 0, 0);
}

void tyreLowPressure() {
	// the 4th bit of the byte 3 (starting with 0 from left) has to be "1"
	sendCAN(0x5A0, 8, 0, 0, 0, 0x0, 0, 0, 0, 0);
}

void blinkers(char leftOn, char rightOn) {
	// pos 0 (from left starting with 0) can be 0-off, 1-leftON, 2-rightON, 3-bothON  (1st & 2nd bits matter)
	char value = leftOn | 2 * rightOn;
	sendCAN(0x531, 8, 0, 0, value, 0, 0, 0, 0, 0);
}


void taskCANSend(void* pdata) {
	unsigned long ulCounter = 0;

    while (1) {
    	immobilizer();
    	// if we send the CAN messages in too quick a succession apparently it creates errors...
    	// should probably listen for an event saying it's been sent before initiating the next one....
    	vTaskDelay(1 / portTICK_RATE_MS);

    	engineOnAndESPEnabled();
    	vTaskDelay(1 / portTICK_RATE_MS);

    	turnOffABSAnd2OtherLights();
    	vTaskDelay(1 / portTICK_RATE_MS);

    	turnOffAirbagLight();
    	vTaskDelay(1 / portTICK_RATE_MS);

    	//tyreLowPressure();
    	vTaskDelay(1 / portTICK_RATE_MS);

    	blinkers(ulCounter > 10, ulCounter > 20);
    	vTaskDelay(1 / portTICK_RATE_MS);

    	// send whatever was provided over serial line (this is just to make testing easier)
    	sendCAN(_ulCANMsg_UART[0], _ulCANMsg_UART[1], _ulCANMsg_UART[2], _ulCANMsg_UART[3], _ulCANMsg_UART[4], _ulCANMsg_UART[5], _ulCANMsg_UART[6], _ulCANMsg_UART[7], _ulCANMsg_UART[8], _ulCANMsg_UART[9]);
    	vTaskDelay(1 / portTICK_RATE_MS);



    	setRPM(2500 *  (ulCounter > 15));
    	vTaskDelay(1 / portTICK_RATE_MS);

    	// SPEED - the below works for small speeds
    	// Beyond 20-30km/h or so one needs to increment the COUNTER but I can't figure out what the rule is.
    	// https://hackaday.io/project/6288-volkswagen-can-bus-gaming#j-discussions-title   Einārs Deksnis gives some code but it doesn't work in my case !
        unsigned int uiSpeed = (20 * 133) * (ulCounter > 25); // km/h
        unsigned int uiCounter = 0;
    	sendCAN(0x5A0, 8, 0, lo8(uiSpeed), hi8(uiSpeed), 0xFF, 0xFF, lo8(uiCounter), hi8(uiCounter), 0);


       	vTaskDelay(30 / portTICK_RATE_MS);
       	ulCounter ++;
       	if(ulCounter > 40) ulCounter = 0;
    }
}



int main(void) {
	initBoard();
	initCAN();
	initUART();
	
	xTaskCreate(taskBlink, (signed char *)"tBlink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 7, NULL);
	xTaskCreate(taskUART, (signed char *)"tUART", 200, NULL, tskIDLE_PRIORITY + 6, NULL);
	xTaskCreate(taskCANDisplay, (signed char *)"tCANDisp", 200, NULL, tskIDLE_PRIORITY + 5, NULL);
	xTaskCreate(taskCANSend, (signed char *)"tCANSend", 500, NULL, tskIDLE_PRIORITY + 4, NULL);

	vTaskStartScheduler(); // This should never return
}


// see vector table in lm3s8962_startup_ccs_gcc.c
void CANIntHandler(void) {
    // Read the CAN interrupt status to find the cause of the interrupt
    unsigned long ulStatus = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    // If the cause is a controller status interrupt, then get the status
    if(ulStatus == CAN_INT_INTID_STATUS)
    {
        // This will return a field of status error bits that can indicate various errors.
        // The act of reading this status will clear the interrupt.
        ulStatus = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        _ulCANStatusFlag = ulStatus;
    } else if(ulStatus == CAN_MSG_OBJ_ID_RX) {
    	// Cause is CAN_MSG_OBJ_ID_RX (nothing to do with CAN ID), used for receiving messages.

        // Clear the message object interrupt. Normally CANMessageGet() clears the interrupt but we don't want to call that yet.
        CANIntClear(CAN0_BASE, CAN_MSG_OBJ_ID_RX);

        _ulCANMsgRXCount++;
        _ucCANNewMsgRX = 1;
    } else if(ulStatus == CAN_MSG_OBJ_ID_TX) {
    	// Cause is CAN_MSG_OBJ_ID_TX (nothing to do with CAN ID), used for transmitting messages.
    	CANIntClear(CAN0_BASE, CAN_MSG_OBJ_ID_TX);
    	_ulCANMsgTXCount++;
    } else {
    	// Something unexpected caused the interrupt.  This should never happen.
    	_ulCANStatusFlag = CAN_UNKNOWN_INT;
    }
}

// see vector table in lm3s8962_startup_ccs_gcc.c
void UARTIntHandler(void) {
	unsigned long ulStatus = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, ulStatus);

    _ucUARTNewCharRX = 1;
}
