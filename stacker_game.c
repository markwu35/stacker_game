#include <stdlib.h>
#include <string.h>
#include <board.h>
/* Handy list of GPIO pins on 17xx/40xx boards.
    Embedded Artists Devkit LPC1788/LPC4088 on base board...

		PORT	PIN	DIMM Pin	Function
		----	---	--------	------------------------------------------
		16	Joystick            center button
		2	Joystick button		LEFT
		4   Joystick button		UP
		0   Joystick button		RIGHT
    	8	Joystick button		DOWN

 */
//GPIO definitions
#define GPIO_INTERRUPT_PIN_LEFT		   2
#define GPIO_INTERRUPT_PIN_UP     	   4
#define GPIO_INTERRUPT_PIN_RIGHT       1
#define GPIO_INTERRUPT_PIN_DOWN        8
#define GPIO_INTERRUPT_PIN_UP_LEFT     6
#define GPIO_INTERRUPT_PIN_DOWN_LEFT  10
#define GPIO_INTERRUPT_PIN_UP_RIGHT    5
#define GPIO_INTERRUPT_PIN_DOWN_RIGHT  9

#define GPIO_INTERRUPT_PIN_CENTER	16
#define GPIO_INTERRUPT_PORT    		GPIOINT_PORT2	/* GPIO port number mapped to interrupt */
#define GPIO_IRQ_HANDLER  			GPIO_IRQHandler/* GPIO interrupt IRQ function name */
#define GPIO_INTERRUPT_NVIC_NAME    GPIO_IRQn	/* GPIO interrupt NVIC interrupt name */

//TIMER definitions
#define TIMER0_IRQ_HANDLER				TIMER0_IRQHandler  // TIMER0 interrupt IRQ function name
#define TIMER0_INTERRUPT_NVIC_NAME		TIMER0_IRQn        // TIMER0 interrupt NVIC interrupt name
bool fDebouncing;  // Boolean variable for tracking debouncing behavior
#define TIMER1_IRQ_HANDLER				TIMER1_IRQHandler  // TIMER1 interrupt IRQ function name
#define TIMER1_INTERRUPT_NVIC_NAME		TIMER1_IRQn        // TIMER1 interrupt NVIC interrupt name

//I2C definitions
#define DEFAULT_I2C          I2C0
#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000
static int mode_poll;   /* Poll/Interrupt mode flag */
static I2C_ID_T i2cDev = DEFAULT_I2C; /* Currently active I2C device */

//LED Matrix Definitions
#define LED_ON					1
#define LED_OFF					0
#define LED_RED					1
#define LED_YELLOW				2
#define LED_GREEN				3
uint8_t i2Cbuffer[16];
int redBuffer[8];
int greenBuffer[8];

int currCursor_x;
int currCursor_y;
int currCursor_color;
int canResetFlag;

int pickUpMode;
int stepsLeft;
int cursorMode;

//uint8_t checkerBoard[8][8] = {
//		{0,0,0,0,0,0,0,0},
//		{0,1,0,0,0,0,0,0},
//		{0,0,0,0,0,0,0,0},
//		{0,0,0,0,0,0,0,0},
//		{0,0,0,0,0,0,0,0},
//		{0,0,0,0,0,1,0,0},
//		{0,0,0,0,0,0,0,0},
//		{0,0,0,0,0,0,0,0}
//	};

uint8_t checkerBoard[8][8] = {
	{0,2,0,2,0,2,0,2},
	{0,1,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,1,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0}
};

void moveCursor(int direction) { // 0=down, 1=up, 2=left, 3=right
	//DEBUGOUT("PREV x: %d, y: %d\n", currCursor_x, currCursor_y);
	if (checkerBoard[currCursor_y][currCursor_x] == 0) {
		drawPixel(currCursor_x,currCursor_y, LED_OFF);
	}
	else if (checkerBoard[currCursor_y][currCursor_x] == 1) {
		drawPixel(currCursor_x,currCursor_y, LED_GREEN);
	}
	else if (checkerBoard[currCursor_y][currCursor_x] == 2) {
		drawPixel(currCursor_x,currCursor_y, LED_RED);
	}
	writeDisplay();
	if (direction == 0) {
		if (currCursor_y == 7) {
			currCursor_y = 0;
		} else {
			currCursor_y +=1;
		}
	} else if (direction == 1) {
		if (currCursor_y == 0) {
			currCursor_y = 7;
		} else {
			currCursor_y -=1;
		}
	} else if (direction == 2) {
		if (currCursor_x == 0) {
			currCursor_x = 7;
		} else {
			currCursor_x -=1;
		}
	} else {
		if (currCursor_x == 7) {
			currCursor_x  = 0;
		} else {
			currCursor_x +=1;
		}
	}
	DEBUGOUT("CURR x: %d, y: %d\n", currCursor_x, currCursor_y);
	drawPixel(currCursor_x, currCursor_y, LED_GREEN);
	writeDisplay();
	return;
}

void movePickUpChecker(int direction) {// 0=up-left, 1=up-right, 2=down-left, 3=down-right
	//DEBUGOUT("PREV x: %d, y: %d\n", currCursor_x, currCursor_y);
	drawPixel(currCursor_x,currCursor_y, LED_OFF);
	writeDisplay();
	if (direction == 0) {
		if (currCursor_y == 0 || currCursor_x == 0) {
			DEBUGOUT("You can't move in that direction!");
		} else {
			currCursor_x -=1;
			currCursor_y -=1;
		}
	} else if (direction == 1) {
		if (currCursor_x == 7 || currCursor_y == 0) {
			DEBUGOUT("You can't move in that direction!");
		} else {
			currCursor_x +=1;
			currCursor_y -=1;
		}
	} else if (direction == 2) {
		if (currCursor_x == 0 || currCursor_y == 7) {
			DEBUGOUT("You can't move in that direction!");
		} else {
			currCursor_x -=1;
			currCursor_y +=1;
		}
	} else {
		if (currCursor_x == 7 || currCursor_y == 7) {
			DEBUGOUT("You can't move in that direction!");
		} else {
			currCursor_x +=1;
			currCursor_y +=1;
		}
	}
	DEBUGOUT("CURR x: %d, y: %d\n", currCursor_x, currCursor_y);
	drawPixel(currCursor_x, currCursor_y, LED_YELLOW);
	writeDisplay();
	return;
}

void pickUpChecker(int x, int y) {

}

/* GPIO INTERRUPT HANDLER */
void joystick_handler()
 {
	if(fDebouncing) {}  // If not debouncing
	else {
		if (Joystick_GetStatus()) {
			DEBUGOUT("%d\n", Joystick_GetStatus());
		}
		if (cursorMode) {
			if (Joystick_GetStatus() == GPIO_INTERRUPT_PIN_UP) {
				//DEBUGOUT("UP\n");
				moveCursor(1);
			}
			if (Joystick_GetStatus() == GPIO_INTERRUPT_PIN_DOWN) {
				//DEBUGOUT("DOWN\n");
				moveCursor(0);
			}
			if (Joystick_GetStatus() == GPIO_INTERRUPT_PIN_LEFT) {
				//DEBUGOUT("LEFT\n");
				moveCursor(2);
			}
			if (Joystick_GetStatus() == GPIO_INTERRUPT_PIN_RIGHT) {
				//DEBUGOUT("RIGHT\n");
				moveCursor(3);
			}
		} else if (pickUpMode) {
			if (Joystick_GetStatus() == GPIO_INTERRUPT_PIN_UP_LEFT) {
				movePickUpChecker(0);
			}
			if (Joystick_GetStatus() == GPIO_INTERRUPT_PIN_UP_RIGHT) {
				movePickUpChecker(1);
			}
			if (Joystick_GetStatus() == GPIO_INTERRUPT_PIN_DOWN_LEFT) {
				movePickUpChecker(2);
			}
			if (Joystick_GetStatus() == GPIO_INTERRUPT_PIN_DOWN_RIGHT) {
				movePickUpChecker(3);
			}
		}

		if (Joystick_GetStatus() == GPIO_INTERRUPT_PIN_CENTER) {
			//DEBUGOUT("CENTER\n");
			if (pickUpMode) {
				pickUpMode = 0;
				cursorMode = 1;
				//dropChecker(currCursor_x, currCursor_y);
			} else {
				pickUpMode = 1;
				cursorMode = 0;
				DEBUGOUT("Picked Up x:%d, y:%d, pickUpMode:%d\n", currCursor_x, currCursor_y, pickUpMode);
				//pickUpChecker();

			}
			Board_LED_Toggle(0);
		}
		// Start debounce delay
		fDebouncing = true;  // Update boolean variable

		// Start timer here
		Chip_TIMER_Enable(LPC_TIMER0);  // Start TIMER0
	}
	return;
 }

/* State machine handler for I2C0 and I2C1 */
static void i2c_state_handling(I2C_ID_T id)
{
	if (Chip_I2C_IsMasterActive(id)) {
		Chip_I2C_MasterStateHandler(id);
	} else {
		Chip_I2C_SlaveStateHandler(id);
	}
}

/* Set I2C mode to polling/interrupt */
static void i2c_set_mode(I2C_ID_T id, int polling)
{
	if(!polling) {
		mode_poll &= ~(1 << id);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
		NVIC_EnableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
	} else {
		mode_poll |= 1 << id;
		NVIC_DisableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);
	}
}

/* Initialize the I2C bus */
static void i2c_app_init(I2C_ID_T id, int speed)
{
	Board_I2C_Init(id);

	/* Initialize I2C */
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, speed);

	/* Set default mode to interrupt */
	i2c_set_mode(id, 1);
}

//Timer 0 used for debouncing
void TIMER0_IRQHandler(void)
{
	fDebouncing = false; 				  // Update boolean variable
	Chip_TIMER_Disable(LPC_TIMER0);		  // Stop TIMER0
	Chip_TIMER_Reset(LPC_TIMER0);		  // Reset TIMER0
	Chip_TIMER_ClearMatch(LPC_TIMER0,0);  // Clear TIMER0 interrupt
}

//Timer 1 used for controlling the speed of the blocks in the current row
void TIMER1_IRQHandler(void)
{

	Chip_TIMER_Reset(LPC_TIMER1);		  // Reset TIMER1
	Chip_TIMER_ClearMatch(LPC_TIMER1,0);  // Clear TIMER1 interrupt
	//DEBUGOUT("%d\n",Joystick_GetStatus());
}


/**
 * @brief	I2C Interrupt Handler
 * @return	None
 */
void I2C1_IRQHandler(void)
{
	i2c_state_handling(I2C1);
}

/**
 * @brief	I2C0 Interrupt handler
 * @return	None
 */
void I2C0_IRQHandler(void)
{
	i2c_state_handling(I2C0);
}

/*****************************************************************************
 * matrixClear() clears the LED matrix
 ****************************************************************************/
void matrixClear() {
	uint8_t i,j;

	for (i = 0; i< 8; i++) {
		for (j = 0; j< 8; j++) {
			drawPixel(i,j, LED_OFF);
		}
	}
	writeDisplay();
}

/*****************************************************************************
 * drawBitMap() draws a graphic according to the bitmap input
 ****************************************************************************/
void drawBitMap(uint8_t bitmap[8][8]) {
	int x, y;
	for (y = 0; y < 8; y++) {
		for (x = 0; x<8; x++) {
			if (bitmap[x][y] == 1) {
				drawPixel(y, x, LED_GREEN);
			} else if (bitmap[x][y] == 2) {
				drawPixel(y, x, LED_RED);
			} else {
				drawPixel(y,x, LED_OFF);
			}
		}
	}
}

/*****************************************************************************
 * drawPixel() draws pixel on the LED matrix
 ****************************************************************************/
void drawPixel(int x, int y, int color) {
	//if ((y < 0) || (y >= 8)) return;
	if ((x < 0) || (x >= 8)) return;

	if (color == LED_GREEN) {
		//Turns on green led
		greenBuffer[y] |= 1 << x;
		//Turns off red led
		y+=1;
		redBuffer[y] &= ~(1 << (x));
	}
	else if (color == LED_RED) {
		//Turn off green led
		greenBuffer[y] &= ~(1 << x);
		y+=1;
		//Turn on red led
		redBuffer[y] |= 1 << (x);
	}
	else if (color == LED_YELLOW) {
		//Turn on red and green led
		greenBuffer[y] |= 1 << x;
		y+=1;
		redBuffer[y] |= 1 << (x);
	}
	else if (color == LED_OFF) {
		//Turn off green and red led
		greenBuffer[y] &= ~(1 << x);
		y+=1;
		redBuffer[y] &= ~(1 << (x));
	}

}

/*****************************************************************************
 * writeDisplay() writes the i2c buffer to the LED matrix
 ****************************************************************************/
void writeDisplay() {
	Chip_I2C_MasterSend(I2C0, 0x70, 0x00, 1);
	uint8_t i;
	for (i = 0; i < 8; i++) {
		i2Cbuffer[2*i+1] =  greenBuffer[i] & 0xFF;
		i2Cbuffer[2*i] = redBuffer[i] & 0xFF;
	}
	Chip_I2C_MasterSend(I2C0, 0x70, i2Cbuffer, 16);
}

void GPIO_Input_Init(uint8_t port, uint8_t pin) {
	Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, IOCON_FUNC0);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
}

void initializeCheckers() {
	pickUpMode = 0;
	cursorMode = 1;
	drawBitMap(checkerBoard);
	writeDisplay();
}

void initializeCursor() {
	currCursor_x = 0;
	currCursor_y = 0;
	currCursor_color = LED_GREEN;
	drawPixel(currCursor_x, currCursor_y, currCursor_color);
	writeDisplay();
}

int main(void) {
	SystemCoreClockUpdate();
	Board_Init();
	Board_Joystick_Init();
	i2c_app_init(I2C0, SPEED_400KHZ);
	i2cDev = I2C0;

	int oscillatorON, oscillatorOFF, blinkrate, brightness;
	oscillatorOFF = 0x20;
	oscillatorON = 0x21;
	blinkrate = 0x81;
	brightness = 0xEF;

	Chip_I2C_MasterSend(I2C0, 0x70, &oscillatorOFF, 2);
	Chip_I2C_MasterSend(I2C0, 0x70, &oscillatorON, 2);	// oscillator
	Chip_I2C_MasterSend(I2C0, 0x70, &blinkrate, 2);   //blinkrate
	Chip_I2C_MasterSend(I2C0, 0x70, &brightness, 2);  //brightness

	fDebouncing = false;	     // Set to false, not debouncing
	initializeCheckers();
	initializeCursor();

	// Initialize TIMER0
	int PrescaleValue = 120000;  // Clock cycle / 1000 (set to ms increments)
	Chip_TIMER_Init(LPC_TIMER0);					   // Initialize TIMER0
	Chip_TIMER_PrescaleSet(LPC_TIMER0,PrescaleValue);  // Set prescale value
	Chip_TIMER_SetMatch(LPC_TIMER0,0,100);			   // Set match value
	Chip_TIMER_MatchEnableInt(LPC_TIMER0, 0);		   // Configure to trigger interrupt on match

	// Initialize TIMER1
	//Chip_TIMER_Init(LPC_TIMER1);					   // Initialize TIMER1
	//Chip_TIMER_PrescaleSet(LPC_TIMER1,60); 			   // Set prescale value
	//Chip_TIMER_SetMatch(LPC_TIMER1,0,30000);		   // Set match value
	//Chip_TIMER_MatchEnableInt(LPC_TIMER1, 0);		   // Configure to trigger interrupt on match
	//Chip_TIMER_Enable(LPC_TIMER1);

	NVIC_ClearPendingIRQ(GPIO_INTERRUPT_NVIC_NAME);
	NVIC_EnableIRQ(GPIO_INTERRUPT_NVIC_NAME);

	NVIC_ClearPendingIRQ(TIMER0_INTERRUPT_NVIC_NAME);
	NVIC_EnableIRQ(TIMER0_INTERRUPT_NVIC_NAME);

	//NVIC_ClearPendingIRQ(TIMER1_INTERRUPT_NVIC_NAME);
	//NVIC_EnableIRQ(TIMER1_INTERRUPT_NVIC_NAME);

	while(1){
		joystick_handler();
	}
	Chip_I2C_DeInit(I2C0);
	return 0;
}
