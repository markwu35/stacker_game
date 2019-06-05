#include <stdlib.h>
#include <string.h>
#include <board.h>
/* Handy list of GPIO pins on 17xx/40xx boards.
    Embedded Artists Devkit LPC1788/LPC4088 on base board...

		PORT	PIN	DIMM Pin	Function
		----	---	--------	------------------------------------------
		2		22	120-GPIO73	Joystick center button
		2		23	121-GPIO74	Joystick button		LEFT
		2		25	122-GPIO75	Joystick button		UP
		2		26	123-GPIO76  Joystick button		RIGHT
    	2		27  124-GPIO77	Joystick button		DOWN

    	2		10  35-GPIO10	Interrupt button SW6
 */
//GPIO definitions
#define GPIO_INTERRUPT_PIN_LEFT		23
#define GPIO_INTERRUPT_PIN_UP     	25
#define GPIO_INTERRUPT_PIN_RIGHT    26
#define GPIO_INTERRUPT_PIN_DOWN     27
#define GPIO_INTERRUPT_PIN_CENTER	22
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

int currCursor_x = 1;
int currCursor_y = 0;
//int currLED_x = 3;
//int currLED_y = 3;
int currLvl;
int prevLvl_LED_x;
int currLvl_LED_x;
int currBlockSize;
bool moveRight;
int leftBorder;
int rightBorder;
int canResetFlag;
int RIGFLAG= 0;

/*****************************************************************************
 * writeBlock() displays the block of the current level
 ****************************************************************************/
void writeBlock(int blockSize, int x, int color) {
	if (blockSize == 3) {
		drawPixel(x-1, currLvl, color);
		drawPixel(x+1, currLvl, color);
	}
	if (blockSize == 2) {
		drawPixel(x+1, currLvl, color);
	}
	drawPixel(x, currLvl, color);
	writeDisplay();
}

/*****************************************************************************
 * moveCurrLvl() moves the block of the current level left and right
 ****************************************************************************/
void moveCurrLvl() {
	writeBlock(currBlockSize, currLvl_LED_x, LED_OFF);
	if (currBlockSize == 2) {
		leftBorder = 0;
		rightBorder = 6;
	} else if (currBlockSize == 1) {
		leftBorder = 0;
		rightBorder = 7;
	}
	if (currLvl_LED_x == rightBorder) {
		moveRight = false;
	}
	if (currLvl_LED_x == leftBorder) {
		moveRight = true;
	}
	if (moveRight) {
		currLvl_LED_x ++;
	} else {
		currLvl_LED_x --;
	}
	if (currBlockSize == 3) {
		writeBlock(currBlockSize, currLvl_LED_x, LED_GREEN);
	} else if (currBlockSize == 2) {
		writeBlock(currBlockSize, currLvl_LED_x, LED_YELLOW);
	} else {
		writeBlock(currBlockSize, currLvl_LED_x, LED_RED);
	}
}

/*****************************************************************************
 * displayLoss() displays the graphics when player loses the game
 ****************************************************************************/
void displayLoss() {
	canResetFlag = 1;
	writeBlock(currBlockSize, currLvl_LED_x, LED_RED);
	uint8_t FACE[8][8] = {
		{0,0,0,0,0,0,0,0},
		{0,1,0,0,0,0,1,0},
		{1,0,1,0,0,1,0,1},
		{0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0},
		{0,1,1,1,1,1,1,0},
		{1,1,0,0,0,0,1,1},
		{0,0,0,0,0,0,0,0}
	};
	drawBitMap(FACE, LED_RED);
	writeDisplay();
}

/*****************************************************************************
 * displayWin() displays the graphics when player wins the game
 ****************************************************************************/
void displayWin() {
	canResetFlag = 1;
	writeBlock(currBlockSize, currLvl_LED_x, LED_GREEN);
	//DEBUGOUT("YOU ARE A WINNER!");
	uint8_t FACE[8][8] = {
		{0,0,0,0,0,0,0,0},
		{0,1,0,0,0,0,1,0},
		{1,1,1,0,0,1,1,1},
		{0,1,0,0,0,0,1,0},
		{0,0,0,0,0,0,0,0},
		{1,0,0,0,0,0,0,1},
		{0,1,0,0,0,0,1,0},
		{0,0,1,1,1,1,0,0}
	};
	drawBitMap(FACE, LED_GREEN);
	writeDisplay();
}

/*****************************************************************************
 * stopBlock() outputs the outcome according to the the location of the current block
 ****************************************************************************/
void stopBlock() {
	if (RIGFLAG != 1 || currLvl == 7) {
		Chip_TIMER_Disable(LPC_TIMER1);
	} else if (RIGFLAG == 1 && currLvl != 7) {
		while (currLvl_LED_x != prevLvl_LED_x) {
			moveCurrLvl();
		}
		Chip_TIMER_Disable(LPC_TIMER1);
	}
	if (currLvl != 7) {
		int xDiff = currLvl_LED_x - prevLvl_LED_x;
		//DEBUGOUT("Curr block x: %d, Prev block x: %d, xDiff: %d\n",currLvl_LED_x,prevLvl_LED_x, xDiff);
		if (abs(xDiff) > 2) {
			displayLoss();
			return;
		}
		if (currLvl == 0) {//Last level and center block location matches last center block
			if (abs(xDiff) < currBlockSize) {
				displayWin();
				return;
			}
		}
		writeBlock(currBlockSize, currLvl_LED_x, LED_OFF);
		if (xDiff == 2) {
			if (currBlockSize == 3){
				currLvl_LED_x -= 1;
				currBlockSize = 1;
			} else if (currBlockSize <= 2) {
				displayLoss();
				return;
			}
		}
		if (xDiff == -2) {
			if (currBlockSize == 3){
				currLvl_LED_x += 1;
				currBlockSize = 1;
			} else if (currBlockSize <= 2) {
				displayLoss();
				return;
			}
		}
		if (xDiff == 1) {
			if (currBlockSize == 3){
				currLvl_LED_x -= 1;
				currBlockSize = 2;
			} else if (currBlockSize == 2) {
				currBlockSize = 1;
			} else if (currBlockSize == 1) {
				displayLoss();
				return;
			}
		}
		if (xDiff == -1) {
			if (currBlockSize == 3){
				currBlockSize = 2;
			} else if (currBlockSize == 2) {
				currLvl_LED_x += 1;
				currBlockSize = 1;
			} else if (currBlockSize == 1) {
				displayLoss();
				return;
			}
		}
		//DEBUGOUT("Curr size: %d\n",currBlockSize);
	}
	writeBlock(currBlockSize, currLvl_LED_x, LED_OFF);
	if (currBlockSize == 3) {
		writeBlock(currBlockSize, currLvl_LED_x, LED_GREEN);
	} else if (currBlockSize == 2) {
		writeBlock(currBlockSize, currLvl_LED_x, LED_YELLOW);
	} else {
		writeBlock(currBlockSize, currLvl_LED_x, LED_RED);
	}
	currLvl --;
	//int currSpd = 100000 * (currLvl/7);
	//Chip_TIMER_SetMatch(LPC_TIMER1,0,currSpd);
	//Chip_TIMER_MatchEnableInt(LPC_TIMER1, 0);
	Chip_TIMER_Enable(LPC_TIMER1);
	prevLvl_LED_x = currLvl_LED_x;
}

void GPIO_Input_Init(uint8_t port, uint8_t pin) {
	Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, IOCON_FUNC0);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
}

/* GPIO INTERRUPT HANDLER */
void GPIO_IRQ_HANDLER(void)
 {
	uint32_t interrupt_bits_f = Chip_GPIOINT_GetStatusFalling(LPC_GPIOINT, 2);
	uint32_t interrupt_bits_r = Chip_GPIOINT_GetStatusRising(LPC_GPIOINT, 2);
	if(fDebouncing) {}  // If not debouncing
	else {
		if (interrupt_bits_r & (1 << GPIO_INTERRUPT_PIN_DOWN)) {
			//DEBUGOUT("DOWN\n");
			if (canResetFlag) {
				initializeStacker();
			} else {
				stopBlock();
			}
			//movePixel(0);
			Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, GPIO_INTERRUPT_PORT, 1 << GPIO_INTERRUPT_PIN_DOWN);
		}
		if (interrupt_bits_f & (1 << GPIO_INTERRUPT_PIN_CENTER)) {
			//DEBUGOUT("CENTER\n");
			if (RIGFLAG) {
				RIGFLAG = 0;
			} else {
				RIGFLAG = 1;
			}
			Board_LED_Toggle(0);
			Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, GPIO_INTERRUPT_PORT, 1 << GPIO_INTERRUPT_PIN_CENTER);
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
	moveCurrLvl();
	Chip_TIMER_Reset(LPC_TIMER1);		  // Reset TIMER1
	Chip_TIMER_ClearMatch(LPC_TIMER1,0);  // Clear TIMER1 interrupt
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
void drawBitMap(uint8_t bitmap[8][8], uint8_t color) {
	int x, y;
	for (y = 0; y < 8; y++) {
		for (x = 0; x<8; x++) {
			if (bitmap[x][y] == 1 ) {
				drawPixel(y, x, color);
			}
			else {
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

/*****************************************************************************
 * initializesStacker() initializes the stacker game
 ****************************************************************************/
void initializeStacker() {
	matrixClear();
	canResetFlag = 0;
	moveRight = true;
	currBlockSize = 3;
	currLvl = 7;
	currLvl_LED_x = 3;
	leftBorder = 1;
	rightBorder = 6;
	//writeBlock(currBlockSize, currLvl_LED_x, LED_GREEN);
	Chip_TIMER_Enable(LPC_TIMER1);  // Start TIMER1
}

int main(void) {
	//system initialization
	SystemCoreClockUpdate();
	Board_Init();

	//i2c initialization
	i2c_app_init(I2C0, SPEED_400KHZ);
	i2cDev = I2C0;

	int oscillatorON, oscillatorOFF, blinkrate, brightness;
	oscillatorOFF = 0x20;
	oscillatorON = 0x21;
	blinkrate = 0x81;
	brightness = 0xEF;

	//initialize LED matrix
	Chip_I2C_MasterSend(I2C0, 0x70, &oscillatorOFF, 2);
	Chip_I2C_MasterSend(I2C0, 0x70, &oscillatorON, 2);	// oscillator
	Chip_I2C_MasterSend(I2C0, 0x70, &blinkrate, 2);   //blinkrate
	Chip_I2C_MasterSend(I2C0, 0x70, &brightness, 2);  //brightness


	// Set timer prescale value and set initial boolean variable state
	int PrescaleValue = 120000;  // Clock cycle / 1000 (set to ms increments)
	fDebouncing = false;	     // Set to false, not debouncing

	// Initialize TIMER0
	Chip_TIMER_Init(LPC_TIMER0);					   // Initialize TIMER0
	Chip_TIMER_PrescaleSet(LPC_TIMER0,PrescaleValue);  // Set prescale value
	Chip_TIMER_SetMatch(LPC_TIMER0,0,100);			   // Set match value
	Chip_TIMER_MatchEnableInt(LPC_TIMER0, 0);		   // Configure to trigger interrupt on match

	// Initialize TIMER1
	Chip_TIMER_Init(LPC_TIMER1);					   // Initialize TIMER1
	Chip_TIMER_PrescaleSet(LPC_TIMER1,60); 			   // Set prescale value
	//the higher the match value is, the slower the block will move
	Chip_TIMER_SetMatch(LPC_TIMER1,0,30000);		   // Set match value
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 0);		   // Configure to trigger interrupt on match

	//GPIO initialization for joystick down and center
	GPIO_Input_Init(GPIO_INTERRUPT_PORT, GPIO_INTERRUPT_PIN_DOWN);
	Chip_GPIOINT_SetIntRising(LPC_GPIOINT, GPIO_INTERRUPT_PORT, 1 << GPIO_INTERRUPT_PIN_DOWN);
	GPIO_Input_Init(GPIO_INTERRUPT_PORT, GPIO_INTERRUPT_PIN_CENTER);
	Chip_GPIOINT_SetIntFalling(LPC_GPIOINT, GPIO_INTERRUPT_PORT, 1 << GPIO_INTERRUPT_PIN_CENTER);

	/* Enable interrupt in the NVIC */
	NVIC_ClearPendingIRQ(GPIO_INTERRUPT_NVIC_NAME);
	NVIC_EnableIRQ(GPIO_INTERRUPT_NVIC_NAME);

	//initialize the stacker game
	initializeStacker();

	NVIC_ClearPendingIRQ(TIMER0_INTERRUPT_NVIC_NAME);
	NVIC_EnableIRQ(TIMER0_INTERRUPT_NVIC_NAME);
	NVIC_ClearPendingIRQ(TIMER1_INTERRUPT_NVIC_NAME);
	NVIC_EnableIRQ(TIMER1_INTERRUPT_NVIC_NAME);

	while(1){
		__WFI();
	}

	Chip_I2C_DeInit(I2C0);
	return 0;
}
