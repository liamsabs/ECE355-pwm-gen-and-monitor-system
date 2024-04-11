// ----------------------------------------------------------------------------

// School: University of Victoria, Canada.

// Course: ECE 355 "Microprocessor-Based Systems".


// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.

// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.

// ----------------------------------------------------------------------------


#include <stdio.h>

#include "diag/Trace.h"

#include "cmsis/cmsis_device.h"

#include "cmsis/stm32f0xx.h"

#include "stm32f0xx_hal.h"

// ----------------------------------------------------------------------------

//

// STM32F0 empty sample (trace via $(trace)).

//

// Trace support is enabled by adding the TRACE macro definition.

// By default the trace messages are forwarded to the $(trace) output,

// but can be rerouted to any device or completely suppressed, by

// changing the definitions required in system/src/diag/trace_impl.c

// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).

//


// ----- main() ---------------------------------------------------------------


// Sample pragmas to cope with warnings. Please note the related line at

// the end of this function, used to pop the compiler diagnostics status.

#pragma GCC diagnostic push

#pragma GCC diagnostic ignored "-Wunused-parameter"

#pragma GCC diagnostic ignored "-Wmissing-declarations"

#pragma GCC diagnostic ignored "-Wreturn-type"



/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)


/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

/* set to 1ms prescaler */
#define myTIM3_PRESCALER (47999)

/*period of 100ms delay time = n x 100ms*/
#define myTIM3_PERIOD (100)


/*MAX value for ADC register IE 12 bits of 1*/
#define ADC_max ((float)0xFFF)

/*corresponds to 5000ohm max of potentiometer to scale DAC value*/
#define POT_max (5000)


/*corresponds to 1111 0000 used to extract upper portion of LCD instruction*/
#define upper (0xF0) 

/*corresponds to 0000 1111, used to extract lower portion of LCD instruction*/
#define lower (0xF)



/*function prototypes*/

/*Function used to set the internal clock of the stm32f0xx to 48Mhz*/
void SystemClock48MHz(void);

/*Function to initialize GPIOA ports*/
void myGPIOA_Init(void);

/*Function to initialize GPIOB ports*/
void myGPIOB_Init(void);

/*Function to initialize TIM2 timer*/
void myTIM2_Init(void);

/*Function to initialize TIM3 timer*/
void myTIM3_Init(void);

/*Function to initialize the external interrupt event mamanger system*/
void myEXTI_Init(void);

/*Function to Initialize ADC*/
void myADC_Init();

/*Function to initialize DAC*/
void myDAC_Init();

/*Function used to process ADC data , convert to potentiometer value, write to DAC register*/
void ADC_IRQHandler();

/*Function used to delay by utilizing TIM3 timer*/
void delay(uint32_t);

/*Function used to initialize SPI parameters using stm32f0xx_hal_spi.c*/
void mySPI_Init();

/*Function to transmit 8 bit data to SPI including delays for proper transmission*/
void mySPI_Send(uint8_t t);

/*Send 10 bit LCD instruction to LCD*/
void mySPI_Formatted_Send(uint16_t d);

/*Initializing LCD to begin*/
void myLCD_Init();

/*Count number of digits given an integer, used for processing frequency and reistance values for LCD writing*/
unsigned int count(unsigned int i);

/*Sets LCD back to Default configurations with Fixed written character*/
void myLCD_Default();

/*Writing the frequency and resistance values to the SPI*/
void myLCD_WriteDigits();



/*Initializing Global variables*/

/*Global variable used when measuring frequency to determine if freqeuncy is in process of measurement with timer*/
int timer_on = 0;

//Global variable for ADC value
uint32_t ADC_value = 0;

//Global variable for Potentiometer value
uint32_t POT_value = 0;

//global value to store frequency
double frequency;

//Gobal variable to determine if number of digits is increasing/decreasing for frequency
int toggle1 = 3;

//Global variable to determine if number of digits is increasing/decreasing for resistance
int toggle2 = 1;

//Creating SPI struct type to interface with SPI*/
SPI_HandleTypeDef SPI_Handle;



//Beginning main
int main(int argc, char* argv[])

{

//Setting system clock
SystemClock48MHz();

trace_printf("Final Project for ECE 355\n");

trace_printf("System clock: %u Hz\n", SystemCoreClock);


/* Initialize GPIOA*/
myGPIOA_Init();

/*Initialize GPIOB*/
myGPIOB_Init();

/* Initialize timer TIM2*/
myTIM2_Init();

/* Initialize timer TIM3 */
myTIM3_Init();

/* Initialize EXTI external interrupt */
myEXTI_Init();

/* initializing ADC */
myADC_Init();

/*Initialing DAC */
myDAC_Init();

/*initializing SPI*/
mySPI_Init();

//initializing LCD comfigurations*/
myLCD_Init();
    
/*Initializing LCD to default configuration with fixed characters*/
myLCD_Default();



while (1){
    
    /*fetching potentiomenter value and writing to LCD*/
	ADC_IRQHandler();
    
    /*Writing updated frequency and resistance to LCD*/
	myLCD_WriteDigits();

}


return 0;

}





void myGPIOA_Init(){

/* Enable clock for GPIOA peripheral */


//PA1 as interrupt input

// Relevant register: RCC->AHBENR

RCC->AHBENR |= RCC_AHBENR_GPIOAEN;



/*Configuring port PA1 as interrupt input for freqeuncy calculating*/

/* Configure PA1 to input mode */
GPIOA->MODER &= ~(GPIO_MODER_MODER1);

/* Ensure no pull-up/pull-down for PA1 */
GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);



/*Configuring port PA4 as analog output for DAC*/

 /*setting PA4 to analog mode*/
GPIOA->MODER |= GPIO_MODER_MODER4;

 /* Ensure no pull-up/pull-down for PA4 */
GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);



/*Configuring port PA5*/

/*setting PA5 to analog mode*/
GPIOA->MODER |= GPIO_MODER_MODER5;

 /* Ensure no pull-up/pull-down for PA5 */
GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5);

}

void myGPIOB_Init(){

/*Relevant register: RCC->AHBENR*/
RCC->AHBENR |= RCC_AHBENR_GPIOBEN;


/*configuring ports for SPI*/

/*PB3: used as clk peripheral of SPI*/

//configuring PB3 as Alternate function mode for SPI sclk
GPIOB->MODER |= GPIO_MODER_MODER3_1;

 //setting alternate function of PA3 to AF0(Do nothing)
 GPIOB->AFR[0] |= 0x00000;

 /* enable pull-up/pull-down for PA3 */
GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);


/*PB4: used as output for storage clock LCK */

//configuring PB4 as output mode for to use as MISO
GPIOB->MODER |= GPIO_MODER_MODER4_0;

/* Ensure no pull-up/pull-down for PA3 */
GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);


/*PB5: used as MOSI for SPI using AF0*/

//configuring PB5 as alternate function mode for MOSI
GPIOB->MODER |= GPIO_MODER_MODER5_1;

//configuring alternate function of PA5 to AF0 for MOSI
GPIOB->AFR[0] |= 0x00000;

/* Ensure no pull-up/pull-down for PA3 */
GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);

}





void myTIM2_Init(){

/* Enable clock for TIM2 peripheral */
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

/* Configuring TIM2 to mode: buffer auto-reload, count up, stop on overflow,
 enable update events, interrupt on overflow only */
TIM2->CR1 = ((uint16_t)0x008C);

/* Set clock prescaler value for TIM2 */
TIM2->PSC = myTIM2_PRESCALER;

/* Set Reload value to 0xFFFFFFFFF for maximum time before overflow*/
TIM2->ARR = myTIM2_PERIOD;

/* Update event generation registers */
TIM2->EGR = ((uint16_t)0x0001);

/* Assign TIM2 interrupt priority = 0 in NVIC */
NVIC_SetPriority(TIM2_IRQn, 0);

/* Enable TIM2 interrupts in NVIC */
NVIC_EnableIRQ(TIM2_IRQn);

/* update interrupt enabled */
TIM2->DIER |= TIM_DIER_UIE;

/* Start counting pulses from timer */
TIM2->CR1 |= TIM_CR1_CEN;

}



void myTIM3_Init(){

/* Enable clock for TIM3 peripheral */
RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

/* Configuring TIM2 to mode: buffer auto-reload, count up, stop on overflow,
 enable update events, interrupt on overflow only */
TIM3->CR1 |= 0x008C;

/* Set clock prescaler value for TIM3 */
TIM3->PSC = myTIM3_PRESCALER;

/* Set auto-reloaded delay: set to 100 for desired period to be used with delay function*/
TIM3->ARR = myTIM3_PERIOD;

/* update interupt enabled */
TIM3->DIER |= TIM_DIER_UIE;

/* Update event generation registers */
TIM3->EGR |= 0x0001;

}



void myEXTI_Init(){

/* Map EXTI2 line to PA1 */
SYSCFG->EXTICR[0] = 0x00000002;


/* EXTI2 line interrupts: set rising-edge trigger */
EXTI->RTSR = 0x00000002;


/* Unmask interrupts from EXTI2 line */
EXTI->IMR =0x00000002;


/* Assign EXTI0 interrupt priority = 0 in NVIC */
NVIC_SetPriority(EXTI0_1_IRQn, 0);


/* Enable EXTI2 interrupts in NVIC */
NVIC_EnableIRQ(EXTI0_1_IRQn);

}


void TIM2_IRQHandler(){

/* Check if update interrupt flag is set */
if ((TIM2->SR & TIM_SR_UIF) != 0){

trace_printf("\nOverflow!\n");

/* Clear update interrupt flag */
TIM2->SR &= ~(TIM_SR_UIF);


/* Restart stopped timer */
TIM2->CR1 |= TIM_CR1_CEN;

}
}


void myADC_Init(){

//enabling clock for ADC
RCC->APB2ENR |= 0x200;


/*ADC_CFGR1: While ADEN in ADC_CR ADEN = 0 these configurations are done*/

//setting CONT bit in ADC_CFGR1 to 1 for continuous sampling
ADC1->CFGR1 |= 0x2000;

//setting resolution to 12 bits
ADC1->CFGR1 |= 0x0000;

//setting data alignment to right aligned
ADC1->CFGR1 |= 0x0000;

//configuring overrun management mode to on
ADC1->CFGR1 |= 0x1000;


/*Now configuring the sampling time register*/

ADC1->SMPR |= 0x000; //setting questions between 000-111


/*Configuring channel selection register for DAC*/

//Configuring channel selection register to PA5
ADC1->CHSELR |= 0x20;


/*calibrating ADC */

//calibrating
ADC1->CR = ADC_CR_ADCAL;

//wait for calibration bit to be set back to 0
while (ADC1->CR == ADC_CR_ADCAL);


/*Analog to digital conversion (Control Register)*/

//enable ADC
ADC1->CR |= 0x0001;


/*Dealing with initial end of conversion flag set to reset to 0 for new conversion to start after function call*/

//waiting for 1st end of conversion
while((ADC1->ISR & ADC_ISR_EOC) != 0);

//clear end of conversion flag for fresh initialization
ADC1->ISR &= ~(ADC_ISR_EOC);

}



void myDAC_Init(){

//enabling clock for DAC
RCC->APB1ENR |= 0x20000000;

//enabling DAC for conversion
DAC->CR |= 0x0001;

}

void delay(uint32_t t){

	//reset counter register to 0
	TIM3-> CNT = (uint32_t)0x0;

	//set auto reload to input value
	TIM3->ARR = t;

	//enable update event generation register
	TIM3->EGR |= 0x0001;

	//start timer
	TIM3->CR1 |= TIM_CR1_CEN;

	//wait for interrupt flag to be set, ie. timer ends
	while((TIM3->SR & TIM_SR_UIF) == 0);

	//reset interrupt flag
	TIM3->SR &= ~(TIM_SR_UIF);

	//stop timer
	TIM3->CR1 &= ~(TIM_CR1_CEN);

	//reset counter register to 0 once again
	TIM3-> CNT = (uint32_t)0x0;

}




/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */

void EXTI0_1_IRQHandler(){

//initializing time to store period value
double time;


/* Check if EXTI2 interrupt pending flag is set */
if (EXTI->PR != 0 && EXTI_PR_PR1 != 0) {

//First Edge
if (EXTI->RTSR && timer_on == 0) {

// Clear TIM2 count register 
TIM2->CNT = 0;

// - Start timer
TIM2->CR1 |= TIM_CR1_CEN;

//set timer_on global variable to indicate timer is running    
timer_on = 1;
    
//Second Edge
}else {

//Stop timer
TIM2->CR1 = 0;

//turn off timer_on global variable    
timer_on = 0;

//Mask count register to time variable ie. set time value
time = TIM2->CNT;

//Calculate frequency global variable
frequency = 48000000/time;

}


//Clear EXTI2 interrupt pending flag(cleared by writing 1)
EXTI->PR |= 0x00000001;

}

}


void ADC_IRQHandler(){

 //start ADC conversion
ADC1->CR |= ADC_CR_ADSTART;

//wait for end of conversion flag to be set to 1
while((ADC1->ISR & ADC_ISR_EOC)== 0);

//clear end of conversion flag
ADC1->ISR &= ~(ADC_ISR_EOC);

//set bits of DAC 12 bit right aligned register to contents of ADC DR
DAC->DHR12R1 = ADC1->DR;
    
//Set ADC_value by &ing data register with 111111111111
ADC_value = (ADC1->DR & ADC_DR_DATA);
    
//Compute potentiometer value 
POT_value  = (((float)ADC_value)/ADC_max)*POT_max;

}

void SystemClock48MHz(){

// Disable the PLL
RCC->CR &= ~(RCC_CR_PLLON);

// Wait for the PLL to unlock
while (( RCC->CR & RCC_CR_PLLRDY ) != 0 );

// Configure the PLL for a 48MHz system clock
RCC->CFGR = 0x00280000;

// Enable the PLL
RCC->CR |= RCC_CR_PLLON;

// Wait for the PLL to lock
while (( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY );

// Switch the processor to the PLL clock source
RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;

// Update the system with the new clock frequency
SystemCoreClockUpdate();

}

void mySPI_Init(){

	//enabling the clock for SPI1
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// Point to SPI1 of the stm32f0
	SPI_Handle.Instance = SPI1;

	// Initialize the SPI_Direction member
	SPI_Handle.Init.Direction = SPI_DIRECTION_1LINE;

	//set mode of SPI to Master
	SPI_Handle.Init.Mode = SPI_MODE_MASTER;

	//set bit size to 8
	SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;

	//set polarity to low
	SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;

	//set phase
	SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;

	//set NSS
	SPI_Handle.Init.NSS = SPI_NSS_SOFT;

	//Set BaudRate Prescaler
	SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;

	//set First bit
	SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;

	//set polynomial
	SPI_Handle.Init.CRCPolynomial = 7;


	// Initialize the SPI interface
	HAL_SPI_Init(&SPI_Handle);

	// Enable the SPI
	__HAL_SPI_ENABLE(&SPI_Handle);

}

void myLCD_Init(){

mySPI_Send(0x2);//sending 00 0000 0010

mySPI_Send(0x82);//sending 01 0000 0010

mySPI_Send(0x2);//sending 00 0000 0010

mySPI_Formatted_Send(0x28); //DL = 0, N = 1, F = 0

mySPI_Formatted_Send(0xC); //D = 1, C = 0, B = 0

mySPI_Formatted_Send(0x6); //I/D = 1, S = 0

mySPI_Formatted_Send(0x1); //clearing Display

}

void mySPI_Send(uint8_t d){

//Set LCK of SPI to low for transmission    
GPIOB->ODR &= ~(GPIO_ODR_4);

//Transmit data with HAL SPI Transmit function    
HAL_SPI_Transmit(&SPI_Handle, &d, (uint16_t)1, HAL_MAX_DELAY);

//delay 1ms    
delay(1);

//set LCK of SPI to high to end transmission
GPIOB->ODR |= GPIO_ODR_4;

//delay 1ms    
delay(1);

}

void mySPI_Formatted_Send(uint16_t d){

//isolating the RS bit of the 10 bit instruction to append to 4 bit format
uint16_t rs = (d & 0x200)>>3;
    
//0x00 xxxx (upper instruction with 0 enable bit)
uint8_t u0 = ((d & upper)>>4)|rs;

//1x00 xxxx (upper instruction with 1 enable bit)    
uint8_t u1 = u0|0x80;

//0x00 xxxx (lower instruction with 0 enable bit)    
uint8_t l0 = (d & lower)|rs;
    
//1x00 xxxx (lower instruction)
uint8_t l1 = l0|0x80;

//sending upper section
mySPI_Send(u0); //en0
mySPI_Send(u1); //en1
mySPI_Send(u0); //en0
    
//sending lower section
mySPI_Send(l0); //en0
mySPI_Send(l1); //en1
mySPI_Send(l0); //en0

}

void myLCD_Default(){

	//Top line of Display "F:xxxxHz"

	/*Writing 'F'*/
	mySPI_Formatted_Send(0b0010000000);//set DDRAM Adress to 00
	mySPI_Formatted_Send(0b1001000110);//writing F

	//Writing ':'*/
	mySPI_Formatted_Send(0b0010000001);//set DDRAM Adress to 01
	mySPI_Formatted_Send(0b1000111010);//writing :

	//Writing 'H'
	mySPI_Formatted_Send(0b0010000110);//set DDRAM Adress to 06
	mySPI_Formatted_Send(0b1001001000);//writing H

	//Writing 'z'
	mySPI_Formatted_Send(0b0010000111);//set DDRAM Adress to 07
	mySPI_Formatted_Send(0b1001111010);//writing z

    
	//Bottom line of Display "R:xxxxOh"

	/*Writing 'R'*/
	mySPI_Formatted_Send(0b0011000000);//set DDRAM Adress to 40
	mySPI_Formatted_Send(0b1001010010);//writing F

	//Writing ':'*/
	mySPI_Formatted_Send(0b0011000001);//set DDRAM Adress to 41
	mySPI_Formatted_Send(0b1000111010);//writing :

	//Writing 'O'
	mySPI_Formatted_Send(0b0011000110);//set DDRAM Adress to 46
	mySPI_Formatted_Send(0b1001001111);//writing O

	//Writing 'h'
	mySPI_Formatted_Send(0b0011000111);//set DDRAM Adress to 47
	mySPI_Formatted_Send(0b1001101000 );//writing h



}

void myLCD_WriteDigits(){

//num1 is typecasted value of frequency from double to int    
unsigned int num1 = (int)frequency;

//dig1 corresponds to the number of digits in the frequency value 
unsigned int dig1 = count(num1);
    
//masked value of frequency size to create array size later
unsigned int size1 = dig1;

//creating array of chars to get digits of frequency    
char f[dig1];

//loop to load digits of frequency to array    
while (dig1--) {

//getting digit with iterated modulo 10    
f[dig1]=num1%10;

//dividing num1 remainder by 10    
num1/=10;

	}

    
//num2 is typecasted value of frequency from double to int
unsigned int num2 = (int)POT_value;
    
//dig2 corresponds to the number of digits in the resistance value 
unsigned int dig2 =count(num2);
    
//masked value of resistance size to create array size later
unsigned int size2 = dig2;
    
//creating array of chars to get digits of resistance
char r[size2];
    
//loop to load digits of resistance to array
while (dig2--) {

//getting digit with iterated modulo 10    
r[dig2]=num2%10;
    
//dividing num1 remainder by 10 
num2/=10;

	}

    
//Writing Frequency Digits
    
//If there are 4 digits ie F = 1134Hz
if(size1 == 4){

    /*Formatting digits of array by | with 0x200 for write bit, and | with 0x30 to convert digit to ascii*/
	uint16_t f0 = f[0]|0x200|0x30;
	uint16_t f1 = f[1]|0x200|0x30;
	uint16_t f2 = f[2]|0x200|0x30;
	uint16_t f3 = f[3]|0x200|0x30;

	/*Writing 'f0'*/
	mySPI_Formatted_Send(0b0010000010);//set DDRAM Adress to 02
	mySPI_Formatted_Send(f0);//writing digit

		//Writing 'f1'*/
	mySPI_Formatted_Send(0b0010000011);//set DDRAM Adress to 03
	mySPI_Formatted_Send(f1);//writing digit

		//Writing 'f2'
	mySPI_Formatted_Send(0b0010000100);//set DDRAM Adress to 04
	mySPI_Formatted_Send(f2);//writing digit

		//Writing 'f3'
	mySPI_Formatted_Send(0b0010000101);//set DDRAM Adress to 05
	mySPI_Formatted_Send(f3);//writing digit
    
    //if the previous frequency written to the screen wasn't size 4
	if(toggle1 != 4){
        
        //set toggle1 to 4 to anticipate more 4 digit values
		toggle1 = 4;

	}


//if the frequency has 3 digits    
} else if(size1 == 3){
    
    //if the previous frequency had 4 digits, clear display to display 3 dig now
	if(toggle1 != 3){
        
    //clearing Display
	mySPI_Formatted_Send(0x1);

    //Writing default characters back to the display    
	myLCD_Default();
    
    //setting toggle1 to 3 to anticipate more 3 digit values    
	toggle1 = 3;

	}
    
    /*Formatting digits of array by | with 0x200 for write bit, and | with 0x30 to convert digit to ascii*/
	uint16_t f0 = f[0]|0x200|0x30;
	uint16_t f1 = f[1]|0x200|0x30;
	uint16_t f2 = f[2]|0x200|0x30;

	/*Writing 'f0'*/
	mySPI_Formatted_Send(0b0010000011);//set DDRAM Adress to 03
	mySPI_Formatted_Send(f0);//writing digit

	//Writing 'f1'*/
	mySPI_Formatted_Send(0b0010000100);//set DDRAM Adress to 04
	mySPI_Formatted_Send(f1);//writing digit

	//Writing 'f2'
	mySPI_Formatted_Send(0b0010000101);//set DDRAM Adress to 05
	mySPI_Formatted_Send(f2);//writing digit


}
    
    
//Writing Resistance Digits

//If the resistance has 4 digits
if(size2 == 4){

     /*Formatting digits of array by | with 0x200 for write bit, and | with 0x30 to convert digit     to ascii*/
	uint16_t r0 = r[0]|0x200|0x30;
	uint16_t r1 = r[1]|0x200|0x30;
	uint16_t r2 = r[2]|0x200|0x30;
	uint16_t r3 = r[3]|0x200|0x30;

	/*Writing 'r0'*/
	mySPI_Formatted_Send(0b0011000010);//set DDRAM Adress to 42
	mySPI_Formatted_Send(r0);//writing digit

		//Writing 'r1'*/
	mySPI_Formatted_Send(0b0011000011);//set DDRAM Adress to 43
	mySPI_Formatted_Send(r1);//writing digit

		//Writing 'r2'
	mySPI_Formatted_Send(0b0011000100);//set DDRAM Adress to 44
	mySPI_Formatted_Send(r2);//writing digit

		//Writing 'r3'
	mySPI_Formatted_Send(0b0011000101);//set DDRAM Adress to 45
	mySPI_Formatted_Send(r3);//writing digit
    
    //If the previous resistance value is not 4 digits
	if(toggle2 != 4){
        
        //set toggle2 to 4 to anticipate more 4 digit values
		toggle2 = 4;
	}
    
//If the resistance has 3 digits    
}else if(size2 == 3){
    
    //if previous resistance value is not 3 digits
	if(toggle2 != 3){
        
        //if the previous resistance value has greater than 3 digits
		if(toggle2 > 3){

            //clearing Display
			mySPI_Formatted_Send(0x1); 

            //Write default characters to LCD
			myLCD_Default();
            
			}
        //set toggle2 to 3 to anticipate more 3 digit values
		toggle2 = 3;

		}
    
         /*Formatting digits of array by | with 0x200 for write bit, and | with 0x30 to convert           digit to ascii*/
		uint16_t r0 = r[0]|0x200|0x30;
		uint16_t r1 = r[1]|0x200|0x30;
		uint16_t r2 = r[2]|0x200|0x30;

		/*Writing 'r0'*/
		mySPI_Formatted_Send(0b0011000011);//set DDRAM Adress to 43
		mySPI_Formatted_Send(r0);//writing digit

		//Writing 'r1'*/
		mySPI_Formatted_Send(0b0011000100);//set DDRAM Adress to 44
		mySPI_Formatted_Send(r1);//writing digit

		//Writing 'r2'
		mySPI_Formatted_Send(0b0011000101);//set DDRAM Adress to 45
		mySPI_Formatted_Send(r2);//writing digit

    //If the resistance has 2 digits
	}else if(size2 == 2){
    
        //if the previous resistance value does not have 2 digits
		if(toggle2 != 2){
            
            //if the previous resistance value has greater than 2 digits
			if(toggle2 > 2){
                
            //clearing Display
			mySPI_Formatted_Send(0x1);
                
            //Write Default characters to LCD
			myLCD_Default();
			}
            
            //set toggle2 to 2 to anticipate more 2 digit values
			toggle2 = 2;

			}

            /*Formatting digits of array by | with 0x200 for write bit, and | with 0x30 to convert           digit to ascii*/
			uint16_t r0 = r[0]|0x200|0x30;
			uint16_t r1 = r[1]|0x200|0x30;

			//Writing 'f1'*/
				mySPI_Formatted_Send(0b0011000100);//set DDRAM Adress to 44
				mySPI_Formatted_Send(r0);//writing digit

				//Writing 'f2'
				mySPI_Formatted_Send(0b0011000101);//set DDRAM Adress to 45
				mySPI_Formatted_Send(r1);//writing digit
    
    //If the resistance has 1 digits
	}else if(size2 == 1){
    
        //If the previous resistance value does not have 1 digit
		if(toggle2 != 1){
            
            //If the previous resistance value has more than 1 digit
			if(toggle2 > 1){
                
                        //Clearing display
						mySPI_Formatted_Send(0x1);

                        //set toggle2 to 2 to anticipate more 2 digit values
						myLCD_Default();
						}

                //set toggle2 to 1 to anticipate more 1 digit values
				toggle2 = 1;

				}
                
                /*Formatting digits of array by | with 0x200 for write bit, and | with 0x30 to                     convert digit to ascii*/
				uint16_t r0 = r[0]|0x200|0x30; 
				

				mySPI_Formatted_Send(0b0011000101);//set DDRAM Adress to 45
				mySPI_Formatted_Send(r0);//writing digit


	}

}


unsigned int count(unsigned int i) {

 //initializing return value   
 unsigned int ret=1;
    
 //while i has non-zero remainder, increase return value
 while (i/=10) ret++;

 //return number of digits
 return ret;

}







#pragma GCC diagnostic pop


// ----------------------------------------------------------------------------
