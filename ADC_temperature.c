	
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h" // manually added for interrupt functions
#include "driverlib/timer.h" // manually added for timer functions

// comment out when configuring HC-05
//#include "inc/tm4c123gh6pm.h" // manually added

// for configuring HC-05 bluetooth communication with each other
#include "inc/hw_ints.h" // manually added
#include "inc/hw_gpio.h" // manually added
#include "driverlib/uart.h" // manually added
#include <math.h> // manually added
#include <stdlib.h> // manually added
#include <stdio.h> // manually added
#include <string.h> //manually added

// for gesture prediction
//#include "Window.h"
//#include "Classifier.h"

// *****************************************************************************
//
//! my_478_project:
//! 1000Hz sampling rate, 4 AIN channels from EMG sensors
//! window length = 50ms, 5 gestures:
//! class_0 = "Rest"
//! class_1 = "HandClose"
//! class_2 = "HandOpen"
//! class_3 = "PointIndex"
//! class_4 = "DevilHorns" 
//
// *****************************************************************************

uint8_t WINDOW_INC = 50; // window increment (ms)
uint32_t WINDOW_LENGTH = 200; // window length (ms)


//uint32_t channel = 4; // number of channels
//uint32_t gestures = 5; // number of classes

//for data recording
//bool isRecording = false; // variable to start recording
//bool feat_ready = false; // variable to start printing features onto serial terminal 
//char command[] = "Enter Command: \n\r"; //array for request input message
uint32_t ui32ADC0Value[4]; // array for storing ADC0 values from different samples/steps
float window[4][50]; // 2d array for window of each channel	

//for UART buffer
static uint8_t ui8UartTxBuffi = 0; // input index

char sFeature_matrix[16]; // buffer for holding features being sent

//variables for monitoring in debug mode
volatile unsigned long samples = 0; // temp variable for keeping track of number of samples recorded
//volatile uint16_t RawEMG0; // temp variable for storing input from AIN0 (PE3)
//volatile uint16_t RawEMG1; // temp variable for storing input from AIN1 (PE2)
//volatile uint16_t RawEMG2; // temp variable for storing input from AIN2 (PE1)
//volatile uint16_t RawEMG3; // temp variable for storing input from AIN3 (PE0)
//volatile float mav0;
//volatile float zc0;
//volatile float wl0;
//volatile float t0;

//for gesture prediction
//char prediction[] = "Gesture: "; // array for prediction


// global variables for storing data and beginning calculations in feature extraction
volatile uint16_t RawEMG[4][64]; // 2d array for storing inputs from [AIN0, AIN1, AIN2, AIN3]
volatile uint8_t RawEMGi = 0; // index for RawEMG; counts up to 64
//volatile uint8_t EMGp = WINDOW_INC-1; // variable to trigger EMGprocess
volatile bool EMGprocess = false; // begins processing Raw EMG when true
volatile uint8_t EMGpend = 49; // variable to indicate end of window
volatile uint8_t EMGpstart = 0; // variable to indicate start of window


// for python classification; floats = 32 bits = 4 bytes
int Feature_matrix[16]; // features array; 16 features, 4 (MAV, ZC, WL, T) for each channel 	
//char current_gesture = '0'; // change for each recording session (0=Rest, 1=HandClose, 2=HandOpen, 3=PointIndex, 4=DevilHorns)

// function to convert integers to string
void int2string(int value) 
{	
	uint8_t index = 0; // sFeature_matrix index
	uint8_t dig = 0; // digit index
	char ch[12]; // max 12 digit long
	for(int i=0; i<sizeof(sFeature_matrix); i++) //initially clear sFeature_matrix buffer with null-characters
	{
		sFeature_matrix[i] = 0;
	}

	if (value == 0) // for case when integer is 0
	{
		sFeature_matrix[index] = '0'; // append '0' character
		index++; // increment index
		index %= 16; // modulate index to be 0-15
		//return;
	}
	while (value != 0) // for cases when integer is not 0
	{
		ch[dig] = (value % 10) + 0x30; // modulus the current ones places
		value /= 10;  // shift decimal to next
		dig++; // increment digit index
	}
	
	while (dig != 0) // reverse digit order to push char into string
	{ 
		dig--; // decrement digit index
		sFeature_matrix[index] = ch[dig]; // append character to sFeature_matrix
		index++; // increment index
		index %= 16; // modulate index to be 0-15
	}

}

//uart initialization function
void uart_Init(void) 
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // activate clock for UART0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // activate clock for GPIOA
	GPIOPinConfigure(GPIO_PA0_U0RX); // configure PA0 to be UART0 receiver
	GPIOPinConfigure(GPIO_PA1_U0TX); // configure PA1 to be UART0 transmitter
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); //configure PA0 and PA1 for UART functions
	
	//configure UART baud rate = 115200, 8-bit data, 1 stop bit, no parity
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, 
											(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8); // FIFO16 TX int when 4 Left, RX int when at least 2 Available
  UARTFIFOEnable(UART0_BASE); // enable UART0 FIFO
	
	IntEnable(INT_UART0); // NVIC enable for UART0 interrupt
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX ); // enable RX and TX interrupts (works)
	//UARTIntDisable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX );
}

//uart0 interrupt handler
void UARTInt_Handler(void) 
{
	uint32_t ui32Status; // variable for storing interrupt status
	
	// for storing recorded data
	//char temp_char; // temp variable for storing UART character
	//char RawEMG_0[6];
	//char RawEMG_1[6];
	//char RawEMG_2[6];
	//char RawEMG_3[6];
  //char sFeature_matrix[16][16]; // 2d char array for storing string type of Feature_matrix
																//Feature_matrix is a 
	
  ui32Status = UARTIntStatus(UART0_BASE, true); // get interrupt status
	//UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts

	
  if(ui32Status == UART_INT_TX) // if UART_TX interrupt; when TX FIFO part way empty
	{	
		UARTIntClear(UART0_BASE, ui32Status); // clear the asserted interrupts
		ui8UartTxBuffi++; // increment buffer index
		//UARTCharPutNonBlocking(UART0_BASE, ','); //print ','
		//UARTCharPutNonBlocking(UART0_BASE, ' '); //print ' '
		//int temp_arr[16];
		//for(int j=0; j<16; j++)
		//{
			//temp_arr[j] = Feature_matrix[j];
		//}
		//int temp = Feature_matrix[ui8UartTxBuffi];
		//sprintf(sFeature_matrix, ",%d", temp); //convert float feature at the buffer index to a string
		//itoa(temp, sFeature_matrix, 10); //implicit declaration not allowed b/c itoa not a standard function
		//snprintf(sFeature_matrix, 16, ",%d", temp); //same issue as sprintf()
		
		/*
		int2string(Feature_matrix[ui8UartTxBuffi]);
		UARTCharPutNonBlocking(UART0_BASE, ',');
		for(int i=0; sFeature_matrix[i] != '\0'; i++)
		{
			UARTCharPutNonBlocking(UART0_BASE, sFeature_matrix[i]);
		}
		if(ui8UartTxBuffi == 15) //if 
		{
			UARTIntDisable(UART0_BASE, UART_INT_TX); //disable UART0 TX interrupt
		}
		*/
	}

}
	
/*	
	while(UARTCharsAvail(UART0_BASE)) //loop while there are characters
  {
		temp_char = UARTCharGetNonBlocking(UART0_BASE); //store input character into temp variable
		
    // for recording data	and printing them out on Serial Monitor	
		if(temp_char == 'r')
		{
			//UARTCharPutNonBlocking(UART0_BASE, temp_char); //echo character r
			isRecording = true;
		}
		else
		{
			isRecording = false;
		}
	}
	if(isRecording == true)
	{
		UARTCharPut(UART0_BASE, '\n'); // start a new line
		UARTCharPut(UART0_BASE, '\r'); // move carriage return to beginning of the line
		
		//for sending extracted features to python as csv file
		if(feat_ready == true)
		{
			feat_ready = false; // acknowledge flag
			for(int i=0; i<16; i++) // print each feature in a 1x16 array 
			{
				sprintf(sFeature_matrix[i] , "%f", Feature_matrix[i]);
				for(int j=0; j<16; j++)
				{
					UARTCharPut(UART0_BASE, sFeature_matrix[i][j]);
				}
				UARTCharPut(UART0_BASE, ',');
			}
			UARTCharPut(UART0_BASE, current_gesture); // append which gesture is being recorded
		}
*/		

/*	//for printing out RawEMG values on Tera Term for viewing
		sprintf(RawEMG_0, "%d", RawEMG0); // converts float value to a character array
		sprintf(RawEMG_1, "%d", RawEMG1); // converts float value to a character array
		sprintf(RawEMG_2, "%d", RawEMG2); // converts float value to a character array
		sprintf(RawEMG_3, "%d", RawEMG3); // converts float value to a character array
		for(int i=0; i<sizeof(RawEMG_0); i++)
		{
			UARTCharPut(UART0_BASE, RawEMG_0[i]);
		}
		UARTCharPut(UART0_BASE, ',');
		for(int i=0; i<sizeof(RawEMG_1); i++)
		{
			UARTCharPut(UART0_BASE, RawEMG_1[i]);
		}
		UARTCharPut(UART0_BASE, ',');
		for(int i=0; i<sizeof(RawEMG_2); i++)
		{
			UARTCharPut(UART0_BASE, RawEMG_2[i]);
		}
		UARTCharPut(UART0_BASE, ',');
		for(int i=0; i<sizeof(RawEMG_3); i++)
		{
			UARTCharPut(UART0_BASE, RawEMG_3[i]);
		}			
*/		
//	}
//}

// ADC0 initializaiton
void ADC0_Init(void)
{
	
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // configure the system clock to be 40MHz
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	// activate the clock of ADC0
		SysCtlDelay(2);	// insert a few cycles after enabling the peripheral to allow the clock to be fully activated.

		ADCSequenceDisable(ADC0_BASE, 1); // disable ADC0 SS1 before the configuration is complete
		//ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0); // will use ADC0, SS1, processor-trigger, priority 0
		ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0); // will use ADC0, SS1, timer-trigger, priority 0
	
		ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0); // ADC0 SS1 Step 0, sample from CH0 (AIN0) (PE3)
		ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1); // ADC0 SS1 Step 1, sample from CH1 (AIN1) (PE2)
		ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2); // ADC0 SS1 Step 2, sample from CH2 (AIN2) (PE1)
		// ADC0 SS1 Step 3, sample from CH3 (AIN3) (PE0), 
	  // completion of this step will set RIS, last sample of the sequence
	  ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH3|ADC_CTL_IE|ADC_CTL_END); 
	
		IntPrioritySet(INT_ADC0SS1, 0x00);  	 // configure ADC0 SS1 interrupt priority as 0
		IntEnable(INT_ADC0SS1);    				// enable interrupt 15 in NVIC (ADC0 SS1)
		ADCIntEnableEx(ADC0_BASE, ADC_INT_SS1);      // arm interrupt of ADC0 SS1
	
		ADCSequenceEnable(ADC0_BASE, 1); // enable ADC0 SS1 after the configuration is complete
}


// initialize AIN0 (PE3), AIN1 (PE2), AIN2 (PE1), and AIN3 (PE0) as an analog input
void PortInitFunction(void)
{
	  //
    // Enable Peripheral Clocks 
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Enable pin PE3 for ADC AIN0
    //
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
	
		// Enable pin PE2 for ADC AIN1
		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
		// Enable pin PE1 for ADC AIN2
		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
		// Enable pin PE0 for ADC AIN3
		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
}

// initialize Timer 0A
void Timer0A_Init(unsigned long period)
{   
	//
  // Enable Peripheral Clocks 
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); 		// configure for 32-bit timer mode
  TimerLoadSet(TIMER0_BASE, TIMER_A, period -1);      // reload value

	//IntPrioritySet(INT_TIMER0A, 0x00);  	 // configure Timer0A interrupt priority as 0
  //IntEnable(INT_TIMER0A);    				// enable interrupt 19 in NVIC (Timer0A)
	//TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);      // arm timeout interrupt

	TimerControlTrigger(TIMER0_BASE, TIMER_A, 1); // enable ADC trigger output

  TimerEnable(TIMER0_BASE, TIMER_A);      // enable Timer0A
}
		
// interrupt handler for ADC0 SS3
void ADC0_Handler(void)
{
		ADCIntClear(ADC0_BASE, 1); // acknowledge trigger flag
	
	  // comment out for Task 3
		//ADCProcessorTrigger(ADC0_BASE, 1); //processor trigger for ADC0 SS1 for the next read
	
		ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value); // get data from ADC0 SS1
	
		for(int i=0; i<4; i++) // loop through each channel
		{
			// modulate ADC readings to be in appropriate range and store in RawEMG array
			RawEMG[i][RawEMGi] = ui32ADC0Value[i]%4096; 
			//window[i][samples] = ui32ADC0Value[i]; //&4095;
		}
		
		samples++; //increment number of samples recorded
		//samples %= 50;
		
		RawEMGi++; // increment the RawEMG array index
		//RawEMGi &= 63; //modulate to 0-63
		RawEMGi %= 64; // modulate the RawEMG index to be 0-63
		
		if(RawEMGi == EMGpend) // have gathered 50 samples from ADC
		{
			EMGprocess = true; // set system flag to begin processing
			EMGpend += WINDOW_INC; // increment RawEMG array end index by the window increment
			//EMGpend &= 63;
			EMGpend %= 64; // modulate RawEMG array index to be 0-63
		}
					
		//RawEMG0 = RawEMG[0][0]; // store first AIN0 value in variable for monitoring
		//RawEMG1 = RawEMG[1][0]; // store first AIN1 value in variable for monitoring
		//RawEMG2 = RawEMG[2][0]; // store first AIN2 value in variable for monitoring
		//RawEMG3 = RawEMG[3][0]; // store first AIN3 value in variable for monitoring
	
}

int main(void)
{
	
		ADC0_Init(); // initialize and configure ADC0
	  uart_Init(); // initialize and configure UART 
		IntMasterEnable(); // globally enable interrupt
		//ADCProcessorTrigger(ADC0_BASE, 1); // processor trigger for ADC0 SS1 for first read
	  PortInitFunction(); // initialize GPIO ports

	
	  unsigned long period0 = SysCtlClockGet()/1000; // reload value to Timer0A for 1000 Hz sampling freq
		Timer0A_Init(period0); // initialize and configure Timer0A for periodic interrupt	

		
		// loop forever
		while(1)
		{
			
			if(EMGprocess == true) // if ready to process
			{
				EMGprocess = false; // acknowledge and clear the flag		
				// variables initialization
				//float window[4][50]; // 2d array for window of each channel	
				//int j = EMGpstart;
				int sum = 0; // sum up values for each channel
				int avg[4] = {0}; // avg for each channel		
				
				int abs_sum[4] = {0}; // absolute value of sum
				int MAV[4] = {0}; // mean absolute value for each channel	
				
				int ZC[4] = {0}; // zero-crossings for each channel
				
				int diff = 0; // difference btw current and prev values
				int delta_abs_sum[4] = {0}; // absolute value of the sum of diff
				int WL[4] = {0}; // wavelength for each channel
				
				int diff_2 = 0; // difference btw the next current and prev values
				int T[4] = {0}; // turns for each channel
				for(int c=0; c<4; c++) // loop through each channel
				{
					// initialize variable values
					sum = 0; //0.0f;
				  avg[c] = 0;
					abs_sum[c] = 0;
					MAV[c] = 0;
					diff = 0;
					delta_abs_sum[c] = 0;
					WL[c] = 0;
					diff_2 = 0;
					
					for(int k=0; k<50; k++) // loop through each sample
					{
						window[c][k] = RawEMG[c][(k+EMGpstart)%64]; // store RawEMG signals into a window for each channel
						sum += window[c][k]; // calculate the sum of values from a channel
						
					}
					
					avg[c] = (sum+25)/50; // calculate the avg for each channel w/ int division

					for(int l=0; l<50; l++) // loop through each sample
					{
						window[c][l] = window[c][l] - avg[c]; // zero each window		
						
					// Feature Extraction
						// calculate the absolute sum for each channel (for MAV)
						if(window[c][l]<0)
						{
							abs_sum[c] = abs_sum[c] - window[c][l] ; 
						}
						else
						{
							abs_sum[c] = window[c][l] + abs_sum[c];
						}
						
						// calculate the number of zero-crossings (ZC) for each channel
						if((window[c][l] * window[c][l+1])<0) // current sample and next sample don't have the same sign
						{
							ZC[c]++; // increment zero-crossing by one
						}
						
						// calculate the absolute value of the difference btw adjacent values for each channel (for WL)
						diff = window[c][l+1] - window[c][l];
						if(diff<0)
						{
							diff = (-1) * diff;
							//diff = window[c][l] - window[c][l+1];
						}
						delta_abs_sum[c] = diff + delta_abs_sum[c];	
						
						// calculate the number of turns (T) for each channel
						diff_2 = window[c][l+2] - window[c][l+1];			
						if((diff * diff_2)<0) //current slope and next slope don't have the same sign
						{
							T[c]++; // increment the number of turns by one
						}
					}

					MAV[c] = (abs_sum[c]+25)/50; // calculate the MAV for each channel
					WL[c] = (delta_abs_sum[c]+25)/50; // calculate the WL for each channel				
					
					// for python; store extracted features in Feature_matrix array for printing out
					Feature_matrix[(c*4)+0] = MAV[c];
					Feature_matrix[(c*4)+1] = ZC[c];
					Feature_matrix[(c*4)+2] = WL[c];
					Feature_matrix[(c*4)+3] = T[c];
		

				}
				EMGpstart += WINDOW_INC; // increment RawEMG start index by the window increment
				//EMGpstart &= 63;
				EMGpstart %= 64; // modulate the RawEMG start index to be 0-63
				//feat_ready = true;
				
				ui8UartTxBuffi = 0; // intialize input index to 0
		
				// for printing out features
				for(; ui8UartTxBuffi<16; ui8UartTxBuffi++) // loop through extracted features in Feature_matrix array
				{
					if(ui8UartTxBuffi == 0) // first feature in Feature_matrix array
					{
						// convert feature value to string; append new line and carriage return to beginning of string of first feature in a set
						sprintf(sFeature_matrix, "\n\r%d", Feature_matrix[ui8UartTxBuffi]); 
					//sprintf(sFeature_matrix, "%d", Feature_matrix[ui8UartTxBuffi]);
					}

					else // other features in Feature_matrix array
					{
						// convert feature values to string; append comma between strings of consecutive features in a set
						sprintf(sFeature_matrix, ",%d", Feature_matrix[ui8UartTxBuffi]); 
					}
					
					for(int i=0; sFeature_matrix[i] != '\0'; i++) // loop while not the end of string character
					{
						UARTCharPut(UART0_BASE, sFeature_matrix[i]); //print out feature
					}
				}
				//UARTIntEnable(UART0_BASE, UART_INT_TX);	
				//UARTIntDisable(UART0_BASE, UART_INT_TX);	
				
//				mav0 = Feature_matrix[0]; //store MAV of channel 0 for monitoring
//				zc0 = Feature_matrix[1]; //store MAV of channel 0 for monitoring
//				wl0 = Feature_matrix[2]; //store MAV of channel 0 for monitoring
//				t0 = Feature_matrix[3]; //store MAV of channel 0 for monitoring
			}							
	

		}
}
