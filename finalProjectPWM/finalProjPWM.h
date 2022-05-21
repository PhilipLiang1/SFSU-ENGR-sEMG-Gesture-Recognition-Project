#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"
#include "tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

//finger movements
volatile uint32_t ServoFingerCount = 0;
uint32_t ServoFingerBase[5];
uint32_t ServoFingerPin[5];
uint32_t ServoFingerPos[5];
uint32_t ServoFingerPosTemp[5];
uint32_t ServoFingerPos[5];
uint32_t ServoFingerNumber = 0;

//configure UART0 for receiving

void UART0_Init(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);	//enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);	//enable GPIOA port pins

    GPIOPinConfigure(GPIO_PA0_U0RX);			//configure pin PA0 for recieving
    GPIOPinConfigure(GPIO_PA1_U0TX);			//configure pin PA1 for transmitting
    GPIOPinTypeUART(0x40004000, GPIO_PIN_0 | GPIO_PIN_1);		

    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //enable GPIO port for LED
    //GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); //enable pin for LED PF2

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));				//allow 8 bits/sec bandwidth, 
																																							//allow one stop bit and no parity bit
    IntMasterEnable(); //enable processor interrupts
    IntEnable(INT_UART0); //enable the UART interrupt
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts
}

/*
void UART0IntHandler(){
     //store receive UART value here
       char tempUART_value; 
	
       uint32_t ui32Status;

       ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status

       UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts
	
       //UARTCharPut(UART0_BASE, tempUART_value);
	
	while(UARTCharsAvail(UART0_BASE)){
		//store received value
		tempUART_value = UARTCharGet(UART0_BASE); 
				
	}

	UARTCharPut(UART0_BASE, UART0IntHandler());
	
	switch(UART0IntHandler()){
		case '3':
		//point index
		for(int index=0; index < sizeof(ServoFingerPin); index++){
			if(ServoFingerPin[index] != GPIO_PIN_4){
				closeFingers(GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
				moveFinger(GPIO_PIN_4);
			}else 
				moveFinger(GPIO_PIN_4);
			}	
		break;
		case '4':
			//gesture devil horns
			moveFinger(GPIO_PIN_4|GPIO_PIN_5);
		break;
				
		case '2':
			//gesture to open hand
			moveFinger(GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_5);
		break;
				
		case '1':
		 //close all fingers
		for(int index=0; index < sizeof(ServoFingerPin); index++){
			if(ServoFingerPin[index] != (GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5)){
				closeFingers(GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
			}else 
				 closeFingers(GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
			}	
						
			//moveFinger(GPIO_PIN_0|GPIO_PIN_0|GPIO_PIN_0|GPIO_PIN_0);
		break;
		default:
						 //rest position, all fingers open
					   moveFinger(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);
		}
}
*/
	

void PortInitialization(){
		//configure timer clock on GPIO port E
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	
		//Set pin as output
		//0x40024000 is the address for GPIO_PORTE_BASE
		GPIOPinTypeGPIOOutput(0x40024000,GPIO_PIN_1);
		GPIOPinTypeGPIOOutput(0x40024000,GPIO_PIN_2);
		GPIOPinTypeGPIOOutput(0x40024000,GPIO_PIN_3);
		GPIOPinTypeGPIOOutput(0x40024000,GPIO_PIN_4);
		GPIOPinTypeGPIOOutput(0x40024000,GPIO_PIN_5);
}


void ServoFingerInterrupt(){

  //Remember to clear the interrupt flags
  uint32_t status=0;

  status = TimerIntStatus(TIMER2_BASE,true);
  TimerIntClear(TIMER2_BASE,status);

 
   //This increments the counter
   //The counter works like a timer in PWM mode in count up mode.
   //The value 4000 sets the wave period.
   //Then we have the "match" values that set the positive pulse width,
   //those are the values saved in the array ServoPos. The outputs start as HIGH and go to LOW
   //when the match value is reached
   
  ServoFingerCount++;
  uint32_t i;
  if(ServoFingerCount > 4000)
	  ServoFingerCount = 0;
  	  
  	   //This loads the next values into the ServoPos array
  	   //It needs to happen in the begining of a period
  	   
	  for(i=0; i < ServoFingerNumber; i++){
		  ServoFingerPos[i] = ServoFingerPosTemp[i];
	  }

 	 for(i=0; i < ServoFingerNumber; i++){
		  if(ServoFingerCount > ServoFingerPos[i])
			  GPIOPinWrite(ServoFingerBase[i],ServoFingerPin[i], 0);
	 	 else
		    GPIOPinWrite(ServoFingerBase[i],ServoFingerPin[i], ServoFingerPin[i]);
  }
}



 //This function changes the servopos
 //It takes values from 0 to 200 and converts them to the real values of 200-400.
 //It saves the position in a auxiliar array
 
int32_t ServoFingerWrite(uint32_t value, uint32_t pos){


	//Check if the values are correct
	if(value > 200)
		return -1;

	//Increment 199 since the real values are from 200 to 400, not 0 to 200
	value = value + 199;

	//Save value in array for the interrupt
	ServoFingerPosTemp[pos] = value;

	return 0;
}

int32_t ServoAttachFingers(uint32_t pin){

	//Remember to check if you aren't controling more servos than the arrays can handle
	if(ServoFingerNumber < 6){
		//0x40024000 is the address of GPIO_PORTE_BASE
		ServoFingerBase[ServoFingerNumber] = 0x40024000;
		ServoFingerPin[ServoFingerNumber] = pin;

		//Increment variable so we know we have 1 more servo
		ServoFingerNumber++;
	}
	else
		return -1;

	return 0;
}



//Timer setup

void TimerFingerBegin(){
	//200Khz - 5uS. (servo pulses precisely of  1000uS to  2000uS or 1mS to 2mS)
 	// 200-400
 	//We set the load value so the timer interrupts each 1ms
 	uint32_t Period;
  	Period = 400;

  	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  	SysCtlDelay(3);
  
  	// Configure the timer as periodic, by omission it's in count down mode.
   	// It counts from the load value to 0 and then resets back to the load value.
 	// REMEMBER: You need to configure the timer before setting the load and match
  
 	 TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
 	 TimerLoadSet(TIMER2_BASE, TIMER_A, Period -1);

 	 TimerIntRegister(TIMER2_BASE, TIMER_A, ServoFingerInterrupt);

  
  	//Enable the timeout interrupt. In count down mode it's when the timer reaches
 	//0 and resets back to load. In count up mode it's when the timer reaches load
 	//and resets back to 0.
  
  	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

  	TimerEnable(TIMER2_BASE, TIMER_A);
}

//this function will move a corresponding finger when called
void moveFinger(uint32_t finger){
	int index;
	ServoAttachFingers(finger);
	while(index < 200){
	//open finger
	      ServoFingerWrite(index,0);
	      //if(i==200){
	      //ServoAttach(SYSCTL_PERIPH_GPIOE,GPIO_PORTE_BASE, 0);
	      //break;
			
	      SysCtlDelay(200000);
	      //ServoWrite(0,0);
	      index++;

			}
		//servo return to zero
		 //ServoFingerWrite(0,0);
			//SysCtlDelay(200000);
}
void closeFingers(uint32_t finger){
     int index;
     ServoAttachFingers(finger);
     while(index <= 200){
     //open finger
     ServoFingerWrite(index,0);
     //if(i==200){
     //ServoAttach(SYSCTL_PERIPH_GPIOE,GPIO_PORTE_BASE, 0);
     //break;
			
     SysCtlDelay(200000);
     //ServoWrite(0,0);
     index--;

     }
     //servo return to zero
     //ServoFingerWrite(0,0);
     //SysCtlDelay(200000);
}

void UART0IntHandler(){
     //store receive UART value here
     char tempUART_value; 
	
     uint32_t ui32Status;

     ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status

     UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts
	
     //UARTCharPut(UART0_BASE, tempUART_value);
	
     while(UARTCharsAvail(UART0_BASE)){
           //UARTCharPut(UART0_BASE);
           //store received value
     	   tempUART_value = UARTCharGet(UART0_BASE);
	   //open hand gesture
     	   if(tempUART_value == '2'){
         	//closeFingers(GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
         	moveFinger(GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_1);
     	   }
     }
					
	/*
     	switch(tempUART_value){
	       case '3':
	       //point index
		for(int index=0; index < sizeof(ServoFingerPin); index++){
		if(ServoFingerPin[index] != GPIO_PIN_4){
		 closeFingers(GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
		 moveFinger(GPIO_PIN_4);
		}else 
		moveFinger(GPIO_PIN_4);
		}	
		break;
		case '4':
		//gesture devil horns
		moveFinger(GPIO_PIN_4|GPIO_PIN_5);
		break;
							
		case '2':
		//gesture to open hand
		moveFinger(GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_5);
		break;
				
		case '1':
		//close all fingers
			for(int index=0; index < sizeof(ServoFingerPin); index++){
			    if(ServoFingerPin[index] != (GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5)){
			    closeFingers(GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
			    }else 
			    closeFingers(GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
			}	
								
		//moveFinger(GPIO_PIN_0|GPIO_PIN_0|GPIO_PIN_0|GPIO_PIN_0);
		break;
		default:
		//rest position, all fingers open
			moveFinger(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);
		}
	}
				*/
}

	
/*
void controlGesture(){
	
	UARTCharPut(UART0_BASE, UART0IntHandler());
	
	switch(UART0IntHandler()){
	case '3':
	//point index
	for(int index=0; index < sizeof(ServoFingerPin); index++){
		if(ServoFingerPin[index] != GPIO_PIN_4){
			closeFingers(GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
			 moveFinger(GPIO_PIN_4);
		}else 
			moveFinger(GPIO_PIN_4);
		}	
		break;
	case '4':
			//gesture devil horns
			 moveFinger(GPIO_PIN_4|GPIO_PIN_5);
	break;
				
	case '2':
		//gesture to open hand
		moveFinger(GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_5);
	break;
		
	case '1':
		//close all fingers
		for(int index=0; index < sizeof(ServoFingerPin); index++){
			if(ServoFingerPin[index] != (GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5)){
				closeFingers(GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
			}else 
				 closeFingers(GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
			}	
						
				//moveFinger(GPIO_PIN_0|GPIO_PIN_0|GPIO_PIN_0|GPIO_PIN_0);
	break;
	default:
		//rest position, all fingers open
		moveFinger(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);
	}
}

*/
