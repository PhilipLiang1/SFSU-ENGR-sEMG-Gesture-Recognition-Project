#include <stdint.h>
#include <stdbool.h>
#include "finalProjPWM.h"
#include "inc/hw_ints.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"
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

void moveFinger(uint32_t finger);

int main(void) {
	//configure system clock
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	
	//inialize ports
	PortInitialization();
	
	//inialize UART0
	UART0_Init();
	//start timer
	TimerFingerBegin();

	//call to move finger 
	 //controlGesture();
   //moveFinger(GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
	 //closeFingers(GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
	//loop forever
	while(1){
		
	}
	
}


