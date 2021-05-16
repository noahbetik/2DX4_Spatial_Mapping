// Noah Betik
// 400246583
// betikn
// April 7, 2021


#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

//uint16_t	dev = 0x29;
uint16_t	dev=0x52;

int status=0;

volatile int IntCount;

//device in interrupt mode (GPIO1 pin signal)
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void PortH_Init(void);
void VL53L1X_XSHUT(void);

//capture values from VL53L1X for inspection
uint16_t distances[32];

void PortH_Init(void){ // motor setup
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0xFF;        								// make Port H output
  GPIO_PORTH_AFSEL_R &= ~0xFF;     								// disable alt funct on H
  GPIO_PORTH_DEN_R |= 0xFF;        								// enable digital I/O on H
  GPIO_PORTH_AMSEL_R &= ~0xFF;     								// disable analog functionality on H		
	return;
}

void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; //activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){}; //allow time for clock to stabilize
	GPIO_PORTM_DIR_R = 0b00000000; // Make PM0 inputs, reading if the button is pressed or not
	GPIO_PORTM_DEN_R = 0b00000001; // Enable PM0
	return;
}

int scan(){
	int dir;
	int input = 0b1;
	
	while(1){	
		input = GPIO_PORTM_DATA_R & 0b1;
		if (input == 0b0){ // check if input is low (active low setup)
			dir = 0;
			return dir;
		}
	}
}

void spinCW(int steps, int speed){
	
	for(int i=0; i<(steps/4); i++){
		GPIO_PORTH_DATA_R = 0b00001100; // motor sequence
		SysTick_Wait1ms(speed); // delay to control speed
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait1ms(speed);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait1ms(speed);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait1ms(speed);
	}

}

void spinCCW(int steps, int speed){ // used to return motor to prev position so the wires dont break
	for(int i=0; i<(steps/4); i++){
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait1ms(speed);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait1ms(speed);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait1ms(speed);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait1ms(speed);
	}
}


int main(void) {
	uint8_t sensorState=0; // useful variables
  uint16_t Distance;
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init();
	PortM_Init();

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(100);
  }
	FlashAllLEDs();

	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);


  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
	
	for (int z = 0; z < 10; z++){ // number of z-axis measurements
		if (scan() == 0){ // press button to begin spin measurement
			for(int i = 0; i < 32; i++) { // we take 32 measurements
				while (dataReady == 0){
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
					FlashLED1(1);
					VL53L1_WaitMs(dev, 2);
				}

					dataReady = 0;
					status = VL53L1X_GetRangeStatus(dev, &RangeStatus); // make sure status ok
					status = VL53L1X_GetDistance(dev, &Distance); // get the distance
					FlashLED1(1); 

					distances[i] = Distance; // put distance in array
					status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
					spinCW(64, 2); // spin 11.25 deg -> 32 measurements
			}
			
			static char signal;
			
			for (int i = 0; i < 32; i++){ // print values to uart one by one from array
				sprintf(printf_buffer,"%u,", distances[i]);
				signal = UART_InChar(); // wait for acknowledgement from PC
				if (signal == 0x31){
					UART_printf(printf_buffer); // send the measurement
				}
			}
			
			spinCCW(2048, 2); // spin back
		}
	}
	
  VL53L1X_StopRanging(dev); // cleanup
}


#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3

                                                                                // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;    // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)

}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. 
// Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

