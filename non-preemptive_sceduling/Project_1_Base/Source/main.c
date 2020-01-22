/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "gpio_defs.h"
#include "LEDs.h"
#include "i2c.h"
#include "mma8451.h"
#include "delay.h"
#include "RTCS.h"

#define FLASH_DELAY 2
#define ACC_SENSITIVITY 90
#define TASK_MOTION_SENSOR_FREQ_HZ (50) 
#define TASK_I2C_SERVER_FREQ_HZ (50)

void Init_Debug_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

	PORTB->PCR[DBG_0] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_1] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_2] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_3] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_4] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_5] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_6] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_7] |= PORT_PCR_MUX(1);          

	PTB->PDDR |= MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);
	PTB->PCOR = MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);

}


void Init_Config_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	PORTE->PCR[CONFIG1_POS] &= ~PORT_PCR_PE_MASK;          
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_PE(1);	
	PORTE->PCR[CONFIG1_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_MUX(1);
	PTE->PDDR &= ~MASK(CONFIG1_POS);

	PORTE->PCR[CONFIG2_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG2_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG2_POS);

	PORTE->PCR[CONFIG3_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG3_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG3_POS);
}
void Task_Motion_Sensor(void) {
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	static int16_t acc_X=0, acc_Y=0, acc_Z=0;
	static uint8_t rf, gf, bf;
	SET_BIT(DEBUG_TASK_MOTION_SENSOR);
	read_full_xyz(&acc_X, &acc_Y, &acc_Z);
	rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
	gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
	bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;
	Control_RGB_LEDs(rf, gf, bf);
	Delay(FLASH_DELAY);
	Control_RGB_LEDs(0, 0, 0);							
	Delay(FLASH_DELAY*2);		
	prev_acc_X = acc_X;
	prev_acc_Y = acc_Y;
	prev_acc_Z = acc_Z;
  CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);
}
void Task_Motion_Sensor_FSM(void) {
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0,reset=0;
	static int16_t acc_X=0, acc_Y=0, acc_Z=0;
	static uint8_t rf, gf, bf;
	
	uint8_t mma_accel_data[1];
	uint8_t data[6];
	int16_t temp[3];
	int i;
	
	static enum {S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11} next_state = S1;
	static int response_delay=0;
	SET_BIT(DEBUG_TASK_MOTION_SENSOR)
	switch (next_state) {
		case S1:   // Upon RESET checking the device by giving READ Command
						if (reset == 0){
							if (g_I2C_Msg.Status == IDLE) {
								g_I2C_Msg.Dev_adx = MMA_ADDR;
								g_I2C_Msg.Reg_adx = REG_WHOAMI;
								g_I2C_Msg.Data_count = 1 ;
								g_I2C_Msg.Command = READ ;
								RTCS_Task_Table[1].ReleasesPending++; // Releasing I2C Task for READ command 
								next_state = S2;
	
								} else {
											RTCS_Task_Table[0].ReleasesPending++; // Releasing itself
											// stay in this state, waiting for IDLE Status
								}
						} else {   //If not RESET then go to State 4 for Reading Accelerometer
									RTCS_Task_Table[0].ReleasesPending++;  // Releasing itself
									next_state = S4;
						}
						break;
		case S2:  // Waiting for READ COMPLETE
						if (g_I2C_Msg.Status == READ_COMPLETE) {
								mma_accel_data[0] = g_I2C_Msg.Data[0];
							  g_I2C_Msg.Status = IDLE;  // Changing Status to IDLE after copying DATA
								if (mma_accel_data[0] != WHOAMI)	{
										// error code
										Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
										while (1)														  	/* not able to initialize mma */
											;
									}	
								Delay(1); // for accelerometer 
								//set active mode, 14 bit samples, 2g full scale, low noise and 800 Hz ODR 
								mma_accel_data[0] = 0x05;
								
								if (g_I2C_Msg.Status == IDLE) {
									g_I2C_Msg.Dev_adx = MMA_ADDR;
									g_I2C_Msg.Reg_adx = REG_CTRL1;
									g_I2C_Msg.Data_count = 1 ;
									g_I2C_Msg.Data[0] = mma_accel_data[0] ;
									g_I2C_Msg.Command = WRITE ;            // Giving WRITE Command
									RTCS_Task_Table[1].ReleasesPending++;  // Releasing I2C Task
									next_state = S3;
								}
							}
						break;
		case S3:   // Waiting for WRITE COMPLETE
						Control_RGB_LEDs(0, 0, 0);
						if (g_I2C_Msg.Status == WRITE_COMPLETE){
						  g_I2C_Msg.Status = IDLE;
							next_state = S4;
							reset = 1;
							RTCS_Task_Table[0].ReleasesPending++;     //Release itself
						} else {
									RTCS_Task_Table[0].ReleasesPending++; //Release itself
									// stay in this state, waiting for WRITE_COMPLETE Status 
						}
						break;
		case S4:   // READ Command given to read Accelerometer readings
						if (g_I2C_Msg.Status == IDLE) {
								g_I2C_Msg.Dev_adx = MMA_ADDR;
								g_I2C_Msg.Reg_adx = REG_XHI;
								g_I2C_Msg.Data_count = 6 ;
								g_I2C_Msg.Command = READ ;             //Giving READ Command
								RTCS_Task_Table[1].ReleasesPending++;  //Realease I2C Task
								next_state = S5;

						} else {
									RTCS_Task_Table[0].ReleasesPending++; //Release itself
									// stay in this state, waiting for IDLE Status
						}
						break;
		case S5:  //Waiting for Read Complete and Proccesing the Accelerometer Data
						if (g_I2C_Msg.Status == READ_COMPLETE) {
							for ( i=0; i<6; i++) {	
							data[i] = g_I2C_Msg.Data[i];
							}
							g_I2C_Msg.Status = IDLE;
							for ( i=0; i<3; i++ ) {
									temp[i] = (int16_t) ((data[2*i]<<8) | data[2*i+1]);
							}
							// Align for 14 bits
							acc_X = temp[0]/4;
							acc_Y = temp[1]/4;
							acc_Z = temp[2]/4;
										
							rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
							gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
							bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;

							Control_RGB_LEDs(rf, gf, bf);   //RGB LED glow Command given
							next_state=S6;
						}
						RTCS_Task_Table[0].ReleasesPending++; // Release itself
						break;
		case S6:  //Delay assigned
							//Delay(FLASH_DELAY);   // If Delay function used.
						RTCS_Task_Table[0].TicksToNextRelease=2;
						next_state=S7;
						break;
		case S7:  //RGB LED OFF 
						Control_RGB_LEDs(0, 0, 0);
						RTCS_Task_Table[0].ReleasesPending++; // Release itself
						next_state=S8;
						break;
		case S8:  // Delay assigned			
							//Delay(FLASH_DELAY*2);		 // If Delay function used
						RTCS_Task_Table[0].TicksToNextRelease=4;
						next_state=S9;
						break;
		case S9:  //Storing the XYZ values
						prev_acc_X = acc_X;
						prev_acc_Y = acc_Y;
						prev_acc_Z = acc_Z;
						RTCS_Task_Table[0].ReleasesPending++;  //Release itself
		        next_state = S4;
						break;
		default: 
						if (reset == 0){				
							RTCS_Task_Table[0].ReleasesPending++; //Release itself
							next_state = S1;
						} else {
									RTCS_Task_Table[0].ReleasesPending++; //Release itself
									next_state = S4;
						}
						break;

	}
	CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);
}

	/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	
	Init_RGB_LEDs();
	Init_Debug_Signals();
	Init_Config_Signals();

	Control_RGB_LEDs(1, 1, 0);								/* yellow: starting up */
	i2c_init();																/* init i2c	*/
	
	Delay(200);
	
	RTCS_Init(SCHED_FREQ_HZ);
	if (PTE->PDIR & MASK(CONFIG1_POS)){       // Checking for PTE3 Bit, If Floating run DEMO Code
		if (!init_mma()) {											/* init mma peripheral */
			Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
			while (1)																/* not able to initialize mma */
			;
		 }
			RTCS_Add_Task(Task_Motion_Sensor, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
	} else{	                                  // If PTE3 Bit is grounded run FSM Code
				RTCS_Add_Task(Task_Motion_Sensor_FSM, 0, 0); // Task Moton Sensor with Higher Priority and 0 Period for Event trigerring
				RTCS_Add_Task(Task_I2C_Server, 1, 0);        // I2C Task with Lower Priority and 0 Period for Event trigerring
	}
	RTCS_Run_Scheduler();
	
}