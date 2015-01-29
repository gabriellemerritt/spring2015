//******************************************************************************
// dymxl_mem_map.h - Mapping of Dynamixel instructions 
//
//  
//
//
//
//
//
//
//
//
//
//
//******************************************************************************
//
#ifndef __DYMXL_MAP_H__
#define __DYMXL_MAP_H__

//!~~~~~~~~~~~~~~~~~~~~ COMMANDS~~~~~~~~~~~~~~~~~~~~~~~~~~!

#define HEADER              0xFF

#define PING                0x01

#define READ_DATA           0x02 // requires 2 params 

#define WRITE_DATA          0x03 // requires >2 params 

#define REG_WRITE           0x04 // similar to write data but dynamixel stays in standby state also requries >2 params 

#define ACTION              0x05 // 0 params use with regwrite

#define RESET               0x06 // restores to factory default setting

#define SYNC_WRITE          0x83 // control more than 1 dynamixels at a time >4 params 

//! ~~~~~~~~~~~~~~~~~~~~ADDRESS MEMMAP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~!
#define DYMXL_MODEL_L 				0x00
#define DYMXL_MODEL_H				0x01
#define DYMXL_FIRM				    0x02
#define DYMXL_ID_NUM				0x03
#define DYMXL_BAUD 					0x04
#define DYMXL_RETURN_DELAY			0x05
#define DYMXL_CW_ANGLE_LIMIT_L		0x06
#define DYMXL_CW_ANGLE_LIMIT_H		0x07
#define DYMXL_CCW_ANGLE_LIMIT_L		0x08
#define DYMXL_CCW_ANGLE_LIMIT_H		0x09
#define DYMXL_HI_LIMIT_TEMP 		0x0B
#define DYMXL_LOW_LIMIT_VOL			0x0C
#define DYMXL_HIGH_LIMIT_VOL		0x0D
#define DYMXL_MAX_TORQUE_L			0x0E
#define DYMXL_MAX_TORQUE_H			0x0F
#define DYMXL_STATUS_RETURN_LEVEL 	0x10
#define DYMXL_ALARM_LED				0x11
#define DYMXL_ALARM_SHUTDOWN		0x12
#define DYMXL_TORQUE_EN 			0x18
#define DYMXL_LED  					0x19
#define DYMXL_D_GAIN				0x1A
#define DYMXL_I_GAIN 				0x1B
#define DYMXL_P_GAIN				0x1C
#define DYMXL_GOAL_POS_L		    0x1E
#define DYMXL_GOAL_POS_H			0x1F
#define DYMXL_VEL_L 				0x20
#define DYMXL_VEL_H					0x21
#define DYMXL_TORQUE_LIMIT_L	    0x22
#define DYMXL_TORQUE_LIMIT_H		0x23
#define DYMXL_CUR_POS_L 			0x24 //READ ONLY
#define DYMXL_CUR_POS_H				0x25 // READ ONLY
#define DYMXL_CUR_VEL_L			    0x26 // READ ONLY
#define DYMXL_CUR_VEL_H				0x27 // READ ONLY
#define DYMXL_CUR_LOAD_L			0x28 // READ ONLY
#define DYMXL_CUR_LOAD_H			0x29 // READ ONLY
#define DYMXL_CUR_VOL			    0x2A // READ ONLY 
#define DYMXL_CUR_TEMP				0x2B // READ ONLY 
#define DYMXL_REGIS_INST			0x2C // READ ONLY is current instruction registered
#define DYMXL_MOVEMENT				0x2E // READ ONLY is the servo moving
#define DYMXL_LOCK				    0x2F // Lock EEProm
#define DYMXL_PUNCH_L				0x30
#define DYMXL_PUNCH_H				0x31
#define DYMXL_CURRENT_L				0x44
#define DYMXL_CURRENT_H			    0x45
#define DYMXL_TORQUE_CTRL_EN		0x46
#define DYMXL_GOAL_TORQUE_L			0x47
#define DYMXL_GOAL_TORQUE_H			0x48
#define DYMXL_GOAL_ACCEL		    0x49

//-----------ERROR DEFS-------------------------//
#define INPUT_ERROR 				0x01 
#define ANGLE_LIMIT_ERROR			0x02 
#define OVER_HEAT_ERROR				0x04 
#define CMD_RANGE_ERROR				0x08 
#define CHECK_SUM_ERROR				0x10 
#define OVERLOAD_ERROR 				0x20 
#define INSTRUCTION_ERROR			0x40


#define UART_RX_RING_LEN			32
extern volatile int ISR_flag; 
extern volatile uint8_t ring_uart_read_buffer[UART_RX_RING_LEN];
extern volatile uint16_t ring_uart_read_pos;
extern volatile uint16_t ring_uart_write_pos; 





//TODO: read the char in the function into the buffer 
void UART3_int_handler(void);
void Timer0BIntHandler(void);


#endif // __DYMXL_MAP_H__
