// schematic: http://www.ti.com/general/docs/lit/getliterature.tsp?baseLiteratureNumber=spmu365&fileType=pdf


// TODO: Make sure struct is being filled with values (seems like it isnt)
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c129encpdt.h"
#include "inc/hw_memmap.h"
//#include "inc/hw_ints.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h" 
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/rhr_pin_map.h" 
#include "driverlib/interrupt.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "dymxl_map.h"

#define MAX_RX_PAYLOAD		512
#define NUMBER_OF_INTS      1000 //  timer length = 1 ms 


//~~~~~~~~~~~~~~~Function Declarations~~~~~~~~~~~~~~~~~~~~~~~~~//
void UARTWrite(uint32_t BASE, char* buffer, uint32_t size); // TESTED. WORKS
void UARTPollRead(uint32_t BASE, char* read_buffer, uint32_t size); // TESTED WORKS
void PingDymxl(uint32_t BASE,uint8_t id); // TESTED WORKS
void DymxlTx(const uint8_t id, const uint8_t instruction, const uint8_t *params, const uint8_t params_len, const uint32_t BASE); //TESTED WORKS
void DymxlRequestReadData(const uint8_t start_idx, const uint8_t len, uint8_t id); //TESTED WORKS
void DymxlReadDataBlocking(const uint8_t start_idx, const uint8_t len, uint8_t id);
void DymxlWriteData(const uint8_t start_idx, const uint8_t len, const uint8_t *data, uint8_t id);
void UARTReadRingBuffer(void); // Working with timer interrupt
void RxParser(uint8_t byte); // WORKING 
void UARTDispatchPacket(void); // PRELIMS TEST WORKING
void UARTTestPing(void); // TESTED WORKS 

//--------------------Global--Vars----------------------------------//
typedef enum
{
	RX_PREABLE0, RX_PREABLE1, RX_ID, RX_LEN, RX_ERROR,
	RX_PAYLOAD, RX_CHECKSUM 
}uart_parser_state_t; 
static uart_parser_state_t uart_parser_state = RX_PREABLE0;
static uint16_t expected_packet_length = 0; 
static uint8_t uart_parser_payload[MAX_RX_PAYLOAD]; 
static uint16_t uart_parser_payload_write_pos =0; 
static uint8_t uart_parser_rx_checksum = 0;  
uint8_t dymxl_id = 0;
volatile int ISR_flag; 
volatile uint8_t ring_uart_read_buffer[UART_RX_RING_LEN];
volatile uint16_t ring_uart_read_pos =0;
volatile uint16_t ring_uart_write_pos =0; 
uint8_t dymxl_rx_params[MAX_RX_PAYLOAD];
uint32_t g_ui32SysClock; 
char buf[32];
char *buffer = buf; //
uint32_t g_freq = 120000000;
int state = 0; 
uint8_t broadcast = 0xFE;
uint8_t g_requested_start_idx;
uint8_t g_request_len; 
volatile uint8_t g_finished_dispatch = 0; 
static volatile uint32_t g_ui32Counter = 0;


//~~~~~~~~~~~~~~~~~~Structs~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
typedef struct dymxl_control_table_t
{
	uint8_t id; 
	uint8_t baud; 
	uint16_t cw_angle_limit;
	uint16_t ccw_angle_limit;
	uint16_t max_torque; 
	uint8_t torque_en; 
	uint8_t led; 
	uint8_t kp; 
	uint8_t ki; 
	uint8_t kd; 
	uint16_t goal_pos; 
	uint16_t velocity; 
	uint16_t torque_limit; 
	uint16_t cur_position; 
	uint16_t cur_vel; 
	uint16_t cur_load; 
	uint8_t cur_voltage; 
	uint8_t cur_temp; 
	uint16_t current; 
	uint8_t torque_control_en; 
	uint16_t goal_torque; 
	uint8_t goal_accel; 
}dymxl_control_table_t;

dymxl_control_table_t g_dymxl;



//-----------------Program------------------------------------------//
int main()
{   
 // specifies a system reset when using external reset pin 
    SysCtlResetBehaviorSet(SYSCTL_ONRST_EXT_SYS);

//------------------set sys clock to 120 MHZ ------------------------//
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ| SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), g_freq); 

    
 //-----------------Enable Periph-------------------------------------//
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); 
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); 
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3); 
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7); 
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);


	ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC);
	ROM_TimerLoadSet(TIMER0_BASE, TIMER_B, g_freq/12000); 
	ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
	ROM_UARTIntEnable(UART3_BASE,UART_INT_RX);
	
    ROM_IntPrioritySet(INT_UART3, 0);
    ROM_IntPrioritySet(INT_TIMER0B,1);
    ROM_IntEnable(INT_UART3);  
    ROM_IntEnable(INT_TIMER0B);



 //-------------Set Pins ---------------------------//
   	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_4);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_5); 
    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    ROM_GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1); 

    ROM_GPIOPinConfigure(GPIO_PC4_U7RX); 
    ROM_GPIOPinConfigure(GPIO_PC5_U7TX);
    ROM_GPIOPinConfigure(GPIO_PJ0_U3RX); 
    ROM_GPIOPinConfigure(GPIO_PJ1_U3TX);

    
    ROM_UARTConfigSetExpClk(UART3_BASE, g_ui32SysClock, 1000000,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE|
                            UART_CONFIG_PAR_NONE));
    ROM_UARTConfigSetExpClk(UART7_BASE, g_ui32SysClock, 1000000,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | 
                            UART_CONFIG_PAR_NONE));

    ROM_TimerEnable(TIMER0_BASE, TIMER_B);
    ROM_UARTEnable(UART3_BASE);
    ROM_UARTEnable(UART7_BASE);
    

    ROM_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_4, GPIO_PIN_4); 
    ROM_IntMasterEnable(); 
    uart_parser_payload_write_pos = 0; 
	uart_parser_state = RX_PREABLE0;
	char write_buf[2];
	char* w_buf = write_buf;
	g_ui32Counter = 0;

// ---------------------MAIN LOOP -----------------------------------// 
    while(1)
    {
    	
    	DymxlReadDataBlocking(36,2,1); // need to turn timer off eventually 

       	state =!state;  
        if(state)
           	 ROM_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_4| GPIO_PIN_5, GPIO_PIN_4); 
       	else
         	 ROM_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_4| GPIO_PIN_5, GPIO_PIN_5); 
    	
    }
    

 	return 0; 
}
//--- Dynmxl Functions ----// 
void DymxlTx(const uint8_t id, const uint8_t instruction,
	         const uint8_t *params, const uint8_t params_len, const uint32_t BASE)
{
	uint8_t checksum = 0; 
	uint8_t tx_buf[256];
	uint8_t* tbuf = tx_buf; 
	ROM_GPIODirModeSet(GPIO_PORTJ_BASE, GPIO_PIN_1, GPIO_DIR_MODE_HW);
	//ROM_GPIODirModeSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN); // find disable reciver 
	HWREG(BASE + UART_O_CTL) &= ~UART_CTL_RXE;


	tx_buf[0] = 0xff;
	tx_buf[1] = 0xff;
	tx_buf[2] = id;
	tx_buf[3] = 2 + params_len;
	tx_buf[4] = instruction;
	checksum = 2+ params_len + id + instruction;
	
	for (uint8_t i = 0; i < params_len; i++)
	{
		tx_buf[5+i] = params[i];
		checksum += params[i];
	}
	
	checksum = ((~checksum) & 255);
	tx_buf[5+params_len] = checksum;
	UARTWrite(BASE,tbuf, 6+params_len);
	while(HWREG(BASE + UART_O_FR) & UART_FR_BUSY) { }
    ROM_GPIODirModeSet(GPIO_PORTJ_BASE, GPIO_PIN_1, GPIO_DIR_MODE_IN);
    HWREG(BASE + UART_O_CTL) |= UART_CTL_RXE;
}
void DymxlRequestReadData(const uint8_t start_idx, const uint8_t len, uint8_t id) // start address of param you want to read and # of byte
{
	g_request_len = len;
	g_requested_start_idx = start_idx; // requested data starting address
	// send a request to read LEN bytes starting at START_IDX from dxl control table
	const uint8_t params[2] = { start_idx, len };
	DymxlTx(id, 0x02, params, sizeof(params), UART3_BASE);
}

void DymxlReadDataBlocking(const uint8_t start_idx, const uint8_t len, uint8_t id) //TODO: go back and clean this up 
{	

	//DymxlRequestReadData(start_idx, len, id);
	for(int i =0; i <500; i++){ // sends them all then dynmxl replies to all at once out of phase 
		DymxlRequestReadData(start_idx,len,id);
		for(volatile int j = 0; j <1000; j++){}
	} 
	UARTReadRingBuffer();
	//while(g_ui32Counter<10){} // this works 
	while(!g_finished_dispatch){ // this is stuck in infinite loop 

		UARTReadRingBuffer();
	}
	
	g_ui32Counter = 0;
	g_finished_dispatch = 0; 

} 

void DymxlWriteData(const uint8_t start_idx, const uint8_t len, const uint8_t *data, uint8_t id)
{
	// send LEN bytes which will be written to START_IDX in dxl control table
	
	g_requested_start_idx = start_idx; 

}


//---- Parsing Rx Data -----//
void UARTReadRingBuffer(void)
{
	uart_parser_payload_write_pos = 0; 
	while (ring_uart_read_pos != ring_uart_write_pos)
	{

		RxParser(ring_uart_read_buffer[ring_uart_read_pos]); 
		if (++ring_uart_read_pos >= UART_RX_RING_LEN)
			ring_uart_read_pos = 0; 
	}
	 
}
void RxParser(uint8_t byte)
{
	uint8_t error_code;
	switch(uart_parser_state)
	{
		case(RX_PREABLE0):
			if (byte == 0xFF)
			{
				uart_parser_state = RX_PREABLE1; 
			}
			break;
		case(RX_PREABLE1):
		   if (byte == 0xFF) 
		   {
		   		uart_parser_state = RX_ID;
		   }
		   else{
		   		uart_parser_state = RX_PREABLE0;
		   }
		   break;

		case(RX_ID):
			dymxl_id = byte;
			g_dymxl.id = byte; 
			uart_parser_state = RX_LEN; 
			break; 

		case(RX_LEN): 
			expected_packet_length = byte; 
			uart_parser_state = RX_ERROR;
			break; 
		case(RX_ERROR):
			error_code = byte; 
			if (error_code == 0){
				if (expected_packet_length == 2)
				{
					uart_parser_state  =RX_CHECKSUM;
				}
				else{
					uart_parser_state = RX_PAYLOAD;
					} 
			}
			else{
				// figureout something todo 
				uart_parser_state = RX_PAYLOAD; 
			}
			break;
		
		case(RX_PAYLOAD):
			//uart_parser_payload[uart_parser_payload_write_pos] = byte; 
			//uart_parser_payload_write_pos++;
			if(uart_parser_payload_write_pos+1 == (expected_packet_length-2)){ // length  =ID (already parsed)+instruction+ params
				uart_parser_state = RX_CHECKSUM; 
			}
			else if (uart_parser_payload_write_pos >= MAX_RX_PAYLOAD)
			{
				uart_parser_state = RX_PREABLE0;
			}
			uart_parser_payload[uart_parser_payload_write_pos] = byte; 
			uart_parser_payload_write_pos++;
			break;
		
		case(RX_CHECKSUM):
			uart_parser_rx_checksum = byte; 
			uint8_t local_checksum = 0; 
			for (int i = 0; i<(expected_packet_length-2); i++){
				local_checksum += uart_parser_payload[i]; 
			}
			local_checksum = local_checksum + expected_packet_length + dymxl_id;
			local_checksum = ((~local_checksum) & 0xFF); 
			if(local_checksum == uart_parser_rx_checksum){
				UARTDispatchPacket(); 
			}
			uart_parser_state = RX_PREABLE0; 
			break; 
		
		default: 
			uart_parser_state = RX_PREABLE0;
			break;   
	}
}

void UARTDispatchPacket(void) // request length that cuts in half high n low bytes 
{   

	const uint8_t len = uart_parser_payload_write_pos; 
	if (len == 0)
	{
		return; 
	}
	const uint8_t actual_length  = uart_parser_payload_write_pos;
	const uint8_t *payload = uart_parser_payload; 
	if(actual_length == g_request_len)
	{
		for(int i = 0; i < actual_length; i++)
		{
			switch (g_requested_start_idx+i)
			{
				case DYMXL_BAUD: 
					g_dymxl.baud=payload[i];

					break; 
				case DYMXL_CW_ANGLE_LIMIT_L: // maybe a flag  
					g_dymxl.cw_angle_limit = (uint16_t)((payload[i+1] << 8)|payload[i]); 
					i++;  
					break; 
				case DYMXL_CCW_ANGLE_LIMIT_L: 
					g_dymxl.ccw_angle_limit = (uint16_t)((payload[i+1] <<8)|payload[i]); 
					i++;
					break;
				case DYMXL_MAX_TORQUE_L: 
					g_dymxl.max_torque = (uint16_t)((payload[i+1] << 8)|payload[i]); 
					i++; 
					break; 
				case DYMXL_TORQUE_EN: 
					g_dymxl.torque_en = payload[i]; 
					break; 
				case DYMXL_LED: 
					g_dymxl.led = payload[i]; 
					break; 
				case DYMXL_D_GAIN: 
					g_dymxl.kd = payload[i]; 
					break; 
				case DYMXL_I_GAIN: 
					g_dymxl.ki = payload[i]; 
					break; 
				case DYMXL_P_GAIN: 
					g_dymxl.kp = payload[i]; 
					break; 
				case DYMXL_GOAL_POS_L: 
					g_dymxl.goal_pos = (uint16_t)((payload[i+1] << 8)|payload[i]);
					i++; 
					break;
				case DYMXL_VEL_L: 
					g_dymxl.velocity =(uint16_t)((payload[i+1] << 8)|payload[i]); 
					i++; 
					break; 
				case DYMXL_TORQUE_LIMIT_L: 
					g_dymxl.torque_limit = (uint16_t)((payload[i+1] << 8)|payload[i]);
					i++; 
					break; 
				case DYMXL_CUR_POS_L: 
					g_dymxl.cur_position = (uint16_t)((payload[i+1] << 8)|payload[i]);
					i++;
					break; 
				case DYMXL_CUR_VEL_L: 
					g_dymxl.cur_vel= (uint16_t)((payload[i+1] << 8)|payload[i]); 
					i++; 
					break; 
				case DYMXL_CUR_LOAD_L: 
					g_dymxl.cur_load = (uint16_t)((payload[i+1] << 8)|payload[i]);
					i++; 
					break; 
				case DYMXL_CUR_VOL:
					g_dymxl.cur_voltage = payload[i]; 
					break; 
				case DYMXL_CUR_TEMP: 
					g_dymxl.cur_temp = payload[i]; 
					break; 
				case DYMXL_CURRENT_L: 
					g_dymxl.current = (uint16_t)((payload[i+1] << 8)|payload[i]);
					i++; 
					break; 
				case DYMXL_TORQUE_CTRL_EN: 
					g_dymxl.torque_control_en = payload[i]; 
					break; 
				case DYMXL_GOAL_TORQUE_L: 
					g_dymxl.goal_torque = (uint16_t)((payload[i+1] << 8)|payload[i]); 
					i++; 
					break; 
				case DYMXL_GOAL_ACCEL: 
					g_dymxl.goal_accel = payload[i]; 
					break;  

				default: 
					//messedup payload 
				g_finished_dispatch = 1;
					break;

			}
		}
		g_finished_dispatch = 1;
	}
	else
	{
		g_finished_dispatch = 1; 
		return; 
	}		
	
}

//---- INT Handlers ----// 
void Timer0BIntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    //
    // Update the periodic interrupt counter.
    //
    g_ui32Counter++;

    //
    // Once NUMBER_OF_INTS interrupts have been received, turn off the
    // TIMER0B interrupt.
    //
    if(g_ui32Counter == NUMBER_OF_INTS)
    {
        //
        // Disable the Timer0B interrupt.
        //
        ROM_IntDisable(INT_TIMER0B);

        //
        // Turn off Timer0B interrupt.
        //
        ROM_TimerIntDisable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

        //
        // Clear any pending interrupt flag.
        //
        ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
    }
    
}

void UART3_int_handler(void)
{
	ISR_flag = 1;
	volatile uint8_t bit = UARTCharGet(UART3_BASE);
	ring_uart_read_buffer[ring_uart_write_pos] = bit; 
	if(++ring_uart_write_pos >= UART_RX_RING_LEN)
	{
		ring_uart_write_pos =0;
	}

	HWREG(UART3_BASE + UART_O_ICR) = UART_INT_RX;

}

//---UART TESTING FUNCTIONS---// 

void PingDymxl (uint32_t BASE,uint8_t id) 
{ 
	DymxlTx(id, 0x1, NULL, 0,UART3_BASE);

}
void UARTTestPing(void)
{

	char test_buf[256];
	char* t_buf = test_buf; 
	t_buf[0] = 0xff;
	t_buf[1] = 0xff;
	t_buf[2] = broadcast;
	t_buf[3] = 2;
	t_buf[4] = 0x01; 
	t_buf[5] = (~(broadcast+2+1)&0xFF);
	UARTWrite(UART3_BASE,t_buf,6);
}

void UARTWrite( uint32_t BASE, char* buffer, uint32_t size)
{
    for( int i = 0; i < size; i++)
    {
       UARTCharPut(BASE, buffer[i]); 
       
    } 
    //CLEAR the FIFO 
}

void UARTPollRead(uint32_t BASE, char* read_buffer, uint32_t size)
{
    while (UARTCharsAvail(BASE))
    {
        for (int i =0; i < size; i++)
        {
           
            read_buffer[i] = UARTCharGet(BASE);
         
            
        }
    }   
}