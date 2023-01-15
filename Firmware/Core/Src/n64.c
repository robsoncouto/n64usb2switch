#include "n64.h"
#include  "main.h"

n64_controller_t controller;


static uint16_t packet[] = { 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,0x0100, 0x01fc, 0x01e0 };
static LL_GPIO_InitTypeDef LL_GPIO_Ini;

static uint16_t in_buff[40] = { 0 };

uint32_t respons = 0;

extern UART_HandleTypeDef huart1;

static uint8_t n64_read_flag=0;


uint32_t  n64_8bit_2_32bit(uint8_t input){

	uint32_t output =0;
	uint one = 0x07;
	uint zero = 0x01;

	for (int i =0;i<8;i++){
		if(input&(1<<i)){
			output|= (one)<<(i*4);
		}else{
			output|= (zero)<<(i*4);
		}
	}

	return output;

}


void n64_init(void){
	LL_GPIO_Ini.Mode = LL_GPIO_MODE_OUTPUT;
	LL_GPIO_Ini.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Ini.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	LL_GPIO_Ini.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

	LL_GPIO_Ini.Pin = LL_GPIO_PIN_8;
	LL_GPIO_Init(GPIOA, &LL_GPIO_Ini);
	LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_8);


	LL_GPIO_Ini.Mode = LL_GPIO_MODE_OUTPUT;
	LL_GPIO_Ini.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Ini.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Ini.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Ini.Pin = LL_GPIO_PIN_10;
	LL_GPIO_Init(GPIOA, &LL_GPIO_Ini);
	LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_10);


}

uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void n64schedule_update(void){
	n64_read_flag = 1;
}

int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void n64_main_loop(void){
	static int8_t min_X=0,max_X=0,max_Y=0,min_Y=0;
	int16_t X=0,Y=0;
	if(!n64_read_flag){
		return;
	}
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
	HAL_StatusTypeDef status;
	HAL_HalfDuplex_EnableTransmitter(&huart1);
	status = HAL_UART_Transmit(&huart1, (uint8_t*) packet, 8, 1);
	if (status != HAL_OK) {
		Error_Handler();
	}
	//while ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) ? SET : RESET) == RESET);

	huart1.Instance->DR = (uint16_t)(0x01e0 & 0x01FFU);
	//if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET, tickstart, 2) != HAL_OK)
	//while ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) ? SET : RESET) == RESET);
	while(!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_9));
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);

	HAL_HalfDuplex_EnableReceiver(&huart1);

	status = HAL_UART_Receive(&huart1, (uint8_t *) in_buff, 33, 1);
	if (status == HAL_OK) {
		respons = 0;
		for (int i = 0; i < 32; i++) {
			if (in_buff[i] & (1 << 5))
				respons |= (1 << i);
		}

		controller = *((n64_controller_t*) &respons);
		controller.AxisX = reverse(controller.AxisX);
		controller.AxisY = reverse(controller.AxisY);

		controller.AxisY = (uint8_t)(((uint16_t)controller.AxisY+1)*-1);

		if(controller.AxisX<min_X) min_X = controller.AxisX;
		if(controller.AxisX>max_X) max_X = controller.AxisX;
		if(controller.AxisY<min_Y) min_Y = controller.AxisY;
		if(controller.AxisY>max_Y) max_Y = controller.AxisY;

		//compensates genuine controllers range
		//X = (127.0/83.0)*controller.AxisX;
		//Y = (127.0/83.0)*controller.AxisY;

		X=map(controller.AxisX,min_X,max_X,-128,127);
		Y=map(controller.AxisY,min_Y,max_Y,-128,127);

		controller.AxisX = (int8_t)X;
		controller.AxisY = (int8_t)Y;

	}
	else if (status == HAL_TIMEOUT){
		__HAL_UART_FLUSH_DRREGISTER(&huart1);
	}
	n64_read_flag=0;

	}

void n64_prepare_hid_report(uint8_t* buffer){
	uint16_t sw_buttons=0;

	uint8_t RightX =127,RightY =127;

	if(controller.ButtonA) sw_buttons|=SWITCH_A;
	if(controller.ButtonB) sw_buttons|=SWITCH_B;
	if(controller.ButtonStart) sw_buttons|=SWITCH_START;
	if(controller.ButtonL) sw_buttons|=SWITCH_L;
	if(controller.ButtonR) sw_buttons|=SWITCH_R;
	if(controller.ButtonZ) sw_buttons|=SWITCH_ZL;

	if(controller.ButtonCUp) RightY=0;
	if(controller.ButtonCDown) RightY=255;
	if(controller.ButtonCLeft) RightX=0;
	if(controller.ButtonCRight) RightX=255;

	uint8_t dpad = 0, hat =0;
	if(controller.ButtonUp) dpad |= 0x80;
	if(controller.ButtonDown) dpad |= 0x40;
	if(controller.ButtonLeft) dpad |= 0x10;
	if(controller.ButtonRight) dpad |= 0x20;

	switch(dpad & 0xF0) {
			case 0x80: // Top
				hat = 0x00;
				break;
			case 0xA0: // Top-Right
				hat = 0x01;
				break;
			case 0x20: // Right
				hat = 0x02;
				break;
			case 0x60: // Bottom-Right
				hat = 0x03;
				break;
			case 0x40: // Bottom
				hat= 0x04;
				break;
			case 0x50: // Bottom-Left
				hat = 0x05;
				break;
			case 0x10: // Left
				hat = 0x06;
				break;
			case 0x90: // Top-Left
				hat = 0x07;
				break;
			default:
				hat = 0x08;
	}

	//LeftY = (uint8_t) ((uint16_t)(controller.AxisY+128));


	if((controller.ButtonL) &(controller.ButtonZ)&(controller.ButtonLeft)){
		sw_buttons|=SWITCH_CAPTURE;
	}
	if((controller.ButtonL) &(controller.ButtonZ)&(controller.ButtonUp)){
		sw_buttons|=SWITCH_SELECT;
	}
	if((controller.ButtonL) &(controller.ButtonZ)&(controller.ButtonRight)){
		sw_buttons|=SWITCH_HOME;
	}

	buffer[0] = sw_buttons&0xff;
	buffer[1] = (sw_buttons>>8)&0xff;
	buffer[2] = hat;
	buffer[3] = (uint8_t)(((int8_t) controller.AxisX)+128);
	buffer[4] = (uint8_t)(((int8_t) controller.AxisY)+128);
	buffer[5] = RightX;
	buffer[6] = RightY;

}
