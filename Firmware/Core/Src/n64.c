#include "n64.h"
#include  "main.h"



static LL_GPIO_InitTypeDef  LL_GPIO;

uint32_t cmd = 0x11111117;
uint32_t response;

n64_controller_t controller;


static uint16_t packet[] = { 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,0x0100, 0x01fc, 0x01e0 };//0x01e0 };
static LL_GPIO_InitTypeDef LL_GPIO_Ini;

static uint16_t in_buff[40] = { 0 };

uint32_t respons = 0;

extern UART_HandleTypeDef huart1;

static uint8_t n64_read=0;


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

void n64_pin_out(void){
	LL_GPIO.Mode  = LL_GPIO_MODE_OUTPUT;
	LL_GPIO.Pull  = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

	LL_GPIO.Pin = LL_GPIO_PIN_4;
	LL_GPIO_Init(GPIOA, &LL_GPIO);
}


void n64_pin_in(void){
	LL_GPIO.Mode  = LL_GPIO_MODE_INPUT;
	LL_GPIO.Pull  = LL_GPIO_PULL_UP;
	LL_GPIO.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

	LL_GPIO.Pin = LL_GPIO_PIN_4;
	LL_GPIO_Init(GPIOA, &LL_GPIO);
}


void n64_start_transmission(void){

	TIMx_CLK_ENABLE();

	ErrorStatus returned;
	LL_GPIO.Mode  = LL_GPIO_MODE_OUTPUT;
	LL_GPIO.Pull  = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

	LL_GPIO.Pin = LL_GPIO_PIN_5;
	returned = LL_GPIO_Init(GPIOA, &LL_GPIO);

	uint32_t sysclk = HAL_RCC_GetSysClockFreq();
	uint32_t Fpclk1 = HAL_RCC_GetPCLK1Freq();

}

void n64_init(void){
	ErrorStatus returned;
	LL_GPIO_Ini.Mode = LL_GPIO_MODE_OUTPUT;
	LL_GPIO_Ini.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Ini.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	LL_GPIO_Ini.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

	LL_GPIO_Ini.Pin = LL_GPIO_PIN_8;
	returned = LL_GPIO_Init(GPIOA, &LL_GPIO_Ini);
	LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_8);


	LL_GPIO_Ini.Mode = LL_GPIO_MODE_OUTPUT;
	LL_GPIO_Ini.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Ini.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Ini.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Ini.Pin = LL_GPIO_PIN_10;
		returned = LL_GPIO_Init(GPIOA, &LL_GPIO_Ini);
		LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_10);


}

/*uint32_t n64_send(uint32_t data){

	n64_pin_out();
	uint32_t response = 0;

	//send data
	for (int8_t i=31;i>=0;i--){
		if(data&(1<<i)){
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
		}else{
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
		}
		delay_1ms();
	}

	//send stop
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
	delay_1ms();
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
	n64_pin_in();

	return;

	while(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4));
	read_delay_1ms();
	read_delay_1ms();

	for (int8_t i=0;i<32;i++){
		//LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
		//read_delay_1ms();
		//LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
		LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);
			if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4)){
				response|=(1<<i);
			}
			read_delay_1ms();
			read_delay_1ms();
			read_delay_1ms();
			read_delay_1ms();
		}

	return response;


}*/

uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void n64schedule_update(void){
	n64_read = 1;
}

void n64_update(void){
	if(!n64_read){
		return;
	}
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
	//__disable_irq();
	//response = n64_send(0x11111117);
	//controller = * ((n64_controller_t*) &response);
	//__enable_irq();
	HAL_StatusTypeDef status;

	n64_controller_t pad;
	uint32_t response;

	// HAL_GPIO_TogglePin(LEDx_GPIO_PORT, LED2_PIN);
	/* Insert delay 100 ms */
	//n64_update();
	//HAL_GPIO_TogglePin(LEDx_GPIO_PORT, LED3_PIN);
	//LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_4);
	//LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
 	HAL_HalfDuplex_EnableTransmitter(&huart1);
	//status=HAL_UART_Transmit(&huart3,packet, sizeof(packet),1000);
	status = HAL_UART_Transmit(&huart1, (uint8_t*) packet, 8, 2);
	if (status != HAL_OK) {
		Error_Handler();
	}
	//while ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) ? SET : RESET) == RESET);

	huart1.Instance->DR = (uint16_t)(0x01e0 & 0x01FFU);
	//if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET, tickstart, 2) != HAL_OK)
	//while ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) ? SET : RESET) == RESET);
	while(!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_9));
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);

	//while ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) ? SET : RESET) == RESET);
	//__disable_irq();
	//LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
	//for (int i = 0; i < 5; i++) {
	//	asm("nop");
	//}
	HAL_HalfDuplex_EnableReceiver(&huart1);
	//LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
	//for (int i = 0; i < 10; i++) {
	//	asm("nop");
	//}
	//__enable_irq();


	//HAL_UART_Transmit(&huart3,packet, sizeof(packet),1000);

//	for (int i = 0; i < 10; i++) {
//		asm("nop");
//	}
	status = HAL_UART_Receive(&huart1, (uint8_t *) in_buff, 33, 2);
	if (status == HAL_OK) {
		//Error_Handler();
		respons = 0;
		for (int i = 0; i < 32; i++) {
			if (in_buff[i] & (1 << 5))
				respons |= (1 << i);
		}

		controller = *((n64_controller_t*) &respons);
		controller.AxisX = reverse(controller.AxisX);
		controller.AxisY = reverse(controller.AxisY);
		controller.AxisY = (uint8_t)(((uint16_t)controller.AxisY+1)*-1);



	}
	else if (status == HAL_TIMEOUT){
		__HAL_UART_FLUSH_DRREGISTER(&huart1);
	}
	n64_read=0;
	//HAL_USART_Transmit(&huart3,packet, sizeof(packet),1000);

	//HAL_Delay(10);
	/* Insert delay 100 ms */
	//HAL_Delay(1000);

	}

void n64_update_buffer(uint8_t* buffer){
	uint16_t sw_buttons=0;

	uint8_t RightX =127,RightY =127, LeftX=127,LeftY=127;

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



	//int16_t helperX, helperY;
	LeftY = (uint8_t) ((uint16_t)(controller.AxisY+128));
	//LeftY = (uint8_t) (((int16_t)LeftY) - 255);
	//LeftX =
	//LeftX = (uint8_t)(((int8_t) reverse(controller.AxisY))+128);



	if((controller.ButtonL) &(controller.ButtonLeft)){
		sw_buttons|=SWITCH_CAPTURE;
	}
	if((controller.ButtonL) &(controller.ButtonUp)){
		sw_buttons|=SWITCH_SELECT;
	}
	if((controller.ButtonL) &(controller.ButtonRight)){
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
