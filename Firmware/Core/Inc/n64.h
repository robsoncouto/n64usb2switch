#ifndef __N64_H
#define __N64_H

#include "main.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

void delay_1ms(void);


typedef struct {
	unsigned ButtonA : 1;
	unsigned ButtonB : 1;
	unsigned ButtonZ : 1;
	unsigned ButtonStart : 1;
	unsigned ButtonUp : 1;
	unsigned ButtonDown : 1;
	unsigned ButtonLeft : 1;
	unsigned ButtonRight : 1;
	unsigned ButtonReset : 1;
	unsigned ButtonZero : 1;
	unsigned ButtonL : 1;
	unsigned ButtonR : 1;
	unsigned ButtonCUp : 1;
	unsigned ButtonCDown : 1;
	unsigned ButtonCLeft : 1;
	unsigned ButtonCRight : 1;
	int8_t AxisX;
	int8_t AxisY;
}n64_controller_t;


//https://github.com/progmem/Switch-Fightstick/blob/master/Joystick.h
// Type Defines
// Enumeration for joystick buttons.
typedef enum {
SWITCH_Y       = 0x01,
SWITCH_B       = 0x02,
SWITCH_A       = 0x04,
SWITCH_X       = 0x08,
SWITCH_L       = 0x10,
SWITCH_R       = 0x20,
SWITCH_ZL      = 0x40,
SWITCH_ZR      = 0x80,
SWITCH_SELECT  = 0x100,
SWITCH_START   = 0x200,
SWITCH_LCLICK  = 0x400,
SWITCH_RCLICK  = 0x800,
SWITCH_HOME    = 0x1000,
SWITCH_CAPTURE = 0x2000,
} JoystickButtons_t;



uint8_t reverse(uint8_t b);

uint32_t n64_send(uint32_t);

/* User can use this section to tailor TIMx instance used and associated 
   resources */
/* Definition for TIMx clock resources */

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM9_IRQn
#define TIMx_IRQHandler                TIM9_IRQHandler


#define TIMx                           TIM9
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM9_CLK_ENABLE()

void n64_main_loop(void);

void n64_init(void);
void n64_prepare_hid_report(uint8_t* buffer);
void n64schedule_update(void);

extern TIM_HandleTypeDef    TimHandle;
#endif
