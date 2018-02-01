/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    LEDs_PIT.c
 * @brief   Application entry point.
 */
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_pit.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

#define LED_RED_PIN 22
#define LED_BLUE_PIN 21
#define LED_GREEN_PIN 20
#define LED_BLUE_PORT PORTB
#define LED_RED_PORT PORTB
#define LED_GREEN_PORT PORTE
#define LED_RED_CLOCK_PORT kCLOCK_PortB
#define LED_BLUE_CLOCK_PORT kCLOCK_PortB
#define LED_GREEN_CLOCK_PORT  kCLOCK_PortE


#define SW2_PIN 4
#define SW3_PIN 6
#define SW2_PORT PORTA
#define SW3_PORT PORTC
#define SW2_CLOCK_PORT kCLOCK_PortA
#define SW3_CLOCK_PORT kCLOCK_PortC
#define SW2_INTERRUPT_FLAG_PIN (1 << SW2_PIN)
#define SW3_INTERRUPT_FLAG_PIN (1 << SW3_PIN)



enum {LED_ON, LED_OFF};
enum {FALSE, TRUE};
typedef struct color {
	uint8_t led_rojo;
	uint8_t led_verde;
	uint8_t led_azul;
}color;

typedef struct  {
	uint8_t bandera_congelado;
	uint8_t bandera_siguiente;
	struct estado* estado_anterior;
	struct color color_del_led;
	struct estado* estado_siguiente;
	uint8_t cambiar;
}estado;

enum {COLOR_SIGUIENTE, COLOR_ANTERIOR};
enum {NO_CONGELADO, CONGELADO};
static volatile color color_rojo ={
		LED_ON,
		LED_OFF,
		LED_OFF,
};

static volatile color color_azul = {
		LED_OFF,
		LED_OFF,
		LED_ON,
};

static volatile color color_verde ={
		LED_OFF,
		LED_ON,
		LED_OFF,
};


 void encender_led( estado Estado){
	 GPIO_PinWrite(GPIOB, LED_RED_PIN,  Estado.color_del_led.led_rojo);
	 GPIO_PinWrite(GPIOB, LED_BLUE_PIN, Estado.color_del_led.led_azul);
	 GPIO_PinWrite(GPIOE, LED_GREEN_PIN, Estado.color_del_led.led_verde);
 }

 static estado * estado_actual = 0;



 void PORTA_IRQHandler(){
 	GPIO_ClearPinsInterruptFlags(GPIOA, SW2_INTERRUPT_FLAG_PIN);
 	estado_actual->bandera_congelado = CONGELADO;
 }

 void PORTC_IRQHandler(){
 	GPIO_ClearPinsInterruptFlags(GPIOC, SW3_INTERRUPT_FLAG_PIN);
 	if(COLOR_SIGUIENTE == estado_actual->bandera_siguiente){
 			estado_actual->bandera_siguiente = COLOR_ANTERIOR;
 	}else{
 		estado_actual->bandera_siguiente = COLOR_SIGUIENTE;
 	}

 }

 void PIT0_IRQHandler (){
	 PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	 estado_actual->cambiar = TRUE;
	 PIT_StartTimer(PIT, kPIT_Chnl_0);
 }

int main(void) {
	 estado estado_rojo ;

	 estado estado_azul;

	 estado estado_verde;




	estado_rojo.bandera_congelado = NO_CONGELADO;
	estado_rojo.bandera_siguiente =  COLOR_SIGUIENTE;
	estado_rojo.estado_anterior =	&estado_verde;
	estado_rojo.color_del_led =		color_rojo;
	estado_rojo.estado_siguiente =	&estado_azul;
	estado_rojo.cambiar = FALSE;

	estado_azul.bandera_congelado = NO_CONGELADO;
	estado_azul.bandera_siguiente =  COLOR_SIGUIENTE;
	estado_azul.estado_anterior =	&estado_rojo;
	estado_azul.color_del_led =		color_azul;
	estado_azul.estado_siguiente =	&estado_verde;
	estado_azul.cambiar = FALSE;


	estado_verde.bandera_congelado = NO_CONGELADO;
	estado_verde.bandera_siguiente =  COLOR_SIGUIENTE;
	estado_verde.estado_anterior =	&estado_azul;
	estado_verde.color_del_led =		color_verde;
	estado_verde.estado_siguiente =	&estado_rojo;
	estado_verde.cambiar = FALSE;

	estado_actual = &estado_rojo;
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

/*
 * Se habilitan los relojes de cada uno de los periféricos.
 */
   CLOCK_EnableClock(LED_RED_CLOCK_PORT);
   CLOCK_EnableClock(LED_BLUE_CLOCK_PORT);
   CLOCK_EnableClock(LED_GREEN_CLOCK_PORT);

   /* Output pin PORT configuration*/
   port_pin_config_t config_blue_led = {
		   kPORT_PullDisable,
       kPORT_FastSlewRate,
       kPORT_PassiveFilterDisable,
       kPORT_OpenDrainDisable,
       kPORT_LowDriveStrength,
       kPORT_MuxAsGpio,
       kPORT_UnlockRegister,
   };

   port_pin_config_t config_red_led = {
		   kPORT_PullDisable,
       kPORT_FastSlewRate,
       kPORT_PassiveFilterDisable,
       kPORT_OpenDrainDisable,
       kPORT_LowDriveStrength,
       kPORT_MuxAsGpio,
       kPORT_UnlockRegister,
   };


   port_pin_config_t config_green_led = {
  		   kPORT_PullDisable,
         kPORT_FastSlewRate,
         kPORT_PassiveFilterDisable,
         kPORT_OpenDrainDisable,
         kPORT_LowDriveStrength,
         kPORT_MuxAsGpio,
         kPORT_UnlockRegister,
     };

   /* Sets the configuration*/
   PORT_SetPinConfig(LED_BLUE_PORT, LED_BLUE_PIN, &config_blue_led);
   PORT_SetPinConfig(LED_RED_PORT, LED_RED_PIN, &config_red_led);
   PORT_SetPinConfig(LED_GREEN_PORT, LED_GREEN_PIN, &config_green_led);




   /*
    * Configuración de los botones.
    */

   CLOCK_EnableClock(SW2_CLOCK_PORT);
   CLOCK_EnableClock(SW3_CLOCK_PORT);

   port_pin_config_t config_SW3 = {
  		   kPORT_PullDisable,
         kPORT_FastSlewRate,
         kPORT_PassiveFilterDisable,
         kPORT_OpenDrainDisable,
         kPORT_LowDriveStrength,
         kPORT_MuxAsGpio,
         kPORT_UnlockRegister,
     };
   port_pin_config_t config_SW2 = {
  		   kPORT_PullDisable,
         kPORT_FastSlewRate,
         kPORT_PassiveFilterDisable,
         kPORT_OpenDrainDisable,
         kPORT_LowDriveStrength,
         kPORT_MuxAsGpio,
         kPORT_UnlockRegister,
     };

   PORT_SetPinConfig(SW2_PORT, SW2_PIN, &config_SW2);
   PORT_SetPinConfig(SW3_PORT, SW3_PIN, &config_SW3);

   /*
    * Congifuracion del PIT 0
    */

   pit_config_t config_pit_0;
   PIT_GetDefaultConfig(&config_pit_0);
   CLOCK_EnableClock(kCLOCK_Pit0);
   PIT_Init(PIT, &config_pit_0);
   uint32_t clock_fq = CLOCK_GetFreq(kCLOCK_Pit0);
   uint8_t seconds = 1;
   uint32_t pit_0_clock_counts = ((uint32_t) seconds ) * clock_fq;
   PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, pit_0_clock_counts);
   PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
   PIT_StartTimer(PIT, kPIT_Chnl_0);
   EnableIRQ(PIT0_IRQn);

	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);


    for(;;)	{
    	if(TRUE == estado_actual->cambiar){
    		if(NO_CONGELADO == estado_actual->bandera_congelado){
    		    		if(COLOR_SIGUIENTE == estado_actual->bandera_siguiente){
    		    			estado_actual = estado_actual->bandera_siguiente;
    		    		}else{
    		    			estado_actual = estado_actual->estado_anterior;
    		    		}
    		    		encender_led(*estado_actual);
    		    	}
    		estado_actual->cambiar = FALSE;
    	}



    }
    return 0 ;
}
