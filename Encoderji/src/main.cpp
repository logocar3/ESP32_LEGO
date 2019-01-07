#include <Arduino.h>
#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include "driver/pcnt.h"

#define RELOAD_TMR      		1
#define PCNT_PULSE_GPIO				32		// gpio for PCNT
#define PCNT_CONTROL_GPIO			35
//#define DIRECTION					25		// gpio for encoder direction input
// 
#define PCNT_H_LIM_VAL      32767 //int16
#define PCNT_L_LIM_VAL     -32767


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
 pcnt_config_t pcnt_config0 = {
		        .pulse_gpio_num = PCNT_PULSE_GPIO,
		        .ctrl_gpio_num = PCNT_CONTROL_GPIO,
            .lctrl_mode = PCNT_MODE_KEEP, 			// Reverse counting direction if low
		        .hctrl_mode = PCNT_MODE_REVERSE,    		            	// Keep the primary counter mode if high
            .pos_mode = PCNT_COUNT_INC,   			// Count up on the positive edge
		        .neg_mode = PCNT_COUNT_DEC,   			// Keep the counter value on the negative edge  
		        .counter_h_lim = PCNT_H_LIM_VAL,
		        .counter_l_lim = PCNT_L_LIM_VAL,
            .unit = PCNT_UNIT_0,
            .channel = PCNT_CHANNEL_0,
		        
		        
		        
		    };
        pcnt_config_t pcnt_config1 = {
		    
            .pulse_gpio_num = PCNT_CONTROL_GPIO,
		        .ctrl_gpio_num = PCNT_PULSE_GPIO,
            .lctrl_mode = PCNT_MODE_REVERSE, 			// Reverse counting direction if low
		        .hctrl_mode = PCNT_MODE_KEEP,    		            	// Keep the primary counter mode if high
            .pos_mode = PCNT_COUNT_INC,   			// Count up on the positive edge
		        .neg_mode = PCNT_COUNT_DEC,   			// Keep the counter value on the negative edge  
		        .counter_h_lim = PCNT_H_LIM_VAL,
		        .counter_l_lim = PCNT_L_LIM_VAL,
            .unit = PCNT_UNIT_0,
            .channel = PCNT_CHANNEL_1,
		        
		    };
       

    pcnt_unit_config(&pcnt_config0);
    pcnt_unit_config(&pcnt_config1);
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    pcnt_intr_enable(PCNT_UNIT_0);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT_0);

}

void loop() {
  // put your main code here, to run repeatedly:
 
 int16_t count = 0;

 while(1){

   pcnt_get_counter_value(PCNT_UNIT_0, &count);
               printf("Current counter value :%d\n", count);
 }


}