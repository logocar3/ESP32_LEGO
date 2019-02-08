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
#define PCNT_PULSE_GPIO1				32		// gpio for PCNT
#define PCNT_CONTROL_GPIO1			35
#define PCNT_PULSE_GPIO2				25		// gpio for PCNT
#define PCNT_CONTROL_GPIO2			33
//#define DIRECTION					25		// gpio for encoder direction input
// 
#define PCNT_H_LIM_VAL      32767 //int16
#define PCNT_L_LIM_VAL     -32767

// desni motor
int MDP1 = 27; //motor desni pin 1 na h-bridge
int MDP2 = 26; //motor desni pin 2
int enableDESNI = 14; 

// levi motor
int MLP3 = 34; //motor levi pin 3
int MLP4 = 12; //motor levi pin 4
int enableLEVI = 13; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannelL = 0;
const int pwmChannelD = 1;
const int resolution = 8;
int dutyCycleL = 120;
int dutyCycleD = 120;

void setup() {
  Serial.begin(9600);
        pcnt_config_t pcnt_config0 = {
		        .pulse_gpio_num = PCNT_PULSE_GPIO1,
		        .ctrl_gpio_num = PCNT_CONTROL_GPIO1,
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
		    
            .pulse_gpio_num = PCNT_CONTROL_GPIO1,
		        .ctrl_gpio_num = PCNT_PULSE_GPIO1,
            .lctrl_mode = PCNT_MODE_REVERSE, 			// Reverse counting direction if low
		        .hctrl_mode = PCNT_MODE_KEEP,    		            	// Keep the primary counter mode if high
            .pos_mode = PCNT_COUNT_INC,   			// Count up on the positive edge
		        .neg_mode = PCNT_COUNT_DEC,   			// Keep the counter value on the negative edge  
		        .counter_h_lim = PCNT_H_LIM_VAL,
		        .counter_l_lim = PCNT_L_LIM_VAL,
            .unit = PCNT_UNIT_0,
            .channel = PCNT_CHANNEL_1, 
		    };
        pcnt_config_t pcnt_config2 = {
		        .pulse_gpio_num = PCNT_PULSE_GPIO2,
		        .ctrl_gpio_num = PCNT_CONTROL_GPIO2,
            .lctrl_mode = PCNT_MODE_KEEP, 			// Reverse counting direction if low
		        .hctrl_mode = PCNT_MODE_REVERSE,    		            	// Keep the primary counter mode if high
            .pos_mode = PCNT_COUNT_INC,   			// Count up on the positive edge
		        .neg_mode = PCNT_COUNT_DEC,   			// Keep the counter value on the negative edge  
		        .counter_h_lim = PCNT_H_LIM_VAL,
		        .counter_l_lim = PCNT_L_LIM_VAL,
            .unit = PCNT_UNIT_1,
            .channel = PCNT_CHANNEL_0,  
		    };
        pcnt_config_t pcnt_config3 = {
		    
            .pulse_gpio_num = PCNT_CONTROL_GPIO2,
		        .ctrl_gpio_num = PCNT_PULSE_GPIO2,
            .lctrl_mode = PCNT_MODE_REVERSE, 			// Reverse counting direction if low
		        .hctrl_mode = PCNT_MODE_KEEP,    		            	// Keep the primary counter mode if high
            .pos_mode = PCNT_COUNT_INC,   			// Count up on the positive edge
		        .neg_mode = PCNT_COUNT_DEC,   			// Keep the counter value on the negative edge  
		        .counter_h_lim = PCNT_H_LIM_VAL,
		        .counter_l_lim = PCNT_L_LIM_VAL,
            .unit = PCNT_UNIT_1,
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

        pcnt_unit_config(&pcnt_config2);
        pcnt_unit_config(&pcnt_config3);
        pcnt_set_filter_value(PCNT_UNIT_1, 100);
        pcnt_filter_enable(PCNT_UNIT_1);

        pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_ZERO);
        pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
        pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);

        /* Initialize PCNT's counter */
        pcnt_counter_pause(PCNT_UNIT_1);
        pcnt_counter_clear(PCNT_UNIT_1);

        pcnt_intr_enable(PCNT_UNIT_1);

        /* Everything is set up, now go to counting */
        pcnt_counter_resume(PCNT_UNIT_1);
        pinMode(MLP3, OUTPUT);
        pinMode(MLP4, OUTPUT);
        pinMode(enableLEVI, OUTPUT);
        ledcSetup(pwmChannelL, freq, resolution);
        ledcAttachPin(enableLEVI, pwmChannelL);

        pinMode(MDP1, OUTPUT);
        pinMode(MDP2, OUTPUT);
        pinMode(enableDESNI, OUTPUT);
        ledcSetup(pwmChannelD, freq, resolution);
        ledcAttachPin(enableDESNI, pwmChannelD);  

}

void loop() {
 int16_t enkoderL = 0;
 int16_t enkoderD = 0;
 int napaka = 0; //razilka med enkoderiji
 int K=dutyCycleD; //ojacanje
    

 while(enkoderD<18000){
    
    pcnt_get_counter_value(PCNT_UNIT_0, &enkoderL);
    pcnt_get_counter_value(PCNT_UNIT_1, &enkoderD);
    napaka = enkoderD-enkoderL;
    printf("napaka: %d\n",napaka);
    dutyCycleL = dutyCycleL + napaka/K;
    //desni motor
    digitalWrite(MDP1, LOW);
    digitalWrite(MDP2, HIGH);
    ledcWrite(pwmChannelD, dutyCycleD);
    //levi motor
    digitalWrite(MLP3, LOW);
    digitalWrite(MLP4, HIGH);
    ledcWrite(pwmChannelL, dutyCycleL);
    delay(100); 
               printf("encoder levi:%d \t encoder desni:%d \n", enkoderL,enkoderD);
 }
 while(1){
    
    
    //desni motor
    digitalWrite(MDP1, LOW);
    digitalWrite(MDP2, LOW);
    
    //levi motor
    digitalWrite(MLP3, LOW);
    digitalWrite(MLP4, LOW);
    
    delay(100); 
              
 }
}