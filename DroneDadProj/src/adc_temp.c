/**
 * \file
 *
 * \brief ADC Temperature Sensor configuration file
 *
 * Copyright (C) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "adc_temp.h"

/* Structure for ADC module instance */
extern struct adc_module adc_inst;


/*  The following variables have referred with respect to device data sheet 
	Equation1 and Equation1b on section "Temperature Sensor Characteristics" 
	of Electrical Characteristics */

float coarse_temp; /* Coarse value of the temperature - tempC */ 
float fine_temp;   /* Finer value of the temperature - tempF */

float tempR;       /* Production Room Temperature value read from NVM memory - tempR */
float tempH;	   /* Production Hot Temperature value read from NVM memory - tempH */
float INT1VR;      /* Room temp 2?s complement of the internal 1V reference value - INT1VR */
float INT1VH;	   /* Hot temp 2?s complement of the internal 1V reference value - INT1VR */
uint16_t ADCR;     /* Production Room Temperature ADC Value read from NVM memory - ADCR */
uint16_t ADCH;     /* Production Hot Temperature ADC Value read from NVM memory - ADCH */

float VADCR;	   /* Room Temperature ADC voltage - VADCR */
float VADCH;	   /* Hot Temperature ADC voltage - VADCH */

/**
* \brief ADC Temperature Sensor mode configuration.
* This function enables internal temperature sensor feature of ADC with below Settings

* GLCK for ADC		-> GCLK_GENERATOR_1 (8MHz)
* CLK_ADC			-> 512 KHz
* REFERENCE			-> internal 1 V
* POSITIVE INPUT	-> INTRENAL Temperature reference
* NEGATIVE INPUT	-> 
* SAMPLES			-> 4
* SAMPLE_LENGTH		-> 4
*/

void configure_adc_temp(void)
{
	struct adc_config conf_adc;
	
	adc_get_config_defaults(&conf_adc);
	
	conf_adc.clock_source = GCLK_GENERATOR_1;
	conf_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
	conf_adc.reference = ADC_REFERENCE_INT1V;
	conf_adc.positive_input = ADC_POSITIVE_INPUT_TEMP;
	conf_adc.negative_input = ADC_NEGATIVE_INPUT_GND;
	conf_adc.sample_length = ADC_TEMP_SAMPLE_LENGTH;
	
	adc_init(&adc_inst, ADC, &conf_adc);
	
	ADC->AVGCTRL.reg = ADC_AVGCTRL_ADJRES(2) | ADC_AVGCTRL_SAMPLENUM_4;
	
	adc_enable(&adc_inst);
}

/**
* \brief Decimal to Fraction Conversation.
* This function converts the decimal value into fractional 
* and return the fractional value for temperature calculation
*/

/**
* \brief Calibration Data.
* This function extract the production calibration data information from
* Temperature log row content and store it variables for temperature calculation
*
*/


/**
* \brief Temperature Calculation.
* This function calculate fine temperature using Equation1 and Equation
* 1b as mentioned in data sheet section "Temperature Sensor Characteristics"
* of Electrical Characteristics.
*
*/
int calculate_temperature(int adc_result)
{	
	int temp = (adc_result *1000)/(4095*25);
	return temp;
}
