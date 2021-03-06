/**
 ******************************************************************************
 * @file    LedDriver.h
 * @author  AST
 * @version V1.0.0
 * @date    April 13th, 2015
 * @brief   This file contains the abstract class describing the interface of a
 *          led_driver component.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Generated with STM32CubeTOO -----------------------------------------------*/


/* Revision ------------------------------------------------------------------*/
/*
	Repository:       http://svn.x-nucleodev.codex.cro.st.com/svnroot/X-NucleoDev
	Branch/Trunk/Tag: trunk
	Based on:         X-CUBE-IKA01A1/trunk/Drivers/BSP/Components/Common/led_driver.h
	Revision:         385
*/


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __LED_DRIVER_CLASS_H
#define __LED_DRIVER_CLASS_H


/* Includes ------------------------------------------------------------------*/

#include <Component.h>


/* Classes  ------------------------------------------------------------------*/

/** An abstract class for LedDriver components.
 */
class LedDriver : public Component
{
public:
	/* ACTION 1 --------------------------------------------------------------*
	 * Declare here the interface's methods.                                  *
	 * They should be:                                                        *
	 *   + Methods with the same name of the C component's virtual table      *
	 *     (and extended virtual table, if any)'s functions, provided that    *
	 *     the component's driver implements them (i.e.: the corresponding    *
	 *     pointer to function is not "0").                                   *
	 *                                                                        *
	 * Example:                                                               *
	 *    virtual int get_value(float *f) = 0;                                *
	 *------------------------------------------------------------------------*/
	 
	/**
	 * @brief  abstract method for setting the duty cycle of LED in LED driver configuration
 	 * @param  dc duty cycle to set 
	 * @retval duty cycle in LED driver configuration
	 */		 
	virtual double set_duty_cycle(double dc) = 0;

    /**
     * @brief Destructor.
     */
    virtual ~LedDriver() {};
};

#endif /* __LED_DRIVER_CLASS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

