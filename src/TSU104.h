/**
 ******************************************************************************
 * @file    TSU104.h
 * @author  Central Labs
 * @version 1.0.0
 * @date    11-February-2016
 * @brief   Portable architecture for TSU104
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
	Based on:         X-CUBE-IKA01A1/trunk/Drivers/BSP/Components/tsu104/tsu104.h
	Revision:         402
*/


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __TSU104_CLASS_H
#define __TSU104_CLASS_H


/* Includes ------------------------------------------------------------------*/

/* ACTION 1 ------------------------------------------------------------------*
 * Include here platform specific header files.                               *
 *----------------------------------------------------------------------------*/		
#include "Arduino.h"
/* ACTION 2 ------------------------------------------------------------------*
 * Include here component specific header files.                              *
 *----------------------------------------------------------------------------*/		
#include "TSU104_def.h"
/* ACTION 3 ------------------------------------------------------------------*
 * Include here interface specific header files.                              *
 *                                                                            *
 * Example:                                                                   *
 *   #include "HumiditySensor.h"                                              *
 *   #include "TemperatureSensor.h"                                           *
 *----------------------------------------------------------------------------*/
#include "Windcomp.h"
#include "PhotoSensor.h"


/* Classes -------------------------------------------------------------------*/

/**
 * @brief Class representing a TSU104 operational amplifier component. The window comparator 
 * configuration allows the user to compare a signal to two threshold voltages. When the signal
 * is out of the required voltage range, the output of the operational amplifier toggles
 */
class TSU104 : public Windcomp, public PhotoSensor
{
public:

	/*** Constructor and Destructor Methods ***/

	/**
	 * @brief Constructor.
	 */
	TSU104(uint8_t photoSensorPin, uint8_t windCmpSignalPin_1,uint8_t windCmpSignalPin_2) : Windcomp(), photo_sensor_pin(photoSensorPin), windCmp_signal_pin_1(windCmpSignalPin_1), windCmp_signal_pin_2(windCmpSignalPin_2)
	{
		pinMode(windCmp_signal_pin_1, INPUT);
		pinMode(windCmp_signal_pin_2, INPUT);
	}
	
	/**
	 * @brief Destructor.
	 */
	virtual ~TSU104(void) {}
	

    /*** Public Component Related Methods ***/

    /* ACTION 5 --------------------------------------------------------------*
     * Implement here the component's public methods, as wrappers of the C    *
     * component's functions.                                                 *
     * They should be:                                                        *
     *   + Methods with the same name of the C component's virtual table's    *
     *     functions (1);                                                     *
     *   + Methods with the same name of the C component's extended virtual   *
     *     table's functions, if any (2).                                     *
     *                                                                        *
     * Example:                                                               *
     *   virtual int get_value(float *p_data) //(1)                           *
     *   {                                                                    *
     *     return COMPONENT_get_value(float *pf_data);                        *
     *   }                                                                    *
     *                                                                        *
     *   virtual int enable_feature(void) //(2)                               *
     *   {                                                                    *
     *     return COMPONENT_enable_feature();                                 *
     *   }                                                                    *
     *------------------------------------------------------------------------*/

    /**
     * @brief Public functions inherited from the Component Class
     */

	/**
	 * @brief initialize the class for TSU104 operational amplifier component
	 * @retval COMPONENT_OK if initialization is successfull
	 * @retval suitable error code otherwise	 
	 */	
	virtual int init(void *init = NULL)
	{
		return (int) TSU104_Init((void *) init);
	}
	
	/**
	 * @brief  obtain component ID for TSU104 perational amplifier
	 * @retval component ID for TSU104 operational amplifier
	 */		
	virtual int read_id(uint8_t *id = NULL)
	{
		return (int) TSU104_ReadID((uint8_t *) id);
	}

    /**
     * @brief Public functions inherited from the LedDriver Class
     */

	/**
	 * @brief  get digital signal value 1 in windows comparator configuration
	 * @retval digital signal value 1
	 */			
	virtual unsigned int get_signal1(void)
	{
		return digitalRead(windCmp_signal_pin_1);
	}

	/**
	 * @brief  get digital signal value 2 in windows comparator configuration
	 * @retval digital signal value 2
	 */			
	virtual unsigned int get_signal2(void)
	{
		return digitalRead(windCmp_signal_pin_2);
	}
	
	
	/**
	 * @brief  obtain the output voltage detected by photo sensor
	 * @param  None.
	 * @retval voltage detected by photo sensor
	 */		
	virtual unsigned int get_voltage(void)
	{
		double voltage = (double) (analogRead(photo_sensor_pin) / 1023.0);
		voltage = voltage * 3.3 * 1000.0;   // voltage in mV
		return voltage;
	}


	/*** Public Interrupt Related Methods ***/

	/* ACTION 6 --------------------------------------------------------------*
	 * Implement here interrupt related methods, if any.                      *
	 * Note that interrupt handling is platform dependent, e.g.:              *
	 *   + mbed:                                                              *
	 *     InterruptIn feature_irq(pin);           //Interrupt object.        *
	 *     feature_irq.fall(callback);             //Attach a callback.       *
	 *     feature_irq.mode(PullNone);             //Set interrupt mode.      *
	 *     feature_irq.enable_irq();               //Enable interrupt.        *
	 *     feature_irq.disable_irq();              //Disable interrupt.       *
	 *   + Arduino:                                                           *
	 *     attachInterrupt(pin, callback, RISING); //Attach a callback.       *
	 *     detachInterrupt(pin);                   //Detach a callback.       *
	 *                                                                        *
	 * Example (mbed):                                                        *
     *   void attach_feature_irq(void (*fptr) (void))                         *
     *   {                                                                    *
     *     feature_irq.rise(fptr);                                            *
     *   }                                                                    *
     *                                                                        *
     *   void enable_feature_irq(void)                                        *
     *   {                                                                    *
     *     feature_irq.enable_irq();                                          *
     *   }                                                                    *
     *                                                                        *
     *   void disable_feature_irq(void)                                       *
     *   {                                                                    *
     *     feature_irq.disable_irq();                                         *
     *   }                                                                    *
	 *------------------------------------------------------------------------*/


protected:

	/*** Protected Component Related Methods ***/

	/**
	 * arduino pin for photosensor output voltage
	 */
	uint32_t photo_sensor_pin;
	
	/**
	 * arduino pin for signal 1 of windows comparator configuration
	 */	
	uint32_t windCmp_signal_pin_1;
	
	/**
	 * arduino pin for signal 1 of windows comparator configuration
	 */		
	uint32_t windCmp_signal_pin_2;


	/* ACTION 7 --------------------------------------------------------------*
	 * Declare here the component's specific methods.                         *
	 * They should be:                                                        *
	 *   + Methods with the same name of the C component's virtual table's    *
	 *     functions (1);                                                     *
	 *   + Methods with the same name of the C component's extended virtual   *
	 *     table's functions, if any (2);                                     *
	 *   + Helper methods, if any, like functions declared in the component's *
	 *     source files but not pointed by the component's virtual table (3). *
	 *                                                                        *
	 * Example:                                                               *
     *   status_t COMPONENT_get_value(float *f);   //(1)                      *
     *   status_t COMPONENT_enable_feature(void);  //(2)                      *
     *   status_t COMPONENT_compute_average(void); //(3)                      *
	 *------------------------------------------------------------------------*/
	/* TSU104's generic functions. */
	status_t TSU104_Init(void *init);
	status_t TSU104_ReadID(void *id);

	/* TSU104's interrupts related functions. */
	status_t TSU104_ConfigIT(void* a);


	/*** Component's I/O Methods ***/

	/*** Component's Instance Variables ***/

	/* ACTION 9 --------------------------------------------------------------*
	 * Declare here interrupt related variables, if needed.                   *
	 * Note that interrupt handling is platform dependent, see                *
	 * "Interrupt Related Methods" above.                                     *
	 *                                                                        *
	 * Example:                                                               *
	 *   + mbed:                                                              *
	 *     InterruptIn feature_irq;                                           *
	 *------------------------------------------------------------------------*/

	/* ACTION 10 -------------------------------------------------------------*
	 * Declare here other pin related variables, if needed.                   *
	 *                                                                        *
	 * Example:                                                               *
	 *   + mbed:                                                              *
	 *     DigitalOut standby_reset;                                          *
	 *------------------------------------------------------------------------*/

	/* ACTION 11 -------------------------------------------------------------*
	 * Declare here communication related variables, if needed.               *
	 *                                                                        *
	 * Example:                                                               *
	 *   + mbed:                                                              *
	 *     DigitalOut address;                                                *
	 *     DevI2C &dev_i2c;                                                   *
	 *------------------------------------------------------------------------*/

	/* ACTION 12 -------------------------------------------------------------*
	 * Declare here identity related variables, if needed.                    *
	 * Note that there should be only a unique identifier for each component, *
	 * which should be the "who_am_i" parameter.                              *
	 *------------------------------------------------------------------------*/
	/* Identity */
	uint8_t who_am_i;

	/* ACTION 13 -------------------------------------------------------------*
	 * Declare here the component's static and non-static data, one variable  *
	 * per line.                                                              *
	 *                                                                        *
	 * Example:                                                               *
	 *   float measure;                                                       *
	 *   int instance_id;                                                     *
	 *   static int number_of_instances;                                      *
	 *------------------------------------------------------------------------*/
};

#endif /* __TSU104_CLASS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/