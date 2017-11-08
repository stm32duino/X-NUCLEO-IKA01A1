/**
 ******************************************************************************
 * @file    XNucleoIKA01A1.h
 * @author  AST / Software Platforms and Cloud
 * @version V1.0
 * @date    October 1st, 2015
 * @brief   Class header file for the X_NUCLEO_IKA01A1 expansion board.
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


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __X_NUCLEO_IKA01A1_CLASS_H
#define __X_NUCLEO_IKA01A1_CLASS_H


/* Includes ------------------------------------------------------------------*/

/* ACTION 1 ------------------------------------------------------------------*
 * Include here platform specific header files.                               *
 *----------------------------------------------------------------------------*/
#include "Arduino.h"
/* ACTION 2 ------------------------------------------------------------------*
 * Include here expansion board configuration's header files.                 *
 *----------------------------------------------------------------------------*/
#include "XNucleoIKA01A1.h"
/* ACTION 3 ------------------------------------------------------------------*
 * Include here expansion board's components' header files.                   *
 *                                                                            *
 * Example:                                                                   *
 *   #include "COMPONENT_1.h"                                                 *
 *   #include "COMPONENT_2.h"                                                 *
 *----------------------------------------------------------------------------*/
#include "TSZ124.h"
#include "TSU104.h"
#include "TSV734.h"


/* Classes -------------------------------------------------------------------*/

/** Class XNucleoIKA01A1 represents multifunctional expansion board based on operational amplifiers. 
  * It provides an easy-to-use and affordable solution for different multifunctional use cases with your 
  * STM32 Nucleo board. For current sensing configuration and the instrumentation amplifier configuration, 
  * a highly accurate operational amplifier (TSZ124) is used. The expansion board also contains Nanopower (TSU104) 
  * and Micropower (TSV734) operational amplifiers for mobile applications.
  * It is intentionally implemented as a singleton because only one
  * XNucleoIKA01A1 at a time might be deployed in a HW component stack.\n
  * In order to get the singleton instance you have to call class method `Instance()`, 
  * e.g.:
  * @code
  * // Inertial & Environmental expansion board singleton instance
  * static XNucleoIKA01A1 *<TODO>_expansion_board = XNucleoIKA01A1::instance();
  * @endcode  
  * 
 */
class XNucleoIKA01A1
{
public:

	/*** Instance, Initialization and Destructor Methods ***/

	/**
	 * @brief Getting a singleton instance of XNucleoIKA01A1 class.
	 * @retval a singleton instance of XNucleoIKA01A1 class.
	 */
	static XNucleoIKA01A1 *instance(uint8_t instrumentAmpPin, uint8_t currentSensorPin, uint8_t photoSensorPin, uint8_t windCmpSignalPin_1,
	                            	uint8_t windCmpSignalPin_2, uint8_t ledDriverPin);

	/**
	 * @brief Initialize the singleton's operational amplifiers to default settings
	 * @retval true if initialization is successful
	 * @retval false otherwise.
	 */
	bool init(void);

	/**
	 * @brief Destructor.
	 */
	~XNucleoIKA01A1(void) {}


	/*** Public Expansion Board Related Attributes ***/

	/* ACTION 4 --------------------------------------------------------------*
	 * Declare here a public attribute for each expansion board's component.  *
	 * You will have to call these attributes' public methods within your     *
	 * main program.                                                          *
	 *                                                                        *
	 *   Example:                                                             *
	 *     COMPONENT_1 *component_1;                                          *
	 *     COMPONENT_2 *component_2;                                          *
	 *------------------------------------------------------------------------*/
	TSZ124 *tsz124;
	TSU104 *tsu104;
	TSV734 *tsv734;


protected:

	/*** Protected Constructor Method ***/

	/**
	 * @brief Constructor.
	 */
	XNucleoIKA01A1(uint8_t instrumentAmpPin, uint8_t currentSensorPin, uint8_t photoSensorPin, uint8_t windCmpSignalPin_1,
	               uint8_t windCmpSignalPin_2, uint8_t ledDriverPin);


	/*** Protected Expansion Board Related Initialization Methods ***/

	/* ACTION 5 --------------------------------------------------------------*
	 * Declare here a protected initialization method for each expansion      *
	 * board's component.                                                     *
	 *                                                                        *
	 * Example:                                                               *
	 *   bool init_COMPONENT_1(void);                                         *
	 *   bool init_COMPONENT_2(void);                                         *
	 *------------------------------------------------------------------------*/
	bool init_TSZ124(void);
	bool init_TSU104(void);
	bool init_TSV734(void);


	/*** Component's Instance Variables ***/

	/* Singleton instance of XNucleoIKA01A1 class. */
	static XNucleoIKA01A1 *_instance;
};

#endif /* __X_NUCLEO_IKA01A1_CLASS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/