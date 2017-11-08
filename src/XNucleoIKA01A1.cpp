/**
 ******************************************************************************
 * @file    XNucleoIKA01A1.cpp
 * @author  AST / Software Platforms and Cloud
 * @version V1.0
 * @date    October 1st, 2015
 * @brief   Implementation file for the X_NUCLEO_IKA01A1 expansion board.
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


/* Includes ------------------------------------------------------------------*/

/* ACTION 1 ------------------------------------------------------------------*
 * Include here platform specific header files.                               *
 *----------------------------------------------------------------------------*/
#include "Arduino.h"
/* ACTION 2 ------------------------------------------------------------------*
 * Include here expansion board specific header files.                        *
 *----------------------------------------------------------------------------*/
#include "XNucleoIKA01A1.h"


/* Variables -----------------------------------------------------------------*/

/* Singleton instance of XNucleoIKA01A1 class. */
XNucleoIKA01A1 *XNucleoIKA01A1::_instance;


/* Methods -------------------------------------------------------------------*/

/**
 * @brief Constructor.
 */
XNucleoIKA01A1::XNucleoIKA01A1(uint8_t instrumentAmpPin, 
							   uint8_t currentSensorPin, 
							   uint8_t photoSensorPin, 
							   uint8_t windCmpSignalPin_1,
							   uint8_t windCmpSignalPin_2,
							   uint8_t ledDriverPin)
{
	/* Instantiating the components. */
	/* ACTION 3 --------------------------------------------------------------*
	 * Instantiate here the expansion board's components.                     *
	 *                                                                   	  *
	 * Example:                                                          	  *
	 *   component_1 = new COMPONENT_1();                                     *
	 *   component_2 = new COMPONENT_2();                                     *
	 *------------------------------------------------------------------------*/
	tsz124 = new TSZ124(instrumentAmpPin, currentSensorPin);
	tsu104 = new TSU104(photoSensorPin, windCmpSignalPin_1, windCmpSignalPin_2);
	tsv734 = new TSV734(ledDriverPin);
}

/**
 * @brief Getting a singleton instance of XNucleoIKA01A1 class.
 * @retval a singleton instance of XNucleoIKA01A1 class.
 */
XNucleoIKA01A1 *XNucleoIKA01A1::instance(uint8_t instrumentAmpPin, 
										 uint8_t currentSensorPin, 
										 uint8_t photoSensorPin, 
										 uint8_t windCmpSignalPin_1,
										 uint8_t windCmpSignalPin_2,
								  		 uint8_t ledDriverPin)
{
	if (_instance == NULL) {
		/* Instantiating the board. */
		_instance = new XNucleoIKA01A1(instrumentAmpPin, currentSensorPin, photoSensorPin, windCmpSignalPin_1, windCmpSignalPin_2, ledDriverPin);

		/* Initializing the components. */
		if (!_instance->init()) {
			Serial.print("Initialization of the X_NUCLEO_IKA01A1 expansion board failed.\n");
		}
	}

	return _instance;
}

/**
 * @brief Initializing the X_NUCLEO_IKA01A1 board.
 * @retval true if initialization is successful
 * @retval false otherwise.
 */
bool XNucleoIKA01A1::init(void)
{
	/* Initializing the components. */
	/* ACTION 4 --------------------------------------------------------------*
	 * Initialize here the expansion board's components.                 	  *
	 *                                                                   	  *
	 * Example:                                                          	  *
	 *   return (init_COMPONENT_1() && init_COMPONENT_2());              	  *
	 *------------------------------------------------------------------------*/
	//return (init_TSZ124() && init_TSU104() && init_TSV734());
	return true;
}


/**
 * @brief  Initialize the TSZ124 component.
 * @retval true if initialization is successful
 * @retval false otherwise
 */
bool XNucleoIKA01A1::init_TSZ124(void)
{
	/* Verifying identity. */
	uint8_t id = 0;
	int ret = tsz124->read_id(&id);
	if ((ret != COMPONENT_OK) || (id != I_AM_TSZ124)) {
		delete tsz124;
		tsz124 = NULL;
		return true;
	}

	/* Configuration. */
	void* init_structure = NULL;
	/* ACTION ----------------------------------------------------------------*
	 * Configure here the component's initialization structure.               *
	 *                                                                        *
	 * Example:                                                          	  *
	 *   init_structure.Property_1 = COMPONENT_PROPERY_1_INIT;                 *
	 *   init_structure.Property_N = COMPONENT_PROPERY_N_INIT;                 *
	 *------------------------------------------------------------------------*/

	/* Initialization. */
	if (tsz124->init(&init_structure) != COMPONENT_OK) {
		return false;
	}

	return true;
}

/**
 * @brief  Initialize the TSU104 component.
 * @retval true if initialization is successful
 * @retval false otherwise 
 */
bool XNucleoIKA01A1::init_TSU104(void)
{
	/* Verifying identity. */
	uint8_t id = 0;
	int ret = tsu104->read_id(&id);
	if ((ret != COMPONENT_OK) || (id != I_AM_TSU104)) {
		delete tsu104;
		tsu104 = NULL;
		return true;
	}

	/* Configuration. */
	void* init_structure = NULL;
	/* ACTION ----------------------------------------------------------------*
	 * Configure here the component's initialization structure.               *
	 *                                                                        *
	 * Example:                                                          	  *
	 *   init_structure.Property_1 = COMPONENT_PROPERY_1_INIT;                 *
	 *   init_structure.Property_N = COMPONENT_PROPERY_N_INIT;                 *
	 *------------------------------------------------------------------------*/

	/* Initialization. */
	if (tsu104->init(&init_structure) != COMPONENT_OK) {
		return false;
	}

	return true;
}

/**
 * @brief  Initialize the TSV734 component.
 * @retval true if initialization is successful
 * @retval false otherwise 
 */
bool XNucleoIKA01A1::init_TSV734(void)
{
	/* Verifying identity. */
	uint8_t id = 0;
	int ret = tsv734->read_id(&id);
	if ((ret != COMPONENT_OK) || (id != I_AM_TSV734)) {
		delete tsv734;
		tsv734 = NULL;
		return true;
	}

	/* Configuration. */
	void* init_structure = NULL;
	/* ACTION ----------------------------------------------------------------*
	 * Configure here the component's initialization structure.               *
	 *                                                                        *
	 * Example:                                                          	  *
	 *   init_structure.Property_1 = COMPONENT_PROPERY_1_INIT;                 *
	 *   init_structure.Property_N = COMPONENT_PROPERY_N_INIT;                 *
	 *------------------------------------------------------------------------*/

	/* Initialization. */
	if (tsv734->init(&init_structure) != COMPONENT_OK) {
		return false;
	}

	return true;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
