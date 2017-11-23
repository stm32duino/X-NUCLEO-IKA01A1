/**
 ******************************************************************************
 * @file    led_driver.h
 * @author  Central Labs
 * @version 1.0.0
 * @date    11-February-2016
 * @brief   Portable architecture for TSZ124
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


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __LED_DRIVER
#define __LED_DRIVER

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include "component_def.h"


/* Types ---------------------------------------------------------------------*/

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */
    
/** @addtogroup INSTRUMENTATION_AMP
  * @{
  */

/** @defgroup INSTRUMENTATION_AMP_Exported_Types
  * @{
  */

/**
 * @brief  INSTRUMENTATION_AMP component identifier enumerator definition.
 */
typedef enum
{
	LED_DRIVER_NONE_COMPONENT = 0,
	LED_DRIVER_TSV734_COMPONENT = 1
} LED_DRIVER_component_t;

/** 
 * @brief  LED_DRIVER driver virtual table structure definition.
 */
typedef struct
{
	/* ACTION ----------------------------------------------------------------*
	 * Declare here the component's generic functions.                        *
	 * Tag this group of functions with the " Generic " C-style comment.      *
	 * A component's interface has to define at least the two generic         *
	 * functions provided here below within the "Example" section, as the     *
	 * first and second functions of its Virtual Table. They have to be       *
	 * specified exactly in the given way.                                    *
	 *                                                                        *
	 * Example:                                                               *
	 *   status_t (*Init)   (void *handle, void *init);                       *
	 *   status_t (*ReadID) (void *handle, uint8_t *id);                      *
	 *------------------------------------------------------------------------*/
	/* Generic */
	status_t (*Init)      (void *handle, void *init);
	status_t (*ReadID)    (void *handle, uint8_t *id);

	/* ACTION ----------------------------------------------------------------*
	 * Declare here the component's specific functions.                       *
	 * Tag this group of functions with the " Specific " C-style comment.     *
	 * Do not specify any function if not required.                           *
	 *                                                                        *
	 * Example:                                                               *
	 *   status_t (*GetValue) (void *handle, float *f);                       *
	 *------------------------------------------------------------------------*/
        /* Specific */
	double (*SetDutyCycle) (void *handle, double dc);
} LED_DRIVER_vt_t;

#ifdef __cplusplus
}
#endif

#endif /* __LED_DRIVER */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

