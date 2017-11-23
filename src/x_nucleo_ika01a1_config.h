/**
 ******************************************************************************
 * @file    x_nucleo_ika01a1_config.h
 * @author  AST / Software Platforms and Cloud
 * @version V1.0
 * @date    October 1st, 2015
 * @brief   Configuration header file for the X_NUCLEO_IKA01A1 expansion board.
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

#ifndef __X_NUCLEO_IKA01A1_CONFIG_H
#define __X_NUCLEO_IKA01A1_CONFIG_H


/* Definitions ---------------------------------------------------------------*/

/* ACTION --------------------------------------------------------------------*
 * Specify here a configuration for I/O and interrupts' pins.                 *
 *                                                                            *
 * Example:                                                                   *
 *   // I2C.                                                                  *
 *   #define EXPANSION_BOARD_PIN_I2C_SCL  (D15)                               *
 *   #define EXPANSION_BOARD_PIN_I2C_SDA  (D14)                               *
 *                                                                            *
 *   // SPI.                                                                  *
 *   #define EXPANSION_BOARD_PIN_SPI_MOSI (D11)                               *
 *   #define EXPANSION_BOARD_PIN_SPI_MISO (D12)                               *
 *   #define EXPANSION_BOARD_PIN_SPI_SCLK (D13)                               *
 *                                                                            *
 *   // Interrupts.                                                           *
 *   #define EXPANSION_BOARD_PIN_INT_1    (A2)                                *
 *----------------------------------------------------------------------------*/
 
/* instrumentation amplifier configuration */
#define X_NUCLEO_IKA01A1_PIN_INSTRUMENTATION_AMP (A1)

/* Current sensing configuration */
#define X_NUCLEO_IKA01A1_PIN_CURRENT_SENSING (A2)
  
/* Photo sensor configuration output pin. */
#define X_NUCLEO_IKA01A1_PIN_PHOTO_SENSOR (A3)

/* Windows  */
#define X_NUCLEO_IKA01A1_PIN_WINDOWS_COMP_SIGNAL_1 (D2)
#define X_NUCLEO_IKA01A1_PIN_WINDOWS_COMP_SIGNAL_2 (D4)

/* LED Driver Configuration PWM out pin. */
#define X_NUCLEO_IKA01A1_PIN_PWM_LED_DRIVER_OUTPUT (D3)

#endif /* __X_NUCLEO_IKA01A1_CONFIG_H */
