/**
 ******************************************************************************
 * @file    X_NUCLEO_IKA01A1_HelloWorld.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    October 17h, 2017
 * @brief   Arduino test application for the STMicroelectronics X-NUCLEO-IKA01A1
 *          Analog expansion board.
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


/* Includes ------------------------------------------------------------------*/

/* Arduino specific header files. */
#include "Arduino.h"

/* Board specific header files. */
#include "XNucleoIKA01A1.h"

#define SerialPort Serial

/* Variables -----------------------------------------------------------------*/

TSZ124 *instr_amp;
TSU104 *photo_sensor_wind_comp;
TSV734 *led_driver;
double duty_cycle;


/* Functions -----------------------------------------------------------------*/

/**
 * @brief  Initialization.
 * @param  None.
 * @retval None.
 */
void setup()
{
    /* Printing to the console. */
    SerialPort.begin(115200);
    SerialPort.print("Analog Application Example\r\n\n");

    /* Initializing Analog Expansion Board. */
    XNucleoIKA01A1 *analog_expansion_board = XNucleoIKA01A1::instance(
        X_NUCLEO_IKA01A1_PIN_INSTRUMENTATION_AMP,
        X_NUCLEO_IKA01A1_PIN_CURRENT_SENSING,
        X_NUCLEO_IKA01A1_PIN_PHOTO_SENSOR,
        X_NUCLEO_IKA01A1_PIN_WINDOWS_COMP_SIGNAL_1,
        X_NUCLEO_IKA01A1_PIN_WINDOWS_COMP_SIGNAL_2,
        X_NUCLEO_IKA01A1_PIN_PWM_LED_DRIVER_OUTPUT);
    analog_expansion_board->init();
    instr_amp = analog_expansion_board->tsz124;
    photo_sensor_wind_comp = analog_expansion_board->tsu104;
    led_driver = analog_expansion_board->tsv734;
    duty_cycle = 0;
}

/**
 * @brief  Main loop.
 * @param  None.
 * @retval None.
 */
void loop()
{         
    unsigned int op_amp_voltage = instr_amp->get_voltage();
    unsigned int op_amp_current = instr_amp->get_current();
    unsigned int photo_sensor_voltage = photo_sensor_wind_comp->get_voltage();                
    unsigned int wind_com_pin1 = photo_sensor_wind_comp->get_signal1();               
    unsigned int wind_com_pin2 = photo_sensor_wind_comp->get_signal2();                  
    
    SerialPort.print("\r\nOpAmp measured Voltage(mV): ");
    SerialPort.print(op_amp_voltage);
    SerialPort.print("\r\nOpAmp measured Current(mA): ");
    SerialPort.print(op_amp_current);
    SerialPort.print("\r\nPhotodiode sensor output voltage(mV): ");
    SerialPort.print(photo_sensor_voltage);
    SerialPort.print("\r\nWindows comparator: signal 1 value(pin D2): ");
    SerialPort.print(wind_com_pin1);
    SerialPort.print("\r\nWindows comparator: signal 1 value(pin D4): ");
    SerialPort.print(wind_com_pin2);
    SerialPort.print("\r\nLED Driver: duty cycle: ");
    SerialPort.print(led_driver->set_duty_cycle(duty_cycle));
    SerialPort.print(" %%\r\n");

    duty_cycle += 0.1;
    if (duty_cycle >= 1) {
        duty_cycle = 0;
    }

    delay(1000);
}

