/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "app_timer.h"
/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "app_timer.h"
#include "nrf_drv_clock.h"

#include "nrf_drv_timer.h"
#include "string.h"

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 1024                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1024                         /**< UART RX buffer size. */

//DS18B20 stuff
#define DS18B20PIN 31
#define TIMER_SHORT 5
#define TIMER_RESET 15
#define TIMER_LONG 20

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }

}

void ds18b20_read(void * data_ptr, unsigned int num_bytes)
{
  char  ch; //Current reading byte buffer
  char * data_buf = data_ptr;
  
  int i=0,u=0;
  for (i=0;i<num_bytes;i++)
  {
    ch=0;
    for(u=0;u<8;u++)
    {
      //Form read slot
      nrf_gpio_cfg_output(DS18B20PIN);
      nrf_gpio_pin_write(DS18B20PIN,0);
      nrf_delay_us(5);
      nrf_gpio_cfg_input(DS18B20PIN, NRF_GPIO_PIN_NOPULL);
      nrf_delay_us(5);
      if(nrf_gpio_pin_read(DS18B20PIN)>0)
      {
        ch |= 1 << u; //There is 1 on the bus
      }
      else
      {
        ch &= ~(1 << u); //There us 0 on the bus
      }

      //Apply "long" timer for make sure that timeslot is end
      nrf_delay_us(60);
    }
    data_buf[i]=ch;
  }
  nrf_gpio_cfg_input(DS18B20PIN, NRF_GPIO_PIN_NOPULL);
}

void ds18b20_write(void * data_ptr, unsigned int num_bytes)
{
  char  ch; //Current reading byte buffer
  char * data_buf = data_ptr;
  
  int i=0,u=0;
  for (i=0;i<num_bytes;i++)
  {
    ch=data_buf[i];
    for(u=0;u<8;u++)
    {
      //Form write slot
      nrf_gpio_cfg_output(DS18B20PIN);
      nrf_gpio_pin_write(DS18B20PIN,0);
      nrf_delay_us(1);
      //write 1 - pull bus to HIGH just after short timer
      if(ch&(1<<u))
      {
        nrf_gpio_cfg_input(DS18B20PIN, NRF_GPIO_PIN_NOPULL); 
      }
      //Apply "long" timer for make sure that timeslot is end
      nrf_delay_us(60);
      //Release bus, if this wasn't done before
      nrf_gpio_cfg_input(DS18B20PIN, NRF_GPIO_PIN_NOPULL);
      data_buf[i]=ch;
    }
  }
  nrf_gpio_cfg_input(DS18B20PIN, NRF_GPIO_PIN_NOPULL);
}

//Perform reset of the bus, and then wait for the presence pulse
bool ds18b20_reset_and_check(void)
{
  int res=0;
  //Form reset pulse
  
  nrf_gpio_cfg_output(DS18B20PIN);
  nrf_gpio_pin_write(DS18B20PIN,0);
  nrf_delay_us(500);
  //Release bus and wait 15-60MS
  nrf_gpio_cfg_input(DS18B20PIN, NRF_GPIO_PIN_NOPULL);
  nrf_delay_us(60);

  //Read from bus
  res=nrf_gpio_pin_read(DS18B20PIN);
  if(res==0)
  {
    nrf_delay_us(500);
    return true;
  }
  return false;
}
/**
 *@}
 **/


float ds18b20_read_remp(void)
{
   char buf[16];
    double f;
        int i;
     int16_t raw_temp=0;

    ds18b20_reset_and_check();

//Read ROM
    buf[0]=0x33;
    ds18b20_write(&buf,1);

//Read the results
    
    ds18b20_read(&buf,8);
    memset(&buf,0,16);

    //Send convert TX cmd
    buf[0]=0x44; //Convert temp
    ds18b20_write(&buf,1);
    nrf_delay_ms(1000);//Wait for finishing of the conversion

    ds18b20_reset_and_check();
    buf[0]=0x33; //Read ROM
    ds18b20_write(&buf,1);
    //Read the results
    ds18b20_read(&buf,8);
    memset(&buf,0,16);
    buf[0]=0xBE;  //Read scratchpad
    ds18b20_write(&buf,1);
    //Read the results
    ds18b20_read(&buf,9);
    raw_temp = (buf[1] << 8) | buf[0];

    //memcpy(&f,&buf,8);
    f=(float)raw_temp / 16.0;
    return(f);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code;
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_115200
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

    printf("\r\nUART example started.\r\n");

  while(1)
  {
    
    printf("read temp: %f \r\n", ds18b20_read_remp());
  }

}


/** @} */


