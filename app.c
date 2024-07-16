/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

// https://github.com/SiliconLabs/peripheral_examples/blob/master/series2/usart/spi_secondary_interrupt/src/main_s2.c

#include "app_log.h"

#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_prs.h"

// SPI ports and pins
#define US0MISO_PORT                  gpioPortC //EXP 6
#define US0MISO_PIN                   1
#define US0MOSI_PORT                  gpioPortC //EXP 4
#define US0MOSI_PIN                   0
#define US0CLK_PORT                   gpioPortC //EXP 8
#define US0CLK_PIN                    2
#define US0CS_PORT                    gpioPortC //EXP 10
#define US0CS_PIN                     3

#define US0MISO_EXT_PORT              gpioPortD //EXP 11 - Pulled Down MISO
#define US0MISO_EXT_PIN               2

#define PRS_CHANNEL_MISO_INTERNAL     6
#define PRS_CHANNEL_CS                7

// Size of the data buffers
#define BUFLEN                        10

// Outgoing data
uint8_t outbuf[BUFLEN];

// Incoming data
uint8_t inbuf[BUFLEN];

// Position in the buffer
uint32_t bufpos;


/**************************************************************************//**
 * @brief
 *    GPIO initialization
 *****************************************************************************/
void initGPIO(void)
{
  // Enable clock
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure MOSI pin as an input -- RX
  GPIO_PinModeSet(US0MOSI_PORT, US0MOSI_PIN, gpioModeInput, 0); // PC 0 - in

  // Configure MISO pin as an output -- TX
  GPIO_PinModeSet(US0MISO_PORT, US0MISO_PIN, gpioModePushPull, 0); // PC 1 - Pulled Down

  // Configure CLK pin as an input
  GPIO_PinModeSet(US0CLK_PORT, US0CLK_PIN, gpioModeInput, 0);

  // Configure CS pin as an input filtered
  GPIO_PinModeSet(US0CS_PORT, US0CS_PIN, gpioModeInput, 1);

  // Generate an interrupt on a CS pin high-to-low transition.
  GPIO_ExtIntConfig(US0CS_PORT, US0CS_PIN, US0CS_PIN, false, true, false);

  // Enable NVIC GPIO interrupt
#if (US0CS_PIN & 1)
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
#else
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
#endif

}

/**************************************************************************//**
 * @brief
 *    USART0 initialization
 *****************************************************************************/
void initUSART0(void)
{
  CMU_ClockEnable(cmuClock_USART0, true);

  // Default asynchronous initializer (main mode, 1 Mbps, 8-bit data)
  USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

  init.master = false;  // Operate as a secondary
  init.msbf = true;     // MSB first transmission for SPI compatibility
  init.enable = usartDisable;  // Do not enable yet

  // Configure but do not enable USART0
  USART_InitSync(USART0, &init);

  // Route USART0 RX, TX, CLK, and CS to the specified pins.
  GPIO->USARTROUTE[0].RXROUTE = (US0MISO_PORT << _GPIO_USART_RXROUTE_PORT_SHIFT)// TX Whne slave
      | (US0MISO_PIN << _GPIO_USART_RXROUTE_PIN_SHIFT);

  GPIO->USARTROUTE[0].TXROUTE = (US0MOSI_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT)// RX when slave
      | (US0MOSI_PIN << _GPIO_USART_TXROUTE_PIN_SHIFT);

  GPIO->USARTROUTE[0].CLKROUTE = (US0CLK_PORT << _GPIO_USART_CLKROUTE_PORT_SHIFT)
      | (US0CLK_PIN << _GPIO_USART_CLKROUTE_PIN_SHIFT);
  GPIO->USARTROUTE[0].CSROUTE = (US0CS_PORT << _GPIO_USART_CSROUTE_PORT_SHIFT)
      | (US0CS_PIN << _GPIO_USART_CSROUTE_PIN_SHIFT);

  // Enable USART interface pins
  GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN  |
                                GPIO_USART_ROUTEEN_TXPEN  |
                                GPIO_USART_ROUTEEN_CLKPEN |
                                GPIO_USART_ROUTEEN_CSPEN;

  // Enable NVIC USART sources
  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
  NVIC_EnableIRQ(USART0_RX_IRQn);

  // Enable receive data valid interrupt
  USART_IntClear(USART0, USART_IF_RXDATAV);
  USART_IntEnable(USART0, USART_IEN_RXDATAV);
}

/**************************************************************************//**
 * @brief
 *    PRS initialization
 *****************************************************************************/
void initPRS(void)
{
  // Enable PRS clock
  CMU_ClockEnable(cmuClock_PRS, true);

  // Enable the activity pin
  GPIO_PinModeSet(US0MISO_EXT_PORT, US0MISO_EXT_PIN, gpioModePushPull, 0);

  //Configure USART0 MISO as source for PRS out, must configure edge detection
  GPIO_ExtIntConfig(US0MISO_PORT, US0MISO_PIN, US0MISO_PIN, 0, 0, false);
  PRS_SourceAsyncSignalSet(PRS_CHANNEL_MISO_INTERNAL, PRS_ASYNC_CH_CTRL_SOURCESEL_GPIO, US0MISO_PIN);

  //Allocate second channel for CS combination
  PRS_SourceAsyncSignalSet(PRS_CHANNEL_CS, PRS_ASYNC_CH_CTRL_SOURCESEL_USART0, prsSignalUSART0_CS);

  //Combine channels and output
  PRS_Combine(PRS_CHANNEL_CS, PRS_CHANNEL_MISO_INTERNAL, prsLogic_NOT_A_AND_B);
  PRS_PinOutput(PRS_CHANNEL_CS, prsTypeAsync, US0MISO_EXT_PORT , US0MISO_EXT_PIN);
}

/**************************************************************************//**
 * @brief GPIO IRQHandler
 *****************************************************************************/
#if (US0CS_PIN & 1)
void GPIO_ODD_IRQHandler(void)
#else
void GPIO_EVEN_IRQHandler(void)
#endif
{
  // Clear the falling edge interrupt flag
  GPIO_IntClear(1 << US0CS_PIN);
}

/**************************************************************************//**
 * @brief
 *    USART0 receive interrupt handler
 *****************************************************************************/
void USART0_RX_IRQHandler(void)
{
  /*
   * Save the byte received concurrent with the transmission of the
   * last bit of the previous outgoing byte, and increment the buffer
   * position to the next byte.  Note that this read clears the
   * USART_IF_RXDATAV interrupt flag.
   */
  inbuf[bufpos++] = USART0->RXDATA;

  // If there are still bytes left to send, transmit the next one
  if (bufpos < BUFLEN){
    USART0->TXDATA = outbuf[bufpos];
  } else {
    bufpos = 0;
  }
}

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  app_log("Secondary App Start\n");

  // Initialize GPIO and USART0
  initGPIO();

    uint8_t i = 0;

    // Zero incoming buffer and populate outgoing data array
    for (i = 0; i < BUFLEN; i++)
    {
      inbuf[i] = 0;
      outbuf[i] = (uint8_t)(i+10);
    }

  initUSART0();
  initPRS();
  USART_Enable(USART0, usartEnable);
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
}
