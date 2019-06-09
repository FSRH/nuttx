/****************************************************************************************************
 * configs/intelliflight/src/intelliflight-v1.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************************************/

#ifndef __CONFIGS_INTELLIFLIGHT_V1_SRC_INTELLIFLIGHT_V1__H
#define __CONFIGS_INTELLIFLIGHT_V1_SRC_INTELLIFLIGHT_V1__H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* procfs File System */

/* Assume that we have everything */

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1
#define HAVE_SDIO       1
#define HAVE_CS43L22    1
#define HAVE_RTC_DRIVER 1
#define HAVE_NETMONITOR 1
#define HAVE_HCIUART    1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB device monitor if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* ItelliFlight v1 LEDs ***********************************************************************/
/* This board has two LED's:
 * - one system-controllable status LED: LED1.
 * - one user-controllable LED: LED2.
 *
 * LED1 is on when PC13 is high.
 * LED2 is on when PD12 is high.
 */

#define GPIO_LED1          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTC | GPIO_PIN13)

#define GPIO_LED2          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTD | GPIO_PIN12)

/* ItelliFlight v1 GPIOs ***********************************************************************/
/* This board has one user-controllable GPIO pin: PB2
 */

#define BOARD_NGPIOIN     0 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    0 /* Amount of GPIO Input w/ Interruption pins */
//
//#define GPIO_IN1          (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN0)
#define GPIO_OUT1         	(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
							GPIO_PORTD | GPIO_PIN12)
//#define GPIO_INT1         (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN2)

/* Pushbutton B1, labelled "User", is connected to GPIO PA0.  A high value will be sensed when the
 * button is depressed. Note that the EXTI interrupt is configured.
 */

#define GPIO_BTN_USER      (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)

#define GPIO_OTGFS_VBUS    (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                            GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
//
///* Sporadic scheduler instrumentation. This configuration has been used for evaluating the NuttX
// * sporadic scheduler.  In this evaluation, two GPIO outputs are used.  One indicating the priority
// * (high or low) of the sporadic thread and one indicating where the thread is running or not.
// *
// * There is nothing special about the pin selections:
// *
// *   Arduino D2 PJ1 - Indicates priority1
// *   Arduino D4 PJ0 - Indicates that the thread is running
// */
//
//#define GPIO_SCHED_HIGHPRI (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
//                            GPIO_PORTJ | GPIO_PIN1)
//#define GPIO_SCHED_RUNNING (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
//                            GPIO_PORTJ | GPIO_PIN0)

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the intelliflight board.
 *
 ****************************************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup USB-related
 *   GPIO pins for the intelliflight board.
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32F7_OTGFS
void stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *   Added by: Stephan Rappensperger + Florian Hölzlwimmer
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int stm32_gpio_initialize(void);
#endif

/****************************************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host functionality. This function will
 *   start a thread that will monitor for device connection/disconnection events.
 *
 ****************************************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
#  error "The Intelliflight-v1 board does not support HOST OTG, only device!"
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_INTELLIFLIGHT_V1_SRC_INTELLIFLIGHT_V1_H */

