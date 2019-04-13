/*
 * intelliflight_v1_pwm.c
 *
 *  Created on: 09.03.2019
 *      Author: hoelzlwimmerf
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/drivers/pwm.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include <stm32/stm32_pwm.h>

#include "intelliflight-v1.h"

#ifdef CONFIG_PWM

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration *******************************************************************/
/* PWM
 *
 * The STM3240G-Eval has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using variously unused pins on the board for
 * PWM output (see board.h for details of pins).
 */

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ************************************************************************************/

int stm32_pwm_setup(void) {
	static bool initialized = false;
	struct pwm_lowerhalf_s *pwm;
	int ret;

	/* Have we already initialized? */

	if (!initialized) {
		/* Call stm32l4_pwminitialize() to get an instance of the PWM interface */

		/* PWM
		 *
		 * The Nucleo-l476rg has no real on-board PWM devices, but the board can be
		 * configured to output a pulse train using TIM1 or 8, or others (see board.h).
		 * Let's figure out which the user has configured.
		 */

#if defined(CONFIG_STM32F7_TIM1_PWM)
#if defined(CONFIG_STM32F7_TIM1_CH1OUT)
		pwm = stm32_pwminitialize(1);
		if (!pwm) {
			aerr("ERROR: Failed to get the STM32F7 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register(DEV_PWM3, pwm);
		if (ret < 0) {
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
#endif
#if defined(CONFIG_STM32F7_TIM1_CH2OUT)
		pwm = stm32_pwminitialize(1);
		if (!pwm) {
			aerr("ERROR: Failed to get the STM32F7 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register(DEV_PWM4, pwm);
		if (ret < 0) {
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
#endif
#if defined(CONFIG_STM32F7_TIM1_CH3OUT)
		pwm = stm32_pwminitialize(1);
		if (!pwm) {
			aerr("ERROR: Failed to get the STM32F7 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register(DEV_PWM5, pwm);
		if (ret < 0) {
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
#endif
#if defined(CONFIG_STM32F7_TIM1_CH4OUT)
		pwm = stm32_pwminitialize(1);
		if (!pwm) {
			aerr("ERROR: Failed to get the STM32F7 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register(DEV_PWMLED0, pwm);
		if (ret < 0) {
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
#endif
#endif

#if defined(CONFIG_STM32F7_TIM2_PWM)
#if defined(CONFIG_STM32F7_TIM2_CH3OUT)
		pwm = stm32_pwminitialize(2);
		if (!pwm) {
			aerr("ERROR: Failed to get the STM32F7 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register(DEV_PWM6, pwm);
		if (ret < 0) {
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
#endif
#if defined(CONFIG_STM32F7_TIM2_CH4OUT)
		pwm = stm32_pwminitialize(2);
		if (!pwm) {
			aerr("ERROR: Failed to get the STM32F7 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register(DEV_PWM7, pwm);
		if (ret < 0) {
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
#endif
#endif

#if defined(CONFIG_STM32F7_TIM3_PWM)
#if defined(CONFIG_STM32F7_TIM3_CH3OUT)
		pwm = stm32_pwminitialize(3);
		if (!pwm) {
			aerr("ERROR: Failed to get the STM32F7 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register(DEV_PWM1, pwm);
		if (ret < 0) {
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
#endif
#if defined(CONFIG_STM32F7_TIM3_CH4OUT)
		pwm = stm32_pwminitialize(3);
		if (!pwm) {
			aerr("ERROR: Failed to get the STM32F7 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register(DEV_PWM2, pwm);
		if (ret < 0) {
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
#endif
#endif

#if defined(CONFIG_STM32F7_TIM4_PWM)
#if defined(CONFIG_STM32F7_TIM4_CH3OUT)
		pwm = stm32_pwminitialize(4);
		if (!pwm) {
			aerr("ERROR: Failed to get the STM32F7 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register(DEV_PWM8, pwm);
		if (ret < 0) {
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
#endif
#if defined(CONFIG_STM32F7_TIM4_CH4OUT)
		pwm = stm32_pwminitialize(4);
		if (!pwm) {
			aerr("ERROR: Failed to get the STM32F7 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register(DEV_PWM9, pwm);
		if (ret < 0) {
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
#endif
#endif

		/* Now we are initialized */

		initialized = true;
	}

	return OK;
}

#endif /* CONFIG_PWM */
