/* Copyright (c) 2014-2015 The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <debug.h>
#include <reg.h>
#include <platform/iomap.h>
#include <platform/gpio.h>
#include <blsp_qup.h>

struct qup_gpio_func {
	uint32_t gpio;
	uint8_t func;
};

struct qup_gpio_func blsp1_qup_gpio_func[][4] = {
	{
		{.gpio = 0, .func = 1},
		{.gpio = 1, .func = 1},
		{.gpio = 2, .func = 1},
		{.gpio = 3, .func = 1},
	},
	{
		{.gpio = 41, .func = 1},
		{.gpio = 42, .func = 1},
		{.gpio = 43, .func = 1},
		{.gpio = 44, .func = 1},
	},
	{
		{.gpio = 45, .func = 1},
		{.gpio = 46, .func = 1},
		{.gpio = 47, .func = 1},
		{.gpio = 48, .func = 1},
	},
	{
		{.gpio = 65, .func = 2},
		{.gpio = 66, .func = 2},
		{.gpio = 67, .func = 2},
		{.gpio = 68, .func = 2},
	},
	{
		{.gpio = 81, .func = 2},
		{.gpio = 82, .func = 2},
		{.gpio = 83, .func = 2},
		{.gpio = 84, .func = 2},
	},
	{
		{.gpio = 25, .func = 3},
		{.gpio = 26, .func = 2},
		{.gpio = 27, .func = 1},
		{.gpio = 28, .func = 1},
	}
};

struct qup_gpio_func blsp2_qup_gpio_func[][4] = {
	{
		{.gpio = 53, .func = 1},
		{.gpio = 54, .func = 1},
		{.gpio = 55, .func = 1},
		{.gpio = 56, .func = 1},
	},
	{
		{.gpio = 4, .func = 1},
		{.gpio = 5, .func = 1},
		{.gpio = 6, .func = 1},
		{.gpio = 7, .func = 1},
	},
	{
		{.gpio = 49, .func = 2},
		{.gpio = 50, .func = 2},
		{.gpio = 51, .func = 2},
		{.gpio = 52, .func = 2},
	},
	{
		{.gpio = 8, .func = 1},
		{.gpio = 9, .func = 1},
		{.gpio = 10, .func = 2},
		{.gpio = 11, .func = 2},
	},
	{
		{.gpio = 58, .func = 3},
		{.gpio = 59, .func = 3},
		{.gpio = 60, .func = 3},
		{.gpio = 61, .func = 3},
	},
	{
		{.gpio = 85, .func = 1},
		{.gpio = 86, .func = 1},
		{.gpio = 87, .func = 1},
		{.gpio = 88, .func = 1},
	}
};

/* Remove the file after the gpio patch to move this to msm_shared gets merged. */
void gpio_tlmm_config(uint32_t gpio, uint8_t func,
		      uint8_t dir, uint8_t pull,
		      uint8_t drvstr, uint32_t enable)
{
	uint32_t val = 0;
	val |= pull;
	val |= func << 2;
	val |= drvstr << 6;
	val |= enable << 9;
	writel(val, (unsigned int *)GPIO_CONFIG_ADDR(gpio));
	return;
}

int gpio_tlmm_config_read(uint32_t gpio)
{
        int val = 0;
        val = readl((unsigned int *)GPIO_CONFIG_ADDR(gpio));
        return val;
}

void gpio_set(uint32_t gpio, uint32_t dir)
{
	writel(dir, (unsigned int *)GPIO_IN_OUT_ADDR(gpio));
	return;
}

int gpio_get(uint32_t gpio)
{
	int val = 0;
	val = readl((unsigned int *)GPIO_IN_OUT_ADDR(gpio));
	return val;
}


/* Configure gpio for blsp uart */
void gpio_config_uart_dm(uint8_t id)
{
    /* configure rx gpio */
	gpio_tlmm_config(5, 2, GPIO_INPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_DISABLE);

    /* configure tx gpio */
	gpio_tlmm_config(4, 2, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_DISABLE);
}


/* Configure gpios for blsp */
void gpio_config_blsp_i2c(uint8_t blsp_id, uint8_t qup_id)
{
	if(blsp_id == BLSP_ID_2) {
		switch (qup_id) {
			case QUP_ID_1:
				/* configure I2C SDA gpio */
				gpio_tlmm_config(GPIO_BLSP2_ACTIVE_1, 3, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_8MA, GPIO_DISABLE);

				/* configure I2C SCL gpio */
				gpio_tlmm_config(GPIO_BLSP2_ACTIVE_2, 3, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_8MA, GPIO_DISABLE);
			break;
			default:
				dprintf(CRITICAL, "Incorrect QUP id %d\n", qup_id);
				ASSERT(0);
		};
	} else {
		dprintf(CRITICAL, "Incorrect BLSP id %d\n",blsp_id);
		ASSERT(0);
	}
}

void gpio_config_blsp_spi(uint8_t blsp_id, uint8_t qup_id)
{
	int i;

	if(blsp_id == BLSP_ID_1) {
		for (i = 0; i < 4; i++)
			gpio_tlmm_config(blsp1_qup_gpio_func[qup_id][i].gpio,
					 blsp1_qup_gpio_func[qup_id][i].func,
					 GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA,
					 GPIO_DISABLE);
	} else if (blsp_id == BLSP_ID_2) {
		for (i = 0; i < 4; i++)
			gpio_tlmm_config(blsp2_qup_gpio_func[qup_id][i].gpio,
					 blsp2_qup_gpio_func[qup_id][i].func,
					 GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA,
					 GPIO_DISABLE);
	} else {
		dprintf(CRITICAL, "Incorrect BLSP id %d\n",blsp_id);
		ASSERT(0);
	}
}
