/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#define BLSP2_QUP1_I2C5_SDA 18
#define BLSP2_QUP1_I2C5_SCL 19

void gpio_tlmm_config(uint32_t gpio, uint8_t func,
			uint8_t dir, uint8_t pull,
			uint8_t drvstr, uint32_t enable)
{
	uint32_t val = 0;

	val |= pull;
	val |= func << 2;
	val |= drvstr << 6;
	val |= enable << 9;

	writel(val, (uint32_t *)GPIO_CONFIG_ADDR(gpio));
	return;
}

void gpio_set_dir(uint32_t gpio, uint32_t dir)
{
	writel(dir, (uint32_t *)GPIO_IN_OUT_ADDR(gpio));

	return;
}

uint32_t gpio_status(uint32_t gpio)
{
	return readl(GPIO_IN_OUT_ADDR(gpio)) & GPIO_IN;
}

/* Configure gpio for blsp uart 2 */
void gpio_config_uart_dm(uint8_t id)
{
	/* configure rx gpio */
	gpio_tlmm_config(5, 2, GPIO_INPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_DISABLE);

	/* configure tx gpio */
	gpio_tlmm_config(4, 2, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_DISABLE);
}

void gpio_config_blsp_i2c(uint8_t blsp_id, uint8_t qup_id)
{
    if (blsp_id == BLSP_ID_2)
    {
        switch (qup_id)
        {
        case QUP_ID_0:
            gpio_tlmm_config(BLSP2_QUP1_I2C5_SDA, 3, GPIO_OUTPUT, GPIO_NO_PULL,
                             GPIO_6MA, GPIO_DISABLE);
            gpio_tlmm_config(BLSP2_QUP1_I2C5_SCL, 3, GPIO_OUTPUT, GPIO_NO_PULL,
                             GPIO_6MA, GPIO_DISABLE);
            break;
        default:
            dprintf(CRITICAL, "gpio_config_blsp_i2c() not supported for BLSP[%d] QUP[%d]\n", blsp_id, qup_id);
            break;
        }
    }
}

