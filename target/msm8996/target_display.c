/* Copyright (c) 2014-2016, 2018-2019, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <debug.h>
#include <string.h>
#include <smem.h>
#include <err.h>
#include <msm_panel.h>
#include <mipi_dsi.h>
#include <mdss_hdmi.h>
#include <pm8x41.h>
#include <pm8x41_wled.h>
#include <qpnp_wled.h>
#include <board.h>
#include <mdp5.h>
#include <endian.h>
#include <regulator.h>
#include <qtimer.h>
#include <arch/defines.h>
#include <platform/gpio.h>
#include <platform/clock.h>
#include <platform/iomap.h>
#include <target/display.h>
#include <target/target_utils.h>
#if ENABLE_QSEED_SCALAR
#include <target/scalar.h>
#endif
#include <mipi_dsi_autopll_thulium.h>
#include <mipi_dsi_i2c.h>

#include "include/panel.h"
#include "include/display_resource.h"
#include "gcdb_display.h"

#define GPIO_STATE_LOW 0
#define GPIO_STATE_HIGH 2
#define RESET_GPIO_SEQ_LEN 3

#define PWM_DUTY_US 13
#define PWM_PERIOD_US 27
#define PMIC_WLED_SLAVE_ID 3
#define PMIC_MPP_SLAVE_ID 2

#define MAX_POLL_READS 15
#define POLL_TIMEOUT_US 1000

#define STRENGTH_SIZE_IN_BYTES_8996	10
#define REGULATOR_SIZE_IN_BYTES_8996	5
#define LANE_SIZE_IN_BYTES_8996		20

#define PANEL_NAME_DELIMITER '+'
#define MAX_PANEL_COUNT      3

/*---------------------------------------------------------------------------*/
/* GPIO configuration                                                        */
/*---------------------------------------------------------------------------*/
static struct gpio_pin reset_gpio = {
  "msmgpio", 8, 3, 1, 0, 1
};

static struct gpio_pin lcd_reg_en = {	/* boost regulator */
  "pmi8994_gpios", 8, 3, 1, 0, 1
};

static struct gpio_pin bklt_gpio = {	/* lcd_bklt_reg_en */
  "pm8994_gpios", 14, 3, 1, 0, 1
};

static struct gpio_pin enable_gpio = {
  "msmgpio", 10, 3, 1, 0, 1
};

static struct gpio_pin dsi2hdmi_switch_gpio = {
  "msmgpio", 105, 3, 1, 0, 1
};

static struct gpio_pin dsi2hdmi2_switch_gpio = {
  "msmgpio", 107, 3, 1, 0, 0
};

/* gpio name, id, strength, direction, pull, state. */
static struct gpio_pin hdmi_cec_gpio = {        /* CEC */
  "msmgpio", 31, 0, 2, 3, 1
};

static struct gpio_pin hdmi_ddc_clk_gpio = {   /* DDC CLK */
  "msmgpio", 32, 0, 2, 3, 1
};

static struct gpio_pin hdmi_ddc_data_gpio = {  /* DDC DATA */
  "msmgpio", 33, 0, 2, 3, 1
};

static struct gpio_pin hdmi_hpd_gpio = {       /* HPD, input */
  "msmgpio", 34, 7, 0, 1, 1
};

struct target_display displays[MAX_NUM_DISPLAY];
struct target_layer_int layers[NUM_TARGET_LAYERS];

extern int msm_display_update(struct fbcon_config *fb, uint32_t pipe_id,
	uint32_t pipe_type, uint32_t *zorder, uint32_t *width, uint32_t *height, uint32_t disp_id);
extern int msm_display_update_pipe(struct fbcon_config *fb, uint32_t pipe_id,
	uint32_t pipe_type, uint32_t *zorder, uint32_t *width, uint32_t *height, uint32_t disp_id);
extern int msm_display_hide_pipe(uint32_t pipe_id, uint32_t pipe_type, uint32_t disp_id);
extern struct fbcon_config* msm_display_get_fb(uint32_t disp_id, uint32_t fb_index);
extern int msm_display_init_count();

bool display_init_done = false;
bool panel_type_is_selected = true;

bool target_display_is_init_done()
{
	return display_init_done;
}

void target_display_set_panel_type(char *panel_name)
{
	if (!strcmp(panel_name, NO_PANEL_CONFIG))
		panel_type_is_selected = false;
}

bool target_display_panel_is_selected()
{
	return panel_type_is_selected;
}

static void target_hdmi_ldo_enable(uint8_t enable)
{
	if (enable)
		regulator_enable(REG_LDO12);
	else
		regulator_disable(REG_LDO12);
}

static void target_hdmi_mpp4_enable(uint8_t enable)
{
	struct pm8x41_mpp mpp;

	/* Enable MPP4 */
	pmi8994_config_mpp_slave_id(0);

        mpp.base = PM8x41_MMP4_BASE;
	mpp.vin = MPP_VIN2;
	mpp.mode = MPP_HIGH;;
	if (enable) {
		pm8x41_config_output_mpp(&mpp);
		pm8x41_enable_mpp(&mpp, MPP_ENABLE);
	} else {
		pm8x41_enable_mpp(&mpp, MPP_DISABLE);
	}

	/* Need delay before power on regulators */
	mdelay(1);
}

int target_hdmi_regulator_ctrl(uint8_t enable)
{
	target_hdmi_ldo_enable(enable);

	target_hdmi_mpp4_enable(enable);

	return 0;
}

int target_hdmi_gpio_ctrl(uint8_t enable)
{
	gpio_tlmm_config(hdmi_cec_gpio.pin_id, 1,	/* gpio 31, CEC */
		hdmi_cec_gpio.pin_direction, hdmi_cec_gpio.pin_pull,
		hdmi_cec_gpio.pin_strength, hdmi_cec_gpio.pin_state);

	gpio_tlmm_config(hdmi_ddc_clk_gpio.pin_id, 1,	/* gpio 32, DDC CLK */
		hdmi_ddc_clk_gpio.pin_direction, hdmi_ddc_clk_gpio.pin_pull,
		hdmi_ddc_clk_gpio.pin_strength, hdmi_ddc_clk_gpio.pin_state);


	gpio_tlmm_config(hdmi_ddc_data_gpio.pin_id, 1,	/* gpio 33, DDC DATA */
		hdmi_ddc_data_gpio.pin_direction, hdmi_ddc_data_gpio.pin_pull,
		hdmi_ddc_data_gpio.pin_strength, hdmi_ddc_data_gpio.pin_state);

	gpio_tlmm_config(hdmi_hpd_gpio.pin_id, 1,	/* gpio 34, HPD */
		hdmi_hpd_gpio.pin_direction, hdmi_hpd_gpio.pin_pull,
		hdmi_hpd_gpio.pin_strength, hdmi_hpd_gpio.pin_state);

	gpio_set(hdmi_cec_gpio.pin_id,      hdmi_cec_gpio.pin_direction);
	gpio_set(hdmi_ddc_clk_gpio.pin_id,  hdmi_ddc_clk_gpio.pin_direction);
	gpio_set(hdmi_ddc_data_gpio.pin_id, hdmi_ddc_data_gpio.pin_direction);
	gpio_set(hdmi_hpd_gpio.pin_id,      hdmi_hpd_gpio.pin_direction);

	return NO_ERROR;
}

int target_hdmi_pll_clock(uint8_t enable, struct msm_panel_info *pinfo)
{
    if (enable) {
        hdmi_phy_reset();
        hdmi_pll_config(pinfo->clk_rate);
        hdmi_vco_enable();
        hdmi_pixel_clk_enable(pinfo->clk_rate);
    } else if(!target_cont_splash_screen()) {
        /* Disable clocks if continuous splash off */
        hdmi_pixel_clk_disable();
        hdmi_vco_disable();
    }

    return NO_ERROR;
}

int target_hdmi_panel_clock(uint8_t enable, struct msm_panel_info *pinfo)
{
	dprintf(SPEW, "%s: target_panel_clock\n", __func__);

	uint32_t board_version = board_soc_version();

	if (board_version == 0x20000 || board_version == 0x20001)
		video_gdsc_enable();

	if (enable) {
		mmss_gdsc_enable();
		mmss_bus_clock_enable();
		mdp_clock_enable();
		hdmi_ahb_core_clk_enable();
	} else if(!target_cont_splash_screen()) {
		hdmi_core_ahb_clk_disable();
		mdp_clock_disable();
		mmss_bus_clock_disable();
		mmss_gdsc_disable();
		if (board_version == 0x20000 || board_version == 0x20001)
			video_gdsc_disable();
	}

	return NO_ERROR;
}

#define TIMER_KHZ 32768

unsigned int place_marker(char *marker_name)
{
        unsigned int marker_value;

        marker_value = readl(MPM2_MPM_SLEEP_TIMETICK_COUNT_VAL);
        dprintf(INFO, "marker name=%s; marker value=%u.%03u seconds\n",
                        marker_name, marker_value/TIMER_KHZ,
                        (((marker_value % TIMER_KHZ)
                        * 1000) / TIMER_KHZ));
        return marker_value;
}

static uint32_t thulium_dsi_pll_lock_status(uint32_t pll_base, uint32_t off,
	uint32_t bit)
{
	uint32_t cnt, status;

	/* check pll lock first */
	for (cnt = 0; cnt < MAX_POLL_READS; cnt++) {
		status = readl(pll_base + off);
		dprintf(SPEW, "%s: pll_base=%x cnt=%d status=%x\n",
				__func__, pll_base, cnt, status);
		status &= BIT(bit); /* bit 5 */
		if (status)
			break;
		udelay(POLL_TIMEOUT_US);
	}

	return status;
}

static uint32_t thulium_dsi_pll_enable_seq(uint32_t phy_base, uint32_t pll_base)
{
	uint32_t pll_locked;

	writel(0x10, phy_base + 0x45c);
	writel(0x01, phy_base + 0x48);
	dmb();

	pll_locked = thulium_dsi_pll_lock_status(pll_base, 0xcc, 5);
	if (pll_locked)
		pll_locked = thulium_dsi_pll_lock_status(pll_base, 0xcc, 0);

	if (!pll_locked)
		dprintf(ERROR, "%s: DSI PLL lock failed\n", __func__);
	else
		dprintf(SPEW, "%s: DSI PLL lock Success\n", __func__);

	return  pll_locked;
}

static int thulium_wled_backlight_ctrl(uint8_t enable)
{
	qpnp_wled_enable_backlight(enable);
	return NO_ERROR;
}

static int thulium_pwm_backlight_ctrl(uint8_t enable)
{
	uint8_t slave_id = 3; /* lpg at pmi */

        if (enable) {
                /* lpg channel 4 */

		 /* LPG_ENABLE_CONTROL */
                pm8x41_lpg_write_sid(slave_id, PWM_BL_LPG_CHAN_ID, 0x46, 0x0);
		mdelay(10);

		 /* LPG_VALUE_LSB, duty cycle = 0x80/0x200 = 1/4 */
                pm8x41_lpg_write_sid(slave_id, PWM_BL_LPG_CHAN_ID, 0x44, 0x80);
		/* LPG_VALUE_MSB */
                pm8x41_lpg_write_sid(slave_id, PWM_BL_LPG_CHAN_ID, 0x45, 0x00);
		/* LPG_PWM_SYNC */
                pm8x41_lpg_write_sid(slave_id, PWM_BL_LPG_CHAN_ID, 0x47, 0x01);

		 /* LPG_PWM_SIZE_CLK, */
                pm8x41_lpg_write_sid(slave_id, PWM_BL_LPG_CHAN_ID, 0x41, 0x13);
		 /* LPG_PWM_FREQ_PREDIV */
                pm8x41_lpg_write_sid(slave_id, PWM_BL_LPG_CHAN_ID, 0x42, 0x02);
		 /* LPG_PWM_TYPE_CONFIG */
                pm8x41_lpg_write_sid(slave_id, PWM_BL_LPG_CHAN_ID, 0x43, 0x20);
		 /* LPG_ENABLE_CONTROL */
                pm8x41_lpg_write_sid(slave_id, PWM_BL_LPG_CHAN_ID, 0x46, 0x04);

		 /* SEC_ACCESS */
                pm8x41_lpg_write_sid(slave_id, PWM_BL_LPG_CHAN_ID, 0xD0, 0xA5);
		 /* DTEST4, OUT_HI */
                pm8x41_lpg_write_sid(slave_id, PWM_BL_LPG_CHAN_ID, 0xE5, 0x01);
		 /* LPG_ENABLE_CONTROL */
                pm8x41_lpg_write_sid(slave_id, PWM_BL_LPG_CHAN_ID, 0x46, 0xA4);
        } else {
		 /* LPG_ENABLE_CONTROL */
                pm8x41_lpg_write_sid(slave_id, PWM_BL_LPG_CHAN_ID, 0x46, 0x0);
        }

        return NO_ERROR;
}

static void lcd_reg_enable(void)
{
	uint8_t slave_id = 2;	/* gpio at pmi */

	struct pm8x41_gpio gpio = {
                .direction = PM_GPIO_DIR_OUT,
                .function = PM_GPIO_FUNC_HIGH,
                .vin_sel = 2,   /* VIN_2 */
                .output_buffer = PM_GPIO_OUT_CMOS,
                .out_strength = PM_GPIO_OUT_DRIVE_MED,
        };

        pm8x41_gpio_config_sid(slave_id, lcd_reg_en.pin_id, &gpio);
	pm8x41_gpio_set_sid(slave_id, lcd_reg_en.pin_id, 1);
}

static void lcd_reg_disable(void)
{
	uint8_t slave_id = 2;	/* gpio at pmi */

	pm8x41_gpio_set_sid(slave_id, lcd_reg_en.pin_id, 0);
}

static void lcd_bklt_reg_enable(void)
{
       struct pm8x41_gpio gpio = {
                .direction = PM_GPIO_DIR_OUT,
                .function = PM_GPIO_FUNC_HIGH,
                .vin_sel = 2,   /* VIN_2 */
                .output_buffer = PM_GPIO_OUT_CMOS,
                .out_strength = PM_GPIO_OUT_DRIVE_LOW,
        };

        pm8x41_gpio_config(bklt_gpio.pin_id, &gpio);
	pm8x41_gpio_set(bklt_gpio.pin_id, 1);
}

static void lcd_bklt_reg_disable(void)
{
	pm8x41_gpio_set(bklt_gpio.pin_id, 0);
}

static int dsi2HDMI_i2c_write_regs(struct msm_panel_info *pinfo,
	struct mipi_dsi_i2c_cmd *cfg, int size, bool second_bridge)
{
	int ret = NO_ERROR;
	int i;
	uint8_t addr;

	if (!cfg || size < 0) {
		dprintf(CRITICAL, "Invalid input: register array is null\n");
		return ERR_INVALID_ARGS;
	}

	for (i = 0; i < size; i++) {
		switch (cfg[i].i2c_addr) {
			case ADV7533_MAIN:
				if (second_bridge)
					addr = pinfo->sadv7533.i2c_main_addr;
				else
					addr = pinfo->adv7533.i2c_main_addr;
				break;
			case ADV7533_CEC_DSI:
				if (second_bridge)
					addr = pinfo->sadv7533.i2c_cec_addr;
				else
					addr = pinfo->adv7533.i2c_cec_addr;
				break;
			default:
				dprintf(CRITICAL, "Invalid I2C addr in array\n");
				ret = ERR_INVALID_ARGS;
				goto w_regs_fail;
		}

		ret = mipi_dsi_i2c_write_byte(addr, cfg[i].reg,
			cfg[i].val);
		if (ret) {
			dprintf(CRITICAL, "mipi_dsi reg writes failed\n");
			goto w_regs_fail;
		}
		if (cfg[i].sleep_in_ms) {
			udelay(cfg[i].sleep_in_ms*1000);
		}
	}
w_regs_fail:
	return ret;
}

int target_display_dsi2hdmi_program_addr(struct msm_panel_info *pinfo, bool second_bridge)
{
	int ret = NO_ERROR;
	uint8_t i2c_8bits;

	if (second_bridge) {
		i2c_8bits = pinfo->sadv7533.i2c_cec_addr << 1;
		ret = mipi_dsi_i2c_write_byte(pinfo->sadv7533.i2c_main_addr,
				0xE1, i2c_8bits);
	} else {
		i2c_8bits = pinfo->adv7533.i2c_cec_addr << 1;
		ret = mipi_dsi_i2c_write_byte(pinfo->adv7533.i2c_main_addr,
				0xE1, i2c_8bits);
	}
	if (ret) {
		dprintf(CRITICAL, "Error in programming CEC DSI addr\n");
	} else {
		dprintf(SPEW, "CEC address programming successful\n");
	}
	return ret;
}

int target_display_dsi2hdmi_config(struct msm_panel_info *pinfo)
{
	int ret = NO_ERROR;

	if (!pinfo) {
		dprintf(CRITICAL, "Invalid input: pinfo is null\n");
		return ERR_INVALID_ARGS;
	}
	if (!pinfo->adv7533.program_i2c_addr) {
		ret = target_display_dsi2hdmi_program_addr(pinfo, false);
		if (ret) {
			dprintf(CRITICAL, "Error in programming cec dsi addr for %s\n",
				(pinfo->dest == DISPLAY_2) ? "DSI1" : "DSI0");
			return ret;
		} else {
			dprintf(SPEW, "successfully programmed cec dsi addr for %s\n",
				(pinfo->dest == DISPLAY_2) ? "DSI1" : "DSI0");
			pinfo->adv7533.program_i2c_addr = 1;
		}
	}
	if ((pinfo->lcdc.split_display) && (!pinfo->sadv7533.program_i2c_addr)) {
		ret = target_display_dsi2hdmi_program_addr(pinfo, true);
		if (ret) {
			dprintf(CRITICAL, "Error in programming cec dsi addr for secondary bridge\n");
			return ret;
		} else {
			dprintf(SPEW, "successfully programmed cec dsi addr for secondary bridge\n");
			pinfo->sadv7533.program_i2c_addr = 1;
		}
	}

	/*
	 * If dsi to HDMI bridge chip connected then
	 * send I2c commands to the chip
	 */
	if (pinfo->adv7533.dsi_setup_cfg_i2c_cmd) {
		ret = dsi2HDMI_i2c_write_regs(pinfo, pinfo->adv7533.dsi_setup_cfg_i2c_cmd,
				pinfo->adv7533.num_of_cfg_i2c_cmds, false);

		if (pinfo->lcdc.split_display)
			ret = dsi2HDMI_i2c_write_regs(pinfo, pinfo->adv7533.dsi_setup_cfg_i2c_cmd,
					pinfo->adv7533.num_of_cfg_i2c_cmds, true);

		if (ret) {
			dprintf(CRITICAL, "Error in writing adv7533 setup registers for %s\n",
							(pinfo->dest == DISPLAY_2) ? "DSI1" : "DSI0");
			return ret;
		}
	}

	if (pinfo->adv7533.dsi_tg_i2c_cmd) {
		ret = dsi2HDMI_i2c_write_regs(pinfo, pinfo->adv7533.dsi_tg_i2c_cmd,
					pinfo->adv7533.num_of_tg_i2c_cmds, false);

		if (pinfo->lcdc.split_display)
			ret = dsi2HDMI_i2c_write_regs(pinfo, pinfo->adv7533.dsi_tg_i2c_cmd,
						pinfo->adv7533.num_of_tg_i2c_cmds, true);

		if (ret) {
			dprintf(CRITICAL, "Error in writing adv7533 timing registers for %s\n",
							(pinfo->dest == DISPLAY_2) ? "DSI1" : "DSI0");
		}
	}
	return ret;
}

int target_backlight_ctrl(struct backlight *bl, uint8_t enable)
{
	uint32_t ret = NO_ERROR;
	struct pm8x41_mpp mpp;
	int rc;

	if (!bl) {
		dprintf(CRITICAL, "backlight structure is not available\n");
		return ERR_INVALID_ARGS;
	}

	switch (bl->bl_interface_type) {
	case BL_WLED:
		/* Enable MPP4 */
		pmi8994_config_mpp_slave_id(PMIC_MPP_SLAVE_ID);
		mpp.base = PM8x41_MMP4_BASE;
		mpp.vin = MPP_VIN2;
		if (enable) {
			pm_pwm_enable(false);
			rc = pm_pwm_config(PWM_DUTY_US, PWM_PERIOD_US);
			if (rc < 0) {
				mpp.mode = MPP_HIGH;
			} else {
				mpp.mode = MPP_DTEST1;
				pm_pwm_enable(true);
			}
			pm8x41_config_output_mpp(&mpp);
			pm8x41_enable_mpp(&mpp, MPP_ENABLE);
		} else {
			pm_pwm_enable(false);
			pm8x41_enable_mpp(&mpp, MPP_DISABLE);
		}
		/* Enable WLED backlight control */
		ret = thulium_wled_backlight_ctrl(enable);
		break;
	case BL_PWM:
		/* Enable MPP1 */
		pmi8994_config_mpp_slave_id(PMIC_MPP_SLAVE_ID);
		mpp.base = PM8x41_MMP1_BASE;
		mpp.vin = MPP_VIN2;
		mpp.mode = MPP_DTEST4;
		if (enable) {
			pm8x41_config_output_mpp(&mpp);
			pm8x41_enable_mpp(&mpp, MPP_ENABLE);
		} else {
			pm8x41_enable_mpp(&mpp, MPP_DISABLE);
		}
		ret = thulium_pwm_backlight_ctrl(enable);
		break;
	default:
		dprintf(CRITICAL, "backlight type:%d not supported\n",
						bl->bl_interface_type);
		return ERR_NOT_SUPPORTED;
	}

	return ret;
}

int target_panel_clock(uint8_t enable, struct msm_panel_info *pinfo)
{
	uint32_t flags, dsi_phy_pll_out;
	uint32_t ret = NO_ERROR;
	uint32_t board_version = board_soc_version();
	uint32_t board_hw_id = board_hardware_id();
	bool video_core_enable = false;
	struct dfps_pll_codes *pll_codes = &pinfo->mipi.pll_codes;

	if (pinfo->dest == DISPLAY_2) {
		flags = MMSS_DSI_CLKS_FLAG_DSI1;
		if (pinfo->lcdc.split_display)
			flags |= MMSS_DSI_CLKS_FLAG_DSI0;
	} else {
		flags = MMSS_DSI_CLKS_FLAG_DSI0;
		if (pinfo->lcdc.split_display)
			flags |= MMSS_DSI_CLKS_FLAG_DSI1;
	}

	/* only required for msm8996 v2 and v2.1 revision */
	video_core_enable = (board_version == 0x20000 || board_version == 0x20001) &&
		!(board_hw_id == MSM8996SG || board_hw_id == APQ8096SG);

	if (!enable) {
		mmss_dsi_clock_disable(flags);

		/* stop pll */
		writel(0x0, pinfo->mipi.phy_base + 0x48);
		dmb();

		goto clks_disable;
	}

	if (video_core_enable)
		video_gdsc_enable();
	mmss_gdsc_enable();
	mmss_bus_clock_enable();
	mdp_clock_enable();
	mdss_dsi_auto_pll_thulium_config(pinfo);

	if (!thulium_dsi_pll_enable_seq(pinfo->mipi.phy_base,
		pinfo->mipi.pll_base)) {
		ret = ERROR;
		dprintf(CRITICAL, "PLL failed to lock!\n");
		goto clks_disable;
	}

	pll_codes->codes[0] = readl_relaxed(pinfo->mipi.pll_base +
			        MMSS_DSI_PHY_PLL_CORE_KVCO_CODE);
	pll_codes->codes[1] = readl_relaxed(pinfo->mipi.pll_base +
			        MMSS_DSI_PHY_PLL_CORE_VCO_TUNE);
	dprintf(SPEW, "codes %d %d\n", pll_codes->codes[0],
			        pll_codes->codes[1]);

	if ((pinfo->mipi.use_dsi1_pll) ||
		((pinfo->mipi.dual_dsi) && (pinfo->dest == DISPLAY_2)))
		dsi_phy_pll_out = DSI1_PHY_PLL_OUT;
	else
		dsi_phy_pll_out = DSI0_PHY_PLL_OUT;
	mmss_dsi_clock_enable(dsi_phy_pll_out, flags);

	return NO_ERROR;

clks_disable:
	mdp_clock_disable();
	mmss_bus_clock_disable();
	mmss_gdsc_disable();
	if (video_core_enable)
		video_gdsc_disable();

	return ret;
}

int target_panel_reset(uint8_t enable, struct panel_reset_sequence *resetseq,
					struct msm_panel_info *pinfo)
{
	uint32_t i = 0;

	if (enable) {
		gpio_tlmm_config(reset_gpio.pin_id, 0,
				reset_gpio.pin_direction, reset_gpio.pin_pull,
				reset_gpio.pin_strength, reset_gpio.pin_state);
		/* reset */
		for (i = 0; i < RESET_GPIO_SEQ_LEN; i++) {
			if (resetseq->pin_state[i] == GPIO_STATE_LOW)
				gpio_set(reset_gpio.pin_id, GPIO_STATE_LOW);
			else
				gpio_set(reset_gpio.pin_id, GPIO_STATE_HIGH);
			mdelay(resetseq->sleep[i]);
		}
		lcd_bklt_reg_enable();
	} else {
		lcd_bklt_reg_disable();
		gpio_set(reset_gpio.pin_id, 0);
	}

	return NO_ERROR;
}

static void wled_init(struct msm_panel_info *pinfo)
{
	struct qpnp_wled_config_data config = {0};
	struct labibb_desc *labibb;
	int display_type = 0;

	// avoid backlight init if bridge chip is being used
	if (pinfo->has_bridge_chip == true)
		return;

	labibb = pinfo->labibb;

	if (labibb)
		display_type = labibb->amoled_panel;

	config.display_type = display_type;
	config.lab_init_volt = 4600000;	/* fixed, see pmi register */
	config.ibb_init_volt = 1400000;	/* fixed, see pmi register */

	if (labibb && labibb->force_config) {
		config.lab_min_volt = labibb->lab_min_volt;
		config.lab_max_volt = labibb->lab_max_volt;
		config.ibb_min_volt = labibb->ibb_min_volt;
		config.ibb_max_volt = labibb->ibb_max_volt;
		config.pwr_up_delay = labibb->pwr_up_delay;
		config.pwr_down_delay = labibb->pwr_down_delay;
		config.ibb_discharge_en = labibb->ibb_discharge_en;
	} else {
		/* default */
		config.pwr_up_delay = 3;
		config.pwr_down_delay =  3;
		config.ibb_discharge_en = 1;
		if (display_type) {	/* amoled */
			config.lab_min_volt = 4600000;
			config.lab_max_volt = 4600000;
			config.ibb_min_volt = 4000000;
			config.ibb_max_volt = 4000000;
		} else { /* lcd */
			config.lab_min_volt = 5500000;
			config.lab_max_volt = 5500000;
			config.ibb_min_volt = 5500000;
			config.ibb_max_volt = 5500000;
		}
	}

	dprintf(SPEW, "%s: %d %d %d %d %d %d %d %d %d %d\n", __func__,
		config.display_type,
		config.lab_min_volt, config.lab_max_volt,
		config.ibb_min_volt, config.ibb_max_volt,
		config.lab_init_volt, config.ibb_init_volt,
		config.pwr_up_delay, config.pwr_down_delay,
		config.ibb_discharge_en);


	/* QPNP WLED init for display backlight */
	pm8x41_wled_config_slave_id(PMIC_WLED_SLAVE_ID);

	qpnp_wled_init(&config);
}

int ldo_ref_cnt = 0;
int target_ldo_ctrl(uint8_t enable, struct msm_panel_info *pinfo)
{
	uint32_t val = BIT(1) | BIT(13) | BIT(27);

	if (enable) {
		if (ldo_ref_cnt == 0) {

			regulator_enable(val);
			mdelay(1);
			wled_init(pinfo);
			qpnp_ibb_enable(true);	/* +5V and -5V */
			mdelay(2);
			if (pinfo->lcd_reg_en)
				lcd_reg_enable();

			ldo_ref_cnt++;
		}
	} else {
		if (ldo_ref_cnt > 0) {
			if (pinfo->lcd_reg_en)
				lcd_reg_disable();

			regulator_disable(val);
			ldo_ref_cnt--;
		}
	}

	return NO_ERROR;
}

int target_display_pre_on()
{
	writel(0xC0000CCC, MDP_CLK_CTRL0);
	writel(0xC0000CCC, MDP_CLK_CTRL1);
	writel(0x00CCCCCC, MDP_CLK_CTRL2);
	writel(0x000000CC, MDP_CLK_CTRL6);
	writel(0x0CCCC0C0, MDP_CLK_CTRL3);
	writel(0xCCCCC0C0, MDP_CLK_CTRL4);
	writel(0xCCCCC0C0, MDP_CLK_CTRL5);
	writel(0x00CCC000, MDP_CLK_CTRL7);

	return NO_ERROR;
}

int target_dsi_phy_config(struct mdss_dsi_phy_ctrl *phy_db)
{
	memcpy(phy_db->strength, panel_strength_ctrl, STRENGTH_SIZE_IN_BYTES_8996 *
		sizeof(uint32_t));
	memcpy(phy_db->regulator, panel_regulator_settings,
		REGULATOR_SIZE_IN_BYTES_8996 * sizeof(uint32_t));
	memcpy(phy_db->laneCfg, panel_lane_config, LANE_SIZE_IN_BYTES_8996);
	return NO_ERROR;
}

bool target_display_panel_node(char *pbuf, uint16_t buf_size)
{
	int prefix_string_len = strlen(DISPLAY_CMDLINE_PREFIX);
	bool ret = true;
	struct oem_panel_data oem = mdss_dsi_get_oem_data();
	char vic_buf[HDMI_VIC_LEN] = "0";

	if ((!strcmp(oem.panel, HDMI_PANEL_NAME)) || \
		((!strlen(oem.panel)) && (platform_is_apq8096_mediabox()))) {
		if (buf_size < (prefix_string_len + LK_OVERRIDE_PANEL_LEN +
				strlen(HDMI_CONTROLLER_STRING))) {
			dprintf(CRITICAL, "command line argument is greater than buffer size\n");
			return false;
		}

		strlcpy(pbuf, DISPLAY_CMDLINE_PREFIX, buf_size);
		buf_size -= prefix_string_len;
		strlcat(pbuf, LK_OVERRIDE_PANEL, buf_size);
		buf_size -= LK_OVERRIDE_PANEL_LEN;
		strlcat(pbuf, HDMI_CONTROLLER_STRING, buf_size);
		buf_size -= strlen(HDMI_CONTROLLER_STRING);
		mdss_hdmi_get_vic(vic_buf);
		strlcat(pbuf, vic_buf, buf_size);
	} else {
		ret = gcdb_display_cmdline_arg(pbuf, buf_size);
	}

	return ret;
}

void target_set_switch_gpio(int enable_dsi2hdmibridge)
{
	gpio_tlmm_config(dsi2hdmi2_switch_gpio.pin_id, 0,
				dsi2hdmi_switch_gpio.pin_direction,
				dsi2hdmi_switch_gpio.pin_pull,
				dsi2hdmi_switch_gpio.pin_strength,
				dsi2hdmi_switch_gpio.pin_state);

	gpio_tlmm_config(dsi2hdmi_switch_gpio.pin_id, 0,
				dsi2hdmi2_switch_gpio.pin_direction,
				dsi2hdmi2_switch_gpio.pin_pull,
				dsi2hdmi2_switch_gpio.pin_strength,
				dsi2hdmi2_switch_gpio.pin_state);

	gpio_set(enable_gpio.pin_id, GPIO_STATE_HIGH);
	if (enable_dsi2hdmibridge)
		gpio_set(enable_gpio.pin_id, GPIO_STATE_LOW); /* DSI2HDMI Bridge */
	else
		gpio_set(enable_gpio.pin_id, GPIO_STATE_HIGH); /* Normal DSI operation */
}

/* Populate the default configurations for each display */
static int target_display_populate(struct target_display *displays)
{
	// the display_id is following the order in targe_disp_init()
	displays[0].display_id = 0;
	displays[0].width = 1280;
	displays[0].height = 720;
	displays[0].dual_pipe = false;
	displays[0].fps = 60;
	displays[0].has_rvc = false;
	displays[0].splitter_display_enabled = false;

	displays[1].display_id = 1;
	displays[1].width = 1280;
	displays[1].height = 720;
	displays[1].dual_pipe = false;
	displays[1].fps = 60;
	displays[1].has_rvc = false;
	displays[1].splitter_display_enabled = false;

	displays[2].display_id = 2;
	displays[2].width = 1920;
	displays[2].height = 1080;
	displays[2].dual_pipe = false;
	displays[2].fps = 60;
	displays[2].has_rvc = false;
	displays[2].splitter_display_enabled = false;

	return 0;
}

static int target_layers_populate(struct target_layer_int *layers)
{
	int i = 0;

	for (i = 0; i < NUM_RGB_PIPES; i++) {
		layers[RGB_PIPE_START + i].layer_id = i;
		layers[RGB_PIPE_START + i].layer_type = RGB_TYPE;
		layers[RGB_PIPE_START + i].assigned = 0;
	}

	for (i = 0; i < NUM_VIG_PIPES; i++) {
		layers[VIG_PIPE_START + i].layer_id = i;
		layers[VIG_PIPE_START + i].layer_type = VIG_TYPE;
		layers[VIG_PIPE_START + i].assigned = 0;
	}

	for (i = 0; i < NUM_DMA_PIPES; i++) {
		layers[DMA_PIPE_START + i].layer_id = i;
		layers[DMA_PIPE_START + i].layer_type = DMA_TYPE;
		layers[DMA_PIPE_START + i].assigned = 0;
	}

	return 0;
}

void target_display_HDMI_resolution (uint32_t *width, uint32_t *height)
{
	uint32_t reg;
	int start_v, end_v, start_h, end_h;

	// Get HDMI resolution
	reg = readl(HDMI_ACTIVE_V);
	start_v = (int)(reg & 0xFFF);
	end_v = (int)(reg >> 16);
	reg = readl(HDMI_ACTIVE_H);
	start_h = (int)(reg & 0xFFF);
	end_h = (int)(reg >> 16);
	*width = end_h - start_h;
	*height = end_v - start_v;
}

static int panel_name_parser (char *panel_name)
{
	int panel_count = 1;
	unsigned int i = 0;
	unsigned int length = strlen(panel_name);

	for (i = 0; i < length; i++) {
		if (panel_name[i] == PANEL_NAME_DELIMITER) {
			panel_count++;
			panel_name[i] = 0;
		}
	}
	return panel_count;
}

static bool composite_panel_name(const char *panel_name)
{
	unsigned int i;

	for (i = 0; i < (strlen(panel_name)); i++)
		if (panel_name[i] == PANEL_NAME_DELIMITER)
			return true;

	return false;
}

void target_display_init(const char *panel_name)
{
	struct oem_panel_data oem;
	uint32_t rvc_disp_id = 0;
	uint32_t shared_disp_id = 0;

	target_display_populate(displays);
	target_layers_populate(layers);

	set_panel_cmd_string(panel_name);
	oem = mdss_dsi_get_oem_data();

	target_display_set_panel_type(oem.panel);

	if (target_utils_validate_input_config(oem.panel, &rvc_disp_id, RVC_DISPLAY))
		displays[rvc_disp_id].has_rvc = true;

	if (target_utils_validate_input_config(oem.panel, &shared_disp_id, SHARE_DISPLAY))
		displays[shared_disp_id].splitter_display_enabled = true;

	if (!strcmp(oem.panel, "")
		|| !strcmp(oem.panel, NO_PANEL_CONFIG)
		|| !strcmp(oem.panel, SIM_VIDEO_PANEL)
		|| !strcmp(oem.panel, SIM_DUALDSI_VIDEO_PANEL)
		|| !strcmp(oem.panel, SIM_CMD_PANEL)
		|| !strcmp(oem.panel, SIM_DUALDSI_CMD_PANEL)
		|| oem.skip) {
		dprintf(INFO, "Selected panel: %s\nSkip panel configuration\n",
			oem.panel);
		goto target_display_init_end;
	} else if (!strcmp(oem.panel, HDMI_PANEL_NAME)) {
		mdss_hdmi_display_init(MDP_REV_50, (void *)HDMI_FB_ADDR,
				displays[DISPLAY_1 - DISPLAY_1].splitter_display_enabled);
		// Get HDMI resolution
		target_display_HDMI_resolution (&displays[DISPLAY_1 - DISPLAY_1].width,
				&displays[DISPLAY_1 - DISPLAY_1].height);

		if (displays[DISPLAY_1 - DISPLAY_1].width > 2560)
			displays[DISPLAY_1 - DISPLAY_1].dual_pipe = true;
		else
			displays[DISPLAY_1 - DISPLAY_1].dual_pipe = false;

		displays[DISPLAY_2 - DISPLAY_1].width = 0;
		displays[DISPLAY_2 - DISPLAY_1].height = 0;
		displays[DISPLAY_3 - DISPLAY_1].width = 0;
		displays[DISPLAY_3 - DISPLAY_1].height = 0;
		goto target_display_init_end;
	} else if (!strcmp(oem.panel, "dual_720p_single_hdmi_video")) {
		// Three display panels init, init DSI0 first
		gcdb_display_init("adv7533_720p_dsi0_video", MDP_REV_50,
			(void *)MIPI_FB_ADDR,
			displays[DISPLAY_1 - DISPLAY_1].splitter_display_enabled);

		// if the panel has different size or color format, they cannot use
		// the same FB buffer
		gcdb_display_init("adv7533_720p_dsi1_video", MDP_REV_50,
				(void *)MIPI_FB_ADDR + 0x1000000,
				displays[DISPLAY_2 - DISPLAY_1].splitter_display_enabled);

		displays[DISPLAY_1 - DISPLAY_1].width = 1280;
		displays[DISPLAY_1 - DISPLAY_1].height = 720;
		displays[DISPLAY_2 - DISPLAY_1].width = 1280;
		displays[DISPLAY_2 - DISPLAY_1].height = 720;

		mdss_hdmi_display_init(MDP_REV_50, (void *)HDMI_FB_ADDR,
				displays[DISPLAY_3 - DISPLAY_1].splitter_display_enabled);
		// Get HDMI resolution
		target_display_HDMI_resolution (&displays[DISPLAY_3 - 1].width,
				&displays[DISPLAY_3 - DISPLAY_1].height);

		if (displays[DISPLAY_3 - DISPLAY_1].width > 2560)
			displays[DISPLAY_3 - DISPLAY_1].dual_pipe = true;
		else
			displays[DISPLAY_3 - DISPLAY_1].dual_pipe = false;

		goto target_display_init_end;
	} else if (!strcmp(oem.panel, "dual_1080p_single_hdmi_video")) {
		// Three display panels init, init DSI0 first
		gcdb_display_init("adv7533_1080p_dsi0_video", MDP_REV_50,
				(void *)MIPI_FB_ADDR,
				displays[DISPLAY_1 - DISPLAY_1].splitter_display_enabled);

		// if the panel has different size or color format, they cannot use
		// the same FB buffer
		gcdb_display_init("adv7533_1080p_dsi1_video", MDP_REV_50,
				(void *)MIPI_FB_ADDR + 0x1000000,
				displays[DISPLAY_2 - DISPLAY_1].splitter_display_enabled);

		displays[DISPLAY_1 - DISPLAY_1].width = 1920;
		displays[DISPLAY_1 - DISPLAY_1].height = 1080;
		displays[DISPLAY_2 - DISPLAY_1].width = 1920;
		displays[DISPLAY_2 - DISPLAY_1].height = 1080;

		mdss_hdmi_display_init(MDP_REV_50, (void *)HDMI_FB_ADDR,
			displays[DISPLAY_3 - DISPLAY_1].splitter_display_enabled);
		// Get HDMI resolution
		target_display_HDMI_resolution (&displays[DISPLAY_3 - DISPLAY_1].width,
				&displays[DISPLAY_3 - DISPLAY_1].height);
		if (displays[DISPLAY_3 - DISPLAY_1].width > 2560)
			displays[DISPLAY_3 - DISPLAY_1].dual_pipe = true;
		else
			displays[DISPLAY_3 - DISPLAY_1].dual_pipe = false;

		goto target_display_init_end;
	} else if (!strcmp(oem.panel, "single_720p_single_hdmi_video")) {
		// Dual display panels init, init DSI0 first
		gcdb_display_init("adv7533_720p_video", MDP_REV_50,
				(void *)MIPI_FB_ADDR,
				displays[DISPLAY_1 - DISPLAY_1].splitter_display_enabled);

		displays[DISPLAY_1 - DISPLAY_1].width = 1280;
		displays[DISPLAY_1 - DISPLAY_1].height = 720;

		mdss_hdmi_display_init(MDP_REV_50, (void *)HDMI_FB_ADDR,
			displays[DISPLAY_2 - DISPLAY_1].splitter_display_enabled);
		// Get HDMI resolution
		target_display_HDMI_resolution (&displays[DISPLAY_2 - DISPLAY_1].width,
				&displays[DISPLAY_2 - DISPLAY_1].height);
		if (displays[DISPLAY_2 - DISPLAY_1].width > 2560)
			displays[DISPLAY_2 - DISPLAY_1].dual_pipe = true;
		else
			displays[DISPLAY_2 - DISPLAY_1].dual_pipe = false;

		displays[DISPLAY_3 - DISPLAY_1].width = 0;
		displays[DISPLAY_3 - DISPLAY_1].height = 0;
		goto target_display_init_end;
	} else if (!strcmp(oem.panel, "single_1080p_single_hdmi_video")) {
		//Single display panel init, init DSI0 only
		gcdb_display_init("adv7533_1080p_video", MDP_REV_50,
				(void *)MIPI_FB_ADDR,
				displays[DISPLAY_1 - DISPLAY_1].splitter_display_enabled);

		displays[DISPLAY_1 - DISPLAY_1].width = 1920;
		displays[DISPLAY_1 - DISPLAY_1].height = 1080;

		mdss_hdmi_display_init(MDP_REV_50, (void *)HDMI_FB_ADDR,
				displays[DISPLAY_2 - DISPLAY_1].splitter_display_enabled);

		// Get HDMI resolution
		target_display_HDMI_resolution (&displays[DISPLAY_2 - DISPLAY_1].width,
				&displays[DISPLAY_2 - DISPLAY_1].height);
		if (displays[DISPLAY_2 - DISPLAY_1].width > 2560)
			displays[DISPLAY_2 - DISPLAY_1].dual_pipe = true;
		else
			displays[DISPLAY_2 - DISPLAY_1].dual_pipe = false;

		displays[DISPLAY_3 - DISPLAY_1].width = 0;
		displays[DISPLAY_3 - DISPLAY_1].height = 0;

		goto target_display_init_end;
	} else if (!strcmp(oem.panel, "adv7533_2560w")) {
		displays[DISPLAY_1 - DISPLAY_1].width = 2560;
		displays[DISPLAY_1 - DISPLAY_1].height = 720;
		displays[DISPLAY_1 - DISPLAY_1].dual_pipe = true;
		displays[DISPLAY_2 - DISPLAY_1].width = 0;
		displays[DISPLAY_2 - DISPLAY_1].height = 0;
		displays[DISPLAY_3 - DISPLAY_1].width = 0;
		displays[DISPLAY_3 - DISPLAY_1].height = 0;
	} else if (!strcmp(oem.panel, "adv7533_3840w")) {
		displays[DISPLAY_1 - DISPLAY_1].width = 3840;
		displays[DISPLAY_1 - DISPLAY_1].height = 1080;
		displays[DISPLAY_1 - DISPLAY_1].dual_pipe = true;
		displays[DISPLAY_2 - DISPLAY_1].width = 0;
		displays[DISPLAY_2 - DISPLAY_1].height = 0;
		displays[DISPLAY_3 - DISPLAY_1].width = 0;
		displays[DISPLAY_3 - DISPLAY_1].height = 0;
	} else if (!strcmp(oem.panel, "adv7533_3840w_hdmi")) {
		/* shared display can't co-exist with dsi split case */
		gcdb_display_init("adv7533_3840w", MDP_REV_50,
				(void *)MIPI_FB_ADDR, false);

		displays[DISPLAY_1 - DISPLAY_1].width = 3840;
		displays[DISPLAY_1 - DISPLAY_1].height = 1080;
		displays[DISPLAY_1 - DISPLAY_1].dual_pipe = true;

		mdss_hdmi_display_init(MDP_REV_50, (void *)HDMI_FB_ADDR,
				displays[DISPLAY_2 - DISPLAY_1].splitter_display_enabled);

		// Get HDMI resolution
		target_display_HDMI_resolution (&displays[DISPLAY_2 - DISPLAY_1].width,
				&displays[DISPLAY_2 - DISPLAY_1].height);
		if (displays[DISPLAY_2 - DISPLAY_1].width > 2560)
			displays[DISPLAY_2 - DISPLAY_1].dual_pipe = true;
		else
			displays[DISPLAY_2 - DISPLAY_1].dual_pipe = false;

		displays[DISPLAY_3 - DISPLAY_1].width = 0;
		displays[DISPLAY_3 - DISPLAY_1].height = 0;

		goto target_display_init_end;
	} else if (!strcmp(oem.panel, "adv7533_2560w_hdmi")) {
		/* shared display can't co-exist with dsi split case */
		gcdb_display_init("adv7533_2560w", MDP_REV_50,
				(void *)MIPI_FB_ADDR, false);

		displays[DISPLAY_1 - DISPLAY_1].width = 2560;
		displays[DISPLAY_1 - DISPLAY_1].height = 720;
		displays[DISPLAY_1 - DISPLAY_1].dual_pipe = true;

		mdss_hdmi_display_init(MDP_REV_50, (void *)HDMI_FB_ADDR,
				displays[DISPLAY_2 - DISPLAY_1].splitter_display_enabled);

		// Get HDMI resolution
		target_display_HDMI_resolution (&displays[DISPLAY_2 - DISPLAY_1].width,
				&displays[DISPLAY_2 - DISPLAY_1].height);
		if (displays[DISPLAY_2 - DISPLAY_1].width > 2560)
			displays[DISPLAY_2 - DISPLAY_1].dual_pipe = true;
		else
			displays[DISPLAY_2 - DISPLAY_1].dual_pipe = false;

		displays[DISPLAY_3 - DISPLAY_1].width = 0;
		displays[DISPLAY_3 - DISPLAY_1].height = 0;

		goto target_display_init_end;
	} else if (!strcmp(oem.panel, "adv7533_3840w_swap_hdmi")) {
		/* shared display can't co-exist with dsi split case */
		gcdb_display_init("adv7533_3840w_swap", MDP_REV_50,
				(void *)MIPI_FB_ADDR, false);

		displays[DISPLAY_1 - DISPLAY_1].width = 3840;
		displays[DISPLAY_1 - DISPLAY_1].height = 1080;
		displays[DISPLAY_1 - DISPLAY_1].dual_pipe = true;

		mdss_hdmi_display_init(MDP_REV_50, (void *)HDMI_FB_ADDR,
				displays[DISPLAY_2 - DISPLAY_1].splitter_display_enabled);

		// Get HDMI resolution
		target_display_HDMI_resolution (&displays[DISPLAY_2 - DISPLAY_1].width,
				&displays[DISPLAY_2 - DISPLAY_1].height);
		if (displays[DISPLAY_2 - DISPLAY_1].width > 2560)
			displays[DISPLAY_2 - DISPLAY_1].dual_pipe = true;
		else
			displays[DISPLAY_2 - DISPLAY_1].dual_pipe = false;

		displays[DISPLAY_3 - DISPLAY_1].width = 0;
		displays[DISPLAY_3 - DISPLAY_1].height = 0;

		goto target_display_init_end;
	} else if (!strcmp(oem.panel, "dsi0_600p_dsi1_720p_hdmi_video")) {
		// Initialize DSI0 in 1024x600 resolution
		gcdb_display_init("adv7533_1024_600p_dsi0_video", MDP_REV_50,
				(void *)MIPI_FB_ADDR,
				displays[DISPLAY_1 - DISPLAY_1].splitter_display_enabled);
		displays[DISPLAY_1 - DISPLAY_1].width = 1024;
		displays[DISPLAY_1 - DISPLAY_1].height = 600;

		// Initialize DSI1 in 1280x720 resolution
		gcdb_display_init("adv7533_720p_dsi1_video", MDP_REV_50,
				(void *)MIPI_FB_ADDR + 0x1000000,
				displays[DISPLAY_2 - DISPLAY_1].splitter_display_enabled);
		displays[DISPLAY_2 - DISPLAY_1].width = 1280;
		displays[DISPLAY_2 - DISPLAY_1].height = 720;

		mdss_hdmi_display_init(MDP_REV_50, (void *)HDMI_FB_ADDR,
			displays[DISPLAY_3 - DISPLAY_1].splitter_display_enabled);
		// Get HDMI resolution
		target_display_HDMI_resolution (&displays[DISPLAY_3 - DISPLAY_1].width,
				&displays[DISPLAY_3 - DISPLAY_1].height);
		if (displays[DISPLAY_3 - DISPLAY_1].width > 2560)
			displays[DISPLAY_3 - DISPLAY_1].dual_pipe = true;
		else
			displays[DISPLAY_3 - DISPLAY_1].dual_pipe = false;

		goto target_display_init_end;
		return;
	} else if (composite_panel_name(oem.panel)) {
		int i, panel_count;
		char* panel[3];
		dprintf(SPEW,"%s is composite_panel_name, lenght:%d\n", oem.panel, strlen(oem.panel));

		panel_count = panel_name_parser (oem.panel);
		panel[0] = oem.panel;
		panel[1] = oem.panel + strlen (oem.panel) + 1;
		if (panel_count == 3)
			panel[2] = oem.panel + strlen (oem.panel) + strlen (panel[1]) + 2;

		for (i = 0; ((i < panel_count) && (i < MAX_PANEL_COUNT)); i++) {
			if (!strcmp(panel[i], HDMI_PANEL_NAME)) {
				dprintf (SPEW, "panel%d name is %s\n", i, panel[i]);
				mdss_hdmi_display_init(MDP_REV_50, (void *)HDMI_FB_ADDR,
					displays[i].splitter_display_enabled);
				// Get HDMI resolution
				target_display_HDMI_resolution (&displays[i].width, &displays[i].height);
				if (displays[i].width > 2560)
					displays[i].dual_pipe = true;
				else
					displays[i].dual_pipe = false;
			} else {
				gcdb_display_init(panel[i], MDP_REV_50, (void *)MIPI_FB_ADDR,
					displays[i].splitter_display_enabled);

				if ((!strcmp(panel[i], "adv7533_1080p_dsi0_video")) ||
					(!strcmp(panel[i], "adv7533_1080p_dsi1_video")) ||
					(!strcmp(panel[i], "adv7533_1080p_video")) ||
					(!strcmp(panel[i], "adv7533_1080p"))) {
					displays[i].width = 1920;
					displays[i].height = 1080;
				} else if ((!strcmp(panel[i], "adv7533_720p_dsi0_video"))||
					(!strcmp(panel[i], "adv7533_720p_dsi1_video")) ||
					(!strcmp(panel[i], "adv7533_720p_video")) ||
					(!strcmp(panel[i], "adv7533_720p"))) {
					displays[i].width = 1280;
					displays[i].height = 720;
				}
			}
		}
		goto target_display_init_end;
	}

	if (gcdb_display_init(oem.panel, MDP_REV_50, (void *)MIPI_FB_ADDR, false)) {
		target_force_cont_splash_disable(true);
		msm_display_off();
	}

	if (!oem.cont_splash) {
		dprintf(INFO, "Forcing continuous splash disable\n");
		target_force_cont_splash_disable(true);
	}

target_display_init_end:
	display_init_done = true;
}

void target_display_shutdown(void)
{
	struct oem_panel_data oem = mdss_dsi_get_oem_data();
	if ((!strcmp(oem.panel, HDMI_PANEL_NAME)) || \
		((!strlen(oem.panel)) && (platform_is_apq8096_mediabox()))) {
		msm_display_off();
	} else {
		gcdb_display_shutdown();
	}
	display_init_done = false;
}

/* DYNAMIC APIS */
void * target_acquire_rbg_pipe(struct target_display *disp)
{
	int i = 0;

	for (i = 0; i < NUM_RGB_PIPES; i++) {
		if (!layers[RGB_PIPE_START + i].assigned) {
			if (disp->dual_pipe) {
				if ((i + 1) < (NUM_RGB_PIPES)) {
					if (!layers[RGB_PIPE_START + i + 1].assigned) {
						layers[RGB_PIPE_START + i + 1].assigned = true;
						layers[RGB_PIPE_START + i + 1].disp = disp;
						dprintf(SPEW,"right RGB%d acquired\n", (i+1));
					}
				}
			}
			layers[RGB_PIPE_START + i].assigned = true;
			layers[RGB_PIPE_START + i].disp = disp;
			return &layers[RGB_PIPE_START + i];
		}
	}
	return NULL;
}

/* DYNAMIC APIS */
void * target_acquire_vig_pipe(struct target_display *disp)
{
	int i = 0;
	for (i = 0; i < NUM_VIG_PIPES ; i++) {
		if (!layers[VIG_PIPE_START + i].assigned) {
			if (disp->dual_pipe) {
				if ((i + 1) < (NUM_VIG_PIPES)) {
					if (!layers[VIG_PIPE_START + i + 1].assigned) {
					    layers[VIG_PIPE_START + i + 1].assigned = true;
					    layers[VIG_PIPE_START + i + 1].disp = disp;
						dprintf(SPEW,"right VIG%d acquired\n", (i+1));
					}
				}
			}
			layers[VIG_PIPE_START + i].assigned = true;
			layers[VIG_PIPE_START + i].disp = disp;
			return &layers[VIG_PIPE_START + i];
		}
	}
	return NULL;
}

void * target_display_open (uint32 display_id)
{
	if (display_id >= MAX_NUM_DISPLAY) {
		dprintf(CRITICAL, "Invalid display id %u\n", display_id);
		return NULL;
	} else {
		return (void *) &displays[display_id];
	}
}

uint32_t target_display_get_rvc_display_id()
{
	uint32_t i;

	for (i = 0; i < MAX_NUM_DISPLAY; i++) {
		if (displays[i].has_rvc)
			return i;
	}

	/* by default, use primary display for rvc */
	return 0;
}

struct target_display * target_get_display_info(void *disp)
{
	return (struct target_display *) disp;
}

void *target_display_acquire_layer(struct target_display *disp, char *client_name, int color_format)
{
	if (color_format < kFormatYCbCr422H2V1Packed)
		return target_acquire_rbg_pipe(disp);
	else
		return target_acquire_vig_pipe(disp);
}

struct fbcon_config* target_display_get_fb(uint32_t disp_id, uint32_t fb_index)
{
	return msm_display_get_fb(disp_id, fb_index);
}

int target_display_update(struct target_display_update *update, uint32_t size, uint32_t disp_id)
{
	uint32_t i = 0;
	uint32_t pipe_type, pipe_id;
	struct target_layer_int *lyr;
	struct target_display *cur_disp;
	uint32_t ret = 0;
#if ENABLE_QSEED_SCALAR
	struct Scale left_scale_setting;
	struct Scale right_scale_setting;
	struct LayerInfo layer;
	uint32_t fb_index = SPLIT_DISPLAY_0;
#endif

	if (update == NULL) {
		dprintf(CRITICAL, "Error: Invalid argument\n");
		return ERR_INVALID_ARGS;
	}

	for (i = 0; i < size; i++) {
		cur_disp = (struct target_display *)update[i].disp;
		lyr = (struct target_layer_int *)update[i].layer_list[0].layer;
		if (lyr == NULL) {
			dprintf(CRITICAL, "Invalid layer entry %p\n",cur_disp);
			return ERR_INVALID_ARGS;
		}
		pipe_id = lyr->layer_id;
		pipe_type = lyr->layer_type;
#if ENABLE_QSEED_SCALAR
		layer.src_format = update[i].layer_list[0].fb[SPLIT_DISPLAY_0].format;
		//setup Layer structure for scaling
		memset((void*)&left_scale_setting, 0, sizeof (struct Scale));
		memset((void*)&right_scale_setting, 0, sizeof (struct Scale));
		layer.left_pipe.scale_data = &left_scale_setting;
		layer.right_pipe.scale_data = &right_scale_setting;
		layer.left_pipe.scale_data->enable_pxl_ext = 0;
		layer.right_pipe.scale_data->enable_pxl_ext = 0;
		layer.left_pipe.id = 1;
		layer.right_pipe.id = 0;
		// call Qseed function to get the scaling data
		if (cur_disp->dual_pipe) {
			/* left pipe */
			layer.left_pipe.horz_deci = 0;
			layer.left_pipe.vert_deci = 0;
			layer.left_pipe.src_width = update[i].layer_list[0].src_width[fb_index];
			layer.left_pipe.src_height = update[i].layer_list[0].src_height[fb_index];

			if (cur_disp->splitter_display_enabled) {
				/* In split case, left pipe owns the whole left fb */
				layer.left_pipe.src_rect.x = update[i].layer_list[0].src_rect_x[fb_index];
				layer.left_pipe.src_rect.y = update[i].layer_list[0].src_rect_y[fb_index];
				layer.left_pipe.src_rect.w = update[i].layer_list[0].src_width[fb_index];
				layer.left_pipe.src_rect.h = update[i].layer_list[0].src_height[fb_index];
				layer.left_pipe.dst_rect.x = update[i].layer_list[0].dst_rect_x[fb_index];
				layer.left_pipe.dst_rect.y = update[i].layer_list[0].dst_rect_y[fb_index];
				layer.left_pipe.dst_rect.w = update[i].layer_list[0].dst_width[fb_index];
				layer.left_pipe.dst_rect.h = update[i].layer_list[0].dst_height[fb_index];
			} else {
				layer.left_pipe.src_rect.x = 0;
				layer.left_pipe.src_rect.y = 0;
				layer.left_pipe.src_rect.w = update[i].layer_list[0].src_width[fb_index] / 2;
				layer.left_pipe.src_rect.h = update[i].layer_list[0].src_height[fb_index];
				layer.left_pipe.dst_rect.x = (cur_disp->width - update[i].layer_list[0].dst_width[fb_index]) / 2;
				layer.left_pipe.dst_rect.y = (cur_disp->height - update[i].layer_list[0].dst_height[fb_index]) / 2;
				layer.left_pipe.dst_rect.w = update[i].layer_list[0].dst_width[fb_index] / 2;
				layer.left_pipe.dst_rect.h = update[i].layer_list[0].dst_height[fb_index];
			}

			/* right pipe */
			if (cur_disp->splitter_display_enabled)
				++fb_index;

			layer.right_pipe.horz_deci = 0;
			layer.right_pipe.vert_deci = 0;
			layer.right_pipe.src_width = update[i].layer_list[0].src_width[fb_index];
			layer.right_pipe.src_height = update[i].layer_list[0].src_height[fb_index];

			if (cur_disp->splitter_display_enabled) {
				/* In split case, right pipe owns the whole right fb */
				layer.left_pipe.src_rect.x = update[i].layer_list[0].src_rect_x[fb_index];
				layer.left_pipe.src_rect.y = update[i].layer_list[0].src_rect_y[fb_index];
				layer.left_pipe.src_rect.w = update[i].layer_list[0].src_width[fb_index];
				layer.left_pipe.src_rect.h = update[i].layer_list[0].src_height[fb_index];
				layer.left_pipe.dst_rect.x = update[i].layer_list[0].dst_rect_x[fb_index];
				layer.left_pipe.dst_rect.y = update[i].layer_list[0].dst_rect_y[fb_index];
				layer.left_pipe.dst_rect.w = update[i].layer_list[0].dst_width[fb_index];
				layer.left_pipe.dst_rect.h = update[i].layer_list[0].dst_height[fb_index];
			} else {
				layer.right_pipe.src_rect.x = update[i].layer_list[0].src_width[fb_index] / 2;
				layer.right_pipe.src_rect.y = 0;
				layer.right_pipe.src_rect.w = update[i].layer_list[0].src_width[fb_index] / 2;
				layer.right_pipe.src_rect.h = update[i].layer_list[0].src_height[fb_index];
				layer.right_pipe.dst_rect.x = cur_disp->width / 2;
				layer.right_pipe.dst_rect.y = (cur_disp->height - update[i].layer_list[0].dst_height[fb_index]) / 2;
				layer.right_pipe.dst_rect.w = update[i].layer_list[0].dst_width[fb_index] / 2;
				layer.right_pipe.dst_rect.h = update[i].layer_list[0].dst_height[fb_index];
			}

			layer.left_pipe.flags = SCALAR_DUAL_PIPE;
			layer.right_pipe.flags = SCALAR_DUAL_PIPE;
			if (pipe_type == MDSS_MDP_PIPE_TYPE_VIG) {
				dualQseedScalar(&layer);
				dprintf(SPEW, "dual pipe Qseed Scalar src:%d-%d\n",
                                                                update[i].layer_list[0].src_width[SPLIT_DISPLAY_0], update[i].layer_list[0].src_height[SPLIT_DISPLAY_0]);
			} else {
				dualRgbScalar(&layer);
				dprintf(SPEW, "dual pipe RGB Scalar src:%d-%d\n",
                                                                update[i].layer_list[0].src_width[SPLIT_DISPLAY_0], update[i].layer_list[0].src_height[SPLIT_DISPLAY_0]);
			}
		} else {
			layer.left_pipe.horz_deci = 0;
			layer.left_pipe.vert_deci = 0;
			layer.left_pipe.src_width = update[i].layer_list[0].src_width[fb_index];
			layer.left_pipe.src_height = update[i].layer_list[0].src_height[fb_index];
			layer.left_pipe.src_rect.x = update[i].layer_list[0].src_rect_x[fb_index];
			layer.left_pipe.src_rect.y = update[i].layer_list[0].src_rect_y[fb_index];
			layer.left_pipe.src_rect.w = update[i].layer_list[0].src_width[fb_index];
			layer.left_pipe.src_rect.h = update[i].layer_list[0].src_height[fb_index];
			layer.left_pipe.dst_rect.x = update[i].layer_list[0].dst_rect_x[fb_index];
			layer.left_pipe.dst_rect.y = update[i].layer_list[0].dst_rect_y[fb_index];
			layer.left_pipe.dst_rect.w = update[i].layer_list[0].dst_width[fb_index];
			layer.left_pipe.dst_rect.h = update[i].layer_list[0].dst_height[fb_index];
			/* make sure the right pipe is empty for single pipe case */
			memset((void*)&layer.right_pipe, 0, sizeof (struct PipeInfo));
			if (pipe_type == MDSS_MDP_PIPE_TYPE_VIG) {
				singleQseedScalar(&layer);
			} else {
				singleRgbScalar(&layer);
			}
		}
		update[i].layer_list[0].fb[SPLIT_DISPLAY_0].layer_scale = &layer;
		update[i].layer_list[0].fb[SPLIT_DISPLAY_1].layer_scale = NULL;
		if (cur_disp->splitter_display_enabled && pipe_type != MDSS_MDP_PIPE_TYPE_VIG)
			update[i].layer_list[0].fb[SPLIT_DISPLAY_0].layer_scale = NULL;
#endif

		ret = msm_display_update(update[i].layer_list[0].fb, pipe_id, pipe_type, update[i].layer_list[0].z_order,
			update[i].layer_list[0].dst_width, update[i].layer_list[0].dst_height, disp_id);
		if (ret != 0)
			dprintf(CRITICAL, "Error in display upadte ret=%u\n",ret);

#if ENABLE_QSEED_SCALAR
		// Clean up the FB structure because it will be reuse
		update[i].layer_list[0].fb[SPLIT_DISPLAY_0].layer_scale = NULL;
		update[i].layer_list[0].fb[SPLIT_DISPLAY_1].layer_scale = NULL;
#endif
	}
	return ret;
}

int target_display_update_pipe(struct target_display_update * update, uint32_t size,
							uint32_t disp_id)
{
	uint32_t i = 0;
	uint32_t pipe_type, pipe_id;
	struct target_layer_int *lyr;
	struct target_display *cur_disp;
	uint32_t ret = 0;

	if (update == NULL) {
		dprintf(CRITICAL, "Error: Invalid argument\n");
		return ERR_INVALID_ARGS;
	}

	for (i = 0; i < size; i++) {
		cur_disp = (struct target_display *)update[i].disp;
		lyr = (struct target_layer_int *)update[i].layer_list[0].layer;
		if (lyr == NULL) {
			dprintf(CRITICAL, "Invalid layer entry %p\n",cur_disp);
			return ERR_INVALID_ARGS;
		}
		pipe_id = lyr->layer_id;
		pipe_type = lyr->layer_type;

		ret = msm_display_update_pipe(update[i].layer_list[0].fb, pipe_id, pipe_type, update[i].layer_list[0].z_order,
			update[i].layer_list[0].dst_width, update[i].layer_list[0].dst_height, disp_id);
		if (ret != 0)
			dprintf(CRITICAL, "Error in display pipe upadte ret=%u\n",ret);

	}
	return ret;
}

int target_release_layer(struct target_layer *layer)
{
	struct target_layer_int *cur_layer = NULL;
	uint32_t disp_id, pipe_id, pipe_type;
	bool    dual_pipe;

	if ((!layer) || (NULL == layer->layer)) {
		return ERR_INVALID_ARGS;
	}

	cur_layer = (struct target_layer_int *) layer->layer;
	disp_id = cur_layer->disp->display_id;
	pipe_id = cur_layer->layer_id;
	pipe_type = cur_layer->layer_type;
	dual_pipe = cur_layer->disp->dual_pipe;
	cur_layer->assigned = false;
	cur_layer->disp = NULL;

	// if display is dual_pipe, the adjacent pipe is allocated too
	if (dual_pipe) {

		cur_layer ++;
		cur_layer->assigned = false;
		cur_layer->disp = NULL;
	}

	msm_display_hide_pipe(pipe_id, pipe_type, disp_id);
	return 0;
}

int target_is_yuv_format(uint32_t format)
{
	if (format < kFormatYCbCr422H2V1Packed)
		return 0;
	else
		return 1;
}

int target_display_close(struct target_display * disp) {
	return 0;
}

int target_get_max_display() {
	return MAX_NUM_DISPLAY;
}

int target_display_init_count() {
	return msm_display_init_count();
}
