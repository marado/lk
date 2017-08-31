/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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
#include <reg.h>
#include <platform/gpio.h>
#include <target/target_audio.h>
#include <stdlib.h>
#include <debug.h>
#include <target.h>
#include <platform.h>
#include <qtimer.h>
#include <mmc.h>
#include <partition_parser.h>
#include <string.h>

#define lpass_io_w(d,a) writel(d,a)
#define lpass_io_r(a) readl(a)

#define LPASS_REG_OFFSET(_virt_addr_, _phys_addr_) \
	(((_virt_addr_)-(_phys_addr_)) >> 2)

#define GPIO_MI2S_FUNC              1
#define GPIO_PIN_PRI_MI2S_SCK      65
#define GPIO_PIN_PRI_MI2S_WS       66
#define GPIO_PIN_PRI_MI2S_DATA1    68
#define GPIO_PIN_SEC_MI2S_SCK      80
#define GPIO_PIN_SEC_MI2S_WS       81
#define GPIO_PIN_SEC_MI2S_DATA1    83
#define GPIO_PIN_TER_MI2S_SCK      75
#define GPIO_PIN_TER_MI2S_WS       76
#define GPIO_PIN_TER_MI2S_DATA1    78
#define GPIO_PIN_QUA_MI2S_SCK      58
#define GPIO_PIN_QUA_MI2S_WS       59
#define GPIO_PIN_QUA_MI2S_DATA1    61

#define GCC_CLK_CTL_BASE           0x00300000
#define LPASS_CC_BASE              0x09000000
#define LPASS_TCSR_BASE            0x09080000
#define LPASS_CSR_BASE             0x090C0000

#define GCC_CLK_CTL_INIT(_gcc_clk_ctl_base_) \
	{_gcc_clk_ctl_base_+GCC_LPASS_SWAY_CBCR_OFFSET, 0x80008001}, \
	{_gcc_clk_ctl_base_+GCC_LPASS_CORE_SMMU_AHB_CBCR_OFFSET, 0x80000001}, \
	{_gcc_clk_ctl_base_+GCC_LPASS_SMMU_AON_AHB_CBCR_OFFSET, 0x80008001},

#define LPASS_INIT(_lpass_cc_base_, _lpass_tcsr_base_, _lpass_csr_base_) \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_BCR_SLP_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_Q6SS_BCR_SLP_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_GDSCR_OFFSET, LPASS_CLK_DISABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_GDSC_XO_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_Q6_SMMU_GDSCR_OFFSET, LPASS_CLK_DISABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_Q6_SMMU_GDSC_XO_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPAIF_CODEC_SPKR_OSR_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPAIF_CODEC_SPKR_IBIT_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPAIF_CODEC_SPKR_EBIT_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPAIF_PRI_IBIT_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPAIF_PRI_EBIT_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPAIF_SEC_IBIT_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPAIF_SEC_EBIT_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPAIF_TER_IBIT_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPAIF_TER_EBIT_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPAIF_QUAD_IBIT_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPAIF_QUAD_EBIT_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_AON_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_AVSYNC_ATIME_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_RESAMPLER_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_AUD_SLIMBUS_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_AUD_SLIMBUS_CORE_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_QCA_SLIMBUS_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_QCA_SLIMBUS_CORE_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPAIF_PCM_DATA_OE_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_AVSYNC_STC_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_LPM_CORE_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_CORE_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_EXT_MCLK0_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_EXT_MCLK1_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_EXT_MCLK2_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_SYSNOC_MPORT_CORE_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_SYSNOC_SWAY_SNOC_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_SYSNOC_SWAY_AON_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_Q6SS_AHBM_AON_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_PERIPHERAL_SMMU_CLIENT_CORE_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_QDSP_SWAY_AON_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_SYSNOC_SWAY_SNOC_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_CORE_PERIPHERAL_SMMU_CFG_CNOC_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_Q6_SMMU_CFG_CNOC_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_Q6SS_Q6_AXIM_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_Q6_SMMU_AXI_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_Q6SS_Q6_CAMSS_DSP_STREAMING_0_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_Q6SS_Q6_CAMSS_DSP_STREAMING_1_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_MPU_CFG_AON_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_Q6_AHBM_MPU_AON_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_Q6SS_AHBS_AON_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_QOS_AHBS_AON_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_QOS_XO_LAT_COUNTER_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_QOS_DMONITOR_FIXED_LAT_COUNTER_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_QOS_DANGER_FIXED_LAT_COUNTER_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_AUDIO_WRAPPER_BUS_TIMEOUT_AON_CBCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_cc_base_+LPASS_DBG_CLK_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_tcsr_base_+LPASS_AUDIO_CORE_LPAIF_PRI_MODE_MUXSEL_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_tcsr_base_+LPASS_AUDIO_CORE_LPAIF_SEC_MODE_MUXSEL_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_tcsr_base_+LPASS_AUDIO_CORE_LPAIF_TER_MODE_MUXSEL_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_tcsr_base_+LPASS_AUDIO_CORE_LPAIF_QUAD_MODE_MUXSEL_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_tcsr_base_+LPASS_AUDIO_WRAPPER_LCC_CSR_AON_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_tcsr_base_+LPASS_AUDIO_WRAPPER_BUS_TIMEOUT_AON_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_tcsr_base_+LPASS_AUDIO_WRAPPER_Q6AHB_AON_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_csr_base_+LPASS_AUDIO_CORE_QOS_CORE_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_csr_base_+LPASS_AUDIO_CORE_AVSYNC_CORE_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_csr_base_+LPASS_AUDIO_CORE_LPAIF_CSR_CORE_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_csr_base_+LPASS_AUDIO_CORE_RESAMPLER_CORE_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_csr_base_+LPASS_AUDIO_CORE_HDMITX_CORE_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_csr_base_+LPASS_AUDIO_CORE_TLB_PRELOAD_CORE_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_csr_base_+LPASS_AUDIO_CORE_ATIMER_CORE_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_csr_base_+LPASS_AUDIO_CORE_SYSNOC_SWAY_CORE_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_csr_base_+LPASS_AUDIO_CORE_QDSP_SWAY_CORE_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_csr_base_+LPASS_AUDIO_CORE_BUS_TIMEOUT_CORE_CGCR_OFFSET, LPASS_CLK_ENABLE}, \
	{_lpass_csr_base_+LPASS_AUDIO_CORE_SYSNOC_MPORT_CORE_CGCR_OFFSET, LPASS_CLK_ENABLE}

#define TIMER_KHZ 32768
#define RDDMA_TIME_THRESHOLD_MS 500
#define EARLY_AUDIO_PARTITION "early-audio"

static unsigned int audio_place_kpi_marker(char *marker_name)
{
	unsigned int marker_value;

	marker_value = readl(MPM2_MPM_SLEEP_TIMETICK_COUNT_VAL);
	dprintf(INFO, "marker name=%s; marker value=%u.%03u seconds\n",
					marker_name, marker_value/TIMER_KHZ,
					(((marker_value % TIMER_KHZ)
					* 1000) / TIMER_KHZ));
	return marker_value;
}

static struct hw_reg_array hw_gcc_clk_ctl_regs[] = {
	GCC_CLK_CTL_INIT(GCC_CLK_CTL_BASE)
};

static struct hw_reg_array hw_lpass_init_regs[] = {
	LPASS_INIT(LPASS_CC_BASE,LPASS_TCSR_BASE,LPASS_CSR_BASE)
};

static lpass_hw_afe_dma_config_t dma_config = {
	.dma_int_reg_addr = LPASS_LPAIF_IRQ_ENa(LPASS_HW_LPAIF_IRQ_OUTPUT_APPS),
	.dma_int_virt_addr = LPASS_LPAIF_IRQ_ENa(LPASS_HW_LPAIF_IRQ_OUTPUT_APPS),
	.rddma_reg_addr = LPASS_LPAIF_RDDMA_CTLa(LPASS_HW_RDDMA_CHANNEL_0),
	.rddma_virt_addr = LPASS_LPAIF_RDDMA_CTLa(LPASS_HW_RDDMA_CHANNEL_0),
	.wrdma_reg_addr = LPASS_LPAIF_WRDMA_CTLa(LPASS_HW_WRDMA_CHANNEL_0),
	.wrdma_virt_addr = LPASS_LPAIF_WRDMA_CTLa(LPASS_HW_WRDMA_CHANNEL_0),
	.stc_rddma_reg_addr = LPASS_LPAIF_RDDMA_STC_LSBa(LPASS_HW_RDDMA_CHANNEL_0),
	.stc_rddma_virt_addr = LPASS_LPAIF_RDDMA_STC_LSBa(LPASS_HW_RDDMA_CHANNEL_0),
	.stc_wrdma_reg_addr = LPASS_LPAIF_WRDMA_STC_LSBa(LPASS_HW_WRDMA_CHANNEL_0),
	.stc_wrdma_virt_addr = LPASS_LPAIF_WRDMA_STC_LSBa(LPASS_HW_WRDMA_CHANNEL_0),
	.buffer_start_addr = (uint32_t *) EARLY_AUDIO_MEM_ADDR,
	.buffer_2nd_period_addr = (uint32_t *) (EARLY_AUDIO_MEM_ADDR + (EARLY_AUDIO_MEM_SIZE / 2)),
	.buf_len_in_words = EARLY_AUDIO_MEM_SIZE / sizeof(uint32_t),
	.period_len_in_words = (EARLY_AUDIO_MEM_SIZE / 2) / sizeof(uint32_t),
	.buf_len_in_bytes = EARLY_AUDIO_MEM_SIZE,
	.period_len_in_bytes = (EARLY_AUDIO_MEM_SIZE / 2),
	//wps count is the number of 32 bit words per sample element (sample element = bytes per sample * number of channels)
	.wps_count = (8*4) >> 2,
	.watermark = 8,
	.burst_size = 16,
	.ifconfig_dma_control = LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF__QUA_SRC
};

static lpass_hw_tdm_config_t tdm_config = {
	.tdm_phys_addr = LPASS_LPAIF_BASE,
	.tdm_virt_addr = LPASS_LPAIF_BASE,
	.sync_src = TDM_MSM_MODE_SLAVE,
	.sync_type = TDM_LONG_SYNC_TYPE,
	.num_channels = 8,
	.bit_width = 32,
	.slot_width = 32,
	.nslots_per_frame = 8,
	.sync_invert = TDM_LONG_SYNC_NORMAL,
	.sync_data_delay = TDM_DATA_DELAY_0_CYCLE,
	.ctrl_data_oe = TDM_CTRL_DATA_OE_DISABLE,
	.slot_mask = 0xFF
};

static early_audio_mmc_partition_t partition = {
	.index = 0,
	.offset = 0,
	.blocksize = 0,
	.pcm_header = {
		.tdm_port = TDM_QUATERNARY,
		.sampling_rate = 48000,
		.bit_width = 16,
		.num_channels = 0,
		.pcm_size = 0,
		.reserved = {0}
		},
	.partition_ptr = NULL,
	.num_periods = 0,
	.dma_prog_cnt = 0,
	.bytes_to_be_read = 0,
	.bytes_written = 0,
	.bytes_in_dma = 0
};

static bool early_audio_enabled = FALSE;
static bool is_partition_fully_read = FALSE;
static uint32_t last_checked_dma_addr = 0;

static void lpass_read_then_set_reg(uint32_t phys_addr, uint32_t virt_offset, uint32_t mask, uint32_t value)
{
	uint32_t read_val;
	volatile uint32_t* addr;

	addr = (volatile uint32_t *) phys_addr;
	read_val = lpass_io_r(addr+virt_offset);
	mask = ~mask;
	read_val = read_val & mask;
	read_val = read_val | value;

	lpass_io_w(read_val, (addr+virt_offset));
}

static void lpass_read_reg(uint32_t phys_addr, uint32_t virt_offset, uint32 *ret_value)
{
	volatile uint32_t* addr;

	addr = (volatile uint32_t *) phys_addr;
	*ret_value = lpass_io_r(addr+virt_offset);
}

static void early_audio_reg_init(struct hw_reg_array *data,int size)
{
	int i = 0;

	for (i = 0; i < size; i++) {
		lpass_io_w(data[i].reg_data, data[i].reg_addr);
	}
}

static void early_audio_init_lpass_hw(void)
{
	int i = 0;
	int array_size = 0;

	dprintf(CRITICAL, "early_audio_start: setup LPASS hw\n");

	array_size = sizeof(hw_lpass_init_regs) / sizeof(hw_lpass_init_regs[0]);
	for (i = 0; i < array_size; i++) {
		lpass_io_w(hw_lpass_init_regs[i].reg_data, hw_lpass_init_regs[i].reg_addr);
		dprintf(CRITICAL, "early_audio_start: writing %d to 0x%08x\n",
			hw_lpass_init_regs[i].reg_data, hw_lpass_init_regs[i].reg_addr);
	}
}

void early_audio_tdm_gpio_config(uint32_t tdm_idx)
{
	switch(tdm_idx)
	{
		case TDM_PRIMARY:
			gpio_tlmm_config(GPIO_PIN_PRI_MI2S_SCK, GPIO_MI2S_FUNC, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_ENABLE);
			gpio_tlmm_config(GPIO_PIN_PRI_MI2S_WS, GPIO_MI2S_FUNC, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_ENABLE);
			gpio_tlmm_config(GPIO_PIN_PRI_MI2S_DATA1, GPIO_MI2S_FUNC, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_ENABLE);
			break;
		case TDM_SECONDARY:
			gpio_tlmm_config(GPIO_PIN_SEC_MI2S_SCK, GPIO_MI2S_FUNC, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_ENABLE);
			gpio_tlmm_config(GPIO_PIN_SEC_MI2S_WS, GPIO_MI2S_FUNC, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_ENABLE);
			gpio_tlmm_config(GPIO_PIN_SEC_MI2S_DATA1, GPIO_MI2S_FUNC, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_ENABLE);
			break;
		case TDM_TERTIARY:
			gpio_tlmm_config(GPIO_PIN_TER_MI2S_SCK, GPIO_MI2S_FUNC, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_ENABLE);
			gpio_tlmm_config(GPIO_PIN_TER_MI2S_WS, GPIO_MI2S_FUNC, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_ENABLE);
			gpio_tlmm_config(GPIO_PIN_TER_MI2S_DATA1, GPIO_MI2S_FUNC, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_ENABLE);
			break;
		case TDM_QUATERNARY:
			gpio_tlmm_config(GPIO_PIN_QUA_MI2S_SCK, GPIO_MI2S_FUNC, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_ENABLE);
			gpio_tlmm_config(GPIO_PIN_QUA_MI2S_WS, GPIO_MI2S_FUNC, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_ENABLE);
			gpio_tlmm_config(GPIO_PIN_QUA_MI2S_DATA1, GPIO_MI2S_FUNC, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_ENABLE);
			break;
		default:
			dprintf(CRITICAL, "early_audio_start: Unrecognized TDM index %d." \
				"GPIO is not set.\n", tdm_idx);
			break;
	}
}

void early_audio_tdm_config(uint32_t tdm_idx, uint32_t dir)
{
	uint32_t mask, val;
	uint32_t reg_addr;
	uint32_t nbits_per_frame;

	dprintf(CRITICAL, "early_audio_start: setup TDM Config\n");

	lpass_read_then_set_reg(LPASS_LPAIF_PCM_I2S_SELa(tdm_idx),
		LPASS_REG_OFFSET(tdm_config.tdm_phys_addr, tdm_config.tdm_virt_addr),
		LPASS_LPAIF_PCM_I2S_SELa__SEL___M,
		LPASS_LPAIF_PCM_I2S_SELa__SEL__PCM_SRC);

	//Reset PCM interface then release
	lpass_read_then_set_reg(LPASS_LPAIF_PCM_CTLa(tdm_idx),
		LPASS_REG_OFFSET(tdm_config.tdm_phys_addr, tdm_config.tdm_virt_addr),
		LPASS_LPAIF_PCM_CTLa__RESET_TX___M,
		LPASS_LPAIF_PCM_CTLa__RESET_TX__ENABLE << LPASS_LPAIF_PCM_CTLa__RESET_TX___S);
	lpass_read_then_set_reg(LPASS_LPAIF_PCM_CTLa(tdm_idx),
		LPASS_REG_OFFSET(tdm_config.tdm_phys_addr, tdm_config.tdm_virt_addr),
		LPASS_LPAIF_PCM_CTLa__RESET_TX___M,
		LPASS_LPAIF_PCM_CTLa__RESET_TX__DISABLE << LPASS_LPAIF_PCM_CTLa__RESET_TX___S);

	//Invert TDM Clock
	switch(tdm_idx)
	{
		case TDM_PRIMARY:
			reg_addr = LPASS_CC_BASE + LPASS_AUDIO_CORE_LPAIF_PRI_CLK_INV_OFFSET;
			break;
		case TDM_SECONDARY:
			reg_addr = LPASS_CC_BASE + LPASS_AUDIO_CORE_LPAIF_SEC_CLK_INV_OFFSET;
			break;
		case TDM_TERTIARY:
			reg_addr = LPASS_CC_BASE + LPASS_AUDIO_CORE_LPAIF_TER_CLK_INV_OFFSET;
			break;
		case TDM_QUATERNARY:
			reg_addr = LPASS_CC_BASE + LPASS_AUDIO_CORE_LPAIF_QUAD_CLK_INV_OFFSET;
			break;
		default:
			dprintf(CRITICAL, "early_audio_start: Unrecognized TDM index %d." \
				"Setting Inverting QUAT_TDM clock by default.\n", tdm_idx);
			reg_addr = LPASS_CC_BASE + LPASS_AUDIO_CORE_LPAIF_QUAD_CLK_INV_OFFSET;
			break;
	}

	lpass_read_then_set_reg(reg_addr,
		LPASS_REG_OFFSET(tdm_config.tdm_phys_addr, tdm_config.tdm_virt_addr),
		LPASS_CLK_ENABLE,
		LPASS_CLK_ENABLE);

	//First Stage: Writing values for LPASS_LPAIF_PCM_CTLa
	val = 0;

	//Setup the mask for sync src, sync type, oe mode
	mask = LPASS_LPAIF_PCM_CTLa__SYNC_SRC___M |
		LPASS_LPAIF_PCM_CTLa__AUX_MODE___M |
		LPASS_LPAIF_PCM_CTLa__CTRL_DATA_OE___M;

	//Setup the sync src: internal or external
	switch (tdm_config.sync_src)
	{
		case TDM_MSM_MODE_MASTER:
			val |= (LPASS_LPAIF_PCM_CTLa__SYNC_SRC__INTERNAL <<
				LPASS_LPAIF_PCM_CTLa__SYNC_SRC___S);
			break;
		case TDM_MSM_MODE_SLAVE:
			val |= (LPASS_LPAIF_PCM_CTLa__SYNC_SRC__EXTERNAL <<
				LPASS_LPAIF_PCM_CTLa__SYNC_SRC___S);
			break;
		default:
			dprintf(CRITICAL, "early_audio_start: Unrecognized TDM sync source %d." \
				"Setting to Slave (External Source) by default.\n", tdm_config.sync_src);
			val |= (LPASS_LPAIF_PCM_CTLa__SYNC_SRC__EXTERNAL <<
				LPASS_LPAIF_PCM_CTLa__SYNC_SRC___S);
		break;
	}

	//Setup the sync type: short or long
	switch(tdm_config.sync_type)
	{
		case TDM_SHORT_SYNC_TYPE:
			val |= (LPASS_LPAIF_PCM_CTLa__AUX_MODE__PCM <<
				LPASS_LPAIF_PCM_CTLa__AUX_MODE___S);
			break;
		case TDM_LONG_SYNC_TYPE:
			val |= (LPASS_LPAIF_PCM_CTLa__AUX_MODE__AUX <<
				LPASS_LPAIF_PCM_CTLa__AUX_MODE___S);
			break;
		case TDM_SLOT_SYNC_TYPE:
			val |= (LPASS_LPAIF_PCM_CTLa__AUX_MODE__PCM <<
				LPASS_LPAIF_PCM_CTLa__AUX_MODE___S);
			val |= (LPASS_LPAIF_PCM_CTLa__ONE_SLOT_SYNC_EN__ENABLE <<
				LPASS_LPAIF_PCM_CTLa__ONE_SLOT_SYNC_EN___S);
			break;
		default:
			dprintf(CRITICAL, "early_audio_start: Unrecognized TDM sync type %d." \
				"Setting to Long by default.\n", tdm_config.sync_type);
			val |= (LPASS_LPAIF_PCM_CTLa__AUX_MODE__AUX <<
				LPASS_LPAIF_PCM_CTLa__AUX_MODE___S);
			break;
	}

	//Setup the OE mode
	switch (tdm_config.ctrl_data_oe)
	{
		case TDM_CTRL_DATA_OE_DISABLE:
			val |= (TDM_CTRL_DATA_OE_DISABLE << LPASS_LPAIF_PCM_CTLa__CTRL_DATA_OE___S);
			break;
		case TDM_CTRL_DATA_OE_ENABLE:
			val |= (TDM_CTRL_DATA_OE_ENABLE << LPASS_LPAIF_PCM_CTLa__CTRL_DATA_OE___S);
			break;
		default:
			dprintf(CRITICAL, "early_audio_start: Unrecognized OE %d." \
				"Setting to Disabled by default.\n", tdm_config.ctrl_data_oe);
			val |= (TDM_CTRL_DATA_OE_DISABLE << LPASS_LPAIF_PCM_CTLa__CTRL_DATA_OE___S);
			break;
	}

	lpass_read_then_set_reg(LPASS_LPAIF_PCM_CTLa(tdm_idx),
		LPASS_REG_OFFSET(tdm_config.tdm_phys_addr, tdm_config.tdm_virt_addr),
		mask,
		val);

	//Second Stage: Writing values for LPASS_LPAIF_PCM_TDM_CTL_a
	//Setup enable TDM, sync-delay, rate, enable diff
	mask = LPASS_LPAIF_PCM_TDM_CTL_a__ENABLE_TDM___M |
		LPASS_LPAIF_PCM_TDM_CTL_a__TDM_SYNC_DELAY___M |
		LPASS_LPAIF_PCM_TDM_CTL_a__TDM_RATE___M |
		LPASS_LPAIF_PCM_TDM_CTL_a__ENABLE_DIFF_SAMPLE_WIDTH___M;

	//Enable TDM mode
	val  = LPASS_LPAIF_PCM_TDM_CTL_a__ENABLE_TDM__ENABLE <<
		LPASS_LPAIF_PCM_TDM_CTL_a__ENABLE_TDM___S;

	//Setup sync delay, delay 1 is the typical use case
	switch(tdm_config.sync_data_delay)
	{
		case TDM_DATA_DELAY_0_CYCLE:
			val |= LPASS_LPAIF_PCM_TDM_CTL_a__TDM_SYNC_DELAY__DELAY_0_CYCLE <<
				LPASS_LPAIF_PCM_TDM_CTL_a__TDM_SYNC_DELAY___S;
			break;
		case TDM_DATA_DELAY_1_CYCLE:
			val |= LPASS_LPAIF_PCM_TDM_CTL_a__TDM_SYNC_DELAY__DELAY_1_CYCLE <<
				LPASS_LPAIF_PCM_TDM_CTL_a__TDM_SYNC_DELAY___S;
			break;
		case TDM_DATA_DELAY_2_CYCLE:
			val |= LPASS_LPAIF_PCM_TDM_CTL_a__TDM_SYNC_DELAY__DELAY_2_CYCLE <<
				LPASS_LPAIF_PCM_TDM_CTL_a__TDM_SYNC_DELAY___S;
			break;
		default:
			dprintf(CRITICAL, "early_audio_start: Unrecognized Sync Data Delay %d." \
				"Setting to 0 Cycle delay by default.\n", tdm_config.sync_data_delay);
			val |= LPASS_LPAIF_PCM_TDM_CTL_a__TDM_SYNC_DELAY__DELAY_0_CYCLE <<
				LPASS_LPAIF_PCM_TDM_CTL_a__TDM_SYNC_DELAY___S;
			break;
	}

	//Setup the rate: slot x slot-width
	nbits_per_frame = tdm_config.nslots_per_frame * tdm_config.slot_width;

	val |= (nbits_per_frame - 1) << LPASS_LPAIF_PCM_TDM_CTL_a__TDM_RATE___S;

	//Based on slot-width, enable or disable DIFF support
	if(tdm_config.bit_width != tdm_config.slot_width)
	{
		val |= LPASS_LPAIF_PCM_TDM_CTL_a__ENABLE_DIFF_SAMPLE_WIDTH__ENABLE <<
			LPASS_LPAIF_PCM_TDM_CTL_a__ENABLE_DIFF_SAMPLE_WIDTH___S;
	}
	else
	{
		val |= LPASS_LPAIF_PCM_TDM_CTL_a__ENABLE_DIFF_SAMPLE_WIDTH__DISABLE <<
			LPASS_LPAIF_PCM_TDM_CTL_a__ENABLE_DIFF_SAMPLE_WIDTH___S;
	}

	//Invert sync or not only for LONG SYNC mode
	//Setup the width regardless of DIFF mode.
	if (TDM_SINK == dir)
	{
		mask |= LPASS_LPAIF_PCM_TDM_CTL_a__TDM_INV_TPCM_SYNC___M;
		val |= tdm_config.sync_invert <<
			LPASS_LPAIF_PCM_TDM_CTL_a__TDM_INV_TPCM_SYNC___S;

		mask |= LPASS_LPAIF_PCM_TDM_CTL_a__TDM_TPCM_WIDTH___M;
		val |= (tdm_config.bit_width - 1) <<
			LPASS_LPAIF_PCM_TDM_CTL_a__TDM_TPCM_WIDTH___S;
	}
	else
	{
	mask |= LPASS_LPAIF_PCM_TDM_CTL_a__TDM_INV_RPCM_SYNC___M;
	val |= tdm_config.sync_invert <<
		LPASS_LPAIF_PCM_TDM_CTL_a__TDM_INV_RPCM_SYNC___S;

	mask |= LPASS_LPAIF_PCM_TDM_CTL_a__TDM_RPCM_WIDTH___M;
	val |= (tdm_config.bit_width - 1) <<
		LPASS_LPAIF_PCM_TDM_CTL_a__TDM_RPCM_WIDTH___S;
	}

	lpass_read_then_set_reg(LPASS_LPAIF_PCM_TDM_CTL_a(tdm_idx),
		LPASS_REG_OFFSET(tdm_config.tdm_phys_addr, tdm_config.tdm_virt_addr), mask, val);

	// Third Stage: Setup LPASS_LPAIF_PCM_TDM_SAMPLE_WIDTH_a if slot and bit widths differ
	if(tdm_config.bit_width != tdm_config.slot_width)
	{
		if(TDM_SINK == dir)
		{
			mask = LPASS_LPAIF_PCM_TDM_SAMPLE_WIDTH_a__TDM_TPCM_SAMPLE_WIDTH___M;
			val = (tdm_config.slot_width - 1) <<
				LPASS_LPAIF_PCM_TDM_SAMPLE_WIDTH_a__TDM_TPCM_SAMPLE_WIDTH___S;
		}
		else
		{
			mask = LPASS_LPAIF_PCM_TDM_SAMPLE_WIDTH_a__TDM_RPCM_SAMPLE_WIDTH___M;
			val = (tdm_config.slot_width - 1) <<
				LPASS_LPAIF_PCM_TDM_SAMPLE_WIDTH_a__TDM_RPCM_SAMPLE_WIDTH___S;
		}
		lpass_read_then_set_reg(LPASS_LPAIF_PCM_TDM_SAMPLE_WIDTH_a(tdm_idx),
			LPASS_REG_OFFSET(tdm_config.tdm_phys_addr, tdm_config.tdm_virt_addr),
			mask,
			val);
	}

	// Fourth Stage: slot mask, LPASS_LPAIF_PCM_TPCM_SLOT_NUM_a or LPASS_LPAIF_PCM_RPCM_SLOT_NUM_a
	if(TDM_SINK == dir)
	{
		reg_addr = LPASS_LPAIF_PCM_TPCM_SLOT_NUM_a(tdm_idx);
		mask = LPASS_LPAIF_PCM_TPCM_SLOT_NUM_a___M;
	}
	else
	{
		reg_addr = LPASS_LPAIF_PCM_RPCM_SLOT_NUM_a(tdm_idx);
		mask = LPASS_LPAIF_PCM_RPCM_SLOT_NUM_a___M;
	}
		val = tdm_config.slot_mask;
		lpass_read_then_set_reg(reg_addr,
			LPASS_REG_OFFSET(tdm_config.tdm_phys_addr,tdm_config.tdm_virt_addr),
			mask,
			val);
	// End Fourth Phase: LPASS_LPAIF_PCM_TPCM_SLOT_NUM_a or LPASS_LPAIF_PCM_RPCM_SLOT_NUM_a
}

void early_audio_tdm_enable(uint32_t tdm_idx, uint32_t dir)
{
	uint32_t mask, val;

	dprintf(CRITICAL, "early_audio_start: enable TDM port\n");

	if(TDM_SINK == dir)
	{
		//enable transmit path
		val = LPASS_LPAIF_PCM_CTLa__ENABLE_TX__ENABLE << LPASS_LPAIF_PCM_CTLa__ENABLE_TX___S;
		mask = LPASS_LPAIF_PCM_CTLa__ENABLE_TX___M;
	}
	else
	{
		//enable receive path
		val = LPASS_LPAIF_PCM_CTLa__ENABLE_RX__ENABLE << LPASS_LPAIF_PCM_CTLa__ENABLE_RX___S;
		mask = LPASS_LPAIF_PCM_CTLa__ENABLE_RX___M;
	}

	lpass_read_then_set_reg(LPASS_LPAIF_PCM_CTLa(tdm_idx),
		LPASS_REG_OFFSET(tdm_config.tdm_phys_addr, tdm_config.tdm_virt_addr), mask, val);
}

void early_audio_tdm_disable(uint32_t tdm_idx, uint32_t dir)
{
	uint32_t mask, val;

	dprintf(CRITICAL, "early_audio_start: disable TDM port\n");

	if(TDM_SINK == dir)
	{
		//disable transmit path
		val = LPASS_LPAIF_PCM_CTLa__ENABLE_TX__DISABLE << LPASS_LPAIF_PCM_CTLa__ENABLE_TX___S;
		mask = LPASS_LPAIF_PCM_CTLa__ENABLE_TX___M;
	}
	else
	{
		//disable receive path
		val = LPASS_LPAIF_PCM_CTLa__ENABLE_RX__DISABLE << LPASS_LPAIF_PCM_CTLa__ENABLE_RX___S;
		mask = LPASS_LPAIF_PCM_CTLa__ENABLE_RX___M;
	}

	lpass_read_then_set_reg(LPASS_LPAIF_PCM_CTLa(tdm_idx),
		LPASS_REG_OFFSET(tdm_config.tdm_phys_addr,tdm_config.tdm_virt_addr),
		mask,
		val);
}

void early_audio_dma_disable_channel(uint32_t dma_dir, uint32_t dma_idx)
{
	uint32_t mask, val;

	dprintf(CRITICAL, "early_audio_start: disable DMA channel\n");

	if(LPASS_HW_DMA_SINK == dma_dir)
	{
		//disable dynamic clock enable.
		mask = LPASS_LPAIF_RDDMA_CTLa__DYNAMIC_CLOCK___M;
		val = LPASS_LPAIF_RDDMA_CTLa__DYNAMIC_CLOCK__OFF << LPASS_LPAIF_RDDMA_CTLa__DYNAMIC_CLOCK___S;
		lpass_read_then_set_reg(LPASS_LPAIF_RDDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.rddma_virt_addr, dma_config.rddma_reg_addr),
			mask,
			val);

		mask = LPASS_LPAIF_RDDMA_CTLa__ENABLE___M | LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF___M;
		val = (LPASS_LPAIF_RDDMA_CTLa__ENABLE__OFF << LPASS_LPAIF_RDDMA_CTLa__ENABLE___S)
			| (LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF__NONE << LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF___S);
		lpass_read_then_set_reg(LPASS_LPAIF_RDDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.rddma_virt_addr, dma_config.rddma_reg_addr),
			mask,
			val);
	}
	else
	{
		//disable dynamic clock enable.
		mask = LPASS_LPAIF_WRDMA_CTLa__DYNAMIC_CLOCK___M;
		val = LPASS_LPAIF_WRDMA_CTLa__DYNAMIC_CLOCK__OFF << LPASS_LPAIF_WRDMA_CTLa__DYNAMIC_CLOCK___S;
		lpass_read_then_set_reg(LPASS_LPAIF_WRDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.wrdma_reg_addr, dma_config.wrdma_virt_addr),
			mask,
			val);

		mask = LPASS_LPAIF_WRDMA_CTLa__ENABLE___M | LPASS_LPAIF_WRDMA_CTLa__AUDIO_INTF___M;
		val = (LPASS_LPAIF_WRDMA_CTLa__ENABLE__OFF << LPASS_LPAIF_WRDMA_CTLa__ENABLE___S)
			| (LPASS_LPAIF_WRDMA_CTLa__AUDIO_INTF__NONE << LPASS_LPAIF_WRDMA_CTLa__AUDIO_INTF___S);
		lpass_read_then_set_reg(LPASS_LPAIF_WRDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.wrdma_reg_addr, dma_config.wrdma_virt_addr),
			mask,
			val);
	}
}

void early_audio_dma_config_channel(uint32_t dma_dir, uint32_t dma_idx)
{
	uint32_t mask, val;

	dprintf(CRITICAL, "early_audio_start: config DMA channel\n");

	// Update the base address of the DMA buffer
	mask = LPASS_LPAIF_RDDMA_BASEa__BASE_ADDR___M;
	val = ((uint32)dma_config.buffer_start_addr >> LPASS_LPAIF_RDDMA_BASEa__BASE_ADDR___S) <<
		LPASS_LPAIF_RDDMA_BASEa__BASE_ADDR___S;

	dprintf(CRITICAL, "early_audio_start: DMA Buffer start Address 0x%08x\n",
		(uint32)dma_config.buffer_start_addr);

	if(LPASS_HW_DMA_SINK == dma_dir)
	{
		lpass_read_then_set_reg(LPASS_LPAIF_RDDMA_BASEa(dma_idx),
		LPASS_REG_OFFSET(dma_config.rddma_virt_addr, dma_config.rddma_reg_addr),
		mask,
		val);
	}
	else
	{
		lpass_read_then_set_reg(LPASS_LPAIF_WRDMA_BASEa(dma_idx),
		LPASS_REG_OFFSET(dma_config.wrdma_reg_addr, dma_config.wrdma_virt_addr),
		mask,
		val);
	}

	// Update the DMA buffer length
	mask = LPASS_LPAIF_RDDMA_BUFF_LENa__LENGTH___M;
	val = (dma_config.buf_len_in_words-1);
	dprintf(CRITICAL, "early_audio_start: DMA Buffer length in words is %d\n", val);

	if(LPASS_HW_DMA_SINK == dma_dir)
	{
		lpass_read_then_set_reg(LPASS_LPAIF_RDDMA_BUFF_LENa(dma_idx),
		LPASS_REG_OFFSET(dma_config.rddma_virt_addr, dma_config.rddma_reg_addr),
		mask,
		val);
	}
	else
	{
		lpass_read_then_set_reg(LPASS_LPAIF_WRDMA_BUFF_LENa(dma_idx),
		LPASS_REG_OFFSET(dma_config.wrdma_reg_addr, dma_config.wrdma_virt_addr),
		mask,
		val);
	}

	// Update the interrupt sample period
	mask = LPASS_LPAIF_RDDMA_PER_LENa___M;
	val = (dma_config.period_len_in_words-1) << LPASS_LPAIF_RDDMA_PER_LENa___S;

	if(LPASS_HW_DMA_SINK == dma_dir)
	{
		lpass_read_then_set_reg(LPASS_LPAIF_RDDMA_PER_LENa(dma_idx),
			LPASS_REG_OFFSET(dma_config.rddma_virt_addr, dma_config.rddma_reg_addr),
			mask,
			val);
	}
	else
	{
		lpass_read_then_set_reg(LPASS_LPAIF_WRDMA_PER_LENa(dma_idx),
			LPASS_REG_OFFSET(dma_config.wrdma_reg_addr, dma_config.wrdma_virt_addr),
			mask,
			val);
	}

	/**
	* Update the audio interface and fifo Watermark in AUDIO_DMA_CTLa(channel)
	* register. Obtain these values from the input configuration structure
	*/
	mask = LPASS_LPAIF_RDDMA_CTLa__BURST_EN___M |
		LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF___M |
		LPASS_LPAIF_RDDMA_CTLa__FIFO_WATERMRK___M |
		LPASS_LPAIF_RDDMA_CTLa__WPSCNT___M;

	val = (dma_config.ifconfig_dma_control) << LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF___S;

	val |=((dma_config.watermark-1) << LPASS_LPAIF_RDDMA_CTLa__FIFO_WATERMRK___S);

	/**
	* Check if burst mode can be enabled
	* if per length is not bigger than fifo length or per buf size is not aligned to 16 bytes,
	* then disable the burst mode and use watermark as 1 dword
	* 8k mono case. The intSamplePeriod will be 8 * 1 / 2 = 4 DWs
	* The burst mode needs to be disabled for this case for latency and drift detection issue
	*/
	if((LPASS_LPAIF_RDDMA_CTLa__FIFO_WATERMRK__ENUM_8+1) < dma_config.period_len_in_words)
	{
		//enable BURST basing on burst_size
		switch(dma_config.burst_size)
		{
			case 4:
			case 8:
			case 16:
				val |= (LPASS_LPAIF_RDDMA_CTLa__BURST_EN__INCR4 << LPASS_LPAIF_RDDMA_CTLa__BURST_EN___S);
				break;
			case 1:
				val |= (LPASS_LPAIF_RDDMA_CTLa__BURST_EN__SINGLE << LPASS_LPAIF_RDDMA_CTLa__BURST_EN___S);
				break;
			default:
				break;
		}
	}
	else
	{
		val |= (LPASS_LPAIF_RDDMA_CTLa__BURST_EN__SINGLE << LPASS_LPAIF_RDDMA_CTLa__BURST_EN___S);
	}

	val |= ((dma_config.wps_count - 1) << LPASS_LPAIF_RDDMA_CTLa__WPSCNT___S);

	if(LPASS_HW_DMA_SINK == dma_dir)
	{
		lpass_read_then_set_reg(LPASS_LPAIF_RDDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.rddma_virt_addr, dma_config.rddma_reg_addr),
			mask,
			val);
	}
	else
	{
		lpass_read_then_set_reg(LPASS_LPAIF_WRDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.wrdma_reg_addr, dma_config.wrdma_virt_addr),
			mask,
			val);
	}
}

void early_audio_enable_dma_channel(uint32_t dma_dir, uint32_t dma_idx)
{
	uint32_t mask, val;

	dprintf(CRITICAL, "early_audio_start: enable DMA channel\n");

	if(LPASS_HW_DMA_SINK == dma_dir)
	{
		mask = LPASS_LPAIF_RDDMA_CTLa__ENABLE___M;
		val = LPASS_LPAIF_RDDMA_CTLa__ENABLE__ON << LPASS_LPAIF_RDDMA_CTLa__ENABLE___S;
		lpass_read_then_set_reg(LPASS_LPAIF_RDDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.rddma_virt_addr, dma_config.rddma_reg_addr),
			mask,
			val);

		//enable dynamic clock control.
		mask = LPASS_LPAIF_RDDMA_CTLa__DYNAMIC_CLOCK___M;
		val = LPASS_LPAIF_RDDMA_CTLa__DYNAMIC_CLOCK__ON << LPASS_LPAIF_RDDMA_CTLa__DYNAMIC_CLOCK___S;
		lpass_read_then_set_reg(LPASS_LPAIF_RDDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.rddma_virt_addr, dma_config.rddma_reg_addr),
			mask,
			val);
	}
	else
	{
		mask = LPASS_LPAIF_WRDMA_CTLa__ENABLE___M;
		val = LPASS_LPAIF_WRDMA_CTLa__ENABLE__ON << LPASS_LPAIF_WRDMA_CTLa__ENABLE___S;
		lpass_read_then_set_reg(LPASS_LPAIF_WRDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.wrdma_reg_addr, dma_config.wrdma_virt_addr),
			mask,
			val);

		//enable dynamic clock control.
		mask = LPASS_LPAIF_WRDMA_CTLa__DYNAMIC_CLOCK___M;
		val = LPASS_LPAIF_WRDMA_CTLa__DYNAMIC_CLOCK__ON << LPASS_LPAIF_WRDMA_CTLa__DYNAMIC_CLOCK___S;
		lpass_read_then_set_reg(LPASS_LPAIF_WRDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.wrdma_reg_addr, dma_config.wrdma_virt_addr),
			mask,
			val);
	}
}

void early_audio_disable_dma_channel(uint32_t dma_dir, uint32_t dma_idx)
{
	uint32_t mask, val;

	dprintf(CRITICAL, "early_audio_start: disable DMA channel\n");

	if(LPASS_HW_DMA_SINK == dma_dir)
	{
		//disable dynamic clock enable.
		mask = LPASS_LPAIF_RDDMA_CTLa__DYNAMIC_CLOCK___M;
		val = LPASS_LPAIF_RDDMA_CTLa__DYNAMIC_CLOCK__OFF << LPASS_LPAIF_RDDMA_CTLa__DYNAMIC_CLOCK___S;
		lpass_read_then_set_reg(LPASS_LPAIF_RDDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.rddma_virt_addr, dma_config.rddma_reg_addr),
			mask,
			val);

		mask = LPASS_LPAIF_RDDMA_CTLa__ENABLE___M | LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF___M;
		val = (LPASS_LPAIF_RDDMA_CTLa__ENABLE__OFF << LPASS_LPAIF_RDDMA_CTLa__ENABLE___S)
			| (LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF__NONE << LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF___S);

		lpass_read_then_set_reg(LPASS_LPAIF_RDDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.rddma_virt_addr, dma_config.rddma_reg_addr),
			mask,
			val);
	}
	else
	{
		//disable dynamic clock enable.
		mask = LPASS_LPAIF_WRDMA_CTLa__DYNAMIC_CLOCK___M;
		val = LPASS_LPAIF_WRDMA_CTLa__DYNAMIC_CLOCK__OFF << LPASS_LPAIF_WRDMA_CTLa__DYNAMIC_CLOCK___S;
		lpass_read_then_set_reg(LPASS_LPAIF_WRDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.wrdma_reg_addr, dma_config.wrdma_virt_addr),
			mask,
			val);

		mask = LPASS_LPAIF_WRDMA_CTLa__ENABLE___M | LPASS_LPAIF_WRDMA_CTLa__AUDIO_INTF___M;
		val = (LPASS_LPAIF_WRDMA_CTLa__ENABLE__OFF << LPASS_LPAIF_WRDMA_CTLa__ENABLE___S)
			| (LPASS_LPAIF_WRDMA_CTLa__AUDIO_INTF__NONE << LPASS_LPAIF_WRDMA_CTLa__AUDIO_INTF___S);
		lpass_read_then_set_reg(LPASS_LPAIF_WRDMA_CTLa(dma_idx),
			LPASS_REG_OFFSET(dma_config.wrdma_reg_addr, dma_config.wrdma_virt_addr),
			mask,
			val);
	}
}

int early_audio_check_rddma_buffer(uint32_t check_addr, uint32_t rddma_idx)
{
	uint32_t curr_addr = 0;

	lpass_read_reg(LPASS_LPAIF_RDDMA_CURR_ADDRa(rddma_idx),
		LPASS_REG_OFFSET(dma_config.dma_int_virt_addr, dma_config.dma_int_reg_addr),
		&curr_addr);

	if (last_checked_dma_addr > curr_addr)
	{
		partition.bytes_in_dma -=
			(uint32_t) dma_config.buffer_start_addr + EARLY_AUDIO_MEM_SIZE - last_checked_dma_addr;
		partition.bytes_in_dma -= curr_addr - (uint32_t) dma_config.buffer_start_addr;
	}
	else
	{
		partition.bytes_in_dma -= curr_addr - last_checked_dma_addr;
	}

	last_checked_dma_addr = curr_addr;

	if (partition.bytes_in_dma >= (int32_t) dma_config.period_len_in_bytes)
		return 0;
	else
	{
		if (is_partition_fully_read)
		{
			if (partition.bytes_in_dma > 0)
				return 0;
			else
			{
				memset((void *)check_addr, 0, dma_config.period_len_in_bytes);
				return 1;
			}
		}
		else
		{
			memset((void *)check_addr, 0, dma_config.period_len_in_bytes);
			return 1;
		}
	}
}

void format_pcm_samples_for_dma(uint8_t *in_data, uint32_t **out_data, uint32_t num_samples)
{
	uint32_t i = 0, j = 0;
	uint32_t bytes_per_sample = partition.pcm_header.bit_width / 8;
	uint32_t bit_shift = tdm_config.bit_width - partition.pcm_header.bit_width;

	memset((void *)out_data[0], 0, dma_config.period_len_in_bytes);

	for (i = 0; i < num_samples; i++)
	{
		for (j = 0; j < tdm_config.num_channels; j++)
		{
			memcpy((void*) &out_data[0][(tdm_config.num_channels*i) + j],
				(void*) (in_data) + (((partition.pcm_header.num_channels*i) +
				(j % partition.pcm_header.num_channels)) * bytes_per_sample),
				bytes_per_sample);

			out_data[0][(tdm_config.num_channels*i) + j] =
				out_data[0][(tdm_config.num_channels*i) + j] << bit_shift;
		}
	}
}

int early_audio_get_partition(void)
{
	int ret = 0;

	partition.index = partition_get_index(EARLY_AUDIO_PARTITION);
	if (partition.index == 0) {
		dprintf(CRITICAL, "ERROR: early-audio partition table not found\n");
		ret = -1;
		goto end;
	}

	partition.offset = partition_get_offset(partition.index);
	if (partition.offset == 0) {
		dprintf(CRITICAL, "ERROR: splash Partition invalid\n");
		ret = -1;
		goto end;
	}

	mmc_set_lun(partition_get_lun(partition.index));

	partition.blocksize = mmc_get_device_blocksize();
	if (partition.blocksize == 0) {
		dprintf(CRITICAL, "ERROR:splash Partition invalid blocksize\n");
		ret = -1;
		goto end;
	}
	dprintf(CRITICAL, "early-audio: early-audio blocksize is %d\n", partition.blocksize);

	if (mmc_read(partition.offset, (uint32_t*) &partition.pcm_header, partition.blocksize)) {
		dprintf(CRITICAL, "ERROR: Cannot read PCM header from partition 1\n");
		ret = -1;
		goto end;
	}

	switch (partition.pcm_header.tdm_port)
	{
		case TDM_PRIMARY:
			dprintf(CRITICAL, "PCM Header: TDM Port set to PRI_TDM\n");
			dma_config.ifconfig_dma_control = LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF__PRI_SRC;
			break;
		case TDM_SECONDARY:
			dprintf(CRITICAL, "PCM Header: TDM Port set to SEC_TDM\n");
			dma_config.ifconfig_dma_control = LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF__SEC_SRC;
			break;
		case TDM_TERTIARY:
			dprintf(CRITICAL, "PCM Header: TDM Port set to TERT_TDM\n");
			dma_config.ifconfig_dma_control = LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF__TER_SRC;
			break;
		case TDM_QUATERNARY:
			dprintf(CRITICAL, "PCM Header: TDM Port set to QUAT_TDM\n");
			dma_config.ifconfig_dma_control = LPASS_LPAIF_RDDMA_CTLa__AUDIO_INTF__QUA_SRC;
			break;
		default:
			dprintf(CRITICAL, "ERROR in PCM Header: TDM Port %d undefined\n",
				partition.pcm_header.tdm_port);
			ret = -1;
			goto end;
	}

	switch (partition.pcm_header.sampling_rate)
	{
		case 48000:
			dprintf(CRITICAL, "PCM Header: PCM Sampling Rate %d\n",
				partition.pcm_header.sampling_rate);
			break;
		default:
			dprintf(CRITICAL, "ERROR in PCM Header: PCM sampling rate %d is unsupported\n",
				partition.pcm_header.sampling_rate);
			ret = -1;
			goto end;
	}

	switch (partition.pcm_header.bit_width)
	{
		case 16:
		case 24:
		case 32:
			dprintf(CRITICAL, "PCM Header: PCM bit width is %d\n",
				partition.pcm_header.bit_width);
			break;
		default:
			dprintf(CRITICAL, "ERROR in PCM Header: PCM bit width %d is unsupported\n",
				partition.pcm_header.bit_width);
			ret = -1;
			goto end;
	}

	switch (partition.pcm_header.num_channels)
	{
		case 1:
		case 2:
			dprintf(CRITICAL, "PCM Header: PCM uses %d channels\n",
				partition.pcm_header.num_channels);
			break;
		default:
			dprintf(CRITICAL, "ERROR in PCM Header: PCM number of channels %d is not supported\n",
				partition.pcm_header.num_channels);
			ret = -1;
			goto end;
	}

	if ((partition.pcm_header.pcm_size & (partition.blocksize - 1)) != 0)
	{
		dprintf(CRITICAL, "ERROR in PCM Header: PCM Size %d unaligned to block size %d\n",
			partition.pcm_header.pcm_size, partition.blocksize);
		ret = -1;
		goto end;
	}
		dprintf(CRITICAL, "PCM Header: Total PCM size is %d\n",
			partition.pcm_header.pcm_size);

	partition.partition_ptr = (uint8_t *)malloc(partition.pcm_header.pcm_size*sizeof(uint8_t));

	if (partition.partition_ptr == NULL) {
		dprintf(CRITICAL, "Cant alloc mem for buffer\n");
		ret = -1;
		goto end;
	}
	if (mmc_read(partition.offset, (uint32_t*) partition.partition_ptr, partition.pcm_header.pcm_size)) {
		dprintf(CRITICAL, "ERROR: Cannot read splash image from partition 1\n");
		ret = -1;
		goto end;
	}

end:
return ret;
}

int early_audio_check_dma_playback(void)
{
	uint32_t num_samples = 0;
	uint32_t pcm_size_in_dma_format = 0;
	uint32_t partition_bytes_per_sample = 0;
	uint32_t remaining_partition_samples = 0;
	uint32_t dma_bytes_per_sample = 0;
	uint32_t rddma_check_addr = 0;

	//Playback completes when the final period is completely played out
	if (partition.dma_prog_cnt > partition.num_periods)
		return EARLY_AUDIO_PLAYBACK_FINISHED;

	partition_bytes_per_sample = partition.pcm_header.bit_width / 8;
	remaining_partition_samples = partition.bytes_to_be_read /
		(partition_bytes_per_sample * partition.pcm_header.num_channels);
	dma_bytes_per_sample = tdm_config.bit_width / 8;

	pcm_size_in_dma_format =
		remaining_partition_samples * dma_bytes_per_sample * tdm_config.num_channels;

	if (pcm_size_in_dma_format < dma_config.period_len_in_bytes)
		num_samples = partition.bytes_to_be_read /
			(partition_bytes_per_sample * partition.pcm_header.num_channels);
	else
		num_samples = dma_config.period_len_in_bytes /
			(tdm_config.num_channels * dma_bytes_per_sample);

	if(partition.dma_prog_cnt % 2)
		rddma_check_addr = (uint32_t) dma_config.buffer_2nd_period_addr;
	else
		rddma_check_addr = (uint32_t) dma_config.buffer_start_addr;

	if (partition.dma_prog_cnt == 0)
	{
		audio_place_kpi_marker("Early Audio Write First Period");
		format_pcm_samples_for_dma(partition.partition_ptr,
			&dma_config.buffer_start_addr,
			num_samples);

		partition.bytes_written += num_samples * partition_bytes_per_sample * partition.pcm_header.num_channels;
		partition.bytes_to_be_read -= num_samples * partition_bytes_per_sample * partition.pcm_header.num_channels;
		partition.bytes_in_dma += num_samples * dma_bytes_per_sample * tdm_config.num_channels;
		partition.dma_prog_cnt++;
		return EARLY_AUDIO_PLAYBACK_NOT_COMPLETE;
	}
	//When the DMA count and periods are the same, the last period was written
	//the previous interation and needs to play out. Playback completes after the last period is played back.
	else if (partition.dma_prog_cnt == partition.num_periods)
		is_partition_fully_read = TRUE;

	if(early_audio_check_rddma_buffer(rddma_check_addr, LPASS_HW_RDDMA_CHANNEL_0))
	{
		audio_place_kpi_marker("Early Audio Write Next Period");
		if(partition.dma_prog_cnt % 2)
		{
			if (!is_partition_fully_read)
				//DMA interrupt cnt is 1, 3, 5
				format_pcm_samples_for_dma((partition.partition_ptr +
					partition.bytes_written),
					&dma_config.buffer_2nd_period_addr,
					num_samples);
		}
		else
		{
			if (!is_partition_fully_read)
				//DMA interrupt cnt is 2, 4, 6
				format_pcm_samples_for_dma((partition.partition_ptr +
					partition.bytes_written),
					&dma_config.buffer_start_addr,
					num_samples);
		}
		if (!is_partition_fully_read)
		{
			partition.bytes_written += num_samples * partition_bytes_per_sample * partition.pcm_header.num_channels;
			partition.bytes_to_be_read -= num_samples * partition_bytes_per_sample * partition.pcm_header.num_channels;
			partition.bytes_in_dma += num_samples * dma_bytes_per_sample * tdm_config.num_channels;
		}

		if (partition.num_periods >= partition.dma_prog_cnt)
		{
			partition.dma_prog_cnt++;
			return EARLY_AUDIO_PLAYBACK_NOT_COMPLETE;
		}
		else
			return EARLY_AUDIO_PLAYBACK_FINISHED;
	}

	return EARLY_AUDIO_PLAYBACK_NOT_COMPLETE;
}

int early_audio_start(void)
{
	int ret = 0;
	uint32_t pcm_size_in_dma_format = 0;
	uint32_t partition_samples = 0;
	uint32_t partition_bytes_per_sample = 0;
	uint32_t tdm_bytes_per_sample = 0;
	audio_place_kpi_marker("Early Audio Start");
	early_audio_reg_init(&hw_gcc_clk_ctl_regs[0],
		sizeof(hw_gcc_clk_ctl_regs) / sizeof(hw_gcc_clk_ctl_regs[0]));

	early_audio_reg_init(&hw_lpass_init_regs[0],
		sizeof(hw_lpass_init_regs) / sizeof(hw_lpass_init_regs[0]));

	ret = early_audio_get_partition();
	if (ret != 0)
	{
		dprintf(CRITICAL, "ERROR: Get Early Audio Partition failure\n");
		goto end;
	}

	//setup
	early_audio_tdm_gpio_config(partition.pcm_header.tdm_port);
	early_audio_tdm_config(partition.pcm_header.tdm_port, TDM_SINK);
	early_audio_dma_disable_channel(LPASS_HW_DMA_SINK, LPASS_HW_RDDMA_CHANNEL_0);
	early_audio_dma_config_channel(LPASS_HW_DMA_SINK, LPASS_HW_RDDMA_CHANNEL_0);
	early_audio_tdm_enable(partition.pcm_header.tdm_port, TDM_SINK);
	early_audio_enable_dma_channel(LPASS_HW_DMA_SINK, LPASS_HW_RDDMA_CHANNEL_0);

	memset(dma_config.buffer_start_addr, 0, EARLY_AUDIO_MEM_SIZE);
	last_checked_dma_addr = (uint32_t) dma_config.buffer_start_addr;
	partition.bytes_to_be_read = partition.pcm_header.pcm_size;
	partition_bytes_per_sample = partition.pcm_header.bit_width / 8;
	partition_samples = partition.pcm_header.pcm_size /
		(partition_bytes_per_sample * partition.pcm_header.num_channels);

	tdm_bytes_per_sample = tdm_config.bit_width / 8;
	pcm_size_in_dma_format = partition_samples * tdm_bytes_per_sample * tdm_config.num_channels;

	if (pcm_size_in_dma_format < dma_config.period_len_in_bytes)
		partition.num_periods = 1;
	else
		//Take the ceiling of data size to find period count
		partition.num_periods = (pcm_size_in_dma_format + dma_config.period_len_in_bytes - 1) / dma_config.period_len_in_bytes;

end:
	return ret;
}

int early_audio_end(void)
{
	early_audio_disable_dma_channel(LPASS_HW_DMA_SINK, LPASS_HW_RDDMA_CHANNEL_0);
	early_audio_tdm_disable(partition.pcm_header.tdm_port, TDM_SINK);
	free(partition.partition_ptr);
	audio_place_kpi_marker("Early Audio End");

	return 0;
}

int early_audio_init(void)
{
	int rc = 0;

	if (early_audio_enabled) {
		rc  = early_audio_start();
	}
	else {
		rc =-1;
	}

	return rc;
}

int early_audio_get_sleep_time_ms(void)
{
	return (RDDMA_TIME_THRESHOLD_MS / 4);
}

/* Sets early audio enabled or disabled */
void set_early_audio_enabled(bool enabled)
{
	early_audio_enabled = enabled;
	dprintf(CRITICAL, "set_early_audio_enabled : %d\n", enabled);
}
