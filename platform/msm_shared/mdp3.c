/* Copyright (c) 2011-2014, 2017, The Linux Foundation. All rights reserved.
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
 *     * Neither the name of The Linux Foundation, Inc. nor the names of its
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
 *
 */
#include <mdp3.h>
#include <debug.h>
#include <reg.h>
#include <msm_panel.h>
#include <err.h>
#include <target/display.h>
#include <platform/timer.h>
#include <platform/iomap.h>
#include <clock.h>

static int mdp_rev;

int mdp_dsi_video_config(struct msm_panel_info *pinfo,
		struct fbcon_config *fb)
{
	unsigned long hsync_period;
	unsigned long vsync_period;
	unsigned long vsync_period_intmd;
	struct lcdc_panel_info *lcdc = NULL;
	int ystride = 3;
	int mdp_rev = mdp_get_revision();
	int display_start_x, display_end_x;
	int display_start_y, display_end_y;
	int active_start_x, active_end_x;
	int active_start_y, active_end_y;
	int xres = pinfo->xres;
	int yres = pinfo->yres;

	if (pinfo == NULL)
		return ERR_INVALID_ARGS;

	lcdc =  &(pinfo->lcdc);
	if (lcdc == NULL)
		return ERR_INVALID_ARGS;

	dprintf(SPEW, "MDP3.0.3 for DSI Video Mode\n");
	xres -= (lcdc->border_left + lcdc->border_right);
	yres -= (lcdc->border_top + lcdc->border_bottom);

	hsync_period = xres + lcdc->h_front_porch + \
			lcdc->h_back_porch + 1;
	vsync_period_intmd = yres + lcdc->v_front_porch + \
				lcdc->v_back_porch + 1;
	if (mdp_rev == MDP_REV_304 || mdp_rev == MDP_REV_305) {
		hsync_period += lcdc->h_pulse_width - 1;
		vsync_period_intmd += lcdc->v_pulse_width - 1;
	}
	hsync_period += lcdc->border_left + lcdc->border_right;
	vsync_period_intmd += lcdc->border_top + lcdc->border_bottom;

	vsync_period = vsync_period_intmd * hsync_period;

	display_start_x = lcdc->h_back_porch + lcdc->h_pulse_width;
	display_end_x = hsync_period - lcdc->h_front_porch - 1;
	display_start_y = (lcdc->v_back_porch + lcdc->v_pulse_width) * hsync_period;
	display_end_y = vsync_period - lcdc->v_front_porch * hsync_period - 1;
	active_start_x = display_start_x + lcdc->border_left;
	active_end_x = display_end_x - lcdc->border_right;
	active_start_y = display_start_y + (lcdc->border_top * hsync_period);
	active_end_y = display_end_y - (lcdc->border_bottom * hsync_period);

	/*Program QOS remapper settings*/
	writel(0x1A9, MDP_DMA_P_QOS_REMAPPER);
	writel(0x0, MDP_DMA_P_WATERMARK_0);
	writel(0x0, MDP_DMA_P_WATERMARK_1);
	writel(0x0, MDP_DMA_P_WATERMARK_2);
	if (pinfo->xres >= 720)
		writel(0xFFFF, MDP_PANIC_LUT0);
	else
		writel(0x00FF, MDP_PANIC_LUT0);
	writel(0x1, MDP_PANIC_ROBUST_CTRL);
	writel(0xFF00, MDP_ROBUST_LUT);

	// ------------- programming MDP_DMA_P_CONFIG ---------------------
	writel(0x1800bf, MDP_DMA_P_CONFIG);	// rgb888


	writel_relaxed(active_start_x | active_start_y << 16, MDP_DMA_P_OUT_XY);
	writel_relaxed(yres << 16 | xres, MDP_DMA_P_SIZE);
	writel(MIPI_FB_ADDR, MDP_DMA_P_BUF_ADDR);
	writel(pinfo->xres * ystride, MDP_DMA_P_BUF_Y_STRIDE);
	writel_relaxed(hsync_period << 16 | lcdc->h_pulse_width,
		MDP_DSI_VIDEO_HSYNC_CTL);
	writel(vsync_period, MDP_DSI_VIDEO_VSYNC_PERIOD);
	writel_relaxed(lcdc->v_pulse_width * hsync_period,
		MDP_DSI_VIDEO_VSYNC_PULSE_WIDTH);
	if (mdp_rev == MDP_REV_304 || mdp_rev == MDP_REV_305) {
		writel_relaxed(display_start_x | (display_end_x << 16),
			MDP_DSI_VIDEO_DISPLAY_HCTL);
		writel_relaxed(display_start_y, MDP_DSI_VIDEO_DISPLAY_V_START);
		writel_relaxed(display_end_y, MDP_DSI_VIDEO_DISPLAY_V_END);
		writel_relaxed(1 << 31 | active_start_x | (active_end_x << 16),
			MDP_DSI_VIDEO_ACTIVE_HCTL);
		writel_relaxed(1 << 31 | active_start_y, MDP_DSI_VIDEO_ACTIVE_V_START);
		writel_relaxed(active_end_y, MDP_DSI_VIDEO_ACTIVE_V_END);
	} else {
		writel((pinfo->xres + lcdc->h_back_porch - 1) << 16 | \
			lcdc->h_back_porch, MDP_DSI_VIDEO_DISPLAY_HCTL);
		writel(lcdc->v_back_porch * hsync_period, \
			MDP_DSI_VIDEO_DISPLAY_V_START);
		writel((pinfo->yres + lcdc->v_back_porch) * hsync_period,
			MDP_DSI_VIDEO_DISPLAY_V_END);
	}
	writel_relaxed(lcdc->border_clr, MDP_DSI_VIDEO_BORDER_CLR);
	writel(0x00000000, MDP_DSI_VIDEO_HSYNC_SKEW);
	writel(0x00000000, MDP_DSI_VIDEO_CTL_POLARITY);
	// end of cmd mdp
	return 0;
}

int mdp_dsi_cmd_config(struct msm_panel_info *pinfo,
                struct fbcon_config *fb)
{
	int ret = 0;
	unsigned short pack_pattern = 0x21;
	unsigned char ystride = 3;

	/*Program QOS remapper settings*/
	writel(0x1A9, MDP_DMA_P_QOS_REMAPPER);
	writel(0x0, MDP_DMA_P_WATERMARK_0);
	writel(0x0, MDP_DMA_P_WATERMARK_1);
	writel(0x0, MDP_DMA_P_WATERMARK_2);
	if (pinfo->xres >= 720)
		writel(0xFFFF, MDP_PANIC_LUT0);
	else
		writel(0x00FF, MDP_PANIC_LUT0);
	writel(0x1, MDP_PANIC_ROBUST_CTRL);
	writel(0xFF00, MDP_ROBUST_LUT);

	writel(0x03ffffff, MDP_INTR_ENABLE);

	// ------------- programming MDP_DMA_P_CONFIG ---------------------
	writel(pack_pattern << 8 | 0x3f | (0 << 25)| (1 << 19) | (1 << 7) , MDP_DMA_P_CONFIG);  // rgb888
	writel(0x00000000, MDP_DMA_P_OUT_XY);
	writel(pinfo->yres << 16 | pinfo->xres, MDP_DMA_P_SIZE);
	writel(MIPI_FB_ADDR, MDP_DMA_P_BUF_ADDR);

	writel(pinfo->xres * ystride, MDP_DMA_P_BUF_Y_STRIDE);

	writel(0x10, MDP_DSI_CMD_MODE_ID_MAP);
	writel(0x11, MDP_DSI_CMD_MODE_TRIGGER_EN);
	mdelay(10);

	return ret;
}

void mdp_disable(void)
{
	if (!target_cont_splash_screen())
		writel(0x00000000, MDP_DSI_VIDEO_EN);
}

int mdp_dsi_video_off(void)
{
	if (!target_cont_splash_screen()) {
		mdp_disable();
		mdelay(60);
	}
	writel(0x00000000, MDP_INTR_ENABLE);
	writel(0x01ffffff, MDP_INTR_CLEAR);
	return NO_ERROR;
}

int mdp_dsi_cmd_off(void)
{
	if (!target_cont_splash_screen()) {
		mdp_dma_off();
		/*
		 * Allow sometime for the DMA channel to
		 * stop the data transfer
		 */
		mdelay(10);
	}
	writel(0x00000000, MDP_INTR_ENABLE);
	writel(0x01ffffff, MDP_INTR_CLEAR);
	return NO_ERROR;
}

void mdp_set_revision(int rev)
{
	mdp_rev = rev;
}

int mdp_get_revision(void)
{
	return mdp_rev;
}

int mdp_dsi_video_on(struct msm_panel_info *pinfo)
{
	int ret = 0;

	writel(0x00000001, MDP_DSI_VIDEO_EN);

	return ret;
}

int mdp_dma_on(struct msm_panel_info *pinfo)
{
	int ret = 0;
	mdelay(100);
	writel(0x00000001, MDP_DMA_P_START);

	return ret;
}

int mdp_dma_off()
{
	int ret = 0;

	if (!target_cont_splash_screen())
		writel(0x00000000, MDP_DMA_P_START);

	return ret;
}

int mdp_edp_config(struct msm_panel_info *pinfo, struct fbcon_config *fb)
{
	return NO_ERROR;
}

int mdp_edp_on(struct msm_panel_info *pinfo)
{
	return NO_ERROR;
}

int mdp_edp_off(void)
{
	return NO_ERROR;
}

int mdss_hdmi_config(struct msm_panel_info *pinfo, struct fbcon_config *fb)
{
	return NO_ERROR;
}

int mdss_hdmi_on(void)
{
	return NO_ERROR;
}

int mdss_hdmi_off(void)
{
	return NO_ERROR;
}
