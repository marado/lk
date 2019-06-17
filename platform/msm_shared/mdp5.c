/* Copyright (c) 2012-2016,2018-2019, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of The Linux Foundation nor
 *     the names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <mdp5.h>
#include <mdp5_rm.h>
#include <debug.h>
#include <reg.h>
#include <target/display.h>
#include <platform/timer.h>
#include <platform/iomap.h>
#include <dev/lcdc.h>
#include <dev/fbcon.h>
#include <bits.h>
#include <msm_panel.h>
#include <mipi_dsi.h>
#include <err.h>
#include <clock.h>
#include <scm.h>
#include <target.h>
#include <arch/defines.h>
#if ENABLE_QSEED_SCALAR
#include <target/scalar.h>
#endif

#define MDSS_MDP_MAX_PREFILL_FETCH	25

static uint32_t external_fb_offset = 0;

int restore_secure_cfg(uint32_t id);

static int mdp_rev;

void mdp_set_revision(int rev)
{
	mdp_rev = rev;
}

int mdp_get_revision()
{
	return mdp_rev;
}

#if ENABLE_QSEED_SCALAR
void _mdp_get_scalar_sspp_op_mode(uint32_t *opmode,
	uint32_t orientation, uint32_t pipe_base)
{
	uint32_t op_mode_val = 0;

	op_mode_val = readl(pipe_base + PIPE_SSPP_SRC_OP_MODE);

	op_mode_val &= ~(MDSS_MDP_OP_MODE_FLIP_LR |
					MDSS_MDP_OP_MODE_FLIP_UD |
					MDSS_MDP_OP_MODE_PIX_OVERRIDE);

	if (orientation & (1 << H_FLIP))
		op_mode_val |= MDSS_MDP_OP_MODE_FLIP_LR;
	if (orientation & (1 << V_FLIP))
		op_mode_val |= MDSS_MDP_OP_MODE_FLIP_UD;

	op_mode_val |= MDSS_MDP_OP_MODE_PIX_OVERRIDE;

	*opmode = op_mode_val;
}
#endif

static inline bool is_software_pixel_ext_config_needed()
{
	return (MDSS_IS_MAJOR_MINOR_MATCHING(readl(MDP_HW_REV),
		MDSS_MDP_HW_REV_107) || MDSS_IS_MAJOR_MINOR_MATCHING(readl(MDP_HW_REV),
		MDSS_MDP_HW_REV_114) || MDSS_IS_MAJOR_MINOR_MATCHING(readl(MDP_HW_REV),
		MDSS_MDP_HW_REV_116) || MDSS_IS_MAJOR_MINOR_MATCHING(readl(MDP_HW_REV),
		MDSS_MDP_HW_REV_115));
}

static inline bool has_fixed_size_smp()
{
	return (MDSS_IS_MAJOR_MINOR_MATCHING(readl(MDP_HW_REV),
		MDSS_MDP_HW_REV_107) || MDSS_IS_MAJOR_MINOR_MATCHING(readl(MDP_HW_REV),
		MDSS_MDP_HW_REV_114) || MDSS_IS_MAJOR_MINOR_MATCHING(readl(MDP_HW_REV),
		MDSS_MDP_HW_REV_116) || MDSS_IS_MAJOR_MINOR_MATCHING(readl(MDP_HW_REV),
		MDSS_MDP_HW_REV_115));
}

uint32_t mdss_mdp_intf_offset()
{
	uint32_t mdss_mdp_intf_off;
	uint32_t mdss_mdp_rev = readl(MDP_HW_REV);

	if ((mdss_mdp_rev == MDSS_MDP_HW_REV_106) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_108) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_111) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_112) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_114) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_116) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_115))
		mdss_mdp_intf_off = 0x59100;
	else if (mdss_mdp_rev >= MDSS_MDP_HW_REV_102)
		mdss_mdp_intf_off = 0;
	else
		mdss_mdp_intf_off = 0xEC00;

	return mdss_mdp_intf_off;
}

static uint32_t mdss_mdp_get_ppb_offset()
{
	uint32_t mdss_mdp_ppb_off = 0;
	uint32_t mdss_mdp_rev = readl(MDP_HW_REV);

	/* return MMSS_MDP_PPB0_CONFIG offset from MDSS base */
	if ((mdss_mdp_rev == MDSS_MDP_HW_REV_108) ||
	    (mdss_mdp_rev == MDSS_MDP_HW_REV_111))
		mdss_mdp_ppb_off = 0x1420;
	else if (mdss_mdp_rev == MDSS_MDP_HW_REV_110)
		mdss_mdp_ppb_off = 0x1334;
	else if (MDSS_IS_MAJOR_MINOR_MATCHING(mdss_mdp_rev, MDSS_MDP_HW_REV_107))
		mdss_mdp_ppb_off = 0x1330;
	else
		dprintf(CRITICAL,"Invalid PPB0_CONFIG offset\n");

	return mdss_mdp_ppb_off;
}

static uint32_t mdss_mdp_vbif_qos_remap_get_offset()
{
	uint32_t mdss_mdp_rev = readl(MDP_HW_REV);

	if ((mdss_mdp_rev == MDSS_MDP_HW_REV_110) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_111) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_114) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_115) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_116))
		return 0xB0020;
	else if (MDSS_IS_MAJOR_MINOR_MATCHING(mdss_mdp_rev, MDSS_MDP_HW_REV_107))
		return 0xB0000;
	else
		return 0xC8020;
}

void mdp_clk_gating_ctrl(void)
{
	uint32_t mdss_mdp_rev = readl(MDP_HW_REV);
	if (MDSS_IS_MAJOR_MINOR_MATCHING(mdss_mdp_rev, MDSS_MDP_HW_REV_107))
		return;

	writel(0x40000000, MDP_CLK_CTRL0);
	udelay(20);
	writel(0x40000040, MDP_CLK_CTRL0);
	writel(0x40000000, MDP_CLK_CTRL1);
	writel(0x00400000, MDP_CLK_CTRL3);
	udelay(20);
	writel(0x00404000, MDP_CLK_CTRL3);
	writel(0x40000000, MDP_CLK_CTRL4);
}

static void mdp_rm_stage_offset(uint32_t stage, uint32_t *blend_off, uint32_t *blend_fg_alpha_off)
{
	uint32_t blend_offset = 0, blend_fg_alpha_offset = 0;

	/* border layer is the bottom layer, so for normal layers, stage value should start from 2 */
	switch (stage) {
	case MDP_STAGE_1:
		blend_offset = LAYER_0_BLEND_OP;
		blend_fg_alpha_offset = LAYER_0_BLEND0_FG_ALPHA;
		break;
	case MDP_STAGE_2:
		blend_offset = LAYER_1_BLEND_OP;
		blend_fg_alpha_offset = LAYER_1_BLEND0_FG_ALPHA;
		break;
	case MDP_STAGE_3:
		blend_offset = LAYER_2_BLEND_OP;
		blend_fg_alpha_offset = LAYER_2_BLEND0_FG_ALPHA;
		break;
	case MDP_STAGE_4:
		blend_offset = LAYER_3_BLEND_OP;
		blend_fg_alpha_offset = LAYER_3_BLEND0_FG_ALPHA;
		break;
	case MDP_STAGE_5:
		blend_offset = LAYER_4_BLEND_OP;
		blend_fg_alpha_offset = LAYER_4_BLEND0_FG_ALPHA;
		break;
	case MDP_STAGE_6:
		blend_offset = LAYER_5_BLEND_OP;
		blend_fg_alpha_offset = LAYER_5_BLEND0_FG_ALPHA;
		break;
	default:
		dprintf(CRITICAL, "%s: invalid stage\n", __func__);
		break;
	}

	*blend_off = blend_offset;
	*blend_fg_alpha_off = blend_fg_alpha_offset;
}

static void mdss_layer_mixer_setup_blend_config(uint32_t lm_base,
	uint32_t stage, uint32_t pipe_type)
{
	uint32_t blend_offset, blend_fg_alpha_offset;
	uint32_t blend_op = 0;

	mdp_rm_stage_offset(stage, &blend_offset, &blend_fg_alpha_offset);

	if (pipe_type == MDSS_MDP_PIPE_TYPE_VIG)
		blend_op = VIG_LAYER_ALPHA_BLEND;
	else
		blend_op = RGB_LAYER_ALPHA_BLEND;

	writel(blend_op, lm_base + blend_offset);
	writel(0xFF, lm_base + blend_fg_alpha_offset);
}

static void mdss_mdp_set_layer_transparent(struct msm_panel_info *pinfo,
				uint32_t pipe_base, bool yuv_pipe)
{
	uint32_t img_size, stride, dst_xy, width;
	struct resource_req *rm = NULL;
	uint32_t op_mode = 0;

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
		return;
	}

	if (pinfo->lcdc.dual_pipe)
		width = pinfo->xres / MAX_SPLIT_DISPLAY;
	else
		width = pinfo->xres;

	img_size = (pinfo->yres << 16) | width;
	stride = width * 4;
	dst_xy = (pinfo->yres << 16) | width;

	writel(img_size, pipe_base + PIPE_SSPP_SRC_SIZE);
	writel(0x0, pipe_base + PIPE_SSPP_SRC_XY);
	writel(img_size, pipe_base + PIPE_SSPP_SRC_OUT_SIZE);
	writel(dst_xy, pipe_base + PIPE_SSPP_OUT_XY);
	writel(stride, pipe_base + PIPE_SSPP_SRC_YSTRIDE);
	writel(0x0, pipe_base + PIPE_SSPP_SRC_YSTRIDE + 0x4);
	writel(0x0, pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_LR);
	writel(0x0, pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_TB);
	writel(img_size, pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_REQ_PIXELS);
	writel(0x0, pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_LR);
	writel(0x0, pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_TB);
	writel(img_size, pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_REQ_PIXELS);
	writel(0x0, pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_LR);
	writel(0x0, pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_TB);
	writel(img_size, pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_REQ_PIXELS);

	/* solid fill of transparent color */
	if (yuv_pipe) {
		writel(SSPP_YUV_SOLID_FILL_FORMAT, pipe_base + PIPE_SSPP_SRC_FORMAT);
		writel(SSPP_YUV_UNPACK_PATTERN, pipe_base + PIPE_SSPP_SRC_UNPACK_PATTERN);
		writel(0xFFFFFFFF, pipe_base + PIPE_SSPP_CONSTANT_COLOR);
	} else {
		writel(SSPP_RGB_SOLID_FILL_FORMAT, pipe_base + PIPE_SSPP_SRC_FORMAT);
		writel(SSPP_RGB_UNPACK_PATTERN, pipe_base + PIPE_SSPP_SRC_UNPACK_PATTERN);
		writel(0x0, pipe_base + PIPE_SSPP_CONSTANT_COLOR);
	}

	if (pinfo->orientation & (1 << H_FLIP))
		op_mode |= MDSS_MDP_OP_MODE_FLIP_LR;
	if (pinfo->orientation & (1 << V_FLIP))
		op_mode |= MDSS_MDP_OP_MODE_FLIP_UD;

	/* CSC is also needed to be disabled. */
	writel(op_mode, pipe_base + PIPE_SSPP_SRC_OP_MODE);
	writel(0x0, pipe_base + PIPE_VP_0_QSEEP2_CONFIG);
}

static inline uint32_t mdp_get_lm_mask(uint32_t lm_base)
{
	uint32_t lm_mask = 0;

	switch (lm_base) {
		case MDP_VP_0_MIXER_0_BASE:
			lm_mask = BIT(6);
			break;
		case MDP_VP_0_MIXER_1_BASE:
			lm_mask = BIT(7);
			break;
		case MDP_VP_0_MIXER_2_BASE:
			lm_mask = BIT(8);
			break;
		case MDP_VP_0_MIXER_5_BASE:
			lm_mask = BIT(20);
			break;
		default:
			dprintf(CRITICAL, "%s: invalid lm_base\n", __func__);
			break;
	}

	return lm_mask;
}

static inline uint32_t mdp_get_bitmask_sspp(uint32_t pipe_base)
{
	uint32_t flushbits = 0;

	switch (pipe_base) {
	case MDP_VP_0_VIG_0_BASE:
		flushbits =  BIT(0);
		break;
	case MDP_VP_0_VIG_1_BASE:
		flushbits = BIT(1);
		break;
	case MDP_VP_0_VIG_2_BASE:
		flushbits = BIT(2);
		break;
	case MDP_VP_0_VIG_3_BASE:
		flushbits = BIT(18);
		break;
	case MDP_VP_0_RGB_0_BASE:
		flushbits = BIT(3);
		break;
	case MDP_VP_0_RGB_1_BASE:
		flushbits = BIT(4);
		break;
	case MDP_VP_0_RGB_2_BASE:
		flushbits = BIT(5);
		break;
	case MDP_VP_0_RGB_3_BASE:
		flushbits = BIT(19);
		break;
	case MDP_VP_0_DMA_0_BASE:
		flushbits = BIT(11);
		break;
	case MDP_VP_0_DMA_1_BASE:
		flushbits = BIT(12);
		break;
	default:
		dprintf(CRITICAL, "%s: invalid pipe_base\n", __func__);
		break;
	}

	return flushbits;
}

static inline uint32_t mdp_get_stage_level(uint32_t mix, uint32_t pipe_base)
{
	uint32_t stage_val = 0;

	switch (pipe_base) {
	case MDP_VP_0_VIG_0_BASE:
		stage_val = mix << 0;
		break;
	case MDP_VP_0_VIG_1_BASE:
		stage_val = mix << 3;
		break;
	case MDP_VP_0_VIG_2_BASE:
		stage_val = mix << 6;
		break;
	case MDP_VP_0_VIG_3_BASE:
		stage_val = mix << 26;
		break;
	case MDP_VP_0_RGB_0_BASE:
		stage_val = mix << 9;
		break;
	case MDP_VP_0_RGB_1_BASE:
		stage_val = mix << 12;
		break;
	case MDP_VP_0_RGB_2_BASE:
		stage_val = mix << 15;
		break;
	case MDP_VP_0_RGB_3_BASE:
		stage_val = mix << 29;
		break;
	case MDP_VP_0_DMA_0_BASE:
		stage_val = mix << 18;
		break;
	case MDP_VP_0_DMA_1_BASE:
		stage_val = mix << 21;
		break;
	default:
		dprintf(CRITICAL, "%s: invalid pipe_base\n", __func__);
		break;
	}

	return stage_val;
}

static inline void _mdp_fill_border_rect(uint32 left, uint32_t right,
	uint32_t top, uint32_t bottom, struct border_rect *rect)
{
	rect->left = left;
	rect->right = right;
	rect->top = top;
	rect->bottom = bottom;
}

/* find the pipe in resource manager with the same zorder and pipe_type */
static uint32_t _mdp_get_pipe(struct resource_req *res_mgr,
	uint32_t search_start, uint32_t zorder,
	uint32_t pipe_type, uint32_t *pipe_idx)
{
	uint32_t i = 0;

	for (i = search_start; i < MDP_STAGE_6; i++) {
		if ((res_mgr->pp_state[i].zorder == zorder) &&
			(res_mgr->pp_state[i].type == pipe_type)) {
			*pipe_idx = i;
			return res_mgr->pp_state[i].base;
		}
	}

	dprintf(CRITICAL, "not get pipe base\n");
	dprintf(INFO, "input zorder=%d, pipe_type=%d\n", zorder, pipe_type);
	for (i = search_start; i < MDP_STAGE_6; i++)
		dprintf(INFO, "pp_state[%d]: zorder=%d, type=%d, base=0x%x\n", i,
		res_mgr->pp_state[i].zorder, res_mgr->pp_state[i].type, res_mgr->pp_state[i].base);
	return 0;
}

static void _mdp_get_external_fb_offset(struct resource_req *res_mgr)
{
	uint32_t i = 0;

	for (i = 0; i < MDP_STAGE_6; i++) {
		/* a pipe base != 0 means the pipe is filled by external layer */
		if (res_mgr->pp_state[i].base == 0) {
			external_fb_offset = i;
			break;
		}
	}
}

static void mdss_scalar_phase(struct LayerInfo *layer, int pipe, uint32_t pipe_base)
{
	if (pipe == LEFT_PIPE) {
		writel(layer->left_pipe.scale_data->init_phase_x[0], pipe_base + PIPE_COMP0_3_INIT_PHASE_X);
		writel(layer->left_pipe.scale_data->init_phase_y[0], pipe_base + PIPE_COMP0_3_INIT_PHASE_Y);
		writel(layer->left_pipe.scale_data->phase_step_x[0], pipe_base + PIPE_COMP0_3_PHASE_STEP_X);
		writel(layer->left_pipe.scale_data->phase_step_y[0], pipe_base + PIPE_COMP0_3_PHASE_STEP_Y);

		writel(layer->left_pipe.scale_data->init_phase_x[1], pipe_base + PIPE_COMP1_2_INIT_PHASE_X);
		writel(layer->left_pipe.scale_data->init_phase_y[1], pipe_base + PIPE_COMP1_2_INIT_PHASE_Y);
		writel(layer->left_pipe.scale_data->phase_step_x[1], pipe_base + PIPE_COMP1_2_PHASE_STEP_X);
		writel(layer->left_pipe.scale_data->phase_step_y[1], pipe_base + PIPE_COMP1_2_PHASE_STEP_Y);
	} else {
		writel(layer->right_pipe.scale_data->init_phase_x[0], pipe_base + PIPE_COMP0_3_INIT_PHASE_X);
		writel(layer->right_pipe.scale_data->init_phase_y[0], pipe_base + PIPE_COMP0_3_INIT_PHASE_Y);
		writel(layer->right_pipe.scale_data->phase_step_x[0], pipe_base + PIPE_COMP0_3_PHASE_STEP_X);
		writel(layer->right_pipe.scale_data->phase_step_y[0], pipe_base + PIPE_COMP0_3_PHASE_STEP_Y);

		writel(layer->right_pipe.scale_data->init_phase_x[1], pipe_base + PIPE_COMP1_2_INIT_PHASE_X);
		writel(layer->right_pipe.scale_data->init_phase_y[1], pipe_base + PIPE_COMP1_2_INIT_PHASE_Y);
		writel(layer->right_pipe.scale_data->phase_step_x[1], pipe_base + PIPE_COMP1_2_PHASE_STEP_X);
		writel(layer->right_pipe.scale_data->phase_step_y[1], pipe_base + PIPE_COMP1_2_PHASE_STEP_Y);
	}
}

static void mdss_scalar_ext(struct LayerInfo *layer, struct msm_panel_info *pinfo, int pipe, uint32_t pipe_base)
{
	struct Scale *left_scale, *right_scale;
	uint32_t lr_pe[4], tb_pe[4], tot_req_pixels[4];
	int color, roi_height;
	uint32_t bytemask = 0xff;
	uint32_t shortmask = 0xffff;

	left_scale = layer->left_pipe.scale_data;
	right_scale = layer->right_pipe.scale_data;

	if (pipe == LEFT_PIPE) {
		for (color = 0; color < 4; color++) {
			/* color 2 has the same set of registers as color 1 */
			if (color == 2)
				continue;

			lr_pe[color] = ((left_scale->right.overfetch[color] & bytemask) << 24)|
				((left_scale->right.repeat[color] & bytemask) << 16)|
				((left_scale->left.overfetch[color] & bytemask) << 8)|
				(left_scale->left.repeat[color] & bytemask);

			tb_pe[color] = ((left_scale->bottom.overfetch[color] & bytemask) << 24)|
				((left_scale->bottom.repeat[color] & bytemask) << 16)|
				((left_scale->top.overfetch[color] & bytemask) << 8)|
				(left_scale->top.repeat[color] & bytemask);

			roi_height = layer->left_pipe.src_height;

			tot_req_pixels[color] = (((roi_height +
							left_scale->top.extension[color] +
							left_scale->bottom.extension[color]) & shortmask) << 16) |
				((left_scale->roi_width[color] +
				  left_scale->left.extension[color] +
				  left_scale->right.extension[color]) & shortmask);
		}

		/* color 0 */
		writel(lr_pe[0], pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_LR);
		writel(tb_pe[0], pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_TB);
		writel(tot_req_pixels[0], pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_REQ_PIXELS);

		/* color 1 and color 2 */
		writel(lr_pe[1], pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_LR);
		writel(tb_pe[1], pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_TB);
		writel(tot_req_pixels[1], pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_REQ_PIXELS);

		/* color 3 */
		writel(lr_pe[3], pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_LR);
		writel(tb_pe[3], pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_TB);
		writel(tot_req_pixels[3], pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_REQ_PIXELS);

		dprintf(SPEW, "left pipe setup tot_req:%x %x %x\n",
			tot_req_pixels[0], tot_req_pixels[1],tot_req_pixels[3]);
		dprintf(SPEW, "left pipe setup lr:%x %x %x  tb:%x %x %x\n",
			lr_pe[0], lr_pe[1], lr_pe[3],tb_pe[0], tb_pe[1], tb_pe[3]);
	} else {
		// Setup right pipe for dual pipe case
		for (color = 0; color < 4; color++) {
			/* color 2 has the same set of registers as color 1 */
			if (color == 2)
				continue;

			lr_pe[color] = ((right_scale->right.overfetch[color] & bytemask) << 24)|
				((right_scale->right.repeat[color] & bytemask) << 16)|
				((right_scale->left.overfetch[color] & bytemask) << 8)|
				(right_scale->left.repeat[color] & bytemask);

			tb_pe[color] = ((right_scale->bottom.overfetch[color] & bytemask) << 24)|
				((right_scale->bottom.repeat[color] & bytemask) << 16)|
				((right_scale->top.overfetch[color] & bytemask) << 8)|
				(right_scale->top.repeat[color] & bytemask);

			roi_height = layer->right_pipe.src_height;

			tot_req_pixels[color] = (((roi_height +
							right_scale->top.extension[color] +
							right_scale->bottom.extension[color]) & shortmask) << 16) |
				((right_scale->roi_width[color] +
				  right_scale->left.extension[color] +
				  right_scale->right.extension[color]) & shortmask);
		}

		/* color 0 */
		writel(lr_pe[0], pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_LR);
		writel(tb_pe[0], pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_TB);
		writel(tot_req_pixels[0], pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_REQ_PIXELS);

		/* color 1 and color 2 */
		writel(lr_pe[1], pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_LR);
		writel(tb_pe[1], pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_TB);
		writel(tot_req_pixels[1], pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_REQ_PIXELS);

		/* color 3 */
		writel(lr_pe[3], pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_LR);
		writel(tb_pe[3], pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_TB);
		writel(tot_req_pixels[3], pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_REQ_PIXELS);

		dprintf(SPEW, "right pipe setup tot_req:%x %x %x\n",
			tot_req_pixels[0], tot_req_pixels[1],tot_req_pixels[3]);
		dprintf(SPEW, "right pipe setup lr:%x %x %x  tb:%x %x %x\n",
			lr_pe[0], lr_pe[1], lr_pe[3],tb_pe[0], tb_pe[1], tb_pe[3]);
	}
}

#if ENABLE_QSEED_SCALAR
int mdp_scalar_config(struct LayerInfo* layer, struct msm_panel_info *pinfo,
			uint32_t left_pipe_offset, uint32_t right_pipe_offset)
{
	bool v_scale_up = false;
	bool h_scale_up = false;
	bool v_scale_down = false;
	bool h_scale_down = false;
	uint32_t reg = 0;
	uint32_t op_mode_val = 0;

	if (!layer)
		return -1;

	if (layer->left_pipe.src_rect.w < layer->left_pipe.dst_rect.w)
		h_scale_up = true;
	else if (layer->left_pipe.src_rect.w > layer->left_pipe.dst_rect.w)
		h_scale_down = true;

	if (layer->left_pipe.src_rect.h < layer->left_pipe.dst_rect.h)
		v_scale_up = true;
	else if	(layer->left_pipe.src_rect.h > layer->left_pipe.dst_rect.h)
		v_scale_down = true;

	mdss_scalar_ext(layer, pinfo, LEFT_PIPE, left_pipe_offset);

	if(pinfo->lcdc.dual_pipe && !pinfo->splitter_is_enabled)
		mdss_scalar_ext(layer, pinfo, RIGHT_PIPE, right_pipe_offset);

	/* skip scaling data if no scaling */
	if (!h_scale_up && !h_scale_down && !v_scale_up && !v_scale_down)
		return 0;

	switch (pinfo->pipe_type) {
		case MDSS_MDP_PIPE_TYPE_RGB:
		case MDSS_MDP_PIPE_TYPE_VIG:
			// enable scaling
			if (h_scale_up)
				reg |= 0x11301;
			else if (h_scale_down)
				reg |= 0x22201;

			if (v_scale_up)
				reg |= 0x44C02;
			else if (v_scale_down)
				reg |= 0x88802;
			writel(reg, left_pipe_offset + PIPE_VP_0_QSEEP2_CONFIG);

			//Setup phase
			mdss_scalar_phase(layer, LEFT_PIPE, left_pipe_offset);

			writel(0x00000020, left_pipe_offset + PIPE_VP_0_QSEEP2_SHARP_SMOOTH_STRENGTH);
			writel(0x00000070, left_pipe_offset + PIPE_VP_0_QSEEP2_SHARP_THRESHOLD_EDGE);
			writel(0x00000008, left_pipe_offset + PIPE_VP_0_QSEEP2_SHARP_THRESHOLD_SMOOTH);
			writel(0x00000002, left_pipe_offset + PIPE_VP_0_QSEEP2_SHARP_THRESHHOLD_NOISE);

			_mdp_get_scalar_sspp_op_mode(&op_mode_val,
				pinfo->orientation, left_pipe_offset);
			writel(op_mode_val, left_pipe_offset + PIPE_SSPP_SRC_OP_MODE);

			/* Setup right pipe for dual pipe case */
			if (pinfo->lcdc.dual_pipe && !pinfo->splitter_is_enabled) {
				// enable scaling
				if (h_scale_up)
					reg |= 0x11301;
				else if (h_scale_down)
					reg |= 0x22201;
				else if (pinfo->pipe_type == MDSS_MDP_PIPE_TYPE_VIG)
					reg |= 0x11301;

				if (v_scale_up)
					reg |= 0x44C02;
				else if (v_scale_down)
					reg |= 0x88802;
				else if (pinfo->pipe_type == MDSS_MDP_PIPE_TYPE_VIG)
					reg |= 0x44C02;

				writel(reg, right_pipe_offset + PIPE_VP_0_QSEEP2_CONFIG);

				//Setup phase
				mdss_scalar_phase(layer, RIGHT_PIPE, right_pipe_offset);

				writel(0x00000020, right_pipe_offset + PIPE_VP_0_QSEEP2_SHARP_SMOOTH_STRENGTH);
				writel(0x00000070, right_pipe_offset + PIPE_VP_0_QSEEP2_SHARP_THRESHOLD_EDGE);
				writel(0x00000008, right_pipe_offset + PIPE_VP_0_QSEEP2_SHARP_THRESHOLD_SMOOTH);
				writel(0x00000002, right_pipe_offset + PIPE_VP_0_QSEEP2_SHARP_THRESHHOLD_NOISE);

				_mdp_get_scalar_sspp_op_mode(&op_mode_val,
					pinfo->orientation, right_pipe_offset);
				writel(op_mode_val, right_pipe_offset + PIPE_SSPP_SRC_OP_MODE);
			}

			break;
		case MDSS_MDP_PIPE_TYPE_DMA:
		default:
			break;
	}
	return 0;
}
#endif

static void _mdp_trigger_flush(struct resource_req *res_mgr,
	uint32_t left_ctl_reg_mask, uint32_t right_ctl_reg_mask)
{
	writel(left_ctl_reg_mask, res_mgr->ctl_base[SPLIT_DISPLAY_0] + CTL_FLUSH);

	if (res_mgr->num_ctl == 2)
		writel(right_ctl_reg_mask, res_mgr->ctl_base[SPLIT_DISPLAY_1] + CTL_FLUSH);
}

static void mdp_wait_for_flush_done(struct resource_req *res_mgr,
	uint32_t left_ctl_reg_val, uint32_t right_ctl_reg_val, bool flush_right)
{
	uint32_t i = 0, flush_register = 0;

	/* wait for pipe flush done for ctrl index 0 */
	for (i = 0; i < SSPP_FLUSH_TIMEOUT_MS; i++) {
		flush_register =
			readl(res_mgr->ctl_base[SPLIT_DISPLAY_0] + CTL_FLUSH);
		dprintf(INFO, "left flush reg=0x%x\n", flush_register);
		if ((flush_register & left_ctl_reg_val) != 0)
			mdelay_optimal(1);
		else
			break;
	}

	/* wait for pipe flush done for ctrl index 1 if there is */
	if (res_mgr->num_ctl == 2 && flush_right) {
		for (i = 0; i < SSPP_FLUSH_TIMEOUT_MS; i++) {
			dprintf(INFO, "right flush_reg=0x%x\n", flush_register);
			flush_register =
				readl(res_mgr->ctl_base[SPLIT_DISPLAY_1] + CTL_FLUSH);
			if ((flush_register & right_ctl_reg_val) != 0)
				mdelay_optimal(1);
			else
				break;
		}
	}

	/* print one error for awareness if there is */
	if (i == SSPP_FLUSH_TIMEOUT_MS)
		dprintf(CRITICAL, "pipe flush may have error.\n");
}

static void mdss_mdp_flush_pipe(struct resource_req *res_mgr,
				uint32_t *left_ctl_mask, uint32_t *right_ctl_mask,
				bool stage_right)
{
	uint32_t i = 0;

	for (i = 0; i < MDP_STAGE_6; i++) {
		if ((res_mgr->pending_pipe_mask & (1 << i)) &&
			(res_mgr->pp_state[i].zorder >= MDP_STAGE_1)) {
			if (res_mgr->pp_state[i].lm_idx == LM_LEFT)
				*left_ctl_mask |=
					mdp_get_bitmask_sspp(res_mgr->pp_state[i].base);
			else if (res_mgr->pp_state[i].lm_idx == LM_RIGHT && stage_right)
				*right_ctl_mask |=
					mdp_get_bitmask_sspp(res_mgr->pp_state[i].base);
		}
	}
}

static void mdss_mdp_set_flush(struct msm_panel_info *pinfo,
				uint32_t *left_flush_mask, uint32_t *right_flush_mask)
{
	uint32_t mdss_mdp_rev = readl(MDP_HW_REV);
	bool dual_pipe_single_ctl = pinfo->lcdc.dual_pipe &&
		!pinfo->mipi.dual_dsi && !pinfo->lcdc.split_display;
	uint32_t left_lm_base = 0, right_lm_base = 0;
	struct resource_req *rm = NULL;
	struct resource_req *display_rm = NULL;

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
		return;
	}

	mdss_mdp_flush_pipe(rm, left_flush_mask, right_flush_mask, true);
	dprintf(INFO, "display_id%d: left_flush_mask=0x%x, right_flush_mask =0x%x\n",
		pinfo->dest, *left_flush_mask, *right_flush_mask);

	mdp_rm_clear_pipe_mask(rm);

	left_lm_base = rm->lm_base[SPLIT_DISPLAY_0];
	right_lm_base = rm->lm_base[SPLIT_DISPLAY_1];

	*left_flush_mask |= mdp_get_lm_mask(left_lm_base);
	if (rm->num_lm == 2)
		*right_flush_mask |= mdp_get_lm_mask(right_lm_base);

	*left_flush_mask |= BIT(17);  //CTL
	*right_flush_mask |= BIT(17);  //CTL
	if (dual_pipe_single_ctl)
		*left_flush_mask |= *right_flush_mask;

	/* For targets from MDP v1.5, MDP INTF registers are double buffered */
	if ((mdss_mdp_rev == MDSS_MDP_HW_REV_106) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_108) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_111) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_112)) {
		if (pinfo->dest == DISPLAY_2) {
			*left_flush_mask |= BIT(31);
			*right_flush_mask |= BIT(30);
		} else {
			*left_flush_mask |= BIT(30);
			*right_flush_mask |= BIT(31);
		}
	} else if ((mdss_mdp_rev == MDSS_MDP_HW_REV_105) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_109) ||
		MDSS_IS_MAJOR_MINOR_MATCHING(mdss_mdp_rev,
			MDSS_MDP_HW_REV_107) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_114) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_116) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_115) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_110)) {
		bool use_second_dsi = false;
		int i;

		/* scan the display resource to see any display is already using DSI0 */
		for (i = 0; i < MAX_NUM_DISPLAY; i++) {
			// skip if it is current display
			if (i == (int)(pinfo->dest - 1))
				continue;

			display_rm = mdp_rm_retrieve_resource(i + DISPLAY_1);
			if (!display_rm) {
				dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
				continue;
			}

			if (display_rm->primary_dsi){
				use_second_dsi = true;
				break;
			}
		}

		if ((use_second_dsi && !pinfo->lcdc.pipe_swap) ||
			(!use_second_dsi && pinfo->lcdc.pipe_swap)) {
			*left_flush_mask |= BIT(29);
			*right_flush_mask |= BIT(30);
		} else {
			*left_flush_mask |= BIT(30);
			*right_flush_mask |= BIT(29);
		}
	}
}

static void mdss_source_pipe_config(struct fbcon_config *fb, struct msm_panel_info
		*pinfo, uint32_t pipe_base, struct border_rect *rect, uint32_t offset_pp_index)
{
	uint32_t img_size, roi_size, out_size, stride;
	uint32_t fb_off = 0;
	uint32_t flip_bits = 0;
	uint32_t src_xy = 0, dst_xy = 0;
	uint32_t height, width;
	struct resource_req *rm = NULL;

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
		return;
	}

#if ENABLE_QSEED_SCALAR
	if (fb->layer_scale) {
		img_size = (fb->layer_scale->left_pipe.src_height << 16) |
					fb->layer_scale->left_pipe.src_width;
		height = fb->layer_scale->left_pipe.src_rect.h;
		width = fb->layer_scale->left_pipe.src_rect.w;
		/* write active region size*/
		roi_size = (height << 16) | width;
		out_size = (fb->layer_scale->left_pipe.dst_rect.h << 16) |
				fb->layer_scale->left_pipe.dst_rect.w;
	} else {
#endif
		height = fb->height - rect->top - rect->bottom;
		width = fb->width - rect->left - rect->right;
		/* write active region size*/
		img_size = (height << 16) | width;
		if (pinfo->lcdc.dual_pipe && !pinfo->splitter_is_enabled) {
			width /= 2;
			roi_size = (height << 16) | width;
			out_size = roi_size;
		} else {
			roi_size = img_size;
			out_size = img_size;
		}
#if ENABLE_QSEED_SCALAR
	}
#endif

	if (pinfo->lcdc.dual_pipe && !pinfo->splitter_is_enabled) {
		if (rm->pp_state[offset_pp_index].base == pipe_base){
			//Left pipe
			fb_off = 0;
		} else {
			//Right pipe
			fb_off = pinfo->lm_split[SPLIT_DISPLAY_0]; //width;
		}
	}
	if (target_is_yuv_format(fb->format))
		fb->bpp = 16;


	if (pinfo->lcdc.dual_pipe && !pinfo->splitter_is_enabled)
		stride = (width * 2 * fb->bpp/8);
	else
		stride = (width * fb->bpp/8);

#if ENABLE_QSEED_SCALAR
	if (fb->layer_scale) {
		if (fb_off == 0) {
			dst_xy = (fb->layer_scale->left_pipe.dst_rect.y << 16) |
				fb->layer_scale->left_pipe.dst_rect.x;
			src_xy = (fb->layer_scale->left_pipe.src_rect.y << 16) |
				fb->layer_scale->left_pipe.src_rect.x;
		} else {
			dst_xy = (fb->layer_scale->right_pipe.dst_rect.y << 16) |
				(fb->layer_scale->left_pipe.dst_rect.x +
				fb->layer_scale->left_pipe.dst_rect.w);
			src_xy = (fb->layer_scale->right_pipe.src_rect.y << 16) |
				fb->layer_scale->right_pipe.src_rect.x;
		}
	} else {
#endif
		if (fb_off == 0) {	/* left */
			src_xy = 0;

			if (pinfo->splitter_is_enabled &&
				(rm->pp_state[offset_pp_index + 1].base == pipe_base)) {
				/*
				 * for shared display case, dst_xy of pipe should move to
				 * right part with one offset lm_split[0].
				 */
				dst_xy = (rect->top << 16) |
						(rect->left + pinfo->lm_split[SPLIT_DISPLAY_0]);
			} else
				dst_xy = (rect->top << 16) | rect->left;
		} else {	/* right */
			dst_xy = (rect->left << 16) | fb_off;
			src_xy = fb_off;
		}
#if ENABLE_QSEED_SCALAR
	}
#endif
	dprintf(SPEW,"%s: src=%x fb_off=%x src_xy=%x dst_xy=%x\n",
			 __func__, out_size, fb_off, src_xy, dst_xy);

	writel((uint32_t) fb->base, pipe_base + PIPE_SSPP_SRC0_ADDR);
	writel(stride, pipe_base + PIPE_SSPP_SRC_YSTRIDE);
	writel(img_size, pipe_base + PIPE_SSPP_SRC_IMG_SIZE);
	writel(roi_size, pipe_base + PIPE_SSPP_SRC_SIZE);
	writel(out_size, pipe_base + PIPE_SSPP_SRC_OUT_SIZE);
	writel(src_xy, pipe_base + PIPE_SSPP_SRC_XY);
	writel(dst_xy, pipe_base + PIPE_SSPP_OUT_XY);
	/* Tight Packing 3bpp 0-Alpha 8-bit R B G */
	writel(0x0002243F, pipe_base + PIPE_SSPP_SRC_FORMAT);
	writel(0x00020001, pipe_base + PIPE_SSPP_SRC_UNPACK_PATTERN);

	if (target_is_yuv_format(fb->format)) {
		writel(0x0082B23F, pipe_base + PIPE_SSPP_SRC_FORMAT);
		writel(0x00020001, pipe_base + PIPE_SSPP_SRC_UNPACK_PATTERN);
		writel(0x00060000, pipe_base + PIPE_VP_0_OP_MODE);

		//coeff 1 and 2
		writel(0x9, pipe_base + 0x134);

		writel(0x00000254, pipe_base + PIPE_VP_0_CSC_1_MATRIX_COEFF_0);
		writel(0x02540396, pipe_base + PIPE_VP_0_CSC_1_MATRIX_COEFF_1);
		writel(0x1eef1f93, pipe_base + PIPE_VP_0_CSC_1_MATRIX_COEFF_2);
		writel(0x043e0254, pipe_base + PIPE_VP_0_CSC_1_MATRIX_COEFF_3);
		writel(0x00000000, pipe_base + PIPE_VP_0_CSC_1_MATRIX_COEFF_4);

		// pre clamp
		writel(0x000010eb, pipe_base + PIPE_VP_0_CSC_1_COMP_0_PRE_CLAMP);
		writel(0x000010f0, pipe_base + PIPE_VP_0_CSC_1_COMP_1_PRE_CLAMP);
		writel(0x000010f0, pipe_base + PIPE_VP_0_CSC_1_COMP_2_PRE_CLAMP);

		//post clamp
		writel(0x000000ff, pipe_base + PIPE_VP_0_CSC_1_COMP_0_POST_CAMP);
		writel(0x000000ff, pipe_base + PIPE_VP_0_CSC_1_COMP_1_POST_CLAMP);
		writel(0x000000ff, pipe_base + PIPE_VP_0_CSC_1_COMP_2_POST_CLAMP);

		//pre bias
		writel(0x0000fff0, pipe_base + PIPE_VP_0_CSC_1_COMP_0_PRE_BIAS);
		writel(0x0000ff80, pipe_base + PIPE_VP_0_CSC_1_COMP_1_PRE_BIAS);
		writel(0x0000ff80, pipe_base + PIPE_VP_0_CSC_1_COMP_2_PRE_BIAS);

		//post bias
		writel(0x00000000, pipe_base + PIPE_VP_0_CSC_1_COMP_0_POST_BIAS);
		writel(0x00000000, pipe_base + PIPE_VP_0_CSC_1_COMP_1_POST_BIAS);
		writel(0x00000000, pipe_base + PIPE_VP_0_CSC_1_COMP_2_POST_BIAS);
	}

	/* bit(0) is set if hflip is required.
	 * bit(1) is set if vflip is required.
	 */
	if (pinfo->orientation & (1 << H_FLIP))
		flip_bits |= MDSS_MDP_OP_MODE_FLIP_LR;
	if (pinfo->orientation & (1 << V_FLIP))
		flip_bits |= MDSS_MDP_OP_MODE_FLIP_UD;

	if (is_software_pixel_ext_config_needed()) {
		flip_bits |= BIT(31);
		writel(out_size, pipe_base + PIPE_SW_PIXEL_EXT_C0_REQ);
		writel(out_size, pipe_base + PIPE_SW_PIXEL_EXT_C1C2_REQ);
		writel(out_size, pipe_base + PIPE_SW_PIXEL_EXT_C3_REQ);
		/* configure phase step 1 for all color components */
		writel(0x200000, pipe_base + PIPE_COMP0_3_PHASE_STEP_X);
		writel(0x200000, pipe_base + PIPE_COMP0_3_PHASE_STEP_Y);
		writel(0x200000, pipe_base + PIPE_COMP1_2_PHASE_STEP_X);
		writel(0x200000, pipe_base + PIPE_COMP1_2_PHASE_STEP_Y);

		if (target_is_yuv_format(fb->format)) {
			//QSEED
			writel(0x00055f03, pipe_base + PIPE_VP_0_QSEEP2_CONFIG);
			writel(0x00100000, pipe_base + PIPE_COMP1_2_PHASE_STEP_X);
			writel(0x00000020, pipe_base + PIPE_VP_0_QSEEP2_SHARP_SMOOTH_STRENGTH);
			writel(0x00000070, pipe_base + PIPE_VP_0_QSEEP2_SHARP_THRESHOLD_EDGE);
			writel(0x00000008, pipe_base + PIPE_VP_0_QSEEP2_SHARP_THRESHOLD_SMOOTH);
			writel(0x00000002, pipe_base + PIPE_VP_0_QSEEP2_SHARP_THRESHHOLD_NOISE);

			//PE
			writel(0x00020001, pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_LR);
			writel(0x00020001, pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_TB);
			if (pinfo->lcdc.split_display)
				writel(0x02d30281, pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_REQ_PIXELS);
			else
				writel(0x02d30503, pipe_base + PIPE_SSPP_SW_PIX_EXT_CO_REQ_PIXELS);
			writel(0x00010000, pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_LR);
			writel(0x00010000, pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_TB);
			if (pinfo->lcdc.split_display)
				writel(0x02d10141, pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_REQ_PIXELS);
			else
				writel(0x02d10281, pipe_base + PIPE_SSPP_SW_PIX_EXT_C1C2_REQ_PIXELS);
			writel(0x00010000, pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_LR);
			writel(0x00010000, pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_TB);

			if (pinfo->lcdc.split_display)
				writel(0x02d10281, pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_REQ_PIXELS);
			else
				writel(0x02d10501, pipe_base + PIPE_SSPP_SW_PIX_EXT_C3_REQ_PIXELS);
			}
#if ENABLE_QSEED_SCALAR
	if (fb->layer_scale) {
		mdss_scalar_phase(fb->layer_scale, fb_off, pipe_base);
		mdss_scalar_ext(fb->layer_scale, pinfo, fb_off, pipe_base);
	}
#endif
	}
	writel(flip_bits, pipe_base + PIPE_SSPP_SRC_OP_MODE);
}

static void mdss_vbif_setup()
{
	uint32_t mdp_hw_rev = readl(MDP_HW_REV);
	int access_secure = false;
	if (!MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev, MDSS_MDP_HW_REV_107))
		access_secure = restore_secure_cfg(SECURE_DEVICE_MDSS);

	if (!access_secure) {
		dprintf(SPEW, "MDSS VBIF registers unlocked by TZ.\n");

		/* Force VBIF Clocks on, needed for 8974 and 8x26 */
		if (mdp_hw_rev < MDSS_MDP_HW_REV_103)
			writel(0x1, VBIF_VBIF_DDR_FORCE_CLK_ON);

		/*
		 * Following configuration is needed because on some versions,
		 * recommended reset values are not stored.
		 */
		if (MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
			MDSS_MDP_HW_REV_100)) {
			writel(0x00000707, VBIF_VBIF_DDR_OUT_MAX_BURST);
			writel(0x00000030, VBIF_VBIF_DDR_ARB_CTRL );
			writel(0x00000001, VBIF_VBIF_DDR_RND_RBN_QOS_ARB);
			writel(0x00000FFF, VBIF_VBIF_DDR_OUT_AOOO_AXI_EN);
			writel(0x0FFF0FFF, VBIF_VBIF_DDR_OUT_AX_AOOO);
			writel(0x22222222, VBIF_VBIF_DDR_AXI_AMEMTYPE_CONF0);
			writel(0x00002222, VBIF_VBIF_DDR_AXI_AMEMTYPE_CONF1);
		} else if (MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
				MDSS_MDP_HW_REV_101)) {
			writel(0x00000707, VBIF_VBIF_DDR_OUT_MAX_BURST);
			writel(0x00000003, VBIF_VBIF_DDR_RND_RBN_QOS_ARB);
		}
	}
}

static uint32_t mdss_smp_alloc(uint32_t client_id, uint32_t smp_cnt,
	uint32_t fixed_smp_cnt, uint32_t free_smp_offset)
{
	uint32_t i, j;
	uint32_t reg_val = 0;

	for (i = fixed_smp_cnt, j = 0; i < smp_cnt; i++) {
		/* max 3 MMB per register */
		reg_val |= client_id << (((j++) % 3) * 8);
		if ((j % 3) == 0) {
			writel(reg_val, MMSS_MDP_SMP_ALLOC_W_BASE +
				free_smp_offset);
			writel(reg_val, MMSS_MDP_SMP_ALLOC_R_BASE +
				free_smp_offset);
			reg_val = 0;
			free_smp_offset += 4;
		}
	}

	if (j % 3) {
		writel(reg_val, MMSS_MDP_SMP_ALLOC_W_BASE + free_smp_offset);
		writel(reg_val, MMSS_MDP_SMP_ALLOC_R_BASE + free_smp_offset);
		free_smp_offset += 4;
	}

	return free_smp_offset;
}

static void mdp_select_pipe_client_id(struct msm_panel_info *pinfo,
		uint32_t *left_sspp_client_id, uint32_t *right_sspp_client_id)
{
	uint32_t mdss_mdp_rev = readl(MDP_HW_REV);
	if (MDSS_IS_MAJOR_MINOR_MATCHING(mdss_mdp_rev, MDSS_MDP_HW_REV_101) ||
		MDSS_IS_MAJOR_MINOR_MATCHING(mdss_mdp_rev, MDSS_MDP_HW_REV_106) ||
		MDSS_IS_MAJOR_MINOR_MATCHING(mdss_mdp_rev, MDSS_MDP_HW_REV_108) ||
		MDSS_IS_MAJOR_MINOR_MATCHING(mdss_mdp_rev, MDSS_MDP_HW_REV_111) ||
		MDSS_IS_MAJOR_MINOR_MATCHING(mdss_mdp_rev, MDSS_MDP_HW_REV_112)) {
		switch (pinfo->pipe_type) {
			case MDSS_MDP_PIPE_TYPE_RGB:
				*left_sspp_client_id = 0x7; /* 7 */
				*right_sspp_client_id = 0x8; /* 8 */
				break;
			case MDSS_MDP_PIPE_TYPE_DMA:
				*left_sspp_client_id = 0x4; /* 4 */
				*right_sspp_client_id = 0xD; /* 13 */
				break;
			case MDSS_MDP_PIPE_TYPE_VIG:
			default:
				*left_sspp_client_id = 0x1; /* 1 */
				*right_sspp_client_id = 0x9; /* 9 */
				break;
		}
	} else {
		switch (pinfo->pipe_type) {
			case MDSS_MDP_PIPE_TYPE_RGB:
				*left_sspp_client_id = 0x10; /* 16 */
				*right_sspp_client_id = 0x11; /* 17 */
				break;
			case MDSS_MDP_PIPE_TYPE_DMA:
				*left_sspp_client_id = 0xA; /* 10 */
				*right_sspp_client_id = 0xD; /* 13 */
				break;
			case MDSS_MDP_PIPE_TYPE_VIG:
			default:
				*left_sspp_client_id = 0x1; /* 1 */
				*right_sspp_client_id = 0x4; /* 4 */
				break;
		}
	}
}

static void mdp_select_pipe_xin_id(struct msm_panel_info *pinfo,
		uint32_t *left_pipe_xin_id, uint32_t *right_pipe_xin_id)
{
	switch (pinfo->pipe_type) {
		case MDSS_MDP_PIPE_TYPE_RGB:
			*left_pipe_xin_id = 0x1; /* 1 */
			*right_pipe_xin_id = 0x5; /* 5 */
			break;
		case MDSS_MDP_PIPE_TYPE_DMA:
			*left_pipe_xin_id = 0x2; /* 2 */
			*right_pipe_xin_id = 0xA; /* 10 */
			break;
		case MDSS_MDP_PIPE_TYPE_VIG:
		default:
			*left_pipe_xin_id = 0x0; /* 0 */
			*right_pipe_xin_id = 0x4; /* 4 */
			break;
	}
}

static void mdss_smp_setup(struct msm_panel_info *pinfo, uint32_t left_pipe,
		uint32_t right_pipe)

{
	uint32_t left_sspp_client_id, right_sspp_client_id;
	uint32_t bpp = 3, free_smp_offset = 0, xres = MDSS_MAX_LINE_BUF_WIDTH;
	uint32_t smp_cnt, smp_size = 4096, fixed_smp_cnt = 0;
	uint32_t mdss_mdp_rev = readl(MDP_HW_REV);

	if ((mdss_mdp_rev == MDSS_MDP_HW_REV_106) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_112)) {
		/* 8Kb per SMP on 8916/8952 */
		smp_size = 8192;
	} else if ((mdss_mdp_rev == MDSS_MDP_HW_REV_108) ||
		(mdss_mdp_rev == MDSS_MDP_HW_REV_111)) {
		/* 10Kb per SMP on 8939/8956 */
		smp_size = 10240;
	} else if ((mdss_mdp_rev >= MDSS_MDP_HW_REV_103) &&
		(mdss_mdp_rev < MDSS_MDP_HW_REV_200)) {
		smp_size = 8192;
		free_smp_offset = 0xC;
		if (pinfo->pipe_type == MDSS_MDP_PIPE_TYPE_RGB)
			fixed_smp_cnt = 2;
		else
			fixed_smp_cnt = 0;
	}

	mdp_select_pipe_client_id(pinfo,
			&left_sspp_client_id, &right_sspp_client_id);

	/* Each pipe driving half the screen */
	if (pinfo->lcdc.dual_pipe && !pinfo->lcdc.force_merge)
		xres = pinfo->lm_split[0];

	/* bpp = bytes per pixel of input image */
	smp_cnt = (xres * bpp * 2) + smp_size - 1;
	smp_cnt /= smp_size;

	if (smp_cnt > 4) {
		dprintf(CRITICAL, "ERROR: %s: Out of SMP's, cnt=%d! \n", __func__,
				smp_cnt);
		ASSERT(0); /* Max 4 SMPs can be allocated per client */
	}

	writel(smp_cnt * 0x40, left_pipe + REQPRIORITY_FIFO_WATERMARK0);
	writel(smp_cnt * 0x80, left_pipe + REQPRIORITY_FIFO_WATERMARK1);
	writel(smp_cnt * 0xc0, left_pipe + REQPRIORITY_FIFO_WATERMARK2);

	if (pinfo->lcdc.dual_pipe && !pinfo->lcdc.force_merge) {
		xres = pinfo->lm_split[1];

		smp_cnt = (xres * bpp * 2) + smp_size - 1;
		smp_cnt /= smp_size;

		writel(smp_cnt * 0x40, right_pipe + REQPRIORITY_FIFO_WATERMARK0);
		writel(smp_cnt * 0x80, right_pipe + REQPRIORITY_FIFO_WATERMARK1);
		writel(smp_cnt * 0xc0, right_pipe + REQPRIORITY_FIFO_WATERMARK2);
	}

	free_smp_offset = mdss_smp_alloc(left_sspp_client_id, smp_cnt,
		fixed_smp_cnt, free_smp_offset);
	if (pinfo->lcdc.dual_pipe && !pinfo->lcdc.force_merge)
		mdss_smp_alloc(right_sspp_client_id, smp_cnt, fixed_smp_cnt,
			free_smp_offset);
}

static void mdss_intf_tg_setup(struct msm_panel_info *pinfo, uint32_t intf_base)
{
	uint32_t hsync_period, vsync_period;
	uint32_t hsync_start_x, hsync_end_x;
	uint32_t display_hctl, hsync_ctl, display_vstart, display_vend;
	uint32_t adjust_xres = 0;
	uint32_t upper = 0, lower = 0;

	struct lcdc_panel_info *lcdc = NULL;
	struct intf_timing_params itp = {0};

	if (pinfo == NULL)
		return;

	lcdc =  &(pinfo->lcdc);
	if (lcdc == NULL)
		return;

	adjust_xres = pinfo->xres;
	if (pinfo->lcdc.split_display) {
		if (pinfo->lcdc.dst_split) {
			adjust_xres /= 2;
		} else if(pinfo->lcdc.dual_pipe) {
			if (intf_base == (MDP_INTF_1_BASE + mdss_mdp_intf_offset()))
				adjust_xres = pinfo->lm_split[0];
			else
				adjust_xres = pinfo->lm_split[1];
		}

		if ((intf_base == (MDP_INTF_1_BASE + mdss_mdp_intf_offset())) &&
				(!pinfo->lcdc.pipe_swap)) {
			lower |= BIT(8);
			upper |= BIT(4);

			writel(lower, MDP_REG_SPLIT_DISPLAY_LOWER_PIPE_CTL);
			writel(upper, MDP_REG_SPLIT_DISPLAY_UPPER_PIPE_CTL);
			writel(0x1, MDP_REG_SPLIT_DISPLAY_EN);
		} if ((intf_base == (MDP_INTF_2_BASE + mdss_mdp_intf_offset())) &&
				(pinfo->lcdc.pipe_swap)) {
			lower |= BIT(4);
			upper |= BIT(8);
			writel(lower, MDP_REG_SPLIT_DISPLAY_LOWER_PIPE_CTL);
			writel(upper, MDP_REG_SPLIT_DISPLAY_UPPER_PIPE_CTL);
			writel(0x1, MDP_REG_SPLIT_DISPLAY_EN);
		}
	}

	if (pinfo->lcdc.dst_split && (intf_base == (MDP_INTF_1_BASE + mdss_mdp_intf_offset()))) {
		uint32_t ppb_offset = mdss_mdp_get_ppb_offset();
		writel(BIT(5), REG_MDP(ppb_offset)); /* MMSS_MDP_PPB0_CNTL */
		writel(BIT(16) | (0x3 << 20), REG_MDP(ppb_offset + 0x4)); /* MMSS_MDP_PPB0_CONFIG */
	}

	if (!pinfo->fbc.enabled || !pinfo->fbc.comp_ratio)
		pinfo->fbc.comp_ratio = 1;

	itp.xres = (adjust_xres / pinfo->fbc.comp_ratio);
	itp.yres = pinfo->yres;
	itp.width =((adjust_xres + pinfo->lcdc.xres_pad) / pinfo->fbc.comp_ratio);

	if (pinfo->compression_mode == COMPRESSION_DSC) {
		itp.xres = pinfo->dsc.pclk_per_line;
		itp.width = pinfo->dsc.pclk_per_line;
	}

	itp.height = pinfo->yres + pinfo->lcdc.yres_pad;
	itp.h_back_porch = pinfo->lcdc.h_back_porch;
	itp.h_front_porch = pinfo->lcdc.h_front_porch;
	itp.v_back_porch =  pinfo->lcdc.v_back_porch;
	itp.v_front_porch = pinfo->lcdc.v_front_porch;
	itp.hsync_pulse_width = pinfo->lcdc.h_pulse_width;
	itp.vsync_pulse_width = pinfo->lcdc.v_pulse_width;

	itp.border_clr = pinfo->lcdc.border_clr;
	itp.underflow_clr = pinfo->lcdc.underflow_clr;
	itp.hsync_skew = pinfo->lcdc.hsync_skew;

	hsync_period = itp.hsync_pulse_width + itp.h_back_porch +
			itp.width + itp.h_front_porch;

	vsync_period = itp.vsync_pulse_width + itp.v_back_porch +
			itp.height + itp.v_front_porch;

	hsync_start_x =
		itp.hsync_pulse_width +
		itp.h_back_porch;
	hsync_end_x =
		hsync_period - itp.h_front_porch - 1;

	display_vstart = (itp.vsync_pulse_width +
			itp.v_back_porch)
		* hsync_period + itp.hsync_skew;
	display_vend = ((vsync_period - itp.v_front_porch) * hsync_period)
		+ itp.hsync_skew - 1;

	if (intf_base == (MDP_INTF_0_BASE + mdss_mdp_intf_offset())) { /* eDP */
		display_vstart += itp.hsync_pulse_width + itp.h_back_porch;
		display_vend -= itp.h_front_porch;
	}

	hsync_ctl = (hsync_period << 16) | itp.hsync_pulse_width;
	display_hctl = (hsync_end_x << 16) | hsync_start_x;

	writel(hsync_ctl, MDP_HSYNC_CTL + intf_base);
	writel(vsync_period*hsync_period, MDP_VSYNC_PERIOD_F0 +
			intf_base);
	writel(0x00, MDP_VSYNC_PERIOD_F1 + intf_base);
	writel(itp.vsync_pulse_width*hsync_period,
			MDP_VSYNC_PULSE_WIDTH_F0 +
			intf_base);
	writel(0x00, MDP_VSYNC_PULSE_WIDTH_F1 + intf_base);
	writel(display_hctl, MDP_DISPLAY_HCTL + intf_base);
	writel(display_vstart, MDP_DISPLAY_V_START_F0 +
			intf_base);
	writel(0x00, MDP_DISPLAY_V_START_F1 + intf_base);
	writel(display_vend, MDP_DISPLAY_V_END_F0 +
			intf_base);
	writel(0x00, MDP_DISPLAY_V_END_F1 + intf_base);
	writel(0x00, MDP_ACTIVE_HCTL + intf_base);
	writel(0x00, MDP_ACTIVE_V_START_F0 + intf_base);
	writel(0x00, MDP_ACTIVE_V_START_F1 + intf_base);
	writel(0x00, MDP_ACTIVE_V_END_F0 + intf_base);
	writel(0x00, MDP_ACTIVE_V_END_F1 + intf_base);
	writel(0xFF, MDP_UNDERFFLOW_COLOR + intf_base);

	if (intf_base == (MDP_INTF_0_BASE + mdss_mdp_intf_offset())) /* eDP */
		writel(0x212A, MDP_PANEL_FORMAT + intf_base);
	else
		writel(0x213F, MDP_PANEL_FORMAT + intf_base);
}

static void mdss_intf_fetch_start_config(struct msm_panel_info *pinfo,
					uint32_t intf_base)
{
	uint32_t mdp_hw_rev = readl(MDP_HW_REV);
	uint32_t v_total, h_total, fetch_start, vfp_start;
	uint32_t prefetch_avail, prefetch_needed;
	uint32_t adjust_xres = 0;
	uint32_t fetch_enable = BIT(31);

	struct lcdc_panel_info *lcdc = NULL;

	if (pinfo == NULL)
		return;

	lcdc =  &(pinfo->lcdc);
	if (lcdc == NULL)
		return;

	/*
	 * MDP programmable fetch is for MDP with rev >= 1.05.
	 * Programmable fetch is not needed if vertical back porch
	 * plus vertical puls width is >= 25.
	 */
	if (mdp_hw_rev < MDSS_MDP_HW_REV_105 ||
			(lcdc->v_back_porch + lcdc->v_pulse_width) >=
			MDSS_MDP_MAX_PREFILL_FETCH)
		return;

	adjust_xres = pinfo->xres;
	if (pinfo->lcdc.split_display) {
		if (pinfo->lcdc.dst_split) {
			adjust_xres /= 2;
		} else if(pinfo->lcdc.dual_pipe) {
			if (intf_base == (MDP_INTF_1_BASE + mdss_mdp_intf_offset()))
				adjust_xres = pinfo->lm_split[0];
			else
				adjust_xres = pinfo->lm_split[1];
		}
	}

	if (pinfo->compression_mode == COMPRESSION_DSC) {
		adjust_xres = pinfo->dsc.pclk_per_line;
	} else if (pinfo->compression_mode == COMPRESSION_FBC) {
		if (pinfo->fbc.enabled && pinfo->fbc.comp_ratio)
			adjust_xres /= pinfo->fbc.comp_ratio;
	}

	/*
	 * Fetch should always be outside the active lines. If the fetching
	 * is programmed within active region, hardware behavior is unknown.
	 */
	v_total = lcdc->v_pulse_width + lcdc->v_back_porch + pinfo->yres +
							lcdc->v_front_porch;
	h_total = lcdc->h_pulse_width + lcdc->h_back_porch + adjust_xres +
							lcdc->h_front_porch;
	vfp_start = lcdc->v_pulse_width + lcdc->v_back_porch + pinfo->yres;

	prefetch_avail = v_total - vfp_start;
	prefetch_needed = MDSS_MDP_MAX_PREFILL_FETCH -
		lcdc->v_back_porch -
		lcdc->v_pulse_width;

	/*
	 * In some cases, vertical front porch is too high. In such cases limit
	 * the mdp fetch lines  as the last (25 - vbp - vpw) lines of vertical front porch.
	 */
	if (prefetch_avail > prefetch_needed)
		prefetch_avail = prefetch_needed;

	fetch_start = (v_total - prefetch_avail) * h_total + 1;

	if (pinfo->dfps.panel_dfps.enabled)
		fetch_enable |= BIT(23);

	writel_relaxed(fetch_start, MDP_PROG_FETCH_START + intf_base);
	writel_relaxed(fetch_enable, MDP_INTF_CONFIG + intf_base);
}

static int mdp_acquire_pipe(uint32_t dest_display_id, struct fbcon_config *fb,
	uint32_t *pipe_base, bool right_stage, char *explicit_pipe_name)
{
	uint32_t index;
	uint32_t pipe_type;
	int ret;

	if (target_is_yuv_format(fb->format))
		pipe_type = MDSS_MDP_PIPE_TYPE_VIG;
	else
		pipe_type = MDSS_MDP_PIPE_TYPE_RGB;

	ret = mdp_rm_search_pipe(pipe_type, dest_display_id,
			&index, explicit_pipe_name);
	if (ret) {
		dprintf(CRITICAL, "%s: pipe search failed\n", __func__);
		return ret;
	}

	dprintf(INFO, "%s:index=0x%x\n", __func__, index);
	ret = mdp_rm_update_pipe_status(index, dest_display_id,
		fb->z_order, right_stage, pipe_base);

	return ret;
}

static bool mdp_check_pipe_right_mixer(struct msm_panel_info *pinfo)
{
	if (pinfo->lcdc.dual_pipe) {
		if (pinfo->splitter_is_enabled && pinfo->lcdc.force_merge)
			return false;
		else if (pinfo->lcdc.split_display ||
			(pinfo->splitter_is_enabled && !pinfo->lcdc.force_merge))
			return true;
	}

	return false;
}

int mdp_setup_pipe(struct msm_panel_info *pinfo,
		struct fbcon_config *fb, uint32_t fb_cnt,
		uint32_t *left_pipe, uint32_t *right_pipe)
{
	uint32_t pipe_base = 0, fb_index = SPLIT_DISPLAY_0;
	struct border_rect rect;
	uint32_t ret = 0, real_fb_cnt = 0;
	bool pipe_on_right = false;
	struct resource_req *res_mgr = NULL;
	uint32_t search_start = 0, pp_index = 0, pipe_type;

	dprintf(INFO, "%s:input fb_cnt=%d\n", __func__, fb_cnt);
	real_fb_cnt = fb_cnt;

	res_mgr = mdp_rm_retrieve_resource(pinfo->dest);
	if (!res_mgr) {
		dprintf(CRITICAL, "%s:get hardware resource failed\n", __func__);
		return ERR_NOT_VALID;
	}

	_mdp_get_external_fb_offset(res_mgr);

	/*
	* In dual pipe & non-spliiter case like 4K or DSI split case, as each fb
	* will use 2 pipes, so fb_cnt * 2 should not exceed the max value of blend stage.
	*/
	if (pinfo->lcdc.dual_pipe & !pinfo->splitter_is_enabled) {
		if (fb_cnt * MAX_SPLIT_DISPLAY > MDP_STAGE_6) {
			dprintf(CRITICAL, "invalid user fb number\n");
			real_fb_cnt = MDP_STAGE_6 / MAX_SPLIT_DISPLAY;
		}
	}

	dprintf(CRITICAL, "%s:real_fb=%d\n", __func__, real_fb_cnt);

	for (fb_index = SPLIT_DISPLAY_0; fb_index < real_fb_cnt; fb_index++) {
		dprintf(INFO, "%s: fb_index=%d", __func__, fb_index);
		pipe_on_right = false;
		search_start = 0;

		/* If one fb's format is invalid, continue for next fb */
		if (!target_format_is_valid(fb[fb_index].format))
			continue;

		ret = mdp_acquire_pipe(pinfo->dest, &fb[fb_index],
				&pipe_base, pipe_on_right, NULL);
		if (ret) {
			dprintf(CRITICAL, "Acquire pipe for fb%d failed\n", fb_index);
			continue;
		}
		dprintf(INFO, "Acquire pipe base=0x%x\n", pipe_base);
		if (target_is_yuv_format(fb[fb_index].format))
			pipe_type = MDSS_MDP_PIPE_TYPE_VIG;
		else
			pipe_type = MDSS_MDP_PIPE_TYPE_RGB;

		if (pipe_base != _mdp_get_pipe(res_mgr, search_start,
			fb[fb_index].z_order, pipe_type, &pp_index)) {
			dprintf(CRITICAL, "pipe_base is not the same, aborted\n");
			continue;
		}

		if ((fb[fb_index].base == NULL) &&
			target_format_is_valid(fb[fb_index].format)) {
			/* set layer to be transparent */
			dprintf(INFO, "transparent base=0x%x\n", pipe_base);
			mdss_mdp_set_layer_transparent(pinfo, pipe_base,
				target_is_yuv_format(fb[fb_index].format));
		} else {
			/* normal layer */
			dprintf(INFO, "normal base=0x%x\n", pipe_base);
			_mdp_fill_border_rect(pinfo->border_left[fb_index],
						pinfo->border_right[fb_index],
						pinfo->border_top[fb_index],
						pinfo->border_bottom[fb_index],
						&rect);

			mdss_source_pipe_config(&fb[fb_index], pinfo,
				pipe_base, &rect, external_fb_offset);
		}

		mdp_rm_update_pipe_pending_mask(res_mgr, pp_index);

		/* For dual pipe & non-splitter case, fetch from the same FB */
		if (pinfo->lcdc.dual_pipe & !pinfo->splitter_is_enabled) {
			/*
			 * For dual pipe & non-splitter case fetching from the same FB,
			 * there are two pipes allocated in resource manager. So when searching
			 * it, it's needed to exclude the the former pipe who has the same zorder
			 * and pipe_type. So the search start will start from actual position + 1.
			 */
			search_start = pp_index + 1;

			pipe_on_right = mdp_check_pipe_right_mixer(pinfo);

			ret = mdp_acquire_pipe(pinfo->dest, &fb[fb_index],
					&pipe_base, pipe_on_right, NULL);
			if (ret) {
				dprintf(CRITICAL,
					"Acquire pipe for fb%d failed in dual pipe case\n", fb_index);
				continue;
			}

			if (pipe_base != _mdp_get_pipe(res_mgr, search_start,
				fb[fb_index].z_order, pipe_type, &pp_index)) {
				dprintf(INFO, "pipe_base is not the same in dual pipe case\n");
				continue;
			}

			if ((fb[fb_index].base == NULL) &&
				target_format_is_valid(fb[fb_index].format)) {
				/* set layer to be transparent */
				mdss_mdp_set_layer_transparent(pinfo, pipe_base,
					target_is_yuv_format(fb[fb_index].format));
			} else {
				/* normal layer */
				mdss_source_pipe_config(&fb[fb_index], pinfo,
					pipe_base, &rect, external_fb_offset);
			}

			mdp_rm_update_pipe_pending_mask(res_mgr, pp_index);
			++fb_index;
		}
	}

	/* to backward compatible with legacy code */
	if ((external_fb_offset + 1) < MDP_STAGE_6) {
		*left_pipe = res_mgr->pp_state[external_fb_offset].base;
		*right_pipe = res_mgr->pp_state[external_fb_offset + 1].base;
	}

	dprintf(INFO, "%s:left_pipe=0x%x, right_pipe=0x%x\n", __func__, *left_pipe, *right_pipe);

	return NO_ERROR;
}

static int mdp_get_pipe_stage_level(struct resource_req *rm,
	uint32_t *left_stage, uint32_t *right_stage)
{
	uint32_t i = 0;

	dprintf(INFO, "%s: pending_pipe_mask=0x%x\n", __func__, rm->pending_pipe_mask);
	for (i = 0; i < MDP_STAGE_6; i++) {
		if ((rm->pending_pipe_mask & (1 << i)) &&
			(rm->pp_state[i].zorder >= MDP_STAGE_1)) {
			if (rm->pp_state[i].lm_idx == LM_LEFT)
				*left_stage |= mdp_get_stage_level(rm->pp_state[i].zorder,
								rm->pp_state[i].base);
			else
				*right_stage |= mdp_get_stage_level(rm->pp_state[i].zorder,
								rm->pp_state[i].base);
		}
	}

	return NO_ERROR;
}

static int mdp_blend_setup(struct resource_req *rm,
		uint32_t left_lm_base, uint32_t right_lm_base)
{
	uint32_t i = 0;

	for (i = 0; i < MDP_STAGE_6; i++) {
		if (rm->pp_state[i].zorder >= MDP_STAGE_1) {
			if (rm->pp_state[i].lm_idx == LM_LEFT)
				mdss_layer_mixer_setup_blend_config(left_lm_base,
				rm->pp_state[i].zorder, rm->pp_state[i].type);
			else
				mdss_layer_mixer_setup_blend_config(right_lm_base,
				rm->pp_state[i].zorder, rm->pp_state[i].type);
		} else
			break;
	}

	return NO_ERROR;
}

int mdss_layer_mixer_hide_pipe(struct msm_panel_info *pinfo,
	struct fbcon_config *fb, uint32_t fb_cnt)
{
	struct resource_req *rm = NULL;
	bool flush_right = true;
	uint32_t left_ctl_mask = 0, right_ctl_mask = 0;
	int ret = 0;

	if (!pinfo) {
		dprintf(CRITICAL, "Invalid input\n");
		return ERROR;
	}

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
		return ERR_NOT_VALID;
	}

	ret = mdp_config_pipe(pinfo, fb, fb_cnt);
	if (ret) {
		dprintf(CRITICAL, "call mdp_config_pipe(fb_cnt=%d) failed\n",
			fb_cnt);
		return ret;
	}

	dprintf(INFO, "pending mask=0x%x\n", rm->pending_pipe_mask);

	ret = mdp_trigger_flush(pinfo, fb, fb_cnt,
		&left_ctl_mask, &right_ctl_mask);
	if (ret) {
		dprintf(CRITICAL, "call mdp_trigger_flush(fb_cnt=%d) failed\n",
			fb_cnt);
		return ret;
	}

	if (pinfo->splitter_is_enabled &&
		(fb[SPLIT_DISPLAY_1].base == NULL) &&
		!target_format_is_valid(fb[SPLIT_DISPLAY_1].format))
		flush_right = false;

	dprintf(INFO, "%s:left_ctl_mask=0x%x, right_ctl_mask=0x%x\n",
		__func__, left_ctl_mask, right_ctl_mask);

	mdp_wait_for_flush_done(rm, left_ctl_mask,
		right_ctl_mask, flush_right);

	return ret;
}

void mdss_layer_mixer_setup(struct fbcon_config *fb, struct msm_panel_info *pinfo)
{
	uint32_t mdp_rgb_size, height, width;
	uint32_t left_staging_level = 0, right_staging_level = 0;
	uint32_t left_mixer_base = 0, right_mixer_base = 0;
	uint32_t left_ctl_base = 0, right_ctl_base = 0;
	struct resource_req *rm= NULL;
	struct resource_req *display_1_rm = NULL;

	/* Update mixer per destination */
	mdp_rm_select_mixer(pinfo);

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
		return;
	}

	if (pinfo->dest != DISPLAY_1) {
		display_1_rm = mdp_rm_retrieve_resource(DISPLAY_1);
		if (!display_1_rm) {
			dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
			return;
		}
	}

	left_mixer_base = rm->lm_base[SPLIT_DISPLAY_0];
	right_mixer_base = rm->lm_base[SPLIT_DISPLAY_1];
	left_ctl_base = rm->ctl_base[SPLIT_DISPLAY_0];
	right_ctl_base = rm->ctl_base[SPLIT_DISPLAY_1];

	height = fb[SPLIT_DISPLAY_0].height;
	width = fb[SPLIT_DISPLAY_0].width;

	if (pinfo->splitter_is_enabled && pinfo->lcdc.dual_pipe && pinfo->lcdc.force_merge) {
		/*
		 * For small resolution case in shared display mode, only one layer mixer
		 * is used, so the active region of layer mixer needs to be adjusted.
		 */
		width = pinfo->xres;
	} else if ((pinfo->lcdc.dual_pipe && !pinfo->lcdc.dst_split)
			|| (pinfo->lcdc.split_display))
		width = pinfo->lm_split[SPLIT_DISPLAY_0];

	/* write active region size*/
	mdp_rgb_size = (height << 16) | width;

	dprintf(SPEW, "Mixer setup left LM  size:%x  base:%x\n", mdp_rgb_size, left_mixer_base);
		writel(mdp_rgb_size, left_mixer_base + LAYER_0_OUT_SIZE);
		writel(0x0, left_mixer_base + LAYER_0_BORDER_COLOR_0);
		writel(0x0, left_mixer_base + LAYER_0_BORDER_COLOR_1);
		writel(0x00, left_mixer_base + LAYER_0_OP_MODE);
		writel(0x100, left_mixer_base + LAYER_0_BLEND_OP);
		writel(0xFF, left_mixer_base + LAYER_0_BLEND0_FG_ALPHA);
		writel(0x100, left_mixer_base + LAYER_1_BLEND_OP);
		writel(0xFF, left_mixer_base + LAYER_1_BLEND0_FG_ALPHA);
		writel(0x100, left_mixer_base + LAYER_2_BLEND_OP);
		writel(0xFF, left_mixer_base + LAYER_2_BLEND0_FG_ALPHA);
		writel(0x100, left_mixer_base + LAYER_3_BLEND_OP);
		writel(0xFF, left_mixer_base + LAYER_3_BLEND0_FG_ALPHA);
		writel(0x100, left_mixer_base + LAYER_4_BLEND_OP);
		writel(0xFF, left_mixer_base + LAYER_4_BLEND0_FG_ALPHA);
		writel(0x100, left_mixer_base + LAYER_5_BLEND_OP);
		writel(0xFF, left_mixer_base + LAYER_5_BLEND0_FG_ALPHA);
	if (pinfo->lcdc.dual_pipe && !pinfo->lcdc.force_merge) {
		dprintf(SPEW, "Mixer setup right LM  size:%x base:%x\n", mdp_rgb_size, right_mixer_base);

		writel(mdp_rgb_size, right_mixer_base + LAYER_0_OUT_SIZE);
		writel(0x0, right_mixer_base + LAYER_0_BORDER_COLOR_0);
		writel(0x0, right_mixer_base + LAYER_0_BORDER_COLOR_1);
		writel(0x80000000, right_mixer_base + LAYER_0_OP_MODE);
		writel(0x100, right_mixer_base + LAYER_0_BLEND_OP);
		writel(0xFF, right_mixer_base + LAYER_0_BLEND0_FG_ALPHA);
		writel(0x100, right_mixer_base + LAYER_1_BLEND_OP);
		writel(0xFF, right_mixer_base + LAYER_1_BLEND0_FG_ALPHA);
		writel(0x100, right_mixer_base + LAYER_2_BLEND_OP);
		writel(0xFF, right_mixer_base + LAYER_2_BLEND0_FG_ALPHA);
		writel(0x100, right_mixer_base + LAYER_3_BLEND_OP);
		writel(0xFF, right_mixer_base + LAYER_3_BLEND0_FG_ALPHA);
		writel(0x100, right_mixer_base + LAYER_4_BLEND_OP);
		writel(0xFF, right_mixer_base + LAYER_4_BLEND0_FG_ALPHA);
		writel(0x100, right_mixer_base + LAYER_5_BLEND_OP);
		writel(0xFF, right_mixer_base + LAYER_5_BLEND0_FG_ALPHA);
	}

	mdp_blend_setup(rm, left_mixer_base, right_mixer_base);

	mdp_get_pipe_stage_level(rm, &left_staging_level, &right_staging_level);
	dprintf(CRITICAL, "left_stage_level=0x%x, right_stage_level=0x%x\n",
		left_staging_level, right_staging_level);

	/* border fill */
	left_staging_level |= BIT(24);
	right_staging_level |= BIT(24);

	/*
	 * Only one layer mixer is used when:
	 * 1. When ping-pong split is enabled and two pipes are used,
	 *    both the pipes need to be staged on the same layer mixer.
	 * 2. When shared display is enabled, while the dst resolution is small(width < 2560).
	 */
	if (pinfo->lcdc.dual_pipe && (pinfo->lcdc.dst_split || pinfo->lcdc.force_merge))
			left_staging_level |= right_staging_level;

	switch (pinfo->dest){
		case DISPLAY_2:
			if (display_1_rm->num_lm == 1) {
				writel(left_staging_level, left_ctl_base + CTL_LAYER_1);
				if (pinfo->lcdc.dual_pipe) {
					if (pinfo->lcdc.split_display)
						writel(right_staging_level, right_ctl_base + CTL_LAYER_2);
					else if (!pinfo->lcdc.force_merge)// single ctl, dual pipe
						writel(right_staging_level, left_ctl_base + CTL_LAYER_2);
				}
			} else if (display_1_rm->num_lm == 2) { //Disp1 used 2 pipes
				writel(left_staging_level, left_ctl_base + CTL_LAYER_2);
				if (pinfo->lcdc.dual_pipe) {
					if (pinfo->lcdc.split_display)
						writel(right_staging_level, right_ctl_base + CTL_LAYER_5);
					else  if (!pinfo->lcdc.force_merge)// single ctl, dual pipe
						writel(right_staging_level, left_ctl_base + CTL_LAYER_5);
				}
			}
			break;
		case DISPLAY_3:
			writel(left_staging_level, left_ctl_base + CTL_LAYER_2);
			if (pinfo->lcdc.dual_pipe && !pinfo->lcdc.force_merge)
				writel(right_staging_level, left_ctl_base + CTL_LAYER_5); /* 1 ctl, 2 layer mixer */
			break;
		case DISPLAY_1:
		default:
			writel(left_staging_level, left_ctl_base + CTL_LAYER_0);
			if (pinfo->lcdc.dual_pipe) {
				if (pinfo->lcdc.split_display)
					writel(right_staging_level, right_ctl_base + CTL_LAYER_1);
				else if (!pinfo->lcdc.force_merge) /* 1 ctl, 2 layer mixer */
					writel(right_staging_level, left_ctl_base + CTL_LAYER_1);
			}
			break;
	}

}

void mdss_fbc_cfg(struct msm_panel_info *pinfo)
{
	uint32_t mode = 0;
	uint32_t budget_ctl = 0;
	uint32_t lossy_mode = 0;
	struct fbc_panel_info *fbc;
	uint32_t enc_mode, width;

	fbc = &pinfo->fbc;

	if (!pinfo->fbc.enabled)
		return;

	/* enc_mode defines FBC version. 0 = FBC 1.0 and 1 = FBC 2.0 */
	enc_mode = (fbc->comp_ratio == 2) ? 0 : 1;

	width = pinfo->xres;
	if (enc_mode)
		width = (pinfo->xres/fbc->comp_ratio);

	if (pinfo->lcdc.split_display)
		width /= 2;

	mode = ((width) << 16) | ((fbc->slice_height) << 11) |
		((fbc->pred_mode) << 10) | (enc_mode) << 9 |
		((fbc->comp_mode) << 8) | ((fbc->qerr_enable) << 7) |
		((fbc->cd_bias) << 4) | ((fbc->pat_enable) << 3) |
		((fbc->vlc_enable) << 2) | ((fbc->bflc_enable) << 1) | 1;

	dprintf(SPEW, "width = %d, slice height = %d, pred_mode =%d, enc_mode = %d, \
			comp_mode %d, qerr_enable = %d, cd_bias = %d\n",
			width, fbc->slice_height, fbc->pred_mode, enc_mode,
			fbc->comp_mode, fbc->qerr_enable, fbc->cd_bias);
	dprintf(SPEW, "pat_enable %d, vlc_enable = %d, bflc_enable = %d\n",
			fbc->pat_enable, fbc->vlc_enable, fbc->bflc_enable);

	budget_ctl = ((fbc->line_x_budget) << 12) |
		((fbc->block_x_budget) << 8) | fbc->block_budget;

	lossy_mode = (((fbc->max_pred_err) << 28) | (fbc->lossless_mode_thd) << 16) |
		((fbc->lossy_mode_thd) << 8) |
		((fbc->lossy_rgb_thd) << 4) | fbc->lossy_mode_idx;

	dprintf(SPEW, "mode= 0x%x, budget_ctl = 0x%x, lossy_mode= 0x%x\n",
			mode, budget_ctl, lossy_mode);
	writel(mode, MDP_PP_0_BASE + MDSS_MDP_REG_PP_FBC_MODE);
	writel(budget_ctl, MDP_PP_0_BASE + MDSS_MDP_REG_PP_FBC_BUDGET_CTL);
	writel(lossy_mode, MDP_PP_0_BASE + MDSS_MDP_REG_PP_FBC_LOSSY_MODE);

	if (pinfo->mipi.dual_dsi) {
		writel(mode, MDP_PP_1_BASE + MDSS_MDP_REG_PP_FBC_MODE);
		writel(budget_ctl, MDP_PP_1_BASE +
				MDSS_MDP_REG_PP_FBC_BUDGET_CTL);
		writel(lossy_mode, MDP_PP_1_BASE +
				MDSS_MDP_REG_PP_FBC_LOSSY_MODE);
	}
}

void mdss_qos_remapper_setup(void)
{
	uint32_t mdp_hw_rev = readl(MDP_HW_REV);
	uint32_t map;

	if (MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev, MDSS_MDP_HW_REV_100) ||
		MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
						MDSS_MDP_HW_REV_102))
		map = 0xE9;
	else if (MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
			MDSS_MDP_HW_REV_101))
		map = 0xA5;
	else if (MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
			MDSS_MDP_HW_REV_106) ||
		 MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
			MDSS_MDP_HW_REV_108) ||
		 MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
			MDSS_MDP_HW_REV_111) ||
		 MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
			MDSS_MDP_HW_REV_112))
		map = 0xE4;
	else if (MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
			MDSS_MDP_HW_REV_105) ||
		 MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
			MDSS_MDP_HW_REV_109) ||
		 MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
			MDSS_MDP_HW_REV_107) ||
		 MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
			MDSS_MDP_HW_REV_110))
		map = 0xA4;
	else if (MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev,
			MDSS_MDP_HW_REV_103))
		map = 0xFA;
	else
		return;

	writel(map, MDP_QOS_REMAPPER_CLASS_0);
}

void mdss_vbif_qos_remapper_setup(struct msm_panel_info *pinfo)
{
	uint32_t mask, reg_val, i;
	uint32_t left_pipe_xin_id, right_pipe_xin_id;
	uint32_t mdp_hw_rev = readl(MDP_HW_REV);
	uint32_t vbif_qos[4] = {0, 0, 0, 0};
	uint32_t vbif_offset;

	mdp_select_pipe_xin_id(pinfo,
			&left_pipe_xin_id, &right_pipe_xin_id);

	if (MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev, MDSS_MDP_HW_REV_106) ||
		 MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev, MDSS_MDP_HW_REV_108) ||
		 MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev, MDSS_MDP_HW_REV_111) ||
		 MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev, MDSS_MDP_HW_REV_112)) {
		vbif_qos[0] = 2;
		vbif_qos[1] = 2;
		vbif_qos[2] = 2;
		vbif_qos[3] = 2;
	} else if (MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev, MDSS_MDP_HW_REV_105) ||
		 MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev, MDSS_MDP_HW_REV_109) ||
		 MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev, MDSS_MDP_HW_REV_107) ||
		 MDSS_IS_MAJOR_MINOR_MATCHING(mdp_hw_rev, MDSS_MDP_HW_REV_110)) {
		vbif_qos[0] = 1;
		vbif_qos[1] = 2;
		vbif_qos[2] = 2;
		vbif_qos[3] = 2;
	} else {
		return;
	}

	vbif_offset = mdss_mdp_vbif_qos_remap_get_offset();

	for (i = 0; i < 4; i++) {
		/* VBIF_VBIF_QOS_REMAP_00 */
		reg_val = readl(REG_MDP(vbif_offset) + i*4);
		mask = 0x3 << (left_pipe_xin_id * 2);
		reg_val &= ~(mask);
		reg_val |= vbif_qos[i] << (left_pipe_xin_id * 2);

		if (pinfo->lcdc.dual_pipe) {
			mask = 0x3 << (right_pipe_xin_id * 2);
			reg_val &= ~(mask);
			reg_val |= vbif_qos[i] << (right_pipe_xin_id * 2);
		}
		writel(reg_val, REG_MDP(vbif_offset) + i*4);
	}
}

static uint32_t mdss_mdp_ctl_out_sel(struct msm_panel_info *pinfo,
	int is_main_ctl)
{
	uint32_t mctl_intf_sel;
	uint32_t sctl_intf_sel;

	if ((pinfo->dest == DISPLAY_2) ||
		((pinfo->dest == DISPLAY_1) && (pinfo->lcdc.pipe_swap))) {
		mctl_intf_sel = BIT(4) | BIT(5); /* Interface 2 */
		sctl_intf_sel = BIT(5); /* Interface 1 */
	} else {
		mctl_intf_sel = BIT(5); /* Interface 1 */
		sctl_intf_sel = BIT(4) | BIT(5); /* Interface 2 */
	}
	dprintf(SPEW, "%s: main ctl dest=%s sec ctl dest=%s\n", __func__,
		(mctl_intf_sel & BIT(4)) ? "Intf2" : "Intf1",
		(sctl_intf_sel & BIT(4)) ? "Intf2" : "Intf1");
	return is_main_ctl ? mctl_intf_sel : sctl_intf_sel;
}

static void mdp_set_intf_base(struct msm_panel_info *pinfo,
	uint32_t *intf_sel, uint32_t *sintf_sel,
	uint32_t *intf_base, uint32_t *sintf_base)
{
	if ((pinfo->dest == DISPLAY_2) && (!pinfo->mipi.dual_dsi)) {
		*intf_sel = BIT(16);
		*sintf_sel = BIT(8);
		*intf_base = MDP_INTF_2_BASE + mdss_mdp_intf_offset();
		*sintf_base = MDP_INTF_1_BASE + mdss_mdp_intf_offset();
	} else {
		*intf_sel = BIT(8);
		*sintf_sel = BIT(16);
		*intf_base = MDP_INTF_1_BASE + mdss_mdp_intf_offset();
		*sintf_base = MDP_INTF_2_BASE + mdss_mdp_intf_offset();
	}
	dprintf(SPEW, "%s: main intf=%s, sec intf=%s\n", __func__,
		(pinfo->dest == DISPLAY_2) ? "Intf2" : "Intf1",
		(pinfo->dest == DISPLAY_2) ? "Intf1" : "Intf2");
}

int mdp_dsi_video_config(struct msm_panel_info *pinfo,
		struct fbcon_config *fb, uint32_t fb_cnt)
{
	uint32_t intf_sel, sintf_sel, old_intf_sel;
	uint32_t intf_base, sintf_base;
	uint32_t left_pipe, right_pipe;
	uint32_t reg;
	int i = 0;
	bool use_second_dsi = false;
	struct resource_req *rm = NULL;

	dprintf(INFO, "%s:destdisplay=%d\n", __func__, pinfo->dest);
	// scan the display resource to see any display is already using DSI0
	for (i = 0; i < MAX_NUM_DISPLAY; i++) {
		if (i == (int)(pinfo->dest - DISPLAY_1))
			continue;
		rm = mdp_rm_retrieve_resource(i + DISPLAY_1);
		if (!rm)
			continue;
		if (rm->primary_dsi) {
			use_second_dsi = true;
			break;
		}
	}

	// update resource requirement per config and then retrieve it
	mdp_rm_update_resource(pinfo, use_second_dsi);

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
		return ERR_NOT_VALID;
	}

	if ((use_second_dsi && !pinfo->lcdc.pipe_swap) ||
		(!use_second_dsi && pinfo->lcdc.pipe_swap)) {
		intf_base = MDP_INTF_2_BASE + mdss_mdp_intf_offset();
		sintf_base = MDP_INTF_1_BASE + mdss_mdp_intf_offset();
	} else {
		intf_base = MDP_INTF_1_BASE + mdss_mdp_intf_offset();
		sintf_base = MDP_INTF_2_BASE + mdss_mdp_intf_offset();
	}
	mdss_intf_tg_setup(pinfo, intf_base);
	mdss_intf_fetch_start_config(pinfo, intf_base);
	if (pinfo->lcdc.split_display) {
		mdss_intf_tg_setup(pinfo, sintf_base);
		mdss_intf_fetch_start_config(pinfo, sintf_base);
	}

	mdp_clk_gating_ctrl();

	mdp_setup_pipe(pinfo, fb, fb_cnt,
		&left_pipe, &right_pipe);
	dprintf(INFO, "%s:fb_cnt %d, left_pipe=0x%x, right_pipe=0x%x\n",
		__func__, fb_cnt, left_pipe, right_pipe);

	mdss_vbif_setup();
	if (!has_fixed_size_smp())
		mdss_smp_setup(pinfo, left_pipe, right_pipe);

	mdss_qos_remapper_setup();
	mdss_vbif_qos_remapper_setup(pinfo);

#if ENABLE_QSEED_SCALAR
	if (fb->layer_scale) {
		/* update the scale setting */
		mdp_scalar_config(fb->layer_scale, pinfo, left_pipe, right_pipe);
	}
#endif

	mdss_layer_mixer_setup(fb, pinfo);

	if ((use_second_dsi && !pinfo->lcdc.pipe_swap) ||
		(!use_second_dsi && pinfo->lcdc.pipe_swap)) {
		reg = 0x1f00 | BIT(4) | BIT(5); /* Interface 2 */
	} else {
		reg = 0x1f00 | BIT(5); /* Interface 1 */
	}

	/* enable 3D mux for dual_pipe but single interface config */
	if (pinfo->lcdc.dual_pipe && !pinfo->mipi.dual_dsi &&
		!pinfo->lcdc.split_display && !pinfo->lcdc.force_merge) {

		if (pinfo->num_dsc_enc != 2)
			reg |= BIT(19) | BIT(20);
	}

	writel(reg, rm->ctl_base[0] + CTL_TOP);

	if (pinfo->lcdc.split_display) {
		if ((use_second_dsi && !pinfo->lcdc.pipe_swap) ||
			(!use_second_dsi && pinfo->lcdc.pipe_swap)) {
			reg = 0x1f00 | BIT(5); /* Interface 1 */
		} else {
			reg = 0x1f00 | BIT(4) | BIT(5); /* Interface 2 */
		}

		writel(reg, rm->ctl_base[1] + CTL_TOP);
	}

	if ((pinfo->compression_mode == COMPRESSION_DSC) &&
		pinfo->dsc.mdp_dsc_config) {
		struct dsc_desc *dsc = &pinfo->dsc;

		if (pinfo->lcdc.dual_pipe && !pinfo->mipi.dual_dsi &&
		    !pinfo->lcdc.split_display && (pinfo->num_dsc_enc == 2)) {
			dsc->mdp_dsc_config(pinfo, MDP_PP_0_BASE,
				MDP_DSC_0_BASE, true, true);
			dsc->mdp_dsc_config(pinfo, MDP_PP_1_BASE,
				MDP_DSC_1_BASE, true, true);

		} else if (pinfo->lcdc.dual_pipe && pinfo->mipi.dual_dsi &&
			pinfo->lcdc.split_display && (pinfo->num_dsc_enc == 1)) {
			dsc->mdp_dsc_config(pinfo, MDP_PP_0_BASE,
				MDP_DSC_0_BASE, false, false);
			dsc->mdp_dsc_config(pinfo, MDP_PP_1_BASE,
				MDP_DSC_1_BASE, false, false);

		} else {
			dsc->mdp_dsc_config(pinfo, MDP_PP_0_BASE,
				MDP_DSC_0_BASE, false, false);
		}
	} else if (pinfo->compression_mode == COMPRESSION_FBC) {
		if (pinfo->fbc.enabled)
			mdss_fbc_cfg(pinfo);
	}

	/*
	 * if dst_split is enabled, intf 1 & 2 needs to be enabled but
	 * CTL_1 path should not be set since CTL_0 itself is going
	 * to split after DSPP block and drive both intf.
	 */
	if ((use_second_dsi && !pinfo->lcdc.pipe_swap) ||
		(!use_second_dsi && pinfo->lcdc.pipe_swap)) {
		intf_sel = BIT(16);
		sintf_sel = BIT(8);
	} else {
		intf_sel = BIT(8);
		sintf_sel = BIT(16);
	}

	if (pinfo->mipi.dual_dsi) {
		if ((pinfo->dest == DISPLAY_2) || (pinfo->lcdc.split_display))
			intf_sel |= sintf_sel; /* INTF 2 enable */
	}

	old_intf_sel = readl(MDP_DISP_INTF_SEL);
	writel(old_intf_sel | intf_sel, MDP_DISP_INTF_SEL);

	writel(0x1111, MDP_VIDEO_INTF_UNDERFLOW_CTL);
	writel(0x01, MDP_UPPER_NEW_ROI_PRIOR_RO_START);
	writel(0x01, MDP_LOWER_NEW_ROI_PRIOR_TO_START);
	return 0;
}

int mdp_edp_config(struct msm_panel_info *pinfo, struct fbcon_config *fb)
{
	uint32_t left_pipe, right_pipe;

	mdss_intf_tg_setup(pinfo, MDP_INTF_0_BASE);

	mdp_clk_gating_ctrl();

	mdss_vbif_setup();

	mdp_setup_pipe(pinfo, fb, 1, &left_pipe, &right_pipe);

	mdss_smp_setup(pinfo, left_pipe, right_pipe);

	mdss_qos_remapper_setup();
	mdss_vbif_qos_remapper_setup(pinfo);

	mdss_layer_mixer_setup(fb, pinfo);

	if (pinfo->lcdc.dual_pipe)
		writel(0x181F10, MDP_CTL_0_BASE + CTL_TOP);
	else
		writel(0x1F10, MDP_CTL_0_BASE + CTL_TOP);

	writel(0x9, MDP_DISP_INTF_SEL);
	writel(0x1111, MDP_VIDEO_INTF_UNDERFLOW_CTL);
	writel(0x01, MDP_UPPER_NEW_ROI_PRIOR_RO_START);
	writel(0x01, MDP_LOWER_NEW_ROI_PRIOR_TO_START);

	return 0;
}

int mdp_config_external_pipe(struct msm_panel_info *pinfo,
	struct fbcon_config *fb, char *actual_pipe_name)
{
	struct resource_req *rm = NULL;
	bool pipe_on_right = false;
	int ret = NO_ERROR;
	uint32_t pipe_base = 0, pp_index = 0;
	uint32_t search_start = 0;
	uint32_t pipe_type;

	/* When entering this function, FB will only be configured as transparent. */
	if (fb->base != NULL || !target_format_is_valid(fb->format))
		return ERR_NOT_VALID;

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s:get hardware resource failed\n", __func__);
		return ERR_NOT_VALID;
	}

	ret = mdp_acquire_pipe(pinfo->dest, fb, &pipe_base,
		pipe_on_right, actual_pipe_name);
	if (ret) {
		dprintf(CRITICAL, "acquire %s pipe failed\n", actual_pipe_name);
		return ret;
	}

	if (target_is_yuv_format(fb->format))
		pipe_type = MDSS_MDP_PIPE_TYPE_VIG;
	else
		pipe_type = MDSS_MDP_PIPE_TYPE_RGB;

	if (pipe_base != _mdp_get_pipe(rm, search_start,
		fb->z_order, pipe_type, &pp_index)) {
		dprintf(CRITICAL, "pipe_base is not the same, aborted\n");
		ret = ERR_NOT_VALID;
		return ret;
	}

	/* set layer to be transparent */
	mdss_mdp_set_layer_transparent(pinfo, pipe_base,
		target_is_yuv_format(fb->format));

	mdp_rm_update_pipe_pending_mask(rm, pp_index);

	return ret;
}

int mdp_config_pipe(struct msm_panel_info *pinfo,
	struct fbcon_config *fb, uint32_t fb_cnt)
{
	struct resource_req *rm = NULL;
	uint32_t fb_index = SPLIT_DISPLAY_0;
	uint32_t pipe_base = 0, pipe_type;
	uint32_t real_fb_cnt = 0;
	uint32_t pp_index = 0, search_start = 0;
	struct border_rect rect;

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s:get hardware resource failed\n", __func__);
		return ERR_NOT_VALID;
	}

	/*
	* In dual pipe & non-splitter case like 4K, as each fb will use 2 pipes,
	* so fb_cnt * 2 should not exceed the max value of blend stage.
	*/
	real_fb_cnt = fb_cnt;
	if (pinfo->lcdc.dual_pipe & !pinfo->splitter_is_enabled) {
		if (fb_cnt * MAX_SPLIT_DISPLAY > MDP_STAGE_6)
			dprintf(CRITICAL, "invalid user fb number\n");
			real_fb_cnt = MDP_STAGE_6 / MAX_SPLIT_DISPLAY;
	}

	for (fb_index = SPLIT_DISPLAY_0; fb_index < real_fb_cnt; fb_index++) {
		search_start = 0;

		/* If one fb's format is invalid, continue for next fb */
		if (!target_format_is_valid(fb[fb_index].format))
			continue;

		if (target_is_yuv_format(fb[fb_index].format))
			pipe_type = MDSS_MDP_PIPE_TYPE_VIG;
		else
			pipe_type = MDSS_MDP_PIPE_TYPE_RGB;

		pipe_base = _mdp_get_pipe(rm, search_start,
			fb[fb_index].z_order, pipe_type, &pp_index);

		/* continue for next fb if pipe_base is 0 */
		if (pipe_base == 0)
			continue;

		if ((fb[fb_index].base == NULL) &&
			target_format_is_valid(fb[fb_index].format)) {
			mdss_mdp_set_layer_transparent(pinfo, pipe_base,
				target_is_yuv_format(fb[fb_index].format));
		} else {
			_mdp_fill_border_rect(pinfo->border_left[fb_index],
					pinfo->border_right[fb_index],
					pinfo->border_top[fb_index],
					pinfo->border_bottom[fb_index],
					&rect);

			mdss_source_pipe_config(&fb[fb_index],
				pinfo, pipe_base, &rect, external_fb_offset);
		}

		mdp_rm_update_pipe_pending_mask(rm, pp_index);
		/*
		 * For dual pipe & non-splitter case fetching from the same FB,
		 * there are two pipes allocated in resource manager. So when searching
		 * it, it's needed to exclude the the former pipe who has the same zorder
		 * and pipe_type. So the search start will start from actual position + 1.
		 */
		 search_start = pp_index + 1;
		if (pinfo->lcdc.dual_pipe & !pinfo->splitter_is_enabled) {
			pipe_base = _mdp_get_pipe(rm, search_start,
				fb[fb_index].z_order, pipe_type, &pp_index);

			/* continue for next fb if pipe_base is 0 */
			if (pipe_base == 0)
				continue;

			if ((fb[fb_index].base == NULL) &&
				target_format_is_valid(fb[fb_index].format)) {
				mdss_mdp_set_layer_transparent(pinfo, pipe_base,
					target_is_yuv_format(fb[fb_index].format));
			} else {
				_mdp_fill_border_rect(pinfo->border_left[fb_index],
						pinfo->border_right[fb_index],
						pinfo->border_top[fb_index],
						pinfo->border_bottom[fb_index],
						&rect);

				mdss_source_pipe_config(&fb[fb_index],
					pinfo, pipe_base, &rect, external_fb_offset);
			}

			mdp_rm_update_pipe_pending_mask(rm, pp_index);

			fb_index++;
		}
	}

	return NO_ERROR;
}

int mdp_trigger_flush(struct msm_panel_info *pinfo,
	struct fbcon_config *fb, uint32_t fb_cnt,
	uint32_t *left_ctl_reg_mask, uint32_t *right_ctl_reg_mask)
{
	uint32_t flush_right = true;
	uint32_t left_mask = 0, right_mask = 0;
	struct resource_req *res_mgr = NULL;

	res_mgr = mdp_rm_retrieve_resource(pinfo->dest);
	if (!res_mgr) {
		dprintf(CRITICAL, "Get hardware resource failed\n");
		return ERR_NOT_VALID;
	}

	if (pinfo->splitter_is_enabled &&
		(fb[SPLIT_DISPLAY_1].base == NULL) &&
		!target_format_is_valid(fb[SPLIT_DISPLAY_1].format))
		flush_right = false;

	mdss_mdp_flush_pipe(res_mgr, &left_mask,
		&right_mask, flush_right);

	_mdp_trigger_flush(res_mgr, left_mask, right_mask);

	mdp_rm_clear_pipe_mask(res_mgr);

	*left_ctl_reg_mask = left_mask;
	*right_ctl_reg_mask = right_mask;

	return NO_ERROR;
}

int mdss_hdmi_config(struct msm_panel_info *pinfo,
	struct fbcon_config *fb, uint32_t fb_cnt)
{
	uint32_t left_pipe = 0, right_pipe = 0, out_size;
	uint32_t old_intf_sel, prg_fetch_start_en;
	struct resource_req *rm = NULL;

	dprintf(INFO, "%s:destdisplay=%d\n", __func__, pinfo->dest);

	/* update resource manager per config and retrieve it next */
	mdp_rm_update_resource(pinfo, false);

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "Get hardware resource failed\n");
		return ERR_NOT_VALID;
	}

	mdss_intf_tg_setup(pinfo, MDP_INTF_3_BASE + mdss_mdp_intf_offset());
	mdss_intf_fetch_start_config(pinfo, MDP_INTF_3_BASE + mdss_mdp_intf_offset());

	mdp_clk_gating_ctrl();
	mdss_vbif_setup();

	mdp_setup_pipe(pinfo, fb, fb_cnt,
		&left_pipe, &right_pipe);

	mdss_smp_setup(pinfo, left_pipe, right_pipe);

	mdss_qos_remapper_setup();

#if ENABLE_QSEED_SCALAR
	if (fb[SPLIT_DISPLAY_0].layer_scale) {
		// update the scale setting
		mdp_scalar_config(fb->layer_scale, pinfo, left_pipe, right_pipe);
	}
#endif

	mdss_layer_mixer_setup(fb, pinfo);

	if (pinfo->lcdc.dual_pipe && !pinfo->lcdc.force_merge)
		writel(0x181F40, rm->ctl_base[0] + CTL_TOP);
	else
		writel(0x1F40, rm->ctl_base[0] + CTL_TOP);

	old_intf_sel = readl(MDP_DISP_INTF_SEL);

	writel(old_intf_sel | BIT(24) | BIT(25), MDP_DISP_INTF_SEL);
	writel(0x11111, MDP_VIDEO_INTF_UNDERFLOW_CTL);
	writel(0x01, MDP_UPPER_NEW_ROI_PRIOR_RO_START);
	writel(0x01, MDP_LOWER_NEW_ROI_PRIOR_TO_START);

	/**
	 * Program the CDM hardware block in HDMI bypass mode, and enable
	 * the HDMI packer.
	 */
	writel(0x01, CDM_HDMI_PACK_OP_MODE);
	writel(0x00, MDP_OUT_CTL_0);
	prg_fetch_start_en = readl(MDP_INTF_3_INTF_CONFIG);
	/* Preserve PROG_FETCH_START_EN bit */
	prg_fetch_start_en &= BIT(31);
	writel(prg_fetch_start_en | 0x00, MDP_INTF_3_INTF_CONFIG);
	out_size = (pinfo->xres & 0xFFFF) | ((pinfo->yres & 0xFFFF) << 16);
	writel(out_size, CDM_CDWN2_OUT_SIZE);
	writel(0x80, CDM_CDWN2_OP_MODE);
	writel(0x3FF0000, CDM_CDWN2_CLAMP_OUT);
	writel(0x0, CDM_CSC_10_OP_MODE);

	return 0;
}

int mdp_dsi_cmd_config(struct msm_panel_info *pinfo,
                struct fbcon_config *fb)
{
	uint32_t intf_sel, sintf_sel;
	uint32_t intf_base, sintf_base;
	uint32_t reg;
	int ret = NO_ERROR;
	uint32_t left_pipe, right_pipe;

	struct lcdc_panel_info *lcdc = NULL;

	if (pinfo == NULL)
		return ERR_INVALID_ARGS;

	lcdc =  &(pinfo->lcdc);
	if (lcdc == NULL)
		return ERR_INVALID_ARGS;

	mdp_set_intf_base(pinfo, &intf_sel, &sintf_sel, &intf_base, &sintf_base);

	if (pinfo->lcdc.split_display) {
		reg = BIT(1); /* Command mode */
		if (pinfo->lcdc.dst_split)
			reg |= BIT(2); /* Enable SMART_PANEL_FREE_RUN mode */
		if (pinfo->lcdc.pipe_swap)
			reg |= BIT(4); /* Use intf2 as trigger */
		else
			reg |= BIT(8); /* Use intf1 as trigger */
		writel(reg, MDP_REG_SPLIT_DISPLAY_UPPER_PIPE_CTL);
		writel(reg, MDP_REG_SPLIT_DISPLAY_LOWER_PIPE_CTL);
		writel(0x1, MDP_REG_SPLIT_DISPLAY_EN);
	}

	if (pinfo->lcdc.dst_split) {
		uint32_t ppb_offset = mdss_mdp_get_ppb_offset();
		writel(BIT(5), REG_MDP(ppb_offset)); /* MMSS_MDP_PPB0_CNTL */
		writel(BIT(16) | (0x3 << 20), REG_MDP(ppb_offset + 0x4)); /* MMSS_MDP_PPB0_CONFIG */
	}

	mdp_clk_gating_ctrl();

	if (pinfo->mipi.dual_dsi)
		intf_sel |= sintf_sel; /* INTF 2 enable */

	writel(intf_sel, MDP_DISP_INTF_SEL);

	mdp_setup_pipe(pinfo, fb, 1, &left_pipe, &right_pipe);

	mdss_vbif_setup();
	if (!has_fixed_size_smp())
		mdss_smp_setup(pinfo, left_pipe, right_pipe);
	mdss_qos_remapper_setup();
	mdss_vbif_qos_remapper_setup(pinfo);

	mdss_layer_mixer_setup(fb, pinfo);

	writel(0x213F, MDP_PANEL_FORMAT + intf_base);
	reg = 0x21f00 | mdss_mdp_ctl_out_sel(pinfo, 1);

	/* enable 3D mux for dual_pipe but single interface config */
	if (pinfo->lcdc.dual_pipe && !pinfo->mipi.dual_dsi &&
		!pinfo->lcdc.split_display) {

		if (pinfo->num_dsc_enc != 2)
			reg |= BIT(19) | BIT(20);
	}

	writel(reg, MDP_CTL_0_BASE + CTL_TOP);

	if ((pinfo->compression_mode == COMPRESSION_DSC) &&
	    pinfo->dsc.mdp_dsc_config) {
		struct dsc_desc *dsc = &pinfo->dsc;

		if (pinfo->lcdc.dual_pipe && !pinfo->mipi.dual_dsi &&
		    !pinfo->lcdc.split_display && (pinfo->num_dsc_enc == 2)) {

			dsc->mdp_dsc_config(pinfo, MDP_PP_0_BASE,
				MDP_DSC_0_BASE, true, true);
			dsc->mdp_dsc_config(pinfo, MDP_PP_1_BASE,
				MDP_DSC_1_BASE, true, true);
		} else {
			dsc->mdp_dsc_config(pinfo, MDP_PP_0_BASE,
				MDP_DSC_0_BASE, false, false);
		}
	} else if (pinfo->compression_mode == COMPRESSION_FBC) {
		if (pinfo->fbc.enabled)
			mdss_fbc_cfg(pinfo);
	}

	if (pinfo->mipi.dual_dsi) {
		writel(0x213F, sintf_base + MDP_PANEL_FORMAT);
		if (!pinfo->lcdc.dst_split) {
			reg = 0x21f00 | mdss_mdp_ctl_out_sel(pinfo, 0);
			writel(reg, MDP_CTL_1_BASE + CTL_TOP);
		}
	}

	return ret;
}

int mdp_dsi_video_on(struct msm_panel_info *pinfo)
{
	uint32_t left_ctl_mask = 0, right_ctl_mask = 0;
	struct resource_req *rm = NULL;

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
		return ERR_NOT_VALID;
	}

	mdss_mdp_set_flush(pinfo, &left_ctl_mask, &right_ctl_mask);
	dprintf(SPEW, "dsi video_on Disp%d flush ctl0:0x%x  ctl1:0x%x\n",
			pinfo->dest, left_ctl_mask, right_ctl_mask);

	_mdp_trigger_flush(rm, left_ctl_mask, right_ctl_mask);

	// clear and disable DSI interrupts
	writel(DSI_ERR_INT_RESET_STATUS, DSI0_ERR_INT_MASK);
	writel(DSI_INT_CTRL_RESET_STATUS, DSI0_INT_CTRL);
	writel(DSI_ERR_INT_RESET_STATUS, DSI1_ERR_INT_MASK);
	writel(DSI_INT_CTRL_RESET_STATUS, DSI1_INT_CTRL);

	if (rm->primary_dsi) {
		writel(0x1, MDP_INTF_1_TIMING_ENGINE_EN + mdss_mdp_intf_offset());
	} else {
		writel(0x01, MDP_INTF_2_TIMING_ENGINE_EN + mdss_mdp_intf_offset());
	}

	if (rm->num_ctl == 2) {
		writel(0x01, MDP_INTF_2_TIMING_ENGINE_EN + mdss_mdp_intf_offset());
	}

	return NO_ERROR;
}

int mdp_dsi_video_update(struct msm_panel_info *pinfo)
{
	uint32_t left_ctl_reg_mask = 0, right_ctl_reg_mask = 0;
	struct resource_req *rm = NULL;

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
		return ERR_NOT_VALID;
	}

	mdss_mdp_set_flush(pinfo, &left_ctl_reg_mask,
		&right_ctl_reg_mask);

	_mdp_trigger_flush(rm, left_ctl_reg_mask,
		right_ctl_reg_mask);

	return NO_ERROR;
}

int mdp_update_pipe(struct msm_panel_info *pinfo,
	struct fbcon_config *fb, uint32_t fb_cnt)
{
	uint32_t left_ctl_reg_mask = 0, right_ctl_reg_mask = 0;
	uint32_t pipe_base = 0, pp_index = 0;
	struct resource_req *rm = NULL;
	bool flush_right = true;
	uint32_t fb_index = 0, search_start = 0;
	uint32_t pipe_type;
	static int cnt = 0;

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
		return ERR_NOT_VALID;
	}

	for (fb_index = SPLIT_DISPLAY_0; fb_index < fb_cnt; fb_index++) {
		search_start = 0;

		if (cnt++ < 6)
			dprintf(INFO, "fb%d: zorder=%d, format=%d, base=0x%x\n",
			fb_index, fb[fb_index].z_order, fb[fb_index].format,
			fb[fb_index].base == NULL ? 0 : (uint32_t)fb[fb_index].base);

		/* When calling this function, fb should be filled by user correctly. */
		if ((fb[fb_index].base == NULL) ||
			!target_format_is_valid(fb[fb_index].format))
			continue;

		if (target_is_yuv_format(fb[fb_index].format))
			pipe_type = MDSS_MDP_PIPE_TYPE_VIG;
		else
			pipe_type = MDSS_MDP_PIPE_TYPE_RGB;

		pipe_base = _mdp_get_pipe(rm, search_start,
					fb[fb_index].z_order,
					pipe_type, &pp_index);

		writel((uint32_t)fb[fb_index].base,
			pipe_base + PIPE_SSPP_SRC0_ADDR);

		mdp_rm_update_pipe_pending_mask(rm, pp_index);

		if (pinfo->lcdc.split_display) {
			/* In split case, right pipe will stage on another ctl */
			search_start = pp_index + 1;

			pipe_base = _mdp_get_pipe(rm, search_start,
						fb[fb_index].z_order,
						pipe_type, &pp_index);

			writel((uint32_t)fb[fb_index].base,
				pipe_base + PIPE_SSPP_SRC0_ADDR);

			mdp_rm_update_pipe_pending_mask(rm, pp_index);

			fb_index++;
		}
	}

	mdss_mdp_flush_pipe(rm, &left_ctl_reg_mask,
		&right_ctl_reg_mask, flush_right);

	mdp_rm_clear_pipe_mask(rm);

	_mdp_trigger_flush(rm, left_ctl_reg_mask,
		right_ctl_reg_mask);

	return 0;
}

int mdp_dsi_video_off(struct msm_panel_info *pinfo)
{
	uint32_t timing_engine_en;

	if (pinfo->dest == DISPLAY_1)
		timing_engine_en = MDP_INTF_1_TIMING_ENGINE_EN;
	else
		timing_engine_en = MDP_INTF_2_TIMING_ENGINE_EN;

	if(!target_cont_splash_screen())
	{
		writel(0x00000000, timing_engine_en + mdss_mdp_intf_offset());
		mdelay(60);
		/* Ping-Pong done Tear Check Read/Write  */
		/* Underrun(Interface 0/1/2/3) VSYNC Interrupt Enable  */
		writel(0xFF777713, MDP_INTR_CLEAR);
	}

	writel(0x00000000, MDP_INTR_EN);

	return NO_ERROR;
}

int mdp_dsi_cmd_off()
{
	if(!target_cont_splash_screen())
	{
		/* Ping-Pong done Tear Check Read/Write  */
		/* Underrun(Interface 0/1/2/3) VSYNC Interrupt Enable  */
		writel(0xFF777713, MDP_INTR_CLEAR);
	}
	writel(0x00000000, MDP_INTR_EN);

	return NO_ERROR;
}

static void mdp_set_cmd_autorefresh_mode(struct msm_panel_info *pinfo)
{
	uint32_t total_lines = 0, vclks_line = 0, cfg = 0;

	if (!pinfo || (pinfo->type != MIPI_CMD_PANEL) ||
				!pinfo->autorefresh_enable)
		return;

	total_lines = pinfo->lcdc.v_front_porch +
			pinfo->lcdc.v_back_porch +
			pinfo->lcdc.v_pulse_width +
			pinfo->border_top[SPLIT_DISPLAY_0] + pinfo->border_bottom[SPLIT_DISPLAY_0] +
			pinfo->yres;
	total_lines *= pinfo->mipi.frame_rate;

	vclks_line = (total_lines) ? 19200000 / total_lines : 0;
	vclks_line = vclks_line * pinfo->mipi.frame_rate * 100 / 6000;

	cfg = BIT(19) | vclks_line;

	/* Configure tearcheck VSYNC param */
	writel(cfg, MDP_REG_PP_0_SYNC_CONFIG_VSYNC);
	if (pinfo->lcdc.dst_split)
		writel(cfg, MDP_REG_PP_SLAVE_SYNC_CONFIG_VSYNC);
	if (pinfo->lcdc.dual_pipe)
		writel(cfg, MDP_REG_PP_1_SYNC_CONFIG_VSYNC);
	dsb();

	/* Enable autorefresh mode */
	writel((BIT(31) | pinfo->autorefresh_framenum),
			MDP_REG_PP_0_AUTOREFRESH_CONFIG);
	if (pinfo->lcdc.dst_split)
		writel((BIT(31) | pinfo->autorefresh_framenum),
			MDP_REG_PP_SLAVE_AUTOREFRESH_CONFIG);
	if (pinfo->lcdc.dual_pipe)
		writel((BIT(31) | pinfo->autorefresh_framenum),
			MDP_REG_PP_1_AUTOREFRESH_CONFIG);
	dsb();
}

int mdp_dma_on(struct msm_panel_info *pinfo)
{
	uint32_t ctl0_reg_val, ctl1_reg_val;
	mdss_mdp_set_flush(pinfo, &ctl0_reg_val, &ctl1_reg_val);
	writel(ctl0_reg_val, MDP_CTL_0_BASE + CTL_FLUSH);
	if (pinfo->lcdc.dual_pipe && !pinfo->lcdc.dst_split)
		writel(ctl1_reg_val, MDP_CTL_1_BASE + CTL_FLUSH);

	if (pinfo->autorefresh_enable)
		mdp_set_cmd_autorefresh_mode(pinfo);
	writel(0x01, MDP_CTL_0_BASE + CTL_START);

	return NO_ERROR;
}

int mdp_edp_on(struct msm_panel_info *pinfo)
{
	uint32_t ctl0_reg_val, ctl1_reg_val;
	mdss_mdp_set_flush(pinfo, &ctl0_reg_val, &ctl1_reg_val);
	writel(ctl0_reg_val, MDP_CTL_0_BASE + CTL_FLUSH);
	writel(0x01, MDP_INTF_0_TIMING_ENGINE_EN  + mdss_mdp_intf_offset());
	return NO_ERROR;
}

int mdss_hdmi_on(struct msm_panel_info *pinfo)
{
	uint32_t left_ctl_flush_mask = 0, right_ctl_flush_mask = 0;
	struct resource_req *rm = NULL;

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
		return ERR_NOT_VALID;
	}

	mdss_mdp_set_flush(pinfo, &left_ctl_flush_mask, &right_ctl_flush_mask);
	left_ctl_flush_mask &= 0x0FFFFFFF;  // remove the interface setting
	right_ctl_flush_mask |= BIT(28);     // enable interface 3

	writel(left_ctl_flush_mask, rm->ctl_base[SPLIT_DISPLAY_0] + CTL_FLUSH);
	writel(0x01, MDP_INTF_3_TIMING_ENGINE_EN + mdss_mdp_intf_offset());

	return NO_ERROR;
}

int mdss_hdmi_update(struct msm_panel_info *pinfo)
{
	uint32_t left_ctl_reg_mask = 0, right_ctl_reg_mask = 0;
	struct resource_req *rm = NULL;

	rm = mdp_rm_retrieve_resource(pinfo->dest);
	if (!rm) {
		dprintf(CRITICAL, "%s: get hardware resource failed\n", __func__);
		return ERR_NOT_VALID;
	}

	mdss_mdp_set_flush(pinfo, &left_ctl_reg_mask, &right_ctl_reg_mask);
	left_ctl_reg_mask &= 0x0FFFFFFF;  // remove the interface setting
	right_ctl_reg_mask |= BIT(28);     // enable interface 3

	_mdp_trigger_flush(rm, left_ctl_reg_mask, right_ctl_reg_mask);

	return NO_ERROR;
}

int mdss_hdmi_off(struct msm_panel_info *pinfo)
{
	if(!target_cont_splash_screen())
	{
		writel(0x00000000, MDP_INTF_3_TIMING_ENGINE_EN + mdss_mdp_intf_offset());
		mdelay(60);
		/* Underrun(Interface 0/1/2/3) VSYNC Interrupt Enable  */
		writel(0xFF777713, MDP_INTR_CLEAR);
	}

	writel(0x00000000, MDP_INTR_EN);

	return NO_ERROR;
}

int mdp_edp_off(void)
{
	if (!target_cont_splash_screen()) {

		writel(0x00000000, MDP_INTF_0_TIMING_ENGINE_EN +
				mdss_mdp_intf_offset());
		mdelay(60);
		/* Ping-Pong done Tear Check Read/Write  */
		/* Underrun(Interface 0/1/2/3) VSYNC Interrupt Enable  */
		writel(0xFF777713, MDP_INTR_CLEAR);
		writel(0x00000000, MDP_INTR_EN);
	}

	writel(0x00000000, MDP_INTR_EN);

	return NO_ERROR;
}
