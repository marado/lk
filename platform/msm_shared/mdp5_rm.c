/*
Copyright (c) 2012-2016,2018-2019, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.
* Neither the name of The Linux Foundation nor the names of its
contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <mdp5_rm.h>
#include <debug.h>
#include <msm_panel.h>
#include <platform/iomap.h>
#include <err.h>
#include <string.h>

static struct resource_req display_req[MAX_NUM_DISPLAY];
static bool ctl_lm_allocated;

static struct source_pipe pipe_req[] = {
	{1 << MDSS_MDP_PIPE_TYPE_RGB, MDP_VP_0_RGB_0_BASE, false, MAX_NUM_DISPLAY, "rgb0"},
	{1 << MDSS_MDP_PIPE_TYPE_RGB, MDP_VP_0_RGB_1_BASE, false, MAX_NUM_DISPLAY, "rgb1"},
	{1 << MDSS_MDP_PIPE_TYPE_RGB, MDP_VP_0_RGB_2_BASE, false, MAX_NUM_DISPLAY, "rgb2"},
	{1 << MDSS_MDP_PIPE_TYPE_RGB, MDP_VP_0_RGB_3_BASE, false, MAX_NUM_DISPLAY, "rgb3"},
	{1 << MDSS_MDP_PIPE_TYPE_RGB, MDP_VP_0_DMA_0_BASE, false, MAX_NUM_DISPLAY, "dma0"},
	{1 << MDSS_MDP_PIPE_TYPE_RGB, MDP_VP_0_DMA_1_BASE, false, MAX_NUM_DISPLAY, "dma1"},
	{1 << MDSS_MDP_PIPE_TYPE_RGB | 1 << MDSS_MDP_PIPE_TYPE_VIG, MDP_VP_0_VIG_0_BASE,
		false, MAX_NUM_DISPLAY, "vig0"},
	{1 << MDSS_MDP_PIPE_TYPE_RGB | 1 << MDSS_MDP_PIPE_TYPE_VIG, MDP_VP_0_VIG_1_BASE,
		false, MAX_NUM_DISPLAY, "vig1"},
	{1 << MDSS_MDP_PIPE_TYPE_RGB | 1 << MDSS_MDP_PIPE_TYPE_VIG, MDP_VP_0_VIG_2_BASE,
		false, MAX_NUM_DISPLAY, "vig2"},
	{1 << MDSS_MDP_PIPE_TYPE_RGB | 1 << MDSS_MDP_PIPE_TYPE_VIG, MDP_VP_0_VIG_3_BASE,
		false, MAX_NUM_DISPLAY, "vig3"},
};

static void _mdp_rm_init(void)
{
	uint32_t i = 0, j = 0;

	for (i = 0; i < MAX_NUM_DISPLAY; i++) {
		if (!ctl_lm_allocated) {
			display_req[i].num_lm = 0;
			display_req[i].num_ctl= 0;
			display_req[i].needs_split_display = 0;
			display_req[i].primary_dsi= 0;

			for (j = 0; j < MAX_SPLIT_DISPLAY; j++) {
				display_req[i].ctl_base[j] = 0;
				display_req[i].lm_base[j] = 0;
			}
		}

		for (j = 0; j < MDP_STAGE_6; j++) {
			display_req[i].pp_state[j].base = 0;
			display_req[i].pp_state[j].zorder = MDP_STAGE_BASE;
			display_req[i].pp_state[j].lm_idx = LM_LEFT;
			display_req[i].pp_state[j].type = MDSS_MDP_PIPE_TYPE_RGB;
		}

		display_req[i].pending_pipe_mask = 0;
	}

	for (i = 0; i < ARRAY_SIZE(pipe_req); i++) {
		pipe_req[i].valid = false;
		pipe_req[i].dest_disp_id = MAX_NUM_DISPLAY;
	}
}

static void _mdp_rm_update_hdmi_display(struct msm_panel_info *pinfo)
{
	if (pinfo->lcdc.dual_pipe && !pinfo->lcdc.force_merge) {
		display_req[pinfo->dest - DISPLAY_1].needs_split_display = true;
		/* layer mixer number is 2 for wide resolution case */
		display_req[pinfo->dest - DISPLAY_1].num_lm = 2;
	}
}

static void _mdp_rm_update_dsi_display(struct msm_panel_info *pinfo, bool use_second_dsi)
{
	if (pinfo->lcdc.dual_pipe &&
		((!pinfo->mipi.dual_dsi && !pinfo->lcdc.split_display) ||
		(pinfo->splitter_is_enabled && !pinfo->lcdc.force_merge))) {
		display_req[pinfo->dest - DISPLAY_1].num_ctl = 1;
		display_req[pinfo->dest - DISPLAY_1].num_lm = 2;
	} else if (pinfo->lcdc.split_display) {
		if (pinfo->dest < DISPLAY_3) {
			display_req[pinfo->dest - DISPLAY_1].needs_split_display = true;
			display_req[pinfo->dest - DISPLAY_1].num_ctl = 2;
			display_req[pinfo->dest - DISPLAY_1].num_lm = 2;
		}
	}

	if (!use_second_dsi)
		display_req[pinfo->dest - DISPLAY_1].primary_dsi = true;
}

static bool _mdp_rm_search_pipe_by_name(
	char *pipe_name, uint32_t *index)
{
	uint32_t i = 0;

	*index = ARRAY_SIZE(pipe_req);

	for (i = 0; i < ARRAY_SIZE(pipe_req); i++) {
		if (!strncmp(pipe_req[i].pipe_name,
				pipe_name, sizeof(pipe_req[i].pipe_name))) {
			if ((pipe_req[i].valid == false) &&
				(pipe_req[i].dest_disp_id == MAX_NUM_DISPLAY)) {
				*index = i;
				return true;
			}
			else
				dprintf(SPEW, "target %s pipe is occupied\n", pipe_name);
		}
	}

	return false;
}

void mdp_rm_update_resource(struct msm_panel_info *pinfo, bool use_second_dsi)
{
	if (pinfo->dest < DISPLAY_1 || pinfo->dest > DISPLAY_3)
		return;

	display_req[pinfo->dest - DISPLAY_1].num_lm = 1;
	display_req[pinfo->dest - DISPLAY_1].num_ctl = 1;
	display_req[pinfo->dest - DISPLAY_1].needs_split_display = false;

	switch(pinfo->type) {
	case MIPI_VIDEO_PANEL:
		_mdp_rm_update_dsi_display(pinfo, use_second_dsi);
		break;
	case HDMI_PANEL:
		_mdp_rm_update_hdmi_display(pinfo);
		break;
	}
}

/* Clear pipe pending mask to prepare for next flush cycle. */
void mdp_rm_clear_pipe_mask(struct resource_req *res_mgr)
{
	res_mgr->pending_pipe_mask = 0;
}

/* Update pipe pending mask for next hardware flush. */
void mdp_rm_update_pipe_pending_mask(struct resource_req *res_mgr,
	uint32_t pipe_index)
{
	res_mgr->pending_pipe_mask |= 1 << pipe_index;
}

void mdp_rm_reset_resource_manager(bool reseted)
{
	if (!reseted) {
		dprintf(SPEW, "call _mdp_rm_init\n");
		_mdp_rm_init();
	}

	ctl_lm_allocated = true;
}

int mdp_rm_search_pipe(uint32_t pipe_type,
	uint32_t dest_display_id, uint32_t *index, char *pipe_name)
{
	uint32_t i = 0;
	bool found = false;

	*index = ARRAY_SIZE(pipe_req);

	for (i = 0; i < ARRAY_SIZE(pipe_req); i++) {
		if (pipe_name)
			found = _mdp_rm_search_pipe_by_name(pipe_name, index);

		if (found)
			break;

		if (1 << pipe_type & pipe_req[i].format_mask) {
			/* get the unused pipes available to all displays */
			if ((pipe_req[i].valid == false) &&
				(pipe_req[i].dest_disp_id == MAX_NUM_DISPLAY)) {
				*index = i;
				break;
			}
		}
	}

	if (*index < ARRAY_SIZE(pipe_req))
		return NO_ERROR;
	else
		return -EINVAL;
}

int mdp_rm_update_pipe_status(uint32_t index,
	uint32_t dest_display_id, uint32_t zorder,
	uint32_t right_mixer, uint32_t *pipe_base)
{
	uint32_t i = 0;

	for (i = 0; i < MDP_STAGE_6; i++) {
		if (display_req[dest_display_id - DISPLAY_1].pp_state[i].base == 0) {
			display_req[dest_display_id - DISPLAY_1].pp_state[i].base =
									pipe_req[index].base;
			display_req[dest_display_id - DISPLAY_1].pp_state[i].zorder = zorder;

			if (right_mixer)
				display_req[dest_display_id - DISPLAY_1].pp_state[i].lm_idx = LM_RIGHT;

			pipe_req[index].valid = true;
			pipe_req[index].dest_disp_id = dest_display_id - DISPLAY_1;

			if (pipe_req[index].format_mask & (1 << MDSS_MDP_PIPE_TYPE_VIG))
				display_req[dest_display_id - DISPLAY_1].pp_state[i].type =
									MDSS_MDP_PIPE_TYPE_VIG;

			dprintf(SPEW, "set pipe 0x%x to display%d, base[%d]=0x%x, zorder=%d\n",
				pipe_req[index].base, dest_display_id,
				i, display_req[dest_display_id - DISPLAY_1].pp_state[i].base,
				display_req[dest_display_id - DISPLAY_1].pp_state[i].zorder);

			*pipe_base = pipe_req[index].base;

			break;
		}
	}

	return NO_ERROR;
}

void mdp_rm_select_mixer(struct msm_panel_info *pinfo)
{
	if (pinfo->dest == DISPLAY_1){
		/* First display usually use CTL path 0 and 1. */
		display_req[pinfo->dest - DISPLAY_1].ctl_base[0] = MDP_CTL_0_BASE;
		display_req[pinfo->dest - DISPLAY_1].lm_base[0] = MDP_VP_0_MIXER_0_BASE;
		if (display_req[pinfo->dest - DISPLAY_1].num_ctl == 2)
			display_req[pinfo->dest - DISPLAY_1].ctl_base[1] = MDP_CTL_1_BASE;
		if (display_req[pinfo->dest - DISPLAY_1].num_lm == 2)
			display_req[pinfo->dest - DISPLAY_1].lm_base[1] = MDP_VP_0_MIXER_1_BASE;
	} else if (pinfo->dest == DISPLAY_2) {
		/* Need to care the resource that display 1 has allocated. */
		if (display_req[0].num_ctl == 2) {
			display_req[pinfo->dest - DISPLAY_1].ctl_base[0] = MDP_CTL_2_BASE;
		} else if (display_req[0].num_ctl == 1) {
			display_req[pinfo->dest - DISPLAY_1].ctl_base[0] = MDP_CTL_1_BASE;
			if (display_req[pinfo->dest - DISPLAY_1].num_ctl == 2)
				display_req[pinfo->dest - DISPLAY_1].ctl_base[1] = MDP_CTL_2_BASE;
		} else {
			dprintf(SPEW, "Display 1 CTL setup incorrect\n");
		}

		if (display_req[0].num_lm == 2) {
			display_req[pinfo->dest - DISPLAY_1].lm_base[0] = MDP_VP_0_MIXER_2_BASE;
			if (display_req[pinfo->dest - DISPLAY_1].num_lm == 2)
				display_req[pinfo->dest - DISPLAY_1].lm_base[1] = MDP_VP_0_MIXER_5_BASE;
		} else if (display_req[0].num_ctl == 1) {
			display_req[pinfo->dest - DISPLAY_1].lm_base[0] = MDP_VP_0_MIXER_1_BASE;
			if (display_req[pinfo->dest - DISPLAY_1].num_lm == 2)
				display_req[pinfo->dest - DISPLAY_1].lm_base[1] = MDP_VP_0_MIXER_2_BASE;
		} else {
			dprintf(SPEW, "Display 2 LM setup incorrect\n");
		}
	} else {
		//pinfo->dest is DISPLAY_3, the only possible CTL path is 2 only
		display_req[pinfo->dest - DISPLAY_1].ctl_base[0] = MDP_CTL_2_BASE;
		display_req[pinfo->dest - DISPLAY_1].lm_base[0] = MDP_VP_0_MIXER_2_BASE;
		if (display_req[pinfo->dest - DISPLAY_1].num_lm == 2)
			display_req[pinfo->dest - DISPLAY_1].lm_base[1] = MDP_VP_0_MIXER_5_BASE;
	}
}

struct resource_req *mdp_rm_retrieve_resource(uint32_t display_id)
{
	if ((display_id >= DISPLAY_1 ) && (display_id <= DISPLAY_3))
		return (struct resource_req *)&display_req[display_id - DISPLAY_1];
	else
		return NULL;
}

