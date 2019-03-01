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

static struct resource_req display_req[MAX_NUM_DISPLAY] = {
    {0,0,0,0, {0,0}, {0,0}, {0,0}},
    {0,0,0,0, {0,0}, {0,0}, {0,0}},
    {0,0,0,0, {0,0}, {0,0}, {0,0}}
};

static void _mdp_rm_update_hdmi_display(struct msm_panel_info *pinfo)
{
	if (pinfo->lcdc.dual_pipe) {
		display_req[pinfo->dest - DISPLAY_1].needs_split_display = true;
		display_req[pinfo->dest - DISPLAY_1].num_lm = 2;
	}

	if (pinfo->dest == DISPLAY_1)
		pinfo->pipe_id = 0;
	else
		pinfo->pipe_id = 2;
}

static void _mdp_rm_update_dsi_display(struct msm_panel_info *pinfo, bool use_second_dsi)
{
	if (pinfo->lcdc.dual_pipe && !pinfo->mipi.dual_dsi &&
		!pinfo->lcdc.split_display) {
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

void mdp_rm_update_resource(struct msm_panel_info *pinfo, bool use_second_dsi)
{
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
			dprintf(CRITICAL, "Display 1 CTL setup incorrect\n");
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
			dprintf(CRITICAL, "Display 2 LM setup incorrect\n");
		}
	} else {
		//pinfo->dest is DISPLAY_3, the only possible CTL path is 2 only
		display_req[pinfo->dest - DISPLAY_1].ctl_base[0] = MDP_CTL_2_BASE;
		display_req[pinfo->dest - DISPLAY_1].lm_base[0] = MDP_VP_0_MIXER_2_BASE;
		if (display_req[pinfo->dest - DISPLAY_1].num_lm == 2)
			display_req[pinfo->dest - DISPLAY_1].lm_base[1] = MDP_VP_0_MIXER_5_BASE;
	}
}

void mdp_rm_update_pipe_base(struct msm_panel_info *pinfo,
		uint32_t *left_pipe, uint32_t *right_pipe)
{
	display_req[pinfo->dest - DISPLAY_1].pipe_base[0] = *left_pipe;
	display_req[pinfo->dest - DISPLAY_1].pipe_base[1] = *right_pipe;
}

struct resource_req *mdp_rm_retrieve_resource(uint32_t display_id)
{
	if ((display_id >= DISPLAY_1 ) && (display_id <= DISPLAY_3))
		return (struct resource_req *)&display_req[display_id - DISPLAY_1];
	else
		return NULL;
}

