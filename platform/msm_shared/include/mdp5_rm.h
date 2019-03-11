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

#ifndef _MDP_5_RM_H_
#define _MDP_5_RM_H_

#include <msm_panel.h>
#include <mdp5.h>

struct pipe_state {
	uint32_t base;
	uint32_t zorder;
	uint32_t lm_idx;
	uint32_t type;
};

struct resource_req {
	int num_lm;
	int num_ctl;
	bool needs_split_display;
	bool primary_dsi;

	uint32_t ctl_base[MAX_SPLIT_DISPLAY];
	uint32_t lm_base[MAX_SPLIT_DISPLAY];

	struct pipe_state pp_state[MDP_STAGE_6];
	uint32_t pending_pipe_mask;
};

/**
 * struct source_pipe - define pipe allocation status
 * @format_mask: formats pipe supports.
 * @base: pipe base address.
 * @valid: pipe is occupied.
 * @dest_disp_id: targe display the pipe is attached to.
 */
struct source_pipe {
	uint32_t format_mask;
	uint32_t base;
	bool valid;
	uint32_t dest_disp_id;
	char pipe_name[MAX_PIPE_NAME_LEN];
};

int mdp_rm_search_pipe(uint32_t pipe_type, uint32_t dest_display_id,
	uint32_t *index, char *pipe_name);

int mdp_rm_update_pipe_status(uint32_t index, uint32_t dest_display_id,
	uint32_t zorder, uint32_t right_mixer, uint32_t *pipe_base);

void mdp_rm_update_resource(struct msm_panel_info *pinfo, bool use_second_dsi);

void mdp_rm_select_mixer(struct msm_panel_info *pinfo);

struct resource_req *mdp_rm_retrieve_resource(uint32_t display_id);

void mdp_rm_clear_pipe_mask(struct resource_req *res_mgr);

void mdp_rm_update_pipe_pending_mask(struct resource_req *res_mgr,
	uint32_t pipe_index);

void mdp_rm_reset_resource_manager();
#endif
