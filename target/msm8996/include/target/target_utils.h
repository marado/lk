/*
Copyright (c) 2014-2016,2018-2019, The Linux Foundation. All rights reserved.

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
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _TARGET_UTILS_H
#define _TARGET_UTILS_H

#include <msm_panel.h>

#define MAX_PANEL_ID_LEN 64

#define LAYER_FOR_EARLY_SPLASH  0x1
#define LAYER_FOR_EARLY_RVC     0x2
#define LAYER_LEFT_STAGED       0x1
#define LAYER_RIGHT_STAGED      0x2
#define MAX_PIPE_NUM		12

enum display_type {
	RVC_DISPLAY,
	SHARE_DISPLAY,
};

struct layer_property {
	char *sspp_name;        //RGB0~RGB3, VIG0~VIG3, DMA0~DMA1
	uint32_t sspp_stage;    //Layer stage
	uint32_t intf_type;     //hdmi, dsi0, dsi1
	uint32_t flag;          //indicator for RVC layer or splash layer
	uint32_t position;      //layer stage position
};

struct early_display_property {
	struct layer_property reserved_layer[MAX_PIPE_NUM];
	bool bootloader_rvc;
	uint32_t intermediate_status;
};

struct panel_display_id_map_list {
	char name[MAX_PANEL_ID_LEN];
	uint32_t disp_id;
};

struct utils_panel_lookup_list {
	char name[MAX_PANEL_ID_LEN];
	struct panel_display_id_map_list list[MAX_NUM_DISPLAY];
};

struct common_config {
	bool is_on;
	char display_type_name[MAX_PANEL_ID_LEN];
};

void target_utils_set_input_config(bool enable,
	const char * rvc_display_name, enum display_type disp_type);

bool target_utils_validate_input_config(const char *panel_name,
	uint32_t *disp_id, enum display_type disp_type);

void target_utils_set_orientation(bool rotation_180);
#endif
