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

enum display_type {
	RVC_DISPLAY,
	SHARE_DISPLAY,
};

struct layer_property {
	char  pipe_name[MAX_PIPE_NAME_LEN]; //RGB0~RGB3, VIG0~VIG3, DMA0~DMA1
	uint32_t zorder;    //zorder
	uint32_t flag;          //indicator for RVC layer or splash layer
	uint32_t position;      //layer stage position
	bool yuv;
	uint32_t dest_display_id;
	char display_name[MAX_PANEL_ID_LEN];
	uint32_t src_x;
	uint32_t src_y;
	uint32_t src_w;
	uint32_t src_h;
	uint32_t dst_x;
	uint32_t dst_y;
	uint32_t dst_w;
	uint32_t dst_h;
	uint32_t format;
};

struct early_display_property {
	struct layer_property reserved_layer[MAX_TARGET_LAYERS];
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

struct layer_format {
	char name[MAX_PIPE_NAME_LEN];
	bool yuv;
};

int target_utils_add_early_app_layer(const char *panel_name,
	const char *early_layer_setup);

void target_utils_set_input_config(bool enable,
	const char * rvc_display_name, enum display_type disp_type);

bool target_utils_validate_input_config(const char *panel_name,
	uint32_t *disp_id, enum display_type disp_type);

void target_utils_set_orientation(bool rotation_180);
uint32_t target_utils_get_early_app_layer_cnt(uint32_t disp_id,
	uint32_t *single, uint32_t *total, uint32_t *index_mask);

char *target_utils_translate_layer_to_fb(struct fbcon_config *fb,
	uint32_t cached_fb_index, struct LayerInfo *layer);

uint32_t get_edrm_format(uint32_t zorder, uint32_t display_id);
#endif
