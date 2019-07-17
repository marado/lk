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

#include <target/target_utils.h>
#include <target/display.h>
#include <string.h>
#include <debug.h>
#include <mdp5.h>
#include <err.h>
#include <stdlib.h>

static struct layer_format layer_format_table[] = {
	{"vig0", true},
	{"vig1", true},
	{"vig2", true},
	{"vig3", true},
	{"rgb0", false},
	{"rgb1", false},
	{"rgb2", false},
	{"rgb3", false},
	{"dma0", false},
	{"dma1", false},
};

//static struct early_display_property splash_property;
static struct utils_panel_lookup_list lookup_table[] = {
	{"dual_1080p_single_hdmi_video", {{"dsi0", 0}, {"dsi1", 1}, {"hdmi", 2}}},
	{"single_1080p_single_hdmi_video", {{"dsi0", 0}, {"hdmi", 1}, {"none", 0}}},
	{"dual_720p_single_hdmi_video", {{"dsi0", 0}, {"dsi1", 1}, {"hdmi", 2}}},
	{"single_720p_single_hdmi_video", {{"dsi0", 0}, {"hdmi", 1}, {"none", 0}}},
	{"adv7533_3840w_swap_hdmi", {{"hdmi", 1}, {"none", 0}, {"none", 0}}},
	{"hdmi", {{"hdmi", 0}, {"none", 0}, {"none", 0}}},
};

static struct common_config usr_config[2];
static uint32_t early_app_layer_cnt = 0;

/* Currently, we allow two early app layer instances */
static struct layer_property early_app_layer[MAX_SPLIT_DISPLAY];

extern int msm_display_add_early_layer(uint32_t disp_id,
	struct layer_property *pipe);


extern void msm_display_set_orientation(uint32_t rot_mask);

static uint32_t display_name_to_disp_id(struct utils_panel_lookup_list panel_list[],
		uint32_t list_size, const char *panel_name, const char *display_name)
{
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t disp_id = MAX_NUM_DISPLAY;

	/* discard the whitespaces at the beginning of the string */
	display_name += strspn(display_name, " ");

	for (i = 0; i < list_size; i++ ) {
		if (!strncmp(panel_name, panel_list[i].name, MAX_PANEL_ID_LEN)) {
			for (j = 0; j < MAX_NUM_DISPLAY; j++) {
				if (!strncmp(display_name,
				panel_list[i].list[j].name, MAX_PANEL_ID_LEN)) {
					disp_id = panel_list[i].list[j].disp_id;
					break;
				}
			}
		}
	}

	return disp_id;
}

static inline bool target_utils_parse_blend_stage(uint32_t blend_stage){
	/* layer blendstage from user starts start from MDP_STAGE_1 to MDP_STAGE_6 */
	if ((blend_stage >= MDP_STAGE_1) && (blend_stage <= MDP_STAGE_6))
		return true;
	else
		return false;
}

static bool target_utils_parse_pipe_validity(char *pipe_name,
	bool *pipe_is_yuv)
{
	bool ret = false;
	uint32_t i = 0;

	for (i = 0; i < ARRAY_SIZE(layer_format_table); i++) {
		if (!strcmp(layer_format_table[i].name, pipe_name)) {
			*pipe_is_yuv = layer_format_table[i].yuv;
			ret = true;
			break;
		}
	}

	return ret;
}

static bool target_utils_parse_cordinates(struct layer_property *pipe, char *src_x, char *src_y, char *dst_x, char *dst_y)
{
	pipe->src_x = atoi(src_x);
	pipe->src_y = atoi(src_y);
	pipe->dst_x = atoi(dst_x);
	pipe->dst_y = atoi(dst_y);

	return true;
}

static bool target_utils_parse_resolution(struct layer_property *pipe, char *src_w, char *src_h, char *dst_w, char *dst_h)
{
        pipe->src_w = atoi(src_w);
        pipe->src_h = atoi(src_h);
        pipe->dst_w = atoi(dst_w);
        pipe->dst_h = atoi(dst_h);

	return true;
}

static bool target_utils_parse_format(struct layer_property *pipe, char *format)
{
	bool ret = false;
	if(!strncmp(format, "uyvy", 4)) {
		pipe->format = kFormatYCbCr422H2V1Packed;
		ret = true;
	} else if(!strncmp(format, "nv12", 4)) {
		pipe->format = kFormatYCbCr420SemiPlanarVenus;
		ret = true;
	} else if (!strncmp(format, "rgb", 3)) {
		pipe->format = kFormatRGB888;
		ret = true;
	}
	return ret;
}

static int target_utils_parse_early_layer_number(char *early_layer_setup)
{
	int panel_count = 1;
	unsigned int i = 0;
	unsigned int length = strlen(early_layer_setup);

	for (i = 0; i < length; i++) {
		if (early_layer_setup[i] == '+')
			panel_count++;
	}

	return panel_count;
}

static int target_utils_parse_early_layer(const char *panel_name,
	const char *early_app_layer_setup)
{
	int ret = NO_ERROR;
	struct layer_property *pipe;
	uint32_t display_id = MAX_NUM_DISPLAY;
	bool stage_valid = false;
	bool pipe_valid = false;
	bool resolution_valid = false;
	bool cordinates_valid = false;
	bool format_valid;
	bool pipe_is_yuv = false;
	char *token = NULL;
	char *pipe_name_token = NULL;
	char *target_disp_name_token = NULL;
	char *blend_stage_token = NULL;
	char *dup_panel_name = NULL;
	char *dup_early_app_layer_setup = NULL;
	char *outer_p = NULL;
	char *inner_p = NULL;
	char *buf = NULL, *subbuf = NULL;
	char *src_x_token = NULL;
        char *src_y_token = NULL;
        char *src_w_token = NULL;
        char *src_h_token = NULL;
        char *dst_x_token = NULL;
        char *dst_y_token = NULL;
        char *dst_w_token = NULL;
        char *dst_h_token = NULL;
	char *format_token = NULL;

	/* when searching string by strtok_r, not breaking original ones */
	dup_panel_name = strdup(panel_name);
	dup_early_app_layer_setup = strdup(early_app_layer_setup);

	if (!dup_panel_name || !dup_early_app_layer_setup)
		return ERR_INVALID_ARGS;

	for (buf = dup_early_app_layer_setup; ; buf = NULL) {
		pipe = &early_app_layer[early_app_layer_cnt];

		token = strtok_r(buf, ";", &outer_p);
		if (token == NULL)
			break;

		subbuf = token;

		pipe_name_token = strtok_r(subbuf, ",", &inner_p);
		pipe_name_token += strspn(pipe_name_token, " ");
		src_x_token = strtok_r(NULL, ",", &inner_p);
                src_y_token = strtok_r(NULL, ",", &inner_p);
                src_w_token = strtok_r(NULL, ",", &inner_p);
                src_h_token = strtok_r(NULL, ",", &inner_p);
                dst_x_token = strtok_r(NULL, ",", &inner_p);
                dst_y_token = strtok_r(NULL, ",", &inner_p);
                dst_w_token = strtok_r(NULL, ",", &inner_p);
                dst_h_token = strtok_r(NULL, ",", &inner_p);
		format_token = strtok_r(NULL, ",", &inner_p);
		blend_stage_token = strtok_r(NULL, "@", &inner_p);
		target_disp_name_token = inner_p;

		dprintf(INFO, "pipe_name=%s, blend_stage=%s, target_disp_name=%s, src_x = %s, src_y = %s,"
				" src_w = %s, src_h = %s, dst_x = %s, dst_y = %s dst_w = %s dst_h = %s format = %s\n",
				pipe_name_token, blend_stage_token, target_disp_name_token, src_x_token, src_y_token,
				src_w_token, src_h_token, dst_x_token, dst_y_token, dst_w_token, dst_h_token, format_token);

		if (!pipe_name_token || !blend_stage_token || !target_disp_name_token || !src_h_token || !dst_h_token || !format_token) {
			ret = ERR_INVALID_ARGS;
			break;
		}

		/* validaty check */
		display_id = display_name_to_disp_id(lookup_table, ARRAY_SIZE(lookup_table),
				dup_panel_name + strspn(dup_panel_name, " "), target_disp_name_token);

		stage_valid = target_utils_parse_blend_stage((uint32_t)(atoi(blend_stage_token)));
		pipe_valid = target_utils_parse_pipe_validity(pipe_name_token, &pipe_is_yuv);
		cordinates_valid = target_utils_parse_cordinates(pipe, src_x_token, src_y_token, dst_x_token, dst_y_token);
		resolution_valid = target_utils_parse_resolution(pipe, src_w_token, src_h_token, dst_w_token, dst_h_token);
		format_valid = target_utils_parse_format(pipe, format_token);

		if ((display_id == MAX_NUM_DISPLAY) || !stage_valid || !pipe_valid ||
				!resolution_valid || !format_valid || !cordinates_valid) {
				dprintf(CRITICAL, "layer instance(%d) is not correct\n", early_app_layer_cnt);
				ret = ERR_INVALID_ARGS;
				break;
		}

		pipe->dest_display_id = display_id + DISPLAY_1;
		pipe->zorder = atoi(blend_stage_token);
		strlcpy(pipe->pipe_name, pipe_name_token, sizeof(pipe->pipe_name));
		pipe->yuv = pipe_is_yuv;
		strlcpy(pipe->display_name, target_disp_name_token, sizeof(pipe->display_name));
		early_app_layer_cnt++;
		if (early_app_layer_cnt >= MAX_SPLIT_DISPLAY)
			break;
	}

	dprintf(CRITICAL, "exit %s\n", __func__);

	if (dup_panel_name) {
		free(dup_panel_name);
		dup_panel_name = NULL;
	}

	if (dup_early_app_layer_setup) {
		free(dup_early_app_layer_setup);
		dup_early_app_layer_setup = NULL;
	}

	return ret;
}

uint32_t target_utils_get_early_app_layer_cnt(uint32_t disp_id,
	uint32_t *single, uint32_t *total, uint32_t *index_mask)
{
	uint32_t i = 0, layer_cnt = 0;

	for (i = 0; i < early_app_layer_cnt; i++) {
		if (early_app_layer[i].dest_display_id == disp_id) {
			*index_mask |= 1 << i;
			layer_cnt++;
		}
	}

	*single = layer_cnt;
	*total = early_app_layer_cnt;

	return NO_ERROR;
}

char *target_utils_translate_layer_to_fb(struct fbcon_config *fb,
	uint32_t cached_fb_index, struct LayerInfo *layer)
{
		fb->base = NULL;

		layer->left_pipe.id = 1;
		layer->right_pipe.id = 0;

		layer->left_pipe.horz_deci = 0;
		layer->left_pipe.vert_deci = 0;
		layer->left_pipe.src_width = early_app_layer[cached_fb_index].src_w;
		layer->left_pipe.src_height = early_app_layer[cached_fb_index].src_h;
		layer->left_pipe.src_rect.x = early_app_layer[cached_fb_index].src_x;
		layer->left_pipe.src_rect.y = early_app_layer[cached_fb_index].src_y;
		layer->left_pipe.src_rect.w = early_app_layer[cached_fb_index].src_w;
		layer->left_pipe.src_rect.h = early_app_layer[cached_fb_index].src_h;
		layer->left_pipe.dst_rect.x = early_app_layer[cached_fb_index].dst_x;
		layer->left_pipe.dst_rect.y = early_app_layer[cached_fb_index].dst_y;
		layer->left_pipe.dst_rect.w = early_app_layer[cached_fb_index].dst_w;
		layer->left_pipe.dst_rect.h = early_app_layer[cached_fb_index].dst_h;
		layer->src_format = early_app_layer[cached_fb_index].format;
		/* make sure the right pipe is empty for single pipe case */
		memset((void*)&layer->right_pipe, 0, sizeof (struct PipeInfo));

		fb->z_order = early_app_layer[cached_fb_index].zorder;
		fb->right = false;
		fb->format = early_app_layer[cached_fb_index].format;

		return early_app_layer[cached_fb_index].pipe_name;
}

uint32_t get_edrm_format(uint32_t zorder, uint32_t display_id)
{
	uint32_t j = 0;
	uint32_t format = kFormatRGB888;
	for(j = 0; j < early_app_layer_cnt; j++) {
                                if((early_app_layer[j].zorder == zorder)
                                && (early_app_layer[j].dest_display_id == display_id)) {
                                        format = early_app_layer[j].format;
					return format;
				}
	}
	/* For non-edrm layers return kFormatRGB888 as default value so that the layers are opaque*/
	return format;
}
bool target_utils_validate_input_config(const char *panel_name,
			uint32_t *disp_id, enum display_type disp_type)
{
	bool ret = false;
	uint32_t display_id = MAX_NUM_DISPLAY;

	if (usr_config[disp_type].is_on) {
		display_id = display_name_to_disp_id(lookup_table, ARRAY_SIZE(lookup_table),
			panel_name, usr_config[disp_type].display_type_name);

		if (display_id < MAX_NUM_DISPLAY) {
			*disp_id = display_id;
			ret = true;
		} else if (display_id == MAX_NUM_DISPLAY) {
			*disp_id = 0;
			ret = true;
		}
	}

	return ret;
}

void target_utils_set_orientation(bool rotation_180)
{
	uint32_t rot_bit_mask = 0;

	if (rotation_180)
		rot_bit_mask = (1 << H_FLIP) | (1 << V_FLIP);

	msm_display_set_orientation(rot_bit_mask);
}

void target_utils_set_input_config(bool enable,
	const char *display_name, enum display_type disp_type)
{
	usr_config[disp_type].is_on = enable;
	strlcpy(usr_config[disp_type].display_type_name, display_name,
		sizeof(usr_config[disp_type].display_type_name));
	dprintf(SPEW, "display_type_name %s\n", usr_config[disp_type].display_type_name);
}

int target_utils_add_early_app_layer(const char *panel_name,
	const char *early_layer_setup)
{
	int ret = NO_ERROR;

	if (!panel_name || !early_layer_setup) {
		dprintf(CRITICAL, "invalid args\n");
		return ERR_NOT_VALID;
	}

	ret = target_utils_parse_early_layer(panel_name, early_layer_setup);
	dprintf(INFO, "panel_name=%s, early_layer_setup=%s\n", panel_name, early_layer_setup);

	return ret;
}
