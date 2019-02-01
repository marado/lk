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
#include <string.h>
#include <debug.h>

//static struct early_display_property splash_property;
static struct utils_panel_lookup_list lookup_table[] = {
	{"dual_1080p_single_hdmi_video", {{" dsi0", 0}, {" dsi1", 1}, {" hdmi", 2}}},
	{"single_1080p_single_hdmi_video", {{" dsi0",0}, {" hdmi", 1}, {"none", 0}}},
	{"hdmi", {{" hdmi", 0}, {"none", 0}, {"none", 0}}},
};

static struct common_config usr_config[2];

static uint32_t display_name_to_disp_id(struct utils_panel_lookup_list panel_list[],
		uint32_t list_size, const char *panel_name, const char *display_name)
{
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t disp_id = MAX_NUM_DISPLAY;

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

void target_utils_set_input_config(bool enable,
	const char *display_name, enum display_type disp_type)
{
	usr_config[disp_type].is_on = enable;
	strlcpy(usr_config[disp_type].display_type_name, display_name,
		sizeof(usr_config[disp_type].display_type_name));
	dprintf(SPEW, "display_type_name %s\n", usr_config[disp_type].display_type_name);
}
