/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
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

#ifndef _PANEL_HOLITECH_GC9305_ID_QVGA_SPI_CMD_H_
#define _PANEL_HOLITECH_GC9305_ID_QVGA_SPI_CMD_H_
/*---------------------------------------------------------------------------*/
/* HEADER files                                                              */
/*---------------------------------------------------------------------------*/
#include "panel.h"

/*---------------------------------------------------------------------------*/
/* Panel configuration                                                       */
/*---------------------------------------------------------------------------*/
static struct panel_config holitech_gc9305_id_qvga_cmd_panel_data = {
	"qcom,mdss_spi_holitech_gc9305_id_qvga_cmd", "spi:0:", "qcom,mdss-spi-panel",
	10, 0, "DISPLAY_1", 0, 0, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/*---------------------------------------------------------------------------*/
/* Panel resolution                                                          */
/*---------------------------------------------------------------------------*/
static struct panel_resolution holitech_gc9305_id_qvga_cmd_panel_res = {
	240, 320, 79, 59, 60, 0, 10, 7, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/*---------------------------------------------------------------------------*/
/* Panel color information                                                   */
/*---------------------------------------------------------------------------*/
static struct color_info holitech_gc9305_id_qvga_cmd_color = {
	16, 0, 0xff, 0, 0, 0
};

/*---------------------------------------------------------------------------*/
/* Panel on/off command information                                          */
/*---------------------------------------------------------------------------*/
static char holitech_gc9305_id_qvga_cmd_on_cmd1[] = {0xfe};
static char holitech_gc9305_id_qvga_cmd_on_cmd2[] = {0xef};
static char holitech_gc9305_id_qvga_cmd_on_cmd3[] = {0x36,0x48};
static char holitech_gc9305_id_qvga_cmd_on_cmd4[] = {0x3a,0x05};
static char holitech_gc9305_id_qvga_cmd_on_cmd5[] = {0xa4,0x44,0x44};
static char holitech_gc9305_id_qvga_cmd_on_cmd6[] = {0xa5,0x42,0x42};
static char holitech_gc9305_id_qvga_cmd_on_cmd7[] = {0xaa,0x88,0x88};
static char holitech_gc9305_id_qvga_cmd_on_cmd8[] = {0xe8,0x21,0x0b};
static char holitech_gc9305_id_qvga_cmd_on_cmd9[] = {0xe3,0x01,0x18};
static char holitech_gc9305_id_qvga_cmd_on_cmd10[] = {0xe1,0x10,0x0a};
static char holitech_gc9305_id_qvga_cmd_on_cmd11[] = {0xac,0x00};
static char holitech_gc9305_id_qvga_cmd_on_cmd12[] = {0xa6,0x29,0x29};
static char holitech_gc9305_id_qvga_cmd_on_cmd13[] = {0xa7,0x27,0x27};
static char holitech_gc9305_id_qvga_cmd_on_cmd14[] = {0xa8,0x17,0x17};
static char holitech_gc9305_id_qvga_cmd_on_cmd15[] = {0xa9,0x26,0x26};
static char holitech_gc9305_id_qvga_cmd_on_cmd16[] = {0xaf,0x67};
static char holitech_gc9305_id_qvga_cmd_on_cmd17[] = {0x2a,0x00,0x00,0x00,0xef};
static char holitech_gc9305_id_qvga_cmd_on_cmd18[] = {0x2b,0x00,0x00,0x01,0x3f};
static char holitech_gc9305_id_qvga_cmd_on_cmd19[] = {0xf0,0x02,0x02,0x00,0x03,0x09,0x0c};
static char holitech_gc9305_id_qvga_cmd_on_cmd20[] = {0xf1,0x01,0x03,0x00,0x04,0x12,0x13};
static char holitech_gc9305_id_qvga_cmd_on_cmd21[] = {0xf2,0x0c,0x07,0x34,0x03,0x04,0x46};
static char holitech_gc9305_id_qvga_cmd_on_cmd22[] = {0xf3,0x14,0x0b,0x42,0x04,0x05,0x50};
static char holitech_gc9305_id_qvga_cmd_on_cmd23[] = {0xf4,0x09,0x13,0x13,0x1e,0x22,0x0f};
static char holitech_gc9305_id_qvga_cmd_on_cmd24[] = {0xf5,0x08,0x0e,0x0e,0x22,0x2b,0x0f};
static char holitech_gc9305_id_qvga_cmd_on_cmd25[] = {0x35,0x00};
static char holitech_gc9305_id_qvga_cmd_on_cmd26[] = {0x44,0x00,0x14};
static char holitech_gc9305_id_qvga_cmd_on_cmd27[] = {0x11};
static char holitech_gc9305_id_qvga_cmd_on_cmd28[] = {0x29};
static char holitech_gc9305_id_qvga_cmd_on_cmd29[] = {0x2c};

static struct mdss_spi_cmd holitech_gc9305_id_qvga_cmd_on_command[] = {
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd1), holitech_gc9305_id_qvga_cmd_on_cmd1, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd2), holitech_gc9305_id_qvga_cmd_on_cmd2, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd3), holitech_gc9305_id_qvga_cmd_on_cmd3, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd4), holitech_gc9305_id_qvga_cmd_on_cmd4, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd5), holitech_gc9305_id_qvga_cmd_on_cmd5, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd6), holitech_gc9305_id_qvga_cmd_on_cmd6, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd7), holitech_gc9305_id_qvga_cmd_on_cmd7, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd8), holitech_gc9305_id_qvga_cmd_on_cmd8, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd9), holitech_gc9305_id_qvga_cmd_on_cmd9, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd10), holitech_gc9305_id_qvga_cmd_on_cmd10, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd11), holitech_gc9305_id_qvga_cmd_on_cmd11, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd12), holitech_gc9305_id_qvga_cmd_on_cmd12, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd13), holitech_gc9305_id_qvga_cmd_on_cmd13, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd14), holitech_gc9305_id_qvga_cmd_on_cmd14, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd15), holitech_gc9305_id_qvga_cmd_on_cmd15, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd16), holitech_gc9305_id_qvga_cmd_on_cmd16, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd17), holitech_gc9305_id_qvga_cmd_on_cmd17, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd18), holitech_gc9305_id_qvga_cmd_on_cmd18, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd19), holitech_gc9305_id_qvga_cmd_on_cmd19, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd20), holitech_gc9305_id_qvga_cmd_on_cmd20, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd21), holitech_gc9305_id_qvga_cmd_on_cmd21, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd22), holitech_gc9305_id_qvga_cmd_on_cmd22, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd23), holitech_gc9305_id_qvga_cmd_on_cmd23, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd24), holitech_gc9305_id_qvga_cmd_on_cmd24, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd25), holitech_gc9305_id_qvga_cmd_on_cmd25, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd26), holitech_gc9305_id_qvga_cmd_on_cmd26, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd27), holitech_gc9305_id_qvga_cmd_on_cmd27, 0x78, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd28), holitech_gc9305_id_qvga_cmd_on_cmd28, 0x00, 0},
	{sizeof(holitech_gc9305_id_qvga_cmd_on_cmd29), holitech_gc9305_id_qvga_cmd_on_cmd29, 0x00, 0},
};

#define HOLITECH_GC9305_ID_QVGA_CMD_ON_COMMAND 29

static char holitech_gc9305_id_qvga_cmdoff_cmd0[] = {
	0x28,
};

static char holitech_gc9305_id_qvga_cmdoff_cmd1[] = {
	0x10,
};

static struct mipi_dsi_cmd holitech_gc9305_id_qvga_cmd_off_command[] = {
	{0x1, holitech_gc9305_id_qvga_cmdoff_cmd0, 0x20},
	{0x1, holitech_gc9305_id_qvga_cmdoff_cmd1, 0x20}
};

#define HOLITECH_GC9305_ID_QVGA_CMD_OFF_COMMAND 2

/*---------------------------------------------------------------------------*/
/* Panel reset sequence                                                      */
/*---------------------------------------------------------------------------*/
static struct panel_reset_sequence holitech_gc9305_id_qvga_cmd_reset_seq = {
	{1, 0, 1, }, {20, 2, 20, }, 2
};

/*---------------------------------------------------------------------------*/
/* Backlight setting                                                         */
/*---------------------------------------------------------------------------*/
static struct backlight holitech_gc9305_id_qvga_cmd_backlight = {
	1, 1, 4095, 100, 1, "PMIC_8941"
};

#define HOLITECH_GC9305_ID_QVGA_CMD_SIGNATURE 0xFFFF

#endif /* PANEL_HOLITECH_GC9305_ID_QVGA_SPI_CMD_H */
