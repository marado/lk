#ifndef PANEL_DETECT_H
#define PANEL_DETECT_H

#include <devinfo.h>
#include <stdint.h>

#define PANEL_ID_NVD        0xD4
#define PANEL_ID_JDI_1      0xD6
#define PANEL_ID_JDI_2      0xD5
#define PANEL_ID_USM        0xE7
int panel_detect_init();
int panel_update_info(struct device_info *info);
uint8_t panel_get_id();
const char *panel_get_name(uint8_t panel_id);

#endif // PANEL_DETECT_H
