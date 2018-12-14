#include <debug.h>
#include <i2c_qup.h>
#include <blsp_qup.h>
#include <board.h>
#include <panel_detect.h>

#define TOUCH_CONTROLLER_HW_I2C_ADDRESS  0x38

struct panel_detect {
    struct qup_i2c_dev  *dev;
    uint8_t             id;
};

static struct panel_detect pndet;

int panel_detect_init() {

    struct qup_i2c_dev *dev = NULL;
    uint32_t soc_ver = 0, src_clk_freq = 0;

    soc_ver = board_soc_version();
    src_clk_freq = (soc_ver >= BOARD_SOC_VERSION2) ? 50000000 : 19200000;
    dev = qup_blsp_i2c_init(BLSP_ID_2, QUP_ID_0, 100000, src_clk_freq );
    if (NULL == dev) {
        dprintf(CRITICAL, "Failed initializing I2c\n");
        return -1;
    }
    pndet.dev = dev;
    return 0;
}

int panel_update_info(struct device_info *info)
{
    /* Create a i2c_msg buffer, to read from eprom */
    int ret = -1;
    uint8_t buf[1];
    struct i2c_msg msg[2];
    uint8_t addr = 0xa8;

    msg[0].addr = TOUCH_CONTROLLER_HW_I2C_ADDRESS;
    msg[0].flags = 0;
    msg[0].len = sizeof(addr);
    msg[0].buf = &addr;

    msg[1].addr = TOUCH_CONTROLLER_HW_I2C_ADDRESS;
    msg[1].flags = I2C_M_RD;
    msg[1].len = sizeof(buf);
    msg[1].buf = buf;

    ret = qup_i2c_xfer(pndet.dev, msg, 2);
    if ( ret < 0 ) {
        dprintf(CRITICAL, "qup_i2c_xfer error %d\n", ret);
        return ret;
    }

    switch(buf[0]) {
    case PANEL_ID_NVD:
        dprintf(CRITICAL, "New Vision Panel Detected...\n");
        strncpy(info->display_panel, "nvd_320p_video", MAX_PANEL_ID_LEN);
        break;
    case PANEL_ID_JDI_1:
    case PANEL_ID_JDI_2:
        dprintf(CRITICAL, "JDI Panel Detected...\n");
        strncpy(info->display_panel, "jdi_320p_video", MAX_PANEL_ID_LEN);
        break;
    case PANEL_ID_USM:
        dprintf(CRITICAL, "US Micro Panel Detected...\n");
        strncpy(info->display_panel, "usm_320p_video", MAX_PANEL_ID_LEN);
        break;
    default:
        dprintf(CRITICAL, "Invalid i2c data read[%x]...\n", buf[0]);
        return -1;
    }
    pndet.id = buf[0];
    return 0;
}

uint8_t panel_get_id()
{
    return pndet.id;
}

const char *panel_get_name(uint8_t panel_id) {
    switch(panel_id) {
    case PANEL_ID_NVD:
        return "qcom,mdss_dsi_nvd_320p_video";
        break;

    case PANEL_ID_JDI_1:
    case PANEL_ID_JDI_2:
        return "qcom,mdss_dsi_jdi_320p_video";
        break;

    case PANEL_ID_USM:
        return "qcom,mdss_dsi_usm_320p_video";
        break;

    default:
        return NULL;
    }
    return NULL;
}

