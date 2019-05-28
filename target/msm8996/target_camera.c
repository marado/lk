/* Copyright (c) 2014-2016, 2018-2019, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/types.h>
#include <gsbi.h>
#include <platform/clock.h>
#include <psci.h>
#include <rpm-ipc.h>
#include <platform/timer.h>
#include <reg.h>
#include <target/display.h>
#include <kernel/thread.h>
#include <dev/keys.h>
#include <platform/timer.h>
#include <pm8x41.h>
#include <debug.h>
#include <pm8x41_hw.h>
#include <string.h>
#include <stdlib.h>
#include <platform/gpio.h>
#include <regulator.h>
#include <early_domain.h>
#include <boot_stats.h>

#include <target/target_camera.h>
#include <platform/iomap.h>

//#define DEBUG_LOGS
#ifdef DEBUG_LOGS
#undef CDBG
#define CDBG _dprintf
#else
#undef CDBG
#define CDBG(fmt, args...) do{}while(0)
#endif
#define pr_err dprintf

#define REV_CHECK_MAX_POLL_TRY 5

#define VFE_PING_ADDR 0xB3FFF000
#define VFE_PONG_ADDR 0xB43FF000
#define PING_PONG_IRQ_MASK 0x100

#define EARLY_CAM_NUM_FRAMES 60*20
#define CAM_RESET_GPIO 23

struct i2c_config_data *cam_data;
void *disp_ptr, *layer_cam_ptr;
struct target_display_update update_cam;
struct target_display *disp;
struct fbcon_config *fb;
int num_configs = 0;
#define VFE_TIME_OUT_POLL_NUM 33 * 3
int queue_id = 1;

#define MAX_POLL_COUNT 100
#define CVBS_HEADER_SIZE 13*720*2
int raw_size;
int csi;
int cci_master;
int msg_count;
#define settle_time_analog 0x10
#define EARLY_CAM_CSI_PORT_NUM_ANALOG 1
#define settle_time_digital 0x1a
#define EARLY_CAM_CSI_PORT_NUM_DIGITAL 2

enum camera_type {
	ANALOG = 1,
	DIGITAL,
};

enum camera_type cam_type;

uint32 frame_counter = 0;
uint32_t read_val = 0;
int ping = 0;
int gpio_triggered = 0;
int toggle =0;
int delay_to_attach_t32 = 0;
static bool early_camera_enabled = FALSE;
uint32_t early_rvc_timeout = 0;
uint32_t early_rvc_gpio = 0;

enum msm_camera_i2c_reg_addr_type {
	MSM_CAMERA_I2C_BYTE_ADDR = 1,
	MSM_CAMERA_I2C_WORD_ADDR,
	MSM_CAMERA_I2C_3B_ADDR,
	MSM_CAMERA_I2C_ADDR_TYPE_MAX,
};

#define TIMER_KHZ 32768

static unsigned int cam_place_kpi_marker(char *marker_name)
{
        unsigned int marker_value;
        marker_value = readl(MPM2_MPM_SLEEP_TIMETICK_COUNT_VAL);
        pr_err(CRITICAL, "marker name=%s; marker value=%u.%03u seconds\n",
                        marker_name, marker_value/TIMER_KHZ,
                        (((marker_value % TIMER_KHZ)
                        * 1000) / TIMER_KHZ));
        return marker_value;
}

int32_t msm_cci_i2c_read(uint32_t address,
						 int32_t length,
						 uint32_t *read_val,
						 unsigned int slave_address,
						 int master,
						 int queue,
						 int addr_type,
						 int data_type);

int32_t msm_cci_init(int master);
int32_t msm_cci_set_clk_param(int master);
enum msm_camera_i2c_data_type {
	MSM_CAMERA_I2C_BYTE_DATA = 1,
	MSM_CAMERA_I2C_WORD_DATA,
	MSM_CAMERA_I2C_DWORD_DATA,
	MSM_CAMERA_I2C_SET_BYTE_MASK,
	MSM_CAMERA_I2C_UNSET_BYTE_MASK,
	MSM_CAMERA_I2C_SET_WORD_MASK,
	MSM_CAMERA_I2C_UNSET_WORD_MASK,
	MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA,
	MSM_CAMERA_I2C_DATA_TYPE_MAX,
};
struct camera_hw_reg_array {
  unsigned int reg_addr;
  unsigned int reg_data;
  unsigned int delay;
};
#define NUM_QUEUES 2
struct msm_camera_cci_master_info {
	uint32_t status;
	uint32_t q_free[NUM_QUEUES];
	uint8_t q_lock[NUM_QUEUES];
	uint8_t reset_pending;
	uint32 done_pending[NUM_QUEUES];
};
#define NUM_MASTERS 2
struct msm_camera_cci_master_info cci_master_info[NUM_MASTERS];

int32_t msm_cci_i2c_write(struct camera_i2c_reg_array *pArray,
						  int arraySize,
						  int slave_address,
						  int queue,
						  int sync_en,
						  int add_size,
						  int data_size,
						  int readback,
						  int master);

int32_t msm_cci_data_queue(struct camera_i2c_reg_array *pArray,
							int arraySize,
							int slave_address,
							int queue,
							int sync_en,
							int add_size,
							int data_size,
							int master);

unsigned msm_cci_poll_irq(int irq_num,int master);

static void msm_cci_flush_queue(int master);

#define MSM_CCI_WRITE_DATA_PAYLOAD_SIZE_11 11
#define MSM_CCI_WRITE_DATA_PAYLOAD_SIZE_10 10
#define BURST_MIN_FREE_SIZE 8
#define CCI_MAX_DELAY 1000000

#define NAME_SIZE_MAX           64
#define MAX_I2C_REG_SET         12
#define MAX_POWER_CONFIG        12
#define U_I2C_SEQ_REG_DATA_MAX  1024

#define MAX_PDAF_WIN        200

enum camera_i2c_freq_mode {
  SENSOR_I2C_MODE_STANDARD,
  SENSOR_I2C_MODE_FAST,
  SENSOR_I2C_MODE_CUSTOM,
  SENSOR_I2C_MODE_FAST_PLUS,
  SENSOR_I2C_MODE_MAX,
};

enum camera_i2c_reg_addr_type {
  CAMERA_I2C_BYTE_ADDR = 1,
  CAMERA_I2C_WORD_ADDR,
  CAMERA_I2C_3B_ADDR,
  CAMERA_I2C_ADDR_TYPE_MAX,
};

enum camera_i2c_data_type {
  CAMERA_I2C_BYTE_DATA = 1,
  CAMERA_I2C_WORD_DATA,
  CAMERA_I2C_DWORD_DATA,
  CAMERA_I2C_SET_BYTE_MASK,
  CAMERA_I2C_UNSET_BYTE_MASK,
  CAMERA_I2C_SET_WORD_MASK,
  CAMERA_I2C_UNSET_WORD_MASK,
  CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA,
  CAMERA_I2C_DATA_TYPE_MAX,
};

enum camera_power_seq_type {
  CAMERA_POW_SEQ_CLK,
  CAMERA_POW_SEQ_GPIO,
  CAMERA_POW_SEQ_VREG,
  CAMERA_POW_SEQ_I2C_MUX,
  CAMERA_POW_SEQ_I2C,
};

enum camera_gpio_type {
  CAMERA_GPIO_RESET,
  CAMERA_GPIO_STANDBY,
  CAMERA_GPIO_AF_PWDM,
  CAMERA_GPIO_VIO,
  CAMERA_GPIO_VANA,
  CAMERA_GPIO_VDIG,
  CAMERA_GPIO_VAF,
  CAMERA_GPIO_FL_EN,
  CAMERA_GPIO_FL_NOW,
  CAMERA_GPIO_FL_RESET,
  CAMERA_GPIO_CUSTOM1,
  CAMERA_GPIO_CUSTOM2,
  CAMERA_GPIO_MAX,
};

enum camera_clk_type {
  CAMERA_MCLK,
  CAMERA_CLK,
  CAMERA_CLK_MAX,
};

enum camera_vreg_name {
  CAMERA_VDIG,
  CAMERA_VIO,
  CAMERA_VANA,
  CAMERA_VAF,
  CAMERA_V_CUSTOM1,
  CAMERA_V_CUSTOM2,
  CAMERA_VREG_MAX,
};

enum actuator_data_type {
  ACTUATOR_BYTE_DATA = 1,
  ACTUATOR_WORD_DATA,
};

enum actuator_addr_type {
  ACTUATOR_BYTE_ADDR = 1,
  ACTUATOR_WORD_ADDR,
};

enum camera_i2c_operation {
  CAMERA_I2C_OP_WRITE = 0,
  CAMERA_I2C_OP_POLL,
  CAMERA_I2C_OP_READ,
};

struct camera_reg_settings_t {
  unsigned short reg_addr;
  enum camera_i2c_reg_addr_type addr_type;
  unsigned short reg_data;
  enum camera_i2c_data_type data_type;
  enum camera_i2c_operation i2c_operation;
  unsigned int delay;
};

struct camera_i2c_reg_setting_array {
  struct camera_i2c_reg_array reg_setting_a[MAX_I2C_REG_SET];
  unsigned short size;
  enum camera_i2c_reg_addr_type addr_type;
  enum camera_i2c_data_type data_type;
  unsigned short delay;
};

struct camera_power_setting {
  enum camera_power_seq_type seq_type;
  unsigned short seq_val;
  long config_val;
  unsigned short delay;
  void *data[10];
};

struct camera_power_setting_array {
  struct camera_power_setting  power_setting_a[MAX_POWER_CONFIG];
  struct camera_power_setting *power_setting;
  unsigned short size;
  struct camera_power_setting  power_down_setting_a[MAX_POWER_CONFIG];
  struct camera_power_setting *power_down_setting;
  unsigned short size_down;
};

struct camera_i2c_seq_reg_array {
  unsigned short reg_addr;
  unsigned char reg_data[U_I2C_SEQ_REG_DATA_MAX];
  unsigned short reg_data_size;
};
struct sensor_id_info_t {
  unsigned short sensor_id_reg_addr;
  unsigned short sensor_id;
  unsigned short sensor_id_mask;
};

#define GPIO_HIGH_VAL 2
#define GPIO_LOW_VAL 0

#define CCI_HW_VERSION_ADDR                                         0x00000000
#define CCI_RESET_CMD_ADDR                                          0x00000004
#define CCI_RESET_CMD_RMSK                                          0x0f73f3f7
#define CCI_M0_RESET_RMSK                                                0x3F1
#define CCI_M1_RESET_RMSK                                              0x3F001
#define CCI_QUEUE_START_ADDR                                        0x00000008
#define CCI_I2C_M0_SCL_CTL_ADDR                                     0x00000100
#define CCI_I2C_M0_SDA_CTL_0_ADDR                                   0x00000104
#define CCI_I2C_M0_SDA_CTL_1_ADDR                                   0x00000108
#define CCI_I2C_M0_SDA_CTL_2_ADDR                                   0x0000010c
#define CCI_I2C_M0_READ_DATA_ADDR                                   0x00000118
#define CCI_I2C_M0_MISC_CTL_ADDR                                    0x00000110
#define CCI_I2C_M0_READ_BUF_LEVEL_ADDR                              0x0000011C
#define CCI_HALT_REQ_ADDR                                           0x00000034
#define CCI_M0_HALT_REQ_RMSK                                               0x1
#define CCI_M1_HALT_REQ_RMSK                                               0x2
#define CCI_I2C_M1_SCL_CTL_ADDR                                     0x00000200
#define CCI_I2C_M1_SDA_CTL_0_ADDR                                   0x00000204
#define CCI_I2C_M1_SDA_CTL_1_ADDR                                   0x00000208
#define CCI_I2C_M1_SDA_CTL_2_ADDR                                   0x0000020c
#define CCI_I2C_M1_MISC_CTL_ADDR                                    0x00000210
#define CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR                             0x00000304
#define CCI_I2C_M0_Q0_CUR_CMD_ADDR                                  0x00000308
#define CCI_I2C_M0_Q0_REPORT_STATUS_ADDR                            0x0000030c
#define CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR                            0x00000300
#define CCI_I2C_M0_Q0_LOAD_DATA_ADDR                                0x00000310
#define CCI_IRQ_MASK_0_ADDR                                         0x00000c04
#define CCI_IRQ_MASK_0_RMSK                                         0x7fff7ff7
#define CCI_IRQ_CLEAR_0_ADDR                                        0x00000c08
#define CCI_IRQ_STATUS_0_ADDR                                       0x00000c0c
#define CCI_IRQ_STATUS_0_I2C_M1_Q0Q1_HALT_ACK_BMSK                   0x4000000
#define CCI_IRQ_STATUS_0_I2C_M0_Q0Q1_HALT_ACK_BMSK                   0x2000000
#define CCI_IRQ_STATUS_0_RST_DONE_ACK_BMSK                           0x1000000
#define CCI_IRQ_STATUS_0_I2C_M1_Q1_REPORT_BMSK                        0x100000
#define CCI_IRQ_STATUS_0_I2C_M1_Q0_REPORT_BMSK                         0x10000
#define CCI_IRQ_STATUS_0_I2C_M1_RD_DONE_BMSK                            0x1000
#define CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK                           0x100
#define CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK                            0x10
#define CCI_IRQ_STATUS_0_I2C_M0_ERROR_BMSK                          0x18000EE6
#define CCI_IRQ_STATUS_0_I2C_M1_ERROR_BMSK                          0x60EE6000
#define CCI_IRQ_STATUS_0_I2C_M0_RD_DONE_BMSK                               0x1
#define CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR                               0x00000c00

#define DEBUG_TOP_REG_START                                                0x0
#define DEBUG_TOP_REG_COUNT                                                 14
#define DEBUG_MASTER_REG_START                                           0x100
#define DEBUG_MASTER_REG_COUNT                                               8
#define DEBUG_MASTER_QUEUE_REG_START                                     0x300
#define DEBUG_MASTER_QUEUE_REG_COUNT                                         6
#define DEBUG_INTR_REG_START                                             0xC00
#define DEBUG_INTR_REG_COUNT                                                 7
#define msm_camera_io_w_mb(d,a) writel(d,a)
#define msm_camera_io_w(d,a) writel(d,a)
#define msm_camera_io_r(a) readl(a)
#define msm_camera_io_r_mb(a) readl(a)
#define CCI_DEV_BASE 0x00A0C000
#define CSI2_PHY_BASE 0x00A36000
#define CSI2_CLK_MUX_BASE 0x00A00040
#define CSI2_D_BASE 0x00A30800

#define CSI1_PHY_BASE 0x00A35000
#define CSI1_CLK_MUX_BASE 0x00A00038
#define CSI1_D_BASE 0x00A30400


#define VFE0_BASE 0x00A10000
#define VFE_SIZE 801
#define VBIF0_BASE 0x00A40000
#define MMSS_A_VFE_0_IRQ_STATUS_0 0x00A1006C
#define MMSS_A_VFE_0_IRQ_CLEAR_0 0x00A10064
#define MMSS_A_VFE_0_IRQ_CLEAR_1 0x00A10068
#define MMSS_A_VFE_0_IRQ_CMD  0x00A10058
#define MMSS_A_VFE_0_BUS_PING_PONG_STATUS 0x00A10338
#define ISPIF_BASE 0x00A31000
#define ISPIF_CLK_MUX_BASE 0x00A00020

#define CSI_PHY_INIT_ADV7481(_csi_phy_base_, _csi_clk_mux_base_) \
	{_csi_phy_base_+0x800,0x1,10}, \
	{_csi_phy_base_+0x800,0x0,0}, \
	{_csi_clk_mux_base_+0x0,EARLY_CAM_CSI_PORT_NUM_ANALOG,0}, \
	{_csi_phy_base_+0x814,0x00000081,0}, \
	{_csi_phy_base_+0x818,0x1,0}, \
	{_csi_phy_base_+0x030,0x2,0}, \
	{_csi_phy_base_+0x02c,0x1,0}, \
	{_csi_phy_base_+0x034,0x10,0}, \
	{_csi_phy_base_+0x028,0x0,0}, \
	{_csi_phy_base_+0x03c,0x000000b8,0}, \
	{_csi_phy_base_+0x004,0x8,0}, \
	{_csi_phy_base_+0x008,settle_time_analog,0}, \
	{_csi_phy_base_+0x000,0x000000d7,0}, \
	{_csi_phy_base_+0x010,0x50,0}, \
	{_csi_phy_base_+0x038,0x1,0}, \
	{_csi_phy_base_+0x01c,0x0000000a,0}, \
	{_csi_phy_base_+0x730,0x2,0}, \
	{_csi_phy_base_+0x72c,0x1,0}, \
	{_csi_phy_base_+0x734,0x10,0}, \
	{_csi_phy_base_+0x728,0x4,0}, \
	{_csi_phy_base_+0x73c,0x000000b8,0}, \
	{_csi_phy_base_+0x704,0x8,0}, \
	{_csi_phy_base_+0x708,settle_time_analog,0}, \
	{_csi_phy_base_+0x700,0x000000c0,0}, \
	{_csi_phy_base_+0x70c,0x000000ff,0}, \
	{_csi_phy_base_+0x710,0x50,0}, \
	{_csi_phy_base_+0x738,0x1,0}, \
	{_csi_phy_base_+0x71c,0x0000000a,0}, \
	{_csi_phy_base_+0x230,0x2,0}, \
	{_csi_phy_base_+0x22c,0x1,0}, \
	{_csi_phy_base_+0x234,0x1,0}, \
	{_csi_phy_base_+0x228,0x0,0}, \
	{_csi_phy_base_+0x23c,0x000000b8,0}, \
	{_csi_phy_base_+0x204,0x8,0}, \
	{_csi_phy_base_+0x200,0x000000d7,0}, \
	{_csi_phy_base_+0x210,0x50,0}, \
	{_csi_phy_base_+0x238,0x1,0}, \
	{_csi_phy_base_+0x21c,0x0000000a,0}, \
	{_csi_phy_base_+0x430,0x2,0}, \
	{_csi_phy_base_+0x42c,0x1,0}, \
	{_csi_phy_base_+0x434,0x1,0}, \
	{_csi_phy_base_+0x428,0x0,0}, \
	{_csi_phy_base_+0x43c,0x000000b8,0}, \
	{_csi_phy_base_+0x404,0x8,0}, \
	{_csi_phy_base_+0x400,0x000000d7,0}, \
	{_csi_phy_base_+0x410,0x50,0}, \
	{_csi_phy_base_+0x438,0x1,0}, \
	{_csi_phy_base_+0x41c,0x0000000a,0}, \
	{_csi_phy_base_+0x630,0x2,0}, \
	{_csi_phy_base_+0x62c,0x1,0}, \
	{_csi_phy_base_+0x634,0x1,0}, \
	{_csi_phy_base_+0x628,0x0,0}, \
	{_csi_phy_base_+0x63c,0x000000b8,0}, \
	{_csi_phy_base_+0x604,0x8,0}, \
	{_csi_phy_base_+0x600,0x000000d7,0}, \
	{_csi_phy_base_+0x610,0x50,0}, \
	{_csi_phy_base_+0x638,0x1,0}, \
	{_csi_phy_base_+0x61c,0x0000000a,0}, \
	{_csi_phy_base_+0x82c,0x000000ff,0}, \
	{_csi_phy_base_+0x830,0x000000ff,0}, \
	{_csi_phy_base_+0x834,0x000000fb,0}, \
	{_csi_phy_base_+0x838,0x000000ff,0}, \
	{_csi_phy_base_+0x83c,0x0000007f,0}, \
	{_csi_phy_base_+0x840,0x000000ff,0}, \
	{_csi_phy_base_+0x844,0x000000ff,0}, \
	{_csi_phy_base_+0x848,0x000000ef,0}, \
	{_csi_phy_base_+0x84c,0x000000ff,0}, \
	{_csi_phy_base_+0x850,0x000000ff,0}, \
	{_csi_phy_base_+0x854,0x000000ff,0},

#define CSI_D_INIT_ADV7481(_csid_base_) \
	{_csid_base_+0x10,0x00007fff,10}, \
	{_csid_base_+0x64,0x800,0}, \
	{_csid_base_+0x04,0x32100,0}, \
	{_csid_base_+0x08,0x0003000f,0}, \
	{_csid_base_+0x14,0x0000001e,0}, \
	{_csid_base_+0x24,0x13,0}, \
	{_csid_base_+0x18,0x00000000,0}, \
	{_csid_base_+0x34,0x0,0}, \
	{_csid_base_+0x1c,0x00000000,0}, \
	{_csid_base_+0x44,0x0,0}, \
	{_csid_base_+0x20,0x00000,0}, \
	{_csid_base_+0x54,0x0,0},

#define VFE_INIT_ADV7481(_vfe_base_, _vfe_vbif_base_) \
	{_vfe_base_ +0x05c,0x80000000,0}, \
	{_vfe_base_ +0x060,0x0,0,}, \
	{_vfe_base_ +0x064,0xffffffff,0}, \
	{_vfe_base_ +0x068,0xffffffff,0}, \
	{_vfe_base_ +0x058,0x1,0,}, \
	{_vfe_base_ +0x018,0x000003ff,10}, \
	{_vfe_base_ +0x064,0x80000000,0}, \
	{_vfe_base_ +0x068,0x0,0,}, \
	{_vfe_base_ +0x058,0x1,0,}, \
	{_vfe_vbif_base_ +0x0004,0x0,0}, \
	{_vfe_base_ +0x404,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x408,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x40c,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x410,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x414,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x418,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x41c,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x420,0x0001aaa9,0}, \
	{_vfe_base_ +0x424,0xcccc0011,0}, \
	{_vfe_base_ +0x428,0xcccc0011,0}, \
	{_vfe_base_ +0x42c,0xcccc0011,0}, \
	{_vfe_base_ +0x430,0xcccc0011,0}, \
	{_vfe_base_ +0x434,0xcccc0011,0}, \
	{_vfe_base_ +0x438,0xcccc0011,0}, \
	{_vfe_base_ +0x43c,0xcccc0011,0}, \
	{_vfe_base_ +0x440,0xcccc0011,0}, \
	{_vfe_base_ +0x444,0xcccc0011,0}, \
	{_vfe_base_ +0x448,0xcccc0011,0}, \
	{_vfe_base_ +0x44c,0xcccc0011,0}, \
	{_vfe_base_ +0x450,0xcccc0011,0}, \
	{_vfe_base_ +0x454,0xcccc0011,0}, \
	{_vfe_base_ +0x458,0xcccc0011,0}, \
	{_vfe_base_ +0x45c,0xcccc0011,0}, \
	{_vfe_base_ +0x460,0xcccc0011,0}, \
	{_vfe_base_ +0x464,0x40000103,0}, \
	{_vfe_vbif_base_ +0x124,3,0}, \
	{_vfe_base_ +0x084,0x001,0}, \
	{_vfe_base_ +0x05c,0xe00000f3,0}, \
	{_vfe_base_ +0x060,0xffffffff,0}, \
	{_vfe_base_ +0x064,0xffffffff,0}, \
	{_vfe_base_ +0x068,0xffffffff,0}, \
	{_vfe_base_ +0x46c,0x5,0}, \
	{_vfe_base_ +0x05c,0xe00001f3,0}, \
	{_vfe_base_ +0x0b4,0x0,0}, \
	{_vfe_base_ +0x0c8,0xffffffff,0}, \
	{_vfe_base_ +0x090,0xc00,0}, \
	{_vfe_base_ +0x0a4,0x83D00000,0}, \
	{_vfe_base_ +0x0a8,0x84532800,0}, \
	{_vfe_base_ +0x0ac,0x83400000,0}, \
	{_vfe_base_ +0x0b0,0x83C32800,0}, \
	{_vfe_base_ +0x0b8,0xd800d7,0}, \
	{_vfe_base_ +0x0bc,0x2c01fa,0}, \
	{_vfe_base_ +0x0c0,0x5a07ea,0}, \
	{_vfe_base_ +0x0e4,0x0,0}, \
	{_vfe_base_ +0x110,0x0,0}, \
	{_vfe_base_ +0x13c,0x0,0}, \
	{_vfe_base_ +0x168,0x0,0}, \
	{_vfe_base_ +0x194,0x0,0}, \
	{_vfe_base_ +0x1c0,0x0,0}, \
	{_vfe_base_ +0x03c,0x1,0}, \
	{_vfe_base_ +0x0c4,0x1,0}, \
	{_vfe_base_ +0x0b4,0x0,0}, \
	{_vfe_base_ +0x0a0,0x1,0}, \
	{_vfe_base_ +0x4ac,0x2,0}, \
	{_vfe_base_ +0x080,0x1,0},

#define ISPIF_INIT_ADV7481(_ispif_base_, _ispif_clk_mux_base_) \
	{_ispif_base_ +0x008,0xfe7f1fff,10}, \
	{_ispif_base_ +0x230,0x8000000,0}, \
	{_ispif_base_ +0x234,0x0,0}, \
	{_ispif_base_ +0x238,0x0,0}, \
	{_ispif_base_ +0x430,0x8000000,0}, \
	{_ispif_base_ +0x434,0x0,0}, \
	{_ispif_base_ +0x438,0x0,0}, \
	{_ispif_base_ +0x01c,0x1,0}, \
	{_ispif_base_ +0x00c,0xfc7f1ff9,10}, \
	{_ispif_base_ +0x230,0x8000000,0}, \
	{_ispif_base_ +0x234,0x0,0}, \
	{_ispif_base_ +0x238,0x0,0}, \
	{_ispif_base_ +0x430,0x8000000,0}, \
	{_ispif_base_ +0x434,0x0,0}, \
	{_ispif_base_ +0x438,0x0,0}, \
	{_ispif_base_ +0x01c,0x1,0}, \
	{_ispif_base_ +0x200,0x40,0}, \
	{_ispif_base_ +0x208,0x0,0}, \
	{_ispif_base_ +0x20c,0x0,0}, \
	{_ispif_base_ +0x210,0x0,0}, \
	{_ispif_base_ +0x230,0xffffffff,0}, \
	{_ispif_base_ +0x234,0xffffffff,0}, \
	{_ispif_base_ +0x238,0xffffffff,0}, \
	{_ispif_base_ +0x244,0x10,0}, \
	{_ispif_base_ +0x248,0xaaaaaaaa,0}, \
	{_ispif_base_ +0x24c,0xaaaaaaaa,0}, \
	{_ispif_base_ +0x254,0x0,0}, \
	{_ispif_base_ +0x258,0x0,0}, \
	{_ispif_base_ +0x264,0x0,0}, \
	{_ispif_base_ +0x268,0x0,0}, \
	{_ispif_base_ +0x26c,0x0,0}, \
	{_ispif_base_ +0x288,0x0,0}, \
	{_ispif_base_ +0x28c,0x0,0}, \
	{_ispif_base_ +0x400,0x40,0}, \
	{_ispif_base_ +0x408,0x0,0}, \
	{_ispif_base_ +0x40c,0x0,0}, \
	{_ispif_base_ +0x410,0x0,0}, \
	{_ispif_base_ +0x430,0xffffffff,0}, \
	{_ispif_base_ +0x434,0xffffffff,0}, \
	{_ispif_base_ +0x438,0xffffffff,0}, \
	{_ispif_base_ +0x444,0x0,0}, \
	{_ispif_base_ +0x448,0xaaaaaaaa,0}, \
	{_ispif_base_ +0x44c,0xaaaaaaaa,0}, \
	{_ispif_base_ +0x454,0x0,0}, \
	{_ispif_base_ +0x458,0x0,0}, \
	{_ispif_base_ +0x464,0x0,0}, \
	{_ispif_base_ +0x468,0x0,0}, \
	{_ispif_base_ +0x46c,0x0,0}, \
	{_ispif_base_ +0x488,0x0,0}, \
	{_ispif_base_ +0x48c,0x0,0}, \
	{_ispif_base_ +0x01c,0x1,0}, \
	{_ispif_base_ +0x208,0x0,0}, \
	{_ispif_base_ +0x20c,0x0,0}, \
	{_ispif_base_ +0x210,0x0,0}, \
	{_ispif_clk_mux_base_ +0x8,0x1,0}, \
	{_ispif_base_ +0x244,0x10,0}, \
	{_ispif_base_ +0x264,0x1,0}, \
	{_ispif_base_ +0x208,0x0a493249,0}, \
	{_ispif_base_ +0x230,0x0a493249,0}, \
	{_ispif_base_ +0x20c,0x2493249,0}, \
	{_ispif_base_ +0x234,0x2493249,0}, \
	{_ispif_base_ +0x210,0x1249,0}, \
	{_ispif_base_ +0x238,0x1249,0}, \
	{_ispif_base_ +0x408,0x0a493249,0}, \
	{_ispif_base_ +0x430,0x0a493249,0}, \
	{_ispif_base_ +0x40c,0x2493249,0}, \
	{_ispif_base_ +0x434,0x2493249,0}, \
	{_ispif_base_ +0x410,0x1249,0}, \
	{_ispif_base_ +0x438,0x1249,0}, \
	{_ispif_base_ +0x01c,0x1,0}, \
	{_ispif_base_ +0x248,0xfffffdff,0}

#define CSI_PHY_INIT(_csi_phy_base_, _csi_clk_mux_base_) \
	{_csi_phy_base_+0x800,0x1,10}, \
	{_csi_phy_base_+0x800,0x0,0}, \
	{_csi_clk_mux_base_+0x0,EARLY_CAM_CSI_PORT_NUM_DIGITAL,0}, \
	{_csi_phy_base_+0x814,0x000000d5,0}, \
	{_csi_phy_base_+0x818,0x1,0}, \
	{_csi_phy_base_+0x030,0x2,0}, \
	{_csi_phy_base_+0x02c,0x1,0}, \
	{_csi_phy_base_+0x034,0x3,0}, \
	{_csi_phy_base_+0x028,0x0,0}, \
	{_csi_phy_base_+0x03c,0x000000b8,0}, \
	{_csi_phy_base_+0x004,0x8,0}, \
	{_csi_phy_base_+0x008,settle_time_digital,0}, \
	{_csi_phy_base_+0x000,0x000000d7,0}, \
	{_csi_phy_base_+0x010,0x52,0}, \
	{_csi_phy_base_+0x038,0x1,0}, \
	{_csi_phy_base_+0x01c,0x0000000a,0}, \
	{_csi_phy_base_+0x730,0x2,0}, \
	{_csi_phy_base_+0x72c,0x1,0}, \
	{_csi_phy_base_+0x734,0x3,0}, \
	{_csi_phy_base_+0x728,0x4,0}, \
	{_csi_phy_base_+0x73c,0x000000b8,0}, \
	{_csi_phy_base_+0x704,0x8,0}, \
	{_csi_phy_base_+0x708,settle_time_digital,0}, \
	{_csi_phy_base_+0x700,0x000000c0,0}, \
	{_csi_phy_base_+0x70c,0x000000a5,0}, \
	{_csi_phy_base_+0x710,0x52,0}, \
	{_csi_phy_base_+0x738,0x1,0}, \
	{_csi_phy_base_+0x71c,0x0000000a,0}, \
	{_csi_phy_base_+0x230,0x2,0}, \
	{_csi_phy_base_+0x22c,0x1,0}, \
	{_csi_phy_base_+0x234,0x3,0}, \
	{_csi_phy_base_+0x228,0x0,0}, \
	{_csi_phy_base_+0x23c,0x000000b8,0}, \
	{_csi_phy_base_+0x204,0x8,0}, \
	{_csi_phy_base_+0x208,settle_time_digital,0}, \
	{_csi_phy_base_+0x200,0x000000d7,0}, \
	{_csi_phy_base_+0x210,0x52,0}, \
	{_csi_phy_base_+0x238,0x1,0}, \
	{_csi_phy_base_+0x21c,0x0000000a,0}, \
	{_csi_phy_base_+0x430,0x2,0}, \
	{_csi_phy_base_+0x42c,0x1,0}, \
	{_csi_phy_base_+0x434,0x3,0}, \
	{_csi_phy_base_+0x428,0x0,0}, \
	{_csi_phy_base_+0x43c,0x000000b8,0}, \
	{_csi_phy_base_+0x404,0x8,0}, \
	{_csi_phy_base_+0x408,settle_time_digital,0}, \
	{_csi_phy_base_+0x400,0x000000d7,0}, \
	{_csi_phy_base_+0x410,0x52,0}, \
	{_csi_phy_base_+0x438,0x1,0}, \
	{_csi_phy_base_+0x41c,0x0000000a,0}, \
	{_csi_phy_base_+0x630,0x2,0}, \
	{_csi_phy_base_+0x62c,0x1,0}, \
	{_csi_phy_base_+0x634,0x3,0}, \
	{_csi_phy_base_+0x628,0x0,0}, \
	{_csi_phy_base_+0x63c,0x000000b8,0}, \
	{_csi_phy_base_+0x604,0x8,0}, \
	{_csi_phy_base_+0x608,settle_time_digital,0}, \
	{_csi_phy_base_+0x600,0x000000d7,0}, \
	{_csi_phy_base_+0x610,0x52,0}, \
	{_csi_phy_base_+0x638,0x1,0}, \
	{_csi_phy_base_+0x61c,0x0000000a,0}, \
	{_csi_phy_base_+0x82c,0x000000ff,0}, \
	{_csi_phy_base_+0x830,0x000000ff,0}, \
	{_csi_phy_base_+0x834,0x000000fb,0}, \
	{_csi_phy_base_+0x838,0x000000ff,0}, \
	{_csi_phy_base_+0x83c,0x0000007f,0}, \
	{_csi_phy_base_+0x840,0x000000ff,0}, \
	{_csi_phy_base_+0x844,0x000000ff,0}, \
	{_csi_phy_base_+0x848,0x000000ef,0}, \
	{_csi_phy_base_+0x84c,0x000000ff,0}, \
	{_csi_phy_base_+0x850,0x000000ff,0}, \
	{_csi_phy_base_+0x854,0x000000ff,0},

#define CSI_D_INIT(_csid_base_) \
	{_csid_base_+0x10,0x00007fff,10}, \
	{_csid_base_+0x64,0x800,0}, \
	{_csid_base_+0x04,0x32103,0}, \
	{_csid_base_+0x08,0x0004000f,0}, \
	{_csid_base_+0x14,0x0000001f,0}, \
	{_csid_base_+0x24,0x23,0}, \
	{_csid_base_+0x18,0x0000001f,0}, \
	{_csid_base_+0x34,0x23,0}, \
	{_csid_base_+0x1c,0x0000001f,0}, \
	{_csid_base_+0x44,0x23,0}, \
	{_csid_base_+0x20,0x0000001f,0}, \
	{_csid_base_+0x54,0x23,0},

#define VFE_INIT(_vfe_base_, _vfe_vbif_base_) \
	{_vfe_base_ +0x05c,0x80000000,0}, \
	{_vfe_base_ +0x060,0x0,0,}, \
	{_vfe_base_ +0x064,0xffffffff,0}, \
	{_vfe_base_ +0x068,0xffffffff,0}, \
	{_vfe_base_ +0x058,0x1,0,}, \
	{_vfe_base_ +0x018,0x000003ff,10}, \
	{_vfe_base_ +0x064,0x80000000,0}, \
	{_vfe_base_ +0x068,0x0,0,}, \
	{_vfe_base_ +0x058,0x1,0,}, \
	{_vfe_vbif_base_ +0x0004,0x0,0}, \
	{_vfe_base_ +0x404,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x408,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x40c,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x410,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x414,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x418,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x41c,0xaaa9aaa9,0}, \
	{_vfe_base_ +0x420,0x0001aaa9,0}, \
	{_vfe_base_ +0x424,0xcccc0011,0}, \
	{_vfe_base_ +0x428,0xcccc0011,0}, \
	{_vfe_base_ +0x42c,0xcccc0011,0}, \
	{_vfe_base_ +0x430,0xcccc0011,0}, \
	{_vfe_base_ +0x434,0xcccc0011,0}, \
	{_vfe_base_ +0x438,0xcccc0011,0}, \
	{_vfe_base_ +0x43c,0xcccc0011,0}, \
	{_vfe_base_ +0x440,0xcccc0011,0}, \
	{_vfe_base_ +0x444,0xcccc0011,0}, \
	{_vfe_base_ +0x448,0xcccc0011,0}, \
	{_vfe_base_ +0x44c,0xcccc0011,0}, \
	{_vfe_base_ +0x450,0xcccc0011,0}, \
	{_vfe_base_ +0x454,0xcccc0011,0}, \
	{_vfe_base_ +0x458,0xcccc0011,0}, \
	{_vfe_base_ +0x45c,0xcccc0011,0}, \
	{_vfe_base_ +0x460,0xcccc0011,0}, \
	{_vfe_base_ +0x464,0x40000103,0}, \
	{_vfe_vbif_base_ +0x124,3,0}, \
	{_vfe_base_ +0x084,0x001,0}, \
	{_vfe_base_ +0x05c,0xe00000f3,0}, \
	{_vfe_base_ +0x060,0xffffffff,0}, \
	{_vfe_base_ +0x064,0xffffffff,0}, \
	{_vfe_base_ +0x068,0xffffffff,0}, \
	{_vfe_base_ +0x46c,0x5,0}, \
	{_vfe_base_ +0x05c,0xe00001f3,0}, \
	{_vfe_base_ +0x0b4,0x2,0}, \
	{_vfe_base_ +0x0c8,0xffffffff,0}, \
	{_vfe_base_ +0x090,0xc00,0}, \
	{_vfe_base_ +0x0a4,0x83D00000,0}, \
	{_vfe_base_ +0x0a8,0x84532800,0}, \
	{_vfe_base_ +0x0ac,0x83400000,0}, \
	{_vfe_base_ +0x0b0,0x83C32800,0}, \
	{_vfe_base_ +0x0b8,0x1db,0}, \
	{_vfe_base_ +0x0e4,0x0,0}, \
	{_vfe_base_ +0x110,0x0,0}, \
	{_vfe_base_ +0x13c,0x0,0}, \
	{_vfe_base_ +0x168,0x0,0}, \
	{_vfe_base_ +0x194,0x0,0}, \
	{_vfe_base_ +0x1c0,0x0,0}, \
	{_vfe_base_ +0x03c,0x1,0}, \
	{_vfe_base_ +0x0c4,0x1,0}, \
	{_vfe_base_ +0x0b4,0x2,0}, \
	{_vfe_base_ +0x0a0,0x1,0}, \
	{_vfe_base_ +0x4ac,0x2,0}, \
	{_vfe_base_ +0x080,0x1,0},

#define ISPIF_INIT(_ispif_base_, _ispif_clk_mux_base_) \
	{_ispif_base_ +0x008,0xfe7f1fff,10}, \
	{_ispif_base_ +0x230,0x8000000,0}, \
	{_ispif_base_ +0x234,0x0,0}, \
	{_ispif_base_ +0x238,0x0,0}, \
	{_ispif_base_ +0x430,0x8000000,0}, \
	{_ispif_base_ +0x434,0x0,0}, \
	{_ispif_base_ +0x438,0x0,0}, \
	{_ispif_base_ +0x01c,0x1,0}, \
	{_ispif_base_ +0x00c,0xfc7f1ff9,10}, \
	{_ispif_base_ +0x230,0x8000000,0}, \
	{_ispif_base_ +0x234,0x0,0}, \
	{_ispif_base_ +0x238,0x0,0}, \
	{_ispif_base_ +0x430,0x8000000,0}, \
	{_ispif_base_ +0x434,0x0,0}, \
	{_ispif_base_ +0x438,0x0,0}, \
	{_ispif_base_ +0x01c,0x1,0}, \
	{_ispif_base_ +0x200,0x40,0}, \
	{_ispif_base_ +0x208,0x0,0}, \
	{_ispif_base_ +0x20c,0x0,0}, \
	{_ispif_base_ +0x210,0x0,0}, \
	{_ispif_base_ +0x230,0xffffffff,0}, \
	{_ispif_base_ +0x234,0xffffffff,0}, \
	{_ispif_base_ +0x238,0xffffffff,0}, \
	{_ispif_base_ +0x244,0x0,0}, \
	{_ispif_base_ +0x248,0xaaaaaaaa,0}, \
	{_ispif_base_ +0x24c,0xaaaaaaaa,0}, \
	{_ispif_base_ +0x254,0x0,0}, \
	{_ispif_base_ +0x258,0x0,0}, \
	{_ispif_base_ +0x264,0x0,0}, \
	{_ispif_base_ +0x268,0x0,0}, \
	{_ispif_base_ +0x26c,0x0,0}, \
	{_ispif_base_ +0x288,0x0,0}, \
	{_ispif_base_ +0x28c,0x0,0}, \
	{_ispif_base_ +0x400,0x40,0}, \
	{_ispif_base_ +0x408,0x0,0}, \
	{_ispif_base_ +0x40c,0x0,0}, \
	{_ispif_base_ +0x410,0x0,0}, \
	{_ispif_base_ +0x430,0xffffffff,0}, \
	{_ispif_base_ +0x434,0xffffffff,0}, \
	{_ispif_base_ +0x438,0xffffffff,0}, \
	{_ispif_base_ +0x444,0x0,0}, \
	{_ispif_base_ +0x448,0xaaaaaaaa,0}, \
	{_ispif_base_ +0x44c,0xaaaaaaaa,0}, \
	{_ispif_base_ +0x454,0x0,0}, \
	{_ispif_base_ +0x458,0x0,0}, \
	{_ispif_base_ +0x464,0x0,0}, \
	{_ispif_base_ +0x468,0x0,0}, \
	{_ispif_base_ +0x46c,0x0,0}, \
	{_ispif_base_ +0x488,0x0,0}, \
	{_ispif_base_ +0x48c,0x0,0}, \
	{_ispif_base_ +0x01c,0x1,0}, \
	{_ispif_base_ +0x208,0x0,0}, \
	{_ispif_base_ +0x20c,0x0,0}, \
	{_ispif_base_ +0x210,0x0,0}, \
	{_ispif_clk_mux_base_ +0x8,0x2,0}, \
	{_ispif_base_ +0x244,0x20,0}, \
	{_ispif_base_ +0x264,0x1,0}, \
	{_ispif_base_ +0x208,0x0a493249,0}, \
	{_ispif_base_ +0x230,0x0a493249,0}, \
	{_ispif_base_ +0x20c,0x2493249,0}, \
	{_ispif_base_ +0x234,0x2493249,0}, \
	{_ispif_base_ +0x210,0x1249,0}, \
	{_ispif_base_ +0x238,0x1249,0}, \
	{_ispif_base_ +0x408,0x0a493249,0}, \
	{_ispif_base_ +0x430,0x0a493249,0}, \
	{_ispif_base_ +0x40c,0x2493249,0}, \
	{_ispif_base_ +0x434,0x2493249,0}, \
	{_ispif_base_ +0x410,0x1249,0}, \
	{_ispif_base_ +0x438,0x1249,0}, \
	{_ispif_base_ +0x01c,0x1,0}, \
	{_ispif_base_ +0x248,0xfffffdff,0}

#define MMSS_HLOS1_VOTE_ALL_MMSS_SMMU_GDS 0x008C8104
#define MMSS_HLOS2_VOTE_ALL_MMSS_SMMU_GDS 0x008C9104
#define MSS_HLOS1_VOTE_VFE_SMMU_GDS  0x008C8098
#define MMSS_HLOS2_VOTE_VFE_SMMU_GDS 0x008C9098
#define MMSS_HLOS1_VOTE_JPEG_SMMU_GDS  0x008C8094
#define MMSS_HLOS2_VOTE_JPEG_SMMU_GDS  0x008C9094
#define MMSS_HLOS1_VOTE_CPP_SMMU_GDS  0x008C8090
#define MMSS_HLOS2_VOTE_CPP_SMMU_GDS  0x008C9090
#define GCC_HLOS1_VOTE_ALL_MMSS_SMMU_GDS  0x0037D040
#define GCC_HLOS2_VOTE_ALL_MMSS_SMMU_GDS  0x0037E040

#define MMSS_HLOS1_VOTE_ALL_MMSS_SMMU_CLK 0x008C8100
#define MMSS_HLOS2_VOTE_ALL_MMSS_SMMU_CLK 0x008C9100

#define MMSS_HLOS1_VOTE_VFE_SMMU_CLK 0x008C8018
#define MMSS_HLOS2_VOTE_VFE_SMMU_CLK 0x008C9018

#define MMSS_HLOS1_VOTE_JPEG_SMMU_CLK 0x008C8014
#define MMSS_HLOS2_VOTE_JPEG_SMMU_CLK 0x008C9014

#define MMSS_HLOS1_VOTE_CPP_SMMU_CLK  0x008C8010
#define MMSS_HLOS2_VOTE_CPP_SMMU_CLK  0x008C9010

#define GCC_HLOS1_VOTE_ALL_MMSS_SMMU_CLK 0x0037D01C
#define GCC_HLOS2_VOTE_ALL_MMSS_SMMU_CLK 0x0037E01C

struct camera_hw_reg_array stop_vfe_stream[] ={
	{0x00A100a0, 0, 0},
	{0x00A104ac, 2, 0},
};

struct camera_hw_reg_array other_init_regs[] ={
	// SMMU setup for VFE
	{0x37E040, 0, 1},
	{0x37E01C, 1, 1},
	{0xDA0800, 0x80000000, 0},
	{0xDA0C00, 0, 0},
	{0xDA1000, 0x10003, 0},
	//gcc_mss_mnoc_bimc_axi_clk
	{0x0038A004, 1, 0},
	// mmagic_bimc_noc_cfg_ahb_clk
	{0x008C5298, 1, 0},
	// mmagic_camss_noc_cfg_ahb_clk
	{0x008C3C48, 1, 0},
};


struct camera_hw_reg_array hw_csi1_phy_init_regs[] = {
	CSI_PHY_INIT_ADV7481(CSI1_PHY_BASE,CSI1_CLK_MUX_BASE) };



struct camera_hw_reg_array hw_csi2_phy_init_regs[] = {
	CSI_PHY_INIT(CSI2_PHY_BASE,CSI2_CLK_MUX_BASE) };

struct camera_hw_reg_array hw_csi1_d_init_regs[] = {
	CSI_D_INIT_ADV7481(CSI1_D_BASE) };


struct camera_hw_reg_array hw_csi2_d_init_regs[] = {
	CSI_D_INIT(CSI2_D_BASE) };

struct camera_hw_reg_array hw_vfe0_init_regs_adv7481[] = {
	VFE_INIT_ADV7481(VFE0_BASE, VBIF0_BASE) };

struct camera_hw_reg_array hw_ispif_init_regs_adv7481[] = {
	ISPIF_INIT_ADV7481(ISPIF_BASE,ISPIF_CLK_MUX_BASE) };

struct camera_hw_reg_array hw_vfe0_init_regs[] = {
	VFE_INIT(VFE0_BASE, VBIF0_BASE) };

struct camera_hw_reg_array hw_ispif_init_regs[] = {
	ISPIF_INIT(ISPIF_BASE,ISPIF_CLK_MUX_BASE) };

// mipi
static uint32_t ldo2[][14]=
{
	{
		LDOA_RES_TYPE, 2,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_ENABLE,
		KEY_MICRO_VOLT, 4, 1250000,
		KEY_CURRENT, 4, 16,
	},
};

// Cam vana supply
static uint32_t ldo29[][14]=
{
	{
		LDOA_RES_TYPE, 29,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_ENABLE,
		KEY_MICRO_VOLT, 4, 2800000,
		KEY_CURRENT, 4, 5,
	},

};

// Cam vana supply
static uint32_t ldo17[][14]=
{
	{
		LDOA_RES_TYPE, 17,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_ENABLE,
		KEY_MICRO_VOLT, 4, 2500000,
		KEY_CURRENT, 4, 5,
	},

};

// Cam vio supply
static uint32_t lvs1[][14]=
{
	{
		VS_RES_TYPE, 1,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_ENABLE,
		KEY_MICRO_VOLT, 4, 1800000,
		KEY_CURRENT, 4, 5,
	},
};

static uint32_t smps3[][14]=
{
	{
		SMPS_RES_TYPE, 3,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_ENABLE,
		KEY_MICRO_VOLT, 4, 1300000,
		KEY_CURRENT, 4, 5,
	},

};




void camera_gpio_init(void) {
	// CSI i2c setup
	gpio_tlmm_config(17, 1, GPIO_OUTPUT, GPIO_PULL_UP,
		GPIO_2MA, GPIO_ENABLE);
	gpio_tlmm_config(18, 1, GPIO_OUTPUT, GPIO_PULL_UP,
		GPIO_2MA, GPIO_ENABLE);

	gpio_tlmm_config(19, 1, GPIO_OUTPUT, GPIO_PULL_UP,
		GPIO_2MA, GPIO_ENABLE);
	gpio_tlmm_config(20, 1, GPIO_OUTPUT, GPIO_PULL_UP,
		GPIO_2MA, GPIO_ENABLE);

	// Cam vana
	rpm_send_data(&ldo29[0][0], 36, RPM_REQUEST_TYPE);
	// Cam VIO
	rpm_send_data(&lvs1[0][0], 36, RPM_REQUEST_TYPE);

	// Turn on mipi rail.
	rpm_send_data(&ldo2[0][0], 36, RPM_REQUEST_TYPE);
}

void camera_adv_gpio_init(void) {
	// CSI i2c setup
	gpio_tlmm_config(17, 1, GPIO_OUTPUT, GPIO_PULL_UP,
		GPIO_2MA, GPIO_ENABLE);
	gpio_tlmm_config(18, 1, GPIO_OUTPUT, GPIO_PULL_UP,
		GPIO_2MA, GPIO_ENABLE);

	gpio_tlmm_config(19, 1, GPIO_OUTPUT, GPIO_PULL_UP,
		GPIO_2MA, GPIO_ENABLE);
	gpio_tlmm_config(20, 1, GPIO_OUTPUT, GPIO_PULL_UP,
		GPIO_2MA, GPIO_ENABLE);

	// Cam vana
	rpm_send_data(&ldo17[0][0], 36, RPM_REQUEST_TYPE);
	// Cam VIO
	rpm_send_data(&lvs1[0][0], 36, RPM_REQUEST_TYPE);
	rpm_send_data(&smps3[0][0], 36, RPM_REQUEST_TYPE);

	// Turn on mipi rail.
	rpm_send_data(&ldo2[0][0], 36, RPM_REQUEST_TYPE);
}

void camera_adv_pmic_gpio_init(void)
{
	struct pm8x41_gpio gpio = {
		.direction = PM_GPIO_DIR_OUT,
		.function = PM_GPIO_FUNC_HIGH,
		.vin_sel = 2,
		.output_buffer = PM_GPIO_OUT_CMOS,
		.out_strength = PM_GPIO_OUT_DRIVE_LOW,
	};

	pm8x41_gpio_config(4, &gpio);
	mdelay_optimal(10);
	pm8x41_gpio_set(4, 0);
	mdelay_optimal(10);
	pm8x41_gpio_set(4, 1);
}

int msm_vfe_poll_irq(int irq_num, int timeout_poll_num)
{
	uint32_t irq;
	int poll_count=0;
	int ping;
	int rc = 0;

	irq = msm_camera_io_r_mb(MMSS_A_VFE_0_IRQ_STATUS_0);

	if(irq_num == PING_PONG_IRQ_MASK) {
		while(!(irq &PING_PONG_IRQ_MASK) ) {
			mdelay_optimal(1);
			irq = msm_camera_io_r_mb(MMSS_A_VFE_0_IRQ_STATUS_0);
			poll_count++;
			if(poll_count>timeout_poll_num) {
				rc = -1;
				break;
			}
		}
		if(rc == 0) {
			msm_camera_io_w_mb(PING_PONG_IRQ_MASK,
				MMSS_A_VFE_0_IRQ_CLEAR_0);
			msm_camera_io_w_mb(0x1, MMSS_A_VFE_0_IRQ_CMD);
		}
	}

	if(rc == 0) {
		ping = msm_camera_io_r_mb(MMSS_A_VFE_0_BUS_PING_PONG_STATUS);
		return (ping&0x1);
	}

	return rc;
}

unsigned msm_cci_poll_irq(int irq_num,int master)
{
	uint32_t irq;
	uint32_t poll_count=0;

	irq = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_IRQ_STATUS_0_ADDR);

	if(irq_num == CCI_IRQ_STATUS_0_RST_DONE_ACK_BMSK) {
		while(!(irq &CCI_IRQ_STATUS_0_RST_DONE_ACK_BMSK)) {
			irq = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_IRQ_STATUS_0_ADDR);
			poll_count++;
			if(poll_count > MAX_POLL_COUNT)
				goto error_exit;
			mdelay_optimal(1);
		}
		msm_camera_io_w_mb(CCI_IRQ_STATUS_0_RST_DONE_ACK_BMSK, CCI_DEV_BASE + CCI_IRQ_CLEAR_0_ADDR);
		msm_camera_io_w_mb(0x1, CCI_DEV_BASE + CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);
		if (cci_master_info[master].reset_pending == TRUE) {
			cci_master_info[master].reset_pending = FALSE;
		}
	}

	if(cci_master == 1) {
		if(irq_num == CCI_IRQ_STATUS_0_I2C_M1_RD_DONE_BMSK) {
			while(!(irq &CCI_IRQ_STATUS_0_I2C_M1_RD_DONE_BMSK)) {
				irq = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_IRQ_STATUS_0_ADDR);
				poll_count++;
				if(poll_count > MAX_POLL_COUNT)
					goto error_exit;
				mdelay_optimal(1);

			}
			msm_camera_io_w_mb(CCI_IRQ_STATUS_0_I2C_M1_RD_DONE_BMSK, CCI_DEV_BASE + CCI_IRQ_CLEAR_0_ADDR);
			msm_camera_io_w_mb(0x1, CCI_DEV_BASE + CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);

			cci_master_info[master].status = 0;
		}
	}

	if(cci_master == 0) {
		if(irq_num == CCI_IRQ_STATUS_0_I2C_M0_RD_DONE_BMSK) {
			while(!(irq &CCI_IRQ_STATUS_0_I2C_M0_RD_DONE_BMSK)) {
				irq = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_IRQ_STATUS_0_ADDR);
				poll_count++;
				if(poll_count > MAX_POLL_COUNT) {
					pr_err(CRITICAL, "CCI_IRQ_STATUS_0_I2C_M0_RD_DONE_BMSK::irq = %d\n", irq);
					goto error_exit;
				}
				mdelay_optimal(1);
			}
			msm_camera_io_w_mb(CCI_IRQ_STATUS_0_I2C_M0_RD_DONE_BMSK, CCI_DEV_BASE + CCI_IRQ_CLEAR_0_ADDR);
			msm_camera_io_w_mb(0x1, CCI_DEV_BASE + CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);

			cci_master_info[master].status = 0;
		}
	}
	if(cci_master == 1) {
		if(irq_num == CCI_IRQ_STATUS_0_I2C_M1_Q1_REPORT_BMSK) {
			while(!(irq &CCI_IRQ_STATUS_0_I2C_M1_Q1_REPORT_BMSK)) {
				irq = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_IRQ_STATUS_0_ADDR);
				poll_count++;
				if(poll_count > MAX_POLL_COUNT) {
					goto error_exit;
				}
				mdelay_optimal(1);
			}
			cci_master_info[master].q_free[1] = 0;
			cci_master_info[master].status = 0;
			if (cci_master_info[master].done_pending[1] == 1) {
				cci_master_info[master].done_pending[1] = 0;
			}
			msm_camera_io_w_mb(CCI_IRQ_STATUS_0_I2C_M1_Q1_REPORT_BMSK, CCI_DEV_BASE + CCI_IRQ_CLEAR_0_ADDR);
			msm_camera_io_w_mb(0x1, CCI_DEV_BASE + CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);
		}
		if(irq_num == CCI_IRQ_STATUS_0_I2C_M1_Q0_REPORT_BMSK) {
			while(!(irq &CCI_IRQ_STATUS_0_I2C_M1_Q0_REPORT_BMSK)) {
				irq = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_IRQ_STATUS_0_ADDR);
				poll_count++;
				if(poll_count > MAX_POLL_COUNT) {
					goto error_exit;
				}
				mdelay_optimal(1);
			}
			cci_master_info[master].q_free[0] = 0;
			cci_master_info[master].status = 0;
			if (cci_master_info[master].done_pending[0] == 1) {
				cci_master_info[master].done_pending[0] =0;
			}
			msm_camera_io_w_mb(CCI_IRQ_STATUS_0_I2C_M1_Q0_REPORT_BMSK, CCI_DEV_BASE + CCI_IRQ_CLEAR_0_ADDR);
			msm_camera_io_w_mb(0x1, CCI_DEV_BASE + CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);
		}
	}

	if(cci_master == 0) {
		if(irq_num == CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK) {
			while(!(irq &CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK)) {
				irq = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_IRQ_STATUS_0_ADDR);
				poll_count++;
				if(poll_count > MAX_POLL_COUNT)
					goto error_exit;
				mdelay_optimal(1);
			}
			cci_master_info[master].q_free[1] = 0;
			cci_master_info[master].status = 0;
			if (cci_master_info[master].done_pending[1] == 1) {
				cci_master_info[master].done_pending[1] = 0;
			}
			msm_camera_io_w_mb(CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK, CCI_DEV_BASE + CCI_IRQ_CLEAR_0_ADDR);
			msm_camera_io_w_mb(0x1, CCI_DEV_BASE + CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);
		}

		if(irq_num == CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK) {
			while(!(irq &CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK)) {
				irq = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_IRQ_STATUS_0_ADDR);
				poll_count++;
				if(poll_count > MAX_POLL_COUNT){
					pr_err(CRITICAL, "CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK ::irq = %d\n", irq);
					goto error_exit;
				}
				mdelay_optimal(1);
			}
			cci_master_info[master].q_free[0] = 0;
			cci_master_info[master].status = 0;
			if (cci_master_info[master].done_pending[0] == 1) {
				cci_master_info[master].done_pending[0] =0;
			}
			msm_camera_io_w_mb(CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK, CCI_DEV_BASE + CCI_IRQ_CLEAR_0_ADDR);
			msm_camera_io_w_mb(0x1, CCI_DEV_BASE + CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);
		}
	}

	return 0;
	error_exit:
		pr_err(CRITICAL, "msm_cci_poll_irq::CCI timeout waiting for %d",irq_num);
		return -1;
}

void msm_hw_init(struct camera_hw_reg_array *pdata,int size,int flag) {
	int i = 0;

	for (i = 0; i < size; i++) {
		msm_camera_io_w_mb(pdata->reg_data,pdata->reg_addr);
		if(pdata->delay)
			mdelay_optimal(pdata->delay);
		pdata++;
	}

}
int32_t msm_cci_init(int master) {
	msm_camera_io_w_mb(0,
		CCI_DEV_BASE + CCI_IRQ_MASK_0_ADDR);

	if (master == 0) {
		msm_camera_io_w_mb(CCI_M0_RESET_RMSK,
			CCI_DEV_BASE + CCI_RESET_CMD_ADDR);
	}
	else {
		msm_camera_io_w_mb(CCI_M1_RESET_RMSK,
			CCI_DEV_BASE + CCI_RESET_CMD_ADDR);
	}

	/* wait for reset done irq */
	msm_cci_poll_irq(CCI_IRQ_STATUS_0_RST_DONE_ACK_BMSK,0);
	return 0;
}

struct msm_cci_clk_params_t {
	uint16_t hw_thigh;
	uint16_t hw_tlow;
	uint16_t hw_tsu_sto;
	uint16_t hw_tsu_sta;
	uint16_t hw_thd_dat;
	uint16_t hw_thd_sta;
	uint16_t hw_tbuf;
	uint8_t hw_scl_stretch_en;
	uint8_t hw_trdhld;
	uint8_t hw_tsp;
};

enum msm_cci_cmd_type {
	MSM_CCI_INIT,
	MSM_CCI_RELEASE,
	MSM_CCI_SET_SID,
	MSM_CCI_SET_FREQ,
	MSM_CCI_SET_SYNC_CID,
	MSM_CCI_I2C_READ,
	MSM_CCI_I2C_WRITE,
	MSM_CCI_I2C_WRITE_SEQ,
	MSM_CCI_I2C_WRITE_ASYNC,
	MSM_CCI_GPIO_WRITE,
	MSM_CCI_I2C_WRITE_SYNC,
	MSM_CCI_I2C_WRITE_SYNC_BLOCK,
};

struct msm_cci_clk_params_t gcci_clk_params =
{
	20,
	28,
	21,
	21,
	13,
	18,
	32,
	0,
	6,
	3,
};

enum msm_cci_i2c_cmd_type {
	CCI_I2C_SET_PARAM_CMD = 1,
	CCI_I2C_WAIT_CMD,
	CCI_I2C_WAIT_SYNC_CMD,
	CCI_I2C_WAIT_GPIO_EVENT_CMD,
	CCI_I2C_TRIG_I2C_EVENT_CMD,
	CCI_I2C_LOCK_CMD,
	CCI_I2C_UNLOCK_CMD,
	CCI_I2C_REPORT_CMD,
	CCI_I2C_WRITE_CMD,
	CCI_I2C_READ_CMD,
	CCI_I2C_WRITE_DISABLE_P_CMD,
	CCI_I2C_READ_DISABLE_P_CMD,
	CCI_I2C_WRITE_CMD2,
	CCI_I2C_WRITE_CMD3,
	CCI_I2C_REPEAT_CMD,
	CCI_I2C_INVALID_CMD,
};
#define CCI_I2C_QUEUE_0_SIZE 64
#define CCI_I2C_QUEUE_1_SIZE 16

int32_t msm_cci_validate_queue(uint32_t len,int master,int queue)
{
	int32_t rc = 0;
	uint32_t read_val = 0;
	uint32_t reg_offset = master * 0x200 + queue * 0x100;
	uint32_t max_queue_size;
	int irq_wait;

    if(master == 0) {
		if(queue == 1) {
			max_queue_size = CCI_I2C_QUEUE_1_SIZE;
			irq_wait = CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK;
		} else {
			max_queue_size = CCI_I2C_QUEUE_0_SIZE;
			irq_wait = CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK;
		}
	} else {
		if(queue == 1) {
			max_queue_size = CCI_I2C_QUEUE_1_SIZE;
			irq_wait = CCI_IRQ_STATUS_0_I2C_M1_Q1_REPORT_BMSK;
		} else {
			max_queue_size = CCI_I2C_QUEUE_0_SIZE;
			irq_wait = CCI_IRQ_STATUS_0_I2C_M1_Q0_REPORT_BMSK;
		}
	}

	read_val = msm_camera_io_r_mb(CCI_DEV_BASE +
		CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR + reg_offset);
	if ((read_val + len + 1) > max_queue_size) {
		uint32_t reg_val = 0;
		uint32_t report_val = CCI_I2C_REPORT_CMD | (1 << 8);
		CDBG("%s:%d CCI_I2C_REPORT_CMD\n", __func__, __LINE__);
		msm_camera_io_w_mb(report_val,
			CCI_DEV_BASE + CCI_I2C_M0_Q0_LOAD_DATA_ADDR +
			reg_offset);
		read_val++;
		CDBG("%s:%d CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR %d, queue: %d\n",
			__func__, __LINE__, read_val, queue);
		msm_camera_io_w_mb(read_val, CCI_DEV_BASE +
			CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR + reg_offset);
		reg_val = 1 << ((master * 2) + queue);
		CDBG("%s:%d CCI_QUEUE_START_ADDR\n", __func__, __LINE__);
		cci_master_info[master].done_pending[queue] = 1;
		msm_camera_io_w_mb(reg_val, CCI_DEV_BASE +
			CCI_QUEUE_START_ADDR);
		CDBG("%s line %d wait_for_completion_timeout\n",
			__func__, __LINE__);
		cci_master_info[master].q_free[queue] = 1;

		rc = msm_cci_poll_irq(irq_wait,master);
		if (rc < 0) {
			pr_err(CRITICAL, "%s: wait_for_completion_timeout %d\n",
				 __func__, __LINE__);
			return rc;
		}
		rc = cci_master_info[master].status;
		if (rc < 0)
			pr_err(CRITICAL, "%s failed rc %d\n", __func__, rc);
	}
	return rc;
}

int32_t msm_cci_set_clk_param(int master)
{
	struct msm_cci_clk_params_t *clk_params = &gcci_clk_params;

	if (0 == master) {
		msm_camera_io_w_mb(clk_params->hw_thigh << 16 |
			clk_params->hw_tlow,
			CCI_DEV_BASE + CCI_I2C_M0_SCL_CTL_ADDR);
		msm_camera_io_w_mb(clk_params->hw_tsu_sto << 16 |
			clk_params->hw_tsu_sta,
			CCI_DEV_BASE + CCI_I2C_M0_SDA_CTL_0_ADDR);
		msm_camera_io_w_mb(clk_params->hw_thd_dat << 16 |
			clk_params->hw_thd_sta,
			CCI_DEV_BASE + CCI_I2C_M0_SDA_CTL_1_ADDR);
		msm_camera_io_w_mb(clk_params->hw_tbuf,
			CCI_DEV_BASE + CCI_I2C_M0_SDA_CTL_2_ADDR);
		msm_camera_io_w_mb(clk_params->hw_scl_stretch_en << 8 |
			clk_params->hw_trdhld << 4 | clk_params->hw_tsp,
			CCI_DEV_BASE + CCI_I2C_M0_MISC_CTL_ADDR);
	} else if (1 == master) {
		msm_camera_io_w_mb(clk_params->hw_thigh << 16 |
			clk_params->hw_tlow,
			CCI_DEV_BASE + CCI_I2C_M1_SCL_CTL_ADDR);
		msm_camera_io_w_mb(clk_params->hw_tsu_sto << 16 |
			clk_params->hw_tsu_sta,
			CCI_DEV_BASE + CCI_I2C_M1_SDA_CTL_0_ADDR);
		msm_camera_io_w_mb(clk_params->hw_thd_dat << 16 |
			clk_params->hw_thd_sta,
			CCI_DEV_BASE + CCI_I2C_M1_SDA_CTL_1_ADDR);
		msm_camera_io_w_mb(clk_params->hw_tbuf,
			CCI_DEV_BASE + CCI_I2C_M1_SDA_CTL_2_ADDR);
		msm_camera_io_w_mb(clk_params->hw_scl_stretch_en << 8 |
			clk_params->hw_trdhld << 4 | clk_params->hw_tsp,
			CCI_DEV_BASE + CCI_I2C_M1_MISC_CTL_ADDR);
	}

	return 0;
}

#define CCI_I2C_MAX_READ 8192
#define CCI_I2C_MAX_WRITE 8192

int32_t msm_cci_write_i2c_queue(int master,int queue,int val, int length)
{
	int32_t rc = 0;
	uint32_t reg_offset = master * 0x200 + queue * 0x100;

	rc = msm_cci_validate_queue(length, master, queue);
	if (rc < 0) {
		pr_err(CRITICAL, "%s: failed %d", __func__, __LINE__);
		return rc;
	}

	msm_camera_io_w_mb(val, CCI_DEV_BASE + CCI_I2C_M0_Q0_LOAD_DATA_ADDR +
		reg_offset);
	return rc;
}

int msm_cci_i2c_read(uint32_t address,
						 int32_t length,
						 uint32_t *read_val,
						 unsigned int slave_address,
						 int master,
						 int queue,
						 int addr_type,
						 int data_type)
{
	int rc = 0;
	uint32_t val = 0;
	int32_t read_words = 0, exp_words = 0;
	int32_t index = 0, first_byte = 0;
	uint32_t i = 0;
	int max_queue_size;

	CDBG("%s line %d\n", __func__, __LINE__);

	/* Set the I2C Frequency */
	rc = msm_cci_set_clk_param(master);//(cci_dev, c_ctrl);
	if (rc < 0) {
		pr_err(CRITICAL, "%s:%d msm_cci_set_clk_param failed rc = %d\n",
			__func__, __LINE__, rc);
		return rc;
	}

	/*
	 * Call validate queue to make sure queue is empty before starting.
	 * If this call fails, don't proceed with i2c_read call. This is to
	 * avoid overflow / underflow of queue
	 */

	if(queue ==1)
		max_queue_size = CCI_I2C_QUEUE_1_SIZE;
	else
		max_queue_size = CCI_I2C_QUEUE_0_SIZE;

	rc = msm_cci_validate_queue(max_queue_size,master, queue);
	if (rc < 0) {
		pr_err(CRITICAL, "%s:%d Initial validataion failed rc %d\n", __func__,
			__LINE__, rc);
		goto ERROR;
	}

	val = CCI_I2C_SET_PARAM_CMD | (slave_address>>1) << 4 |
		3 << 16 |
		0 << 18;
	rc = msm_cci_write_i2c_queue(master, queue,val, length);
	if (rc < 0) {
		CDBG("%s failed line %d\n", __func__, __LINE__);
		goto ERROR;
	}

	val = CCI_I2C_LOCK_CMD;
	rc = msm_cci_write_i2c_queue(master, queue,val, length);
	if (rc < 0) {
		CDBG("%s failed line %d\n", __func__, __LINE__);
		goto ERROR;
	}

	if (addr_type >= MSM_CAMERA_I2C_ADDR_TYPE_MAX) {
		CDBG("%s failed line %d\n", __func__, __LINE__);
		goto ERROR;
	}

	if (addr_type == MSM_CAMERA_I2C_BYTE_ADDR)
		val = CCI_I2C_WRITE_DISABLE_P_CMD | (addr_type << 4) |
			((address & 0xFF) << 8);
	if (addr_type == MSM_CAMERA_I2C_WORD_ADDR)
		val = CCI_I2C_WRITE_DISABLE_P_CMD | (addr_type << 4) |
			(((address & 0xFF00) >> 8) << 8) |
			((address & 0xFF) << 16);
	rc = msm_cci_write_i2c_queue(master, queue,val, length);
	if (rc < 0) {
		CDBG("%s failed line %d\n", __func__, __LINE__);
		goto ERROR;
	}

	val = CCI_I2C_READ_CMD | (length << 4);
	rc = msm_cci_write_i2c_queue(master, queue,val, length);
	if (rc < 0) {
		CDBG("%s failed line %d\n", __func__, __LINE__);
		goto ERROR;
	}

	val = CCI_I2C_UNLOCK_CMD;
	rc = msm_cci_write_i2c_queue(master, queue,val, length);
	if (rc < 0) {
		CDBG("%s failed line %d\n", __func__, __LINE__);
		goto ERROR;
	}

	val = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR
			+ master * 0x200 + queue * 0x100);
	msm_camera_io_w_mb(val, CCI_DEV_BASE + CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR
			+ master * 0x200 + queue * 0x100);

	val = 1 << ((master * 2) + queue);
	msm_camera_io_w_mb(val, CCI_DEV_BASE + CCI_QUEUE_START_ADDR);

	// Wait for read done.
	if(master == 0) {
		rc = msm_cci_poll_irq(CCI_IRQ_STATUS_0_I2C_M0_RD_DONE_BMSK,master);
	} else {
		rc = msm_cci_poll_irq(CCI_IRQ_STATUS_0_I2C_M1_RD_DONE_BMSK,master);
	}

	if (rc < 0) {
		CDBG("%s failed to get IRQ line %d\n", __func__, __LINE__);
		goto ERROR;
	}

	read_words = msm_camera_io_r_mb(CCI_DEV_BASE +
		CCI_I2C_M0_READ_BUF_LEVEL_ADDR + master * 0x100);
	exp_words = ((length / 4) + 1);
	if (read_words != exp_words) {
		pr_err(CRITICAL, "%s:%d read_words = %d, exp words = %d\n", __func__,
			__LINE__, read_words, exp_words);
		val = 0;
		rc = -1;
		goto ERROR;
	}
	index = 0;
	first_byte = 0;
	do {
		uint32_t data = 0;
		val = msm_camera_io_r_mb(CCI_DEV_BASE +
			CCI_I2C_M0_READ_DATA_ADDR + master * 0x100);

		for (i = 0; (i < 4) && (index < length); i++) {
			if (!first_byte) {
				first_byte++;
			} else {
				data =
					(val  >> (i * 8)) & 0xFF;
				CDBG("%s data[%d] 0x%x\n", __func__, index,
					data);
				*read_val = data;
				index++;
			}
		}
	} while (--read_words > 0);
	CDBG("sa = %x address = %x val = %x\n", slave_address, address, *read_val);

ERROR:
	msm_cci_flush_queue(cci_master);
	return rc;
}

int32_t msm_cci_calc_cmd_len(int cmd_size,
							struct camera_i2c_reg_array *pArray,
							uint32_t *pack,
							int add_size,
							int data_size,
							int queue)
{
	uint8_t i;
	uint32_t len = 0;
	uint8_t data_len = 0, addr_len = 0;
	uint8_t pack_max_len;
	uint32_t size = cmd_size;
	struct camera_i2c_reg_array *cmd = pArray;
	int addr_type = add_size;
	int data_type = data_size;
	uint8_t payload_size = MSM_CCI_WRITE_DATA_PAYLOAD_SIZE_11;

	*pack = 0;
	addr_len = addr_type;
	data_len =data_type;
		len = data_len + addr_len;
		pack_max_len = size < (payload_size-len) ?
			size : (payload_size-len);
		for (i = 0; i < pack_max_len;) {
			if (cmd->delay || ((cmd - pArray) >= (cmd_size - 1)))
				break;
			if (cmd->reg_addr + 1 == (cmd+1)->reg_addr) {
				len += data_len;
				*pack += data_len;
			} else {
				break;
			}
			i += data_len;
			cmd++;
		}

	if (len > payload_size) {
		pr_err(CRITICAL, "Len error: %d", len);
		return -1;
	}

	len += 1; /*add i2c WR command*/
	len = len/4 + 1;

	return len;
}

void msm_cci_load_report_cmd(int  master,int queue)
{
	uint32_t reg_offset = master * 0x200 + queue * 0x100;
	uint32_t read_val = msm_camera_io_r_mb(CCI_DEV_BASE +
		CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR + reg_offset);
	uint32_t report_val = CCI_I2C_REPORT_CMD | (1 << 8);

	msm_camera_io_w_mb(report_val,
		CCI_DEV_BASE + CCI_I2C_M0_Q0_LOAD_DATA_ADDR +
		reg_offset);
	read_val++;

	msm_camera_io_w_mb(read_val, CCI_DEV_BASE+
		CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR + reg_offset);
}


uint32_t msm_cci_wait(int master,int queue)
{
	int32_t rc = 0;

	if(queue == 1 && master == 1)
		rc = msm_cci_poll_irq(CCI_IRQ_STATUS_0_I2C_M1_Q1_REPORT_BMSK, master);
	else if (queue == 0 && master == 1)
		rc = msm_cci_poll_irq(CCI_IRQ_STATUS_0_I2C_M1_Q0_REPORT_BMSK,master);
	else if (queue == 1 && master == 0)
		rc = msm_cci_poll_irq(CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK, master);
	else
		rc = msm_cci_poll_irq(CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK, master);

	if (rc < 0) {
		pr_err(CRITICAL, "%s: %d failed rc %d\n", __func__, __LINE__, rc);
		return rc;
	}

	rc = cci_master_info[master].status;
	if (rc < 0) {
		pr_err(CRITICAL, "%s: %d failed rc %d\n", __func__, __LINE__, rc);
		return rc;
	}
	return 0;
}

int32_t msm_cci_wait_report_cmd(int master,int queue)
{
	uint32_t reg_val = 1 << ((master * 2) + queue);
	msm_cci_load_report_cmd( master, queue);
	cci_master_info[master].q_free[queue] = 1;
	cci_master_info[master].done_pending[queue] = 1;
	msm_camera_io_w_mb(reg_val, CCI_DEV_BASE +
		CCI_QUEUE_START_ADDR);
	return msm_cci_wait( master, queue);
}

void msm_cci_process_half_q(int master,int queue)
{
	uint32_t reg_val = 1 << ((master * 2) + queue);
	if (0 == cci_master_info[master].q_free[queue]) {
		msm_cci_load_report_cmd( master, queue);
		cci_master_info[master].q_free[queue] = 1;
		msm_camera_io_w_mb(reg_val, CCI_DEV_BASE +
			CCI_QUEUE_START_ADDR);
	}
}

int32_t msm_cci_process_full_q(int master,int queue)
{
	int32_t rc = 0;
	if (1 == cci_master_info[master].q_free[queue]) {
		cci_master_info[master].done_pending[queue] = 1;
		rc = msm_cci_wait(master, queue);
		if (rc < 0) {
			pr_err(CRITICAL, "%s: %d failed rc %d\n", __func__, __LINE__, rc);
			return rc;
		}
	} else {
		rc = msm_cci_wait_report_cmd( master, queue);
		if (rc < 0) {
			pr_err(CRITICAL, "%s: %d failed rc %d\n", __func__, __LINE__, rc);
			return rc;
		}
	}
	return rc;
}

int32_t msm_cci_get_queue_free_size(int master,int queue)
{
	uint32_t read_val = 0;
	uint32_t reg_offset = master * 0x200 + queue * 0x100;
	int max_queue_size;

	if(queue ==1)
		max_queue_size = CCI_I2C_QUEUE_1_SIZE;
	else
		max_queue_size = CCI_I2C_QUEUE_0_SIZE;

	read_val = msm_camera_io_r_mb(CCI_DEV_BASE +
		CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR + reg_offset);

	return max_queue_size -read_val;
}

#define PRIORITY_QUEUE 0

int32_t msm_cci_lock_queue(int master,int queue, uint32_t en)
{
	uint32_t val;

	if (queue != PRIORITY_QUEUE)
		return 0;

	val = en ? CCI_I2C_LOCK_CMD : CCI_I2C_UNLOCK_CMD;
	return msm_cci_write_i2c_queue(master, queue,val,1);
}


int32_t msm_cci_transfer_end(int master,int queue)
{
	int32_t rc = 0;

	if (0 == cci_master_info[master].q_free[queue]) {
		rc = msm_cci_lock_queue(master, queue, 0);
		if (rc < 0) {
			pr_err(CRITICAL, "%s failed line %d\n", __func__, __LINE__);
			return rc;
		}
		rc = msm_cci_wait_report_cmd(master, queue);
		if (rc < 0) {
			pr_err(CRITICAL, "%s: %d failed rc %d\n", __func__, __LINE__, rc);
			return rc;
		}
	} else {
		cci_master_info[master].done_pending[queue] =1;
		rc = msm_cci_wait(master, queue);
		if (rc < 0) {
			pr_err(CRITICAL, "%s: %d failed rc %d\n", __func__, __LINE__, rc);
			return rc;
		}
		rc = msm_cci_lock_queue(master, queue, 0);
		if (rc < 0) {
			pr_err(CRITICAL, "%s failed line %d\n", __func__, __LINE__);
			return rc;
		}
		rc = msm_cci_wait_report_cmd(master, queue);
		if (rc < 0) {
			pr_err(CRITICAL, "%s: %d failed rc %d\n", __func__, __LINE__, rc);
			return rc;
		}
	}
	return rc;
}

int32_t msm_cci_data_queue(struct camera_i2c_reg_array *pArray,
							int arraySize,
							int slave_address,
							int queue,
							int sync_en,
							int add_size,
							int data_size,
							int master)
{
	uint16_t i = 0, j = 0, k = 0, h = 0, len = 0;
	int32_t rc = 0, free_size = 0, en_seq_write = 0;
	uint32_t cmd = 0, delay = 0;
	uint8_t data[2*65];
	uint16_t reg_addr = 0;
	uint16_t cmd_size = arraySize;//i2c_msg->size;
	struct camera_i2c_reg_array *i2c_cmd = pArray;

	uint32_t read_val = 0;
	uint32_t reg_offset;
	uint32_t val = 0;
	uint32_t max_queue_size;
	int data_type = data_size;
	int addr_type = add_size;

	if (addr_type >= MSM_CAMERA_I2C_ADDR_TYPE_MAX) {
		pr_err(CRITICAL, "%s:%d failed: invalid addr_type 0x%X\n",
			__func__, __LINE__, addr_type);
		return -1;
	}
	if (data_type >= MSM_CAMERA_I2C_DATA_TYPE_MAX) {
		pr_err(CRITICAL, "%s:%d failed: invalid data_type 0x%X\n",
			__func__, __LINE__, data_type);
		return -1;
	}
	reg_offset = master * 0x200 + queue * 0x100;

	val = CCI_I2C_SET_PARAM_CMD | (slave_address>>1) << 4 |
		3 << 16 |
		0 << 18;

	msm_camera_io_w_mb(val, CCI_DEV_BASE + CCI_I2C_M0_Q0_LOAD_DATA_ADDR +
		reg_offset);

	cci_master_info[master].q_free[queue] = 0;

	if(queue ==1)
		max_queue_size = CCI_I2C_QUEUE_1_SIZE;
	else
		max_queue_size = CCI_I2C_QUEUE_0_SIZE;

	reg_addr = pArray->reg_addr;

	rc = msm_cci_lock_queue(master, queue, 1);
	if (rc < 0) {
		pr_err(CRITICAL, "%s failed line %d\n", __func__, __LINE__);
		return rc;
	}

	while (cmd_size) {
		uint32_t pack = 0;

		len = msm_cci_calc_cmd_len(cmd_size,
			i2c_cmd, &pack, add_size,data_size,queue);
		if (len <= 0) {
			pr_err(CRITICAL, "%s failed line %d\n", __func__, __LINE__);
			return -1;
		}

		read_val = msm_camera_io_r_mb(CCI_DEV_BASE +
			CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR + reg_offset);
		/* + 1 - space alocation for Report CMD*/
		if ((read_val + len + 1) > max_queue_size/2) {
			if ((read_val + len + 1) > max_queue_size) {
				rc = msm_cci_process_full_q(master, queue);
				if (rc < 0) {
					pr_err(CRITICAL, "%s failed line %d\n",
						__func__, __LINE__);
					return rc;
				}
				continue;
			}
			msm_cci_process_half_q(master, queue);
		}

		delay = i2c_cmd->delay;
		i = 0;
		data[i++] = CCI_I2C_WRITE_CMD;

		reg_addr = i2c_cmd->reg_addr;

		if (en_seq_write == 0) {
			/* either byte or word addr */
			if (addr_type == MSM_CAMERA_I2C_BYTE_ADDR)
				data[i++] = reg_addr;
			else {
				data[i++] = (reg_addr & 0xFF00) >> 8;
				data[i++] = reg_addr & 0x00FF;
			}
		}
		/* max of 10 data bytes */
		do {
			if (data_type == MSM_CAMERA_I2C_BYTE_DATA) {
				data[i++] = i2c_cmd->reg_data;
				reg_addr++;
			} else {
				if ((i + 1) <= MSM_CCI_WRITE_DATA_PAYLOAD_SIZE_11) {
					data[i++] = (i2c_cmd->reg_data &
						0xFF00) >> 8; /* MSB */
					data[i++] = i2c_cmd->reg_data &
						0x00FF; /* LSB */
					reg_addr++;
				} else
					break;
			}
			i2c_cmd++;
			--cmd_size;
		} while (((cmd == MSM_CCI_I2C_WRITE_SEQ) || pack--) &&
				(cmd_size > 0) && (i <= MSM_CCI_WRITE_DATA_PAYLOAD_SIZE_11));//cci_dev->payload_size));
		free_size = msm_cci_get_queue_free_size( master,
				queue);
		if ((cmd == MSM_CCI_I2C_WRITE_SEQ) &&
			((i-1) == MSM_CCI_WRITE_DATA_PAYLOAD_SIZE_11) &&
			0 && cmd_size > 0 &&
			free_size > BURST_MIN_FREE_SIZE) {
				data[0] |= 0xF0;
				en_seq_write = 1;
		} else {
			data[0] |= ((i-1) << 4);
			en_seq_write = 0;
		}
		len = ((i-1)/4) + 1;

		read_val = msm_camera_io_r_mb(CCI_DEV_BASE +
			CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR + reg_offset);
		for (h = 0, k = 0; h < len; h++) {
			cmd = 0;
			for (j = 0; (j < 4 && k < i); j++)
				cmd |= (data[k++] << (j * 8));
			msm_camera_io_w_mb(cmd, CCI_DEV_BASE +
				CCI_I2C_M0_Q0_LOAD_DATA_ADDR +
				master * 0x200 + queue * 0x100);

			read_val += 1;
			msm_camera_io_w_mb(read_val, CCI_DEV_BASE +
				CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR + reg_offset);
		}

		if ((delay > 0) && (delay < CCI_MAX_DELAY) &&
			en_seq_write == 0) {
			cmd = (uint32_t)((delay * 4915) /
				0x100);
			cmd <<= 4;
			cmd |= CCI_I2C_WAIT_CMD;
			msm_camera_io_w_mb(cmd, CCI_DEV_BASE +
				CCI_I2C_M0_Q0_LOAD_DATA_ADDR +
				master * 0x200 + queue * 0x100);
			read_val += 1;
			msm_camera_io_w_mb(read_val, CCI_DEV_BASE +
				CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR + reg_offset);
		}
	}

	rc = msm_cci_transfer_end(master, queue);
	if (rc < 0) {
		pr_err(CRITICAL, "%s: %d failed rc %d\n", __func__, __LINE__, rc);
		return rc;
	}
	return rc;
}

int32_t msm_cci_i2c_write(struct camera_i2c_reg_array *pArray,
						  int arraySize,
						  int slave_address,
						  int queue,
						  int sync_en,
						  int add_size,
						  int data_size,
						  int readback,
						  int master){
	int32_t rc = 0;
	uint32_t max_queue_size;

	CDBG("%s: addr = 0x%X, size = %d, master = %d, queue = %d\n", __func__, slave_address, arraySize, master, queue);

	/* Set the I2C Frequency */
	rc = msm_cci_set_clk_param(master);
	if (rc < 0) {
		pr_err(CRITICAL, "%s:%d msm_cci_set_clk_param failed rc = %d\n",
			__func__, __LINE__, rc);
		return rc;
	}
	if(queue == 1)
		max_queue_size = CCI_I2C_QUEUE_1_SIZE;
	else
		max_queue_size = CCI_I2C_QUEUE_0_SIZE;

	/*
	 * Call validate queue to make sure queue is empty before starting.
	 * If this call fails, don't proceed with i2c_write call. This is to
	 * avoid overflow / underflow of queue
	 */
	rc = msm_cci_validate_queue(max_queue_size-1,
		master, queue);

	if (rc < 0) {
		pr_err(CRITICAL, "%s:%d Initial validataion failed rc %d\n",
		__func__, __LINE__, rc);
		goto ERROR;
	}

	rc = msm_cci_data_queue(pArray,
							arraySize,
							slave_address,
							queue,
							sync_en,
							add_size,
							data_size,
							master);

	if (rc < 0) {
		CDBG("%s failed line %d\n", __func__, __LINE__);
		goto ERROR;
	}
	return 0;
ERROR:
	msm_cci_flush_queue(cci_master);
	return rc;
}

static void msm_cci_flush_queue(int master)
{
	msm_camera_io_w_mb(1 << master,CCI_DEV_BASE + CCI_HALT_REQ_ADDR);
	msm_cci_init(master);
}

void target_early_camera_init(const char *camera_type)
{

	if (!strcmp(camera_type,"analog")) {
		dprintf(CRITICAL, "camera is %s\n", camera_type);
		cam_type = ANALOG;
		raw_size = 720*507*2;
		csi = 1;
		cci_master = 0;
		msg_count = 0;

	} else if (!strcmp(camera_type,"digital")) {
		dprintf(CRITICAL, "camera is %s\n", camera_type);
		cam_type = DIGITAL;
		raw_size = 1280*720*2;
		csi = 2;
		cci_master = 1;
	}


#ifdef EARLYDOMAIN_SUPPORT
	num_configs = get_cam_data(csi, &cam_data);
	if (num_configs) {
		camera_gdsc_enable(1);
		if (cci_master == 0) {
			camera_clocks_enable(csi, 1);
			msm_cci_init(0);
			camera_adv_gpio_init();
			camera_adv_pmic_gpio_init();
		} else {
			camera_clocks_enable(csi, 1);
			msm_cci_init(1);
			camera_gpio_init();
		}
	}
#endif
}

int early_camera_check_rev(unsigned int *pExpected_rev_id, unsigned int num_id, unsigned int revision)
{
	unsigned int i = 0;
	for (i=0; i < num_id; i++) {
		if (*pExpected_rev_id == revision)
			return 0;
		pExpected_rev_id++;
	}
	return -1;
}

static void early_camera_setup_layer(int display_id,
	struct target_layer *cam_layer)
{
	uint32_t rvc_display_id = 0;
	uint32_t fb_index = SPLIT_DISPLAY_0;

	if (!cam_layer->layer) {
		cam_layer->layer = target_display_acquire_layer(disp_ptr,
							"as", kFormatYCbCr422H2V1Packed);
		if (!cam_layer->layer) {
			dprintf(CRITICAL, "Camera Layer acquire failed\n");
			cam_layer->layer = NULL;
			return;
		}
	}

	/* RVC FB usually uses FB0 */
	rvc_display_id = target_display_get_rvc_display_id();
	fb = target_display_get_fb(rvc_display_id, fb_index);
	if (fb == NULL){
		dprintf(CRITICAL, "Display FB acquire failed\n");
		return;
	}

	update_cam.disp = disp_ptr;
	update_cam.layer_list = cam_layer;

	update_cam.num_layers = 1;
	fb->bpp = 16;
	fb->format = kFormatYCbCr422H2V1Packed;
	fb->z_order = RVC_LAYER_ZORDER;
	cam_layer->fb[fb_index] = *fb;

	cam_layer->src_rect_x[fb_index] = 0;
	cam_layer->src_rect_y[fb_index] = 0;
	cam_layer->dst_rect_x[fb_index] = 0;
	cam_layer->dst_rect_y[fb_index] = 0;

	if (cam_type == ANALOG) {
		cam_layer->src_width[fb_index] =  720;
		cam_layer->src_height[fb_index] =  240;
		cam_layer->dst_width[fb_index] =   1920;
		cam_layer->dst_height[fb_index] =  1080;
	} else if (cam_type == DIGITAL) {
		cam_layer->src_width[fb_index]  = 1280;
		cam_layer->src_height[fb_index] = 720;
		cam_layer->dst_width[fb_index]  = 1920;
		cam_layer->dst_height[fb_index] = 1080;
	}

	if (disp->splitter_display_enabled)
		cam_layer->dst_width[fb_index] /= MAX_SPLIT_DISPLAY;

	dprintf(INFO, "fb_cnt in camera_layer = %d\n", cam_layer->valid_fb_cnt);
}

static void early_camera_remove_layer(struct target_layer *cam_layer)
{
	struct fbcon_config *cam_fb = NULL;

	if (!cam_layer) {
		dprintf(CRITICAL, "invalid cam_layer\n");
		return;
	}

	cam_fb = target_display_search_fb(cam_layer,
		kFormatYCbCr422H2V1Packed, RVC_LAYER_ZORDER);
	if (cam_fb) {
		target_display_setup_fb(cam_fb, NULL,
			kFormatYCbCr422H2V1Packed, RVC_LAYER_ZORDER, false, 24);
		cam_layer->valid_fb_cnt = 1;

		target_release_layer(cam_layer, cam_fb);
	}
}

static int early_camera_setup_display(void)
{
	uint32_t rvc_display_id;

	rvc_display_id = target_display_get_rvc_display_id();
	disp_ptr = target_display_open(rvc_display_id);

	if (disp_ptr == NULL) {
		pr_err(CRITICAL, "Display open failed\n");
		return -1;
	}
	disp = target_get_display_info(disp_ptr);
	if (disp == NULL){
		pr_err(CRITICAL, "Display info failed\n");
		return -1;
	}
	return 0;
}

static int early_camera_start(void *arg) {
	int rc = -1;
	int i2c_wr_iter = 0;
	int retry_count = 0;
	int index = 0;

	num_configs = get_cam_data(csi, &cam_data);

	if(num_configs == 0) {
		pr_err(CRITICAL,
			"Early Camera not configured for this target exiting\n");
		return -1;
	}

	early_camera_setup_display();

	if (cam_type == ANALOG) {
		hw_vfe0_init_regs_adv7481[46].reg_data = (unsigned int)VFE_PING_ADDR;
		hw_vfe0_init_regs_adv7481[47].reg_data = hw_vfe0_init_regs_adv7481[46].reg_data +raw_size;
		hw_vfe0_init_regs_adv7481[48].reg_data = (unsigned int)VFE_PONG_ADDR;
		hw_vfe0_init_regs_adv7481[49].reg_data = hw_vfe0_init_regs_adv7481[48].reg_data+ raw_size;
	} else if (cam_type == DIGITAL) {
		hw_vfe0_init_regs[46].reg_data = (unsigned int)VFE_PING_ADDR;
		hw_vfe0_init_regs[47].reg_data = hw_vfe0_init_regs[46].reg_data +raw_size;
		hw_vfe0_init_regs[48].reg_data = (unsigned int)VFE_PONG_ADDR;
		hw_vfe0_init_regs[49].reg_data = hw_vfe0_init_regs[48].reg_data+ raw_size;
	}

	read_val = 0;
	while(rc != 0) {
		rc = msm_cci_i2c_read(cam_data[0].i2c_revision_id_reg,
						1,
						&read_val,
						cam_data[0].i2c_slave_address,
						cci_master,
						queue_id,
						cam_data[0].i2c_num_bytes_address,
						cam_data[0].i2c_num_bytes_data);
		if(rc == 0) {
			rc = early_camera_check_rev(&cam_data[0].i2c_revision_id_val[0],
				cam_data[0].i2c_revision_id_num, read_val);
			if(rc == 0)
				break;
		} else {
			if(retry_count > REV_CHECK_MAX_POLL_TRY) {
				break;
			}
			read_val = 0;
		}
		retry_count++;
		pr_err(CRITICAL, "Early Camera retry revision check %d\n",retry_count);
		mdelay_optimal(5);
	}

	if (rc == -1) {
		unsigned int i = 0;
		for (i = 0; i < cam_data[0].i2c_revision_id_num; i++) {
			pr_err(CRITICAL,
				"Early Camera - I2c revision %d doesn't match for this target %d exiting\n",
				read_val,
				cam_data[0].i2c_revision_id_val[i]);
		}
		goto exit;
	}

	if (cam_type == ANALOG)
		i2c_wr_iter = 1;
	else if (cam_type == DIGITAL)
		i2c_wr_iter = num_configs - 2;

	// Write setup configs for i2c devices.
	// Last config is for starting the camera.
	for(index = 0;index < i2c_wr_iter; index++) {
		if(index == 1) {

			rc = -1;
			while( rc != 0) {
				mdelay_optimal(5);
				// Check if sensor is present and exit if not
				rc = msm_cci_i2c_read(cam_data[index].i2c_revision_id_reg,
								1,
								&read_val,
								cam_data[index].i2c_slave_address,
								cci_master,
								queue_id,
								cam_data[index].i2c_num_bytes_address,
								cam_data[index].i2c_num_bytes_data);
				if(rc == 0) {
					rc = early_camera_check_rev(&cam_data[index].i2c_revision_id_val[0],
						cam_data[index].i2c_revision_id_num, read_val);
					if(rc == 0)
						break;
				} else {
					if(retry_count > REV_CHECK_MAX_POLL_TRY) {
						break;
					}
					read_val = 0;
				}
				retry_count++;
				pr_err(CRITICAL,
					"Early Camera retry sensor revision check %d\n",retry_count);
			}
			if (rc == -1) {
				unsigned int i = 0;
				for (i = 0; i < cam_data[index].i2c_revision_id_num; i++) {
					pr_err(CRITICAL,
					"Early Camera - I2c sensor rev id %x doesn't match for this target %x exiting\n",
					read_val,
					cam_data[index].i2c_revision_id_val[i]);
				}
				goto exit;
			}
		}
		msm_cci_i2c_write(cam_data[index].i2c_regs,
						cam_data[index].size,
						cam_data[index].i2c_slave_address,
						queue_id,
						0,
						cam_data[index].i2c_num_bytes_address,
						cam_data[index].i2c_num_bytes_data,
						0,
						cci_master);
	}

	// SMMU and anything else missed.
	msm_hw_init(&other_init_regs[0],
		sizeof(other_init_regs) / sizeof(other_init_regs[0]),0);

	if (csi == 1) {
		msm_hw_init(&hw_csi1_phy_init_regs[0],
				sizeof(hw_csi1_phy_init_regs) / sizeof(hw_csi1_phy_init_regs[0]),0);
		msm_hw_init(&hw_csi1_d_init_regs[0],
				sizeof(hw_csi1_d_init_regs) / sizeof(hw_csi1_d_init_regs[0]),0);

		msm_hw_init(&hw_vfe0_init_regs_adv7481[0],
			sizeof(hw_vfe0_init_regs_adv7481) / sizeof(hw_vfe0_init_regs_adv7481[0]),0);
		msm_hw_init(&hw_ispif_init_regs_adv7481[0],
			sizeof(hw_ispif_init_regs_adv7481) / sizeof(hw_ispif_init_regs_adv7481[0]),0);


	} else {
		msm_hw_init(&hw_csi2_phy_init_regs[0],
			sizeof(hw_csi2_phy_init_regs) / sizeof(hw_csi2_phy_init_regs[0]),0);

		msm_hw_init(&hw_csi2_d_init_regs[0],
			sizeof(hw_csi2_d_init_regs) / sizeof(hw_csi2_d_init_regs[0]),0);

		msm_hw_init(&hw_vfe0_init_regs[0],
			sizeof(hw_vfe0_init_regs) / sizeof(hw_vfe0_init_regs[0]),0);
		msm_hw_init(&hw_ispif_init_regs[0],
			sizeof(hw_ispif_init_regs) / sizeof(hw_ispif_init_regs[0]),0);

	}

	if (cam_type == ANALOG) {
		// Select CVBS SDP in ADV Bridge chip
		msm_cci_i2c_write(cam_data[1].i2c_regs,
						cam_data[1].size,
						cam_data[1].i2c_slave_address,
						queue_id,
						0,
						cam_data[1].i2c_num_bytes_address,
						cam_data[1].i2c_num_bytes_data,
						0,cci_master);

		// Start CVBS Camera Port.
		msm_cci_i2c_write(cam_data[2].i2c_regs,
						cam_data[2].size,
						cam_data[2].i2c_slave_address,
						queue_id,
						0,
						cam_data[2].i2c_num_bytes_address,
						cam_data[2].i2c_num_bytes_data,
						0,cci_master);

		msm_cci_i2c_write(cam_data[3].i2c_regs,
						cam_data[3].size,
						cam_data[3].i2c_slave_address,
						queue_id,
						0,
						cam_data[3].i2c_num_bytes_address,
						cam_data[3].i2c_num_bytes_data,
						0,cci_master);

		msm_cci_i2c_write(cam_data[4].i2c_regs,
						cam_data[4].size,
						cam_data[4].i2c_slave_address,
						queue_id,
						0,
						cam_data[4].i2c_num_bytes_address,
						cam_data[4].i2c_num_bytes_data,
						0,cci_master);

		// wait for lock 1
		rc = -1;
		while (rc == -1) {
			msm_cci_i2c_read(cam_data[5].i2c_revision_id_reg,
								1,
								&read_val,
								cam_data[5].i2c_slave_address,
								cci_master,
								queue_id,
								cam_data[5].i2c_num_bytes_address,
								cam_data[5].i2c_num_bytes_data);

			rc = early_camera_check_rev(&cam_data[5].i2c_revision_id_val[0],
				cam_data[5].i2c_revision_id_num, read_val);


			if (rc == -1) {
				if (msg_count == 0)
					pr_err(CRITICAL,
							"Early Camera - lock1 not detected %x doesn't match for this target %x retrying\n",
							read_val,
							cam_data[5].i2c_revision_id_val[0]);
				msg_count++;
				mdelay_optimal(1);
			} else {
				break;
			}
			if(msg_count > MAX_CAM_ERROR_EXIT) {
				pr_err(CRITICAL,
						"Early Camera - lock1 not detected exiting after %d retrys\n",
						msg_count);
				goto exit;
			}
		}

		msm_cci_i2c_write(cam_data[6].i2c_regs,
						cam_data[6].size,
						cam_data[6].i2c_slave_address,
						queue_id,
						0,
						cam_data[6].i2c_num_bytes_address,
						cam_data[6].i2c_num_bytes_data,
						0,cci_master);


		//////////////////////
		msg_count = 0;
		// wait for lock 2
		rc = -1;
		while(rc==-1) {
			msm_cci_i2c_read(cam_data[7].i2c_revision_id_reg,
								1,
								&read_val,
								cam_data[7].i2c_slave_address,
								cci_master,
								queue_id,
								cam_data[7].i2c_num_bytes_address,
								cam_data[7].i2c_num_bytes_data);

			rc = early_camera_check_rev(&cam_data[7].i2c_revision_id_val[0],
					cam_data[7].i2c_revision_id_num, read_val);


			if(rc==-1) {
				if(msg_count == 0)
					pr_err(CRITICAL, "Early Camera - lock2 not detected %x doesn't match for this target %x retrying\n",
							read_val,
							cam_data[7].i2c_revision_id_val[0]);
				msg_count++;
				mdelay_optimal(1);
			} else {
				break;
			}
			if(msg_count > MAX_CAM_ERROR_EXIT) {
				pr_err(CRITICAL,
						"Early Camera - lock2 not detected exiting after %d retrys\n",
						msg_count);
				goto exit;
			}
		}

		mdelay_optimal(100);

		adv7481_intr_enable();



		msm_cci_i2c_write(cam_data[8].i2c_regs,
						cam_data[8].size,
						cam_data[8].i2c_slave_address,
						queue_id,
						0,
						cam_data[8].i2c_num_bytes_address,
						cam_data[8].i2c_num_bytes_data,
						0,cci_master);

		msm_cci_i2c_write(cam_data[9].i2c_regs,
						cam_data[9].size,
						cam_data[9].i2c_slave_address,
						queue_id,
						0,
						cam_data[9].i2c_num_bytes_address,
						cam_data[9].i2c_num_bytes_data,
						0,cci_master);
	} else if (cam_type == DIGITAL) {
		// Start cam
		msm_cci_i2c_write(cam_data[num_configs-2].i2c_regs,
						cam_data[num_configs-2].size,
						cam_data[num_configs-2].i2c_slave_address,
						queue_id,
						0,
						cam_data[num_configs-2].i2c_num_bytes_address,
						cam_data[num_configs-2].i2c_num_bytes_data,
						0,
						cci_master);
	}

	// Signal Kernel early camera is active.
	set_early_service_active_bit(EARLY_CAMERA);

	return 0;
	exit:
	return -1;
}

#define SENSOR_STOP_IDX 3
void early_camera_stop(void *cam_layer)
{
	struct target_layer *camera_layer;
	unsigned int csid_irq = 0;
	unsigned int irq_cci = 0;
	struct fbcon_config *cam_fb = NULL;

	if (!cam_layer) {
		dprintf(CRITICAL, "Invalid cam_layer\n");
		return;
	}
	camera_layer = (struct target_layer *)cam_layer;
	// stop VFE writes to memory
	msm_hw_init(&stop_vfe_stream[0],
		sizeof(stop_vfe_stream) / sizeof(stop_vfe_stream[0]),0);

	if (cam_type == ANALOG) {
		struct camera_i2c_reg_array reg_stop[1] = {{ 0x0, 0x81, 0}};

		// Stop ispif
		msm_camera_io_w_mb(0xaaaaaaaa,0x00a31248);
		// Reset ping pong to top of buffer.
		msm_camera_io_w_mb(1,0x00A10080);
		//stop csi tx
		msm_cci_i2c_write(&reg_stop[0],
						1,
						cam_data[9].i2c_slave_address,
						queue_id,
						0,
						1,
						1,
						0,cci_master);
	} else if (cam_type == DIGITAL) {
		// Stop cam
		msm_cci_i2c_write(cam_data[SENSOR_STOP_IDX].i2c_regs,
						cam_data[SENSOR_STOP_IDX].size,
						cam_data[SENSOR_STOP_IDX].i2c_slave_address,
						0,
						0,
						cam_data[SENSOR_STOP_IDX].i2c_num_bytes_address,
						cam_data[SENSOR_STOP_IDX].i2c_num_bytes_data,
						0,
						cci_master);
	}

	cam_fb = target_display_search_fb(camera_layer,
		kFormatYCbCr422H2V1Packed, RVC_LAYER_ZORDER);
	if (cam_fb) {
		target_display_setup_fb(cam_fb, NULL,
			kFormatYCbCr422H2V1Packed, RVC_LAYER_ZORDER, false, 24);
		camera_layer->valid_fb_cnt = 1;

		target_release_layer(camera_layer, cam_fb);
	}

	// Clear any interrupt status registers before exit.
	msm_camera_io_w_mb(0xffffffff, MMSS_A_VFE_0_IRQ_CLEAR_0);
	msm_camera_io_w_mb(0xffffffff, MMSS_A_VFE_0_IRQ_CLEAR_1);
	msm_camera_io_w_mb(0xffffffff, MMSS_A_VFE_0_IRQ_CMD);

	irq_cci = msm_camera_io_r(CCI_DEV_BASE + CCI_IRQ_STATUS_0_ADDR);
	msm_camera_io_w_mb(irq_cci, CCI_DEV_BASE + CCI_IRQ_CLEAR_0_ADDR);
	msm_camera_io_w_mb(0x1, CCI_DEV_BASE + CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);

	csid_irq = msm_camera_io_r(0x00A3046C);
	msm_camera_io_w_mb(csid_irq, 0x00A30464);

	// Signal Kernel were done to allow camera daemon to start.
	clear_early_service_active_bit(EARLY_CAMERA);
}

int early_camera_on(void)
{
#ifdef DEBUG_T32
	while(delay_to_attach_t32 != 1)
	{
		mdelay_optimal(1000);
		pr_err(CRITICAL, "Waiting to attach to t.32\n");
	}
#endif

	if(early_rvc_gpio < MAX_GPIO_COUNT)
		gpio_triggered = gpio_get(early_rvc_gpio);
	else if (early_rvc_timeout > 0)
		gpio_triggered = TRUE;
	// else Neither RVC GPIO is set correctly nor RVC time out is set correctly.
	// Exit with GPIO Triggered as False (default value)
	return gpio_triggered;
}

//Clear frame buffer in case of error
void early_camera_clear_buffers(void)
{
	memset((void *)VFE_PING_ADDR, EARLY_CAMERA_FILL_PATTERN, raw_size);
	memset((void *)VFE_PONG_ADDR, EARLY_CAMERA_FILL_PATTERN, raw_size);
}

int early_camera_flip(void *cam_layer, bool firstframe, bool mode_change)
{
	static int buffer_cleared = 0;
	uint32_t rvc_display_id;

	struct target_layer *camera_layer;
#ifdef DEBUG_T32
	while(delay_to_attach_t32 != 1) {
		mdelay_optimal(1000);
		pr_err(CRITICAL, "Waiting to attach to t.32\n");
	}
#endif

	if (!cam_layer) {
		dprintf(CRITICAL, "Input invalid\n");
		return -1;
	}
	camera_layer = (struct target_layer *)cam_layer;

	if (cam_type == ANALOG) {
		//static int msg_count = 0;
		static int camera_lock_status = 1;
		struct camera_i2c_reg_array reg_stop[1] = {{ 0x0, 0x81, 0}};
		struct camera_i2c_reg_array reg_start[1] = {{ 0x0, 0x21, 0}};

		camera_lock_status = adv7481_lock_status();

		// There has been a change in lock status.
		if(camera_lock_status == 0)
		{
			// Stop vfe
			msm_camera_io_w_mb(0,0x00A100a0);
			msm_camera_io_w_mb(0xf,0x00A104ac);
			// Stop ispif
			msm_camera_io_w_mb(0xaaaaaaaa,0x00a31248);
			// Reset ping pong to top of buffer.
			msm_camera_io_w_mb(1,0x00A10080);
			//stop csi tx
			msm_cci_i2c_write(&reg_stop[0],
							1,
							cam_data[9].i2c_slave_address,
							queue_id,
							0,
							1,
							1,
							0,cci_master);
			if(buffer_cleared == 0) {
				early_camera_clear_buffers();
				buffer_cleared =1;
			}
			goto ERROR;
		}
		if (camera_lock_status == 1) {
			//start csi tx
			msm_cci_i2c_write(&reg_start[0],
							1,
							cam_data[9].i2c_slave_address,
							queue_id,
							0,
							1,
							1,
							0,cci_master);
			// Start ispif on next frame.
			msm_camera_io_w_mb(0xfffffdff,0x00a31248);
			// Start vfe write masters.
			//start vfe
			msm_camera_io_w_mb(1,0x00A100a0);
			// reg update
			msm_camera_io_w_mb(0xf,0x00A104ac);
		}
		if(camera_lock_status == 0)
			goto ERROR;
	}

	// wait for ping pong irq;
	ping = msm_vfe_poll_irq(PING_PONG_IRQ_MASK,VFE_TIME_OUT_POLL_NUM);
	if(ping==-1)
	{
		ping = 0;
		if(buffer_cleared == 0) {
			pr_err(CRITICAL, "timeout waiting for ping/pong\n");
			// Stop vfe
			msm_camera_io_w_mb(0,0x00A100a0);
			msm_camera_io_w_mb(2,0x00A104ac);
			// Stop ispif
			msm_camera_io_w_mb(0xaaaaaaaa,0x00a31248);
			// Reset ping pong to top of buffer.
			msm_camera_io_w_mb(1,0x00A10080);
			early_camera_clear_buffers();
			buffer_cleared =1;
			// Start ispif on next frame.
			msm_camera_io_w_mb(0xfffffdff,0x00a31248);
			// Start vfe write masters.
			// update ping /pong
			msm_camera_io_w_mb(2,0x00A104ac);
			//start vfe
			msm_camera_io_w_mb(1,0x00A100a0);
		}
		goto ERROR;
	}
	buffer_cleared = 0;

	if (firstframe == true) {
		cam_place_kpi_marker("Camera Image in memory");
		bs_set_timestamp(BS_EARLY_CAMERA_START);
	}

	rvc_display_id = target_display_get_rvc_display_id();

	frame_counter++;
	if (gpio_triggered) {
		if(firstframe || mode_change)
			early_camera_setup_layer(rvc_display_id,
				camera_layer);

		if (camera_layer->fb) {
			if (cam_type == ANALOG) {
				if (ping)
					camera_layer->fb[SPLIT_DISPLAY_0].base =
						(void *)(VFE_PING_ADDR+CVBS_HEADER_SIZE);
				else
					camera_layer->fb[SPLIT_DISPLAY_0].base =
						(void *)(VFE_PONG_ADDR+CVBS_HEADER_SIZE);

			} else if (cam_type == DIGITAL) {
				if (ping)
					camera_layer->fb[SPLIT_DISPLAY_0].base = (void *)VFE_PING_ADDR;
				else
					camera_layer->fb[SPLIT_DISPLAY_0].base = (void *)VFE_PONG_ADDR;
			}
			camera_layer->valid_fb_cnt++;
		}

		if (firstframe || mode_change) {
				/* setup one inactive fb for status switching between RVC and animation */
					target_display_setup_fb(&camera_layer->fb[camera_layer->valid_fb_cnt],
						NULL, kFormatRGB888, SPLASH_SPLIT_0_LAYER_ZORDER, false, 24);
					camera_layer->valid_fb_cnt++;
			if (firstframe)
				dprintf(INFO, "camera flip setup once(display id=%d)\n", rvc_display_id);
			target_display_update(&update_cam, 1, rvc_display_id, true, firstframe);
		} else
			target_display_update_pipe(&update_cam, 1, rvc_display_id);
	} else {
		camera_layer->fb[SPLIT_DISPLAY_0].z_order = SPLASH_SPLIT_0_LAYER_ZORDER;
		early_camera_remove_layer(camera_layer);
		camera_layer->layer = NULL;
	}

	if (firstframe == true)
		cam_place_kpi_marker("Camera display post done");

	return 0;

ERROR:
	return -1;
}

int early_camera_init(void)
{
	int msg = 0;
	int rc = 0;

	if (early_camera_enabled) {
		rc  = early_camera_start(&msg);
	}
	else {
		rc =-1;
	}

	return rc;
}

/* Sets early camera enabled or disabled */
void set_early_camera_enabled(bool enabled, uint32_t rvc_timeout, uint32_t rvc_gpio)
{
	early_camera_enabled = enabled;
	pr_err(CRITICAL, "set_early_camera_enabled : %d\n", enabled);
	// update the RVC Time out value in Seconds
	early_rvc_timeout = rvc_timeout;
	// Update the RVC TLMM GPIO value
	early_rvc_gpio = rvc_gpio;
}

