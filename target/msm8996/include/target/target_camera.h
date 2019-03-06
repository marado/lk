/* Copyright (c) 2015-2016, 2018-2019, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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
 *
 */
#ifndef _TARGET_CAMERA_H
#define _TARGET_CAMERA_H

#include <sys/types.h>

//#define BRIDGE_REV_1  // For adashub rev 1 of TI 960 Bridge chip.
#define MAX_CAM_ERROR_EXIT 1500
// Pattern used to fill buffer displayed during error
#define EARLY_CAMERA_FILL_PATTERN 0

// Enable ADV7481 early camera on CSI1
// If not defined TI 964 on CSI2 is used.
#define ADV7481


#define MAX_REV_ID 2

struct camera_i2c_reg_array {
	unsigned short reg_addr;
	unsigned short reg_data;
	unsigned int delay;
};

struct i2c_config_data {
	struct camera_i2c_reg_array *i2c_regs;	// Array of i2c registers to be written
	unsigned int	size;					// Number of elements in the array
	unsigned int	i2c_slave_address;		// Slave address to use for the write
	unsigned int	i2c_num_bytes_address;// Number of bytes used for i2c address
	unsigned int	i2c_num_bytes_data;	// Number of bytes used for i2c data
	unsigned int	i2c_revision_id_num;	// Number of revision id's to check
	unsigned int	i2c_revision_id_val[MAX_REV_ID];	// Expected revision id's of the device
	unsigned int	i2c_revision_id_reg;	// Address of the expected revision id of the device
};

int get_cam_data(int csi, struct i2c_config_data **cam_data);
int early_camera_init(void);
void target_early_camera_init(void);
int early_camera_flip(void);
int early_camera_on(void);

void early_camera_stop(void);

void set_early_camera_enabled(bool enabled);
int msm_cci_i2c_read(uint32_t address,
						 int32_t length,
						 uint32_t *read_val,
						 unsigned int slave_address,
						 int master,
						 int queue,
						 int addr_type,
						 int data_type);

int32_t msm_cci_i2c_write(struct camera_i2c_reg_array *pArray,
						  int arraySize,
						  int slave_address,
						  int queue,
						  int sync_en,
						  int add_size,
						  int data_size,
						  int readback,
						  int master);


#ifdef ADV7481
int adv7481_intr_enable(void);
int adv7481_sdp_isr(void);
void adv7481_isr(void);
int adv7481_lock_status(void);
#endif

#endif
