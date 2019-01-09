/* Copyright (c) 2014-2016, The Linux Foundation. All rights reserved.
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
#include <platform/gpio.h>
#include <gsbi.h>
#include <platform/clock.h>
#include <psci.h>
#include <regulator.h>
#include <rpm-ipc.h>
#include <pm8x41.h>
#include <platform/timer.h>
#include <string.h>
#include <stdlib.h>
#include <reg.h>
#include <target/display.h>
#include <kernel/thread.h>
#include <target/target_camera.h>

//#define DEBUG_LOGS
#ifdef DEBUG_LOGS
#undef CDBG
#define CDBG _dprintf
#else
#undef CDBG
#define CDBG(fmt, args...) do{}while(0)
#endif
#define pr_err dprintf


#define CAMERA_SLAVEADDR 0x48
#define BRIDGE_SLAVEADDR 0x7a

#define ANALOG_CAMERA_SLAVEADDR 0x94 // REG_CSI_TXB_SADDR
#define ADV_BRIDGE_SLAVEADDR 0xE0 // ADV7481_IO_MAP_SLAVE_ADDR (0x70<<1)
#define INT_DEBOUNCE_NUM 10 // Number of locks to check in a row before we consider it locked.

static struct camera_i2c_reg_array ti960_init_regs[] =
{
	/* Global Settings */
	{ 0x10, 0x81, 0},
	{ 0x11, 0x85, 0},
	{ 0x12, 0x89, 0},
	{ 0x13, 0x8d, 0},
	{ 0x1f, 0x6, 0},
	{ 0x32, 0x1, 0},
	{ 0x33, 0x1, 0},
	{ 0x4c, 0x1, 0},
	{ 0x6e, 0x8, 5000},
	{ 0x6e, 0x9, 5000},
	{ 0x70, 0x1f, 0},
	{ 0x58, 0x58, 0},
	{ 0x5c, 0x18, 0},
	{ 0x5d, 0x60, 0},
	{ 0x65, 0x48, 0},
	{ 0x7c, 0x81, 0},
	{ 0x6f, 0x8, 0},
	{ 0x6d, 0x7f, 0}, };

// Start TI 960 RX 0 port
static struct camera_i2c_reg_array ti960_start_regs[] = {
	{ 0xb0, 0x1c, 0},
	{ 0xb1, 0x13, 0},
	{ 0xb2, 0x1f, 0},
	{ 0x33, 0x03, 0},
	{ 0x20, 0xe0, 0},
};

// Start TI 960 RX 0 port
static struct camera_i2c_reg_array ti960_stop_regs[] = {
	{ 0x4c, 0x1, 0},
	{ 0x6e, 0x8, 5000},
};

// Camera init registers
struct camera_i2c_reg_array ov10635_regs[] = {
	{ 0x0103, 0x01, 0 },
	{ 0x301b, 0xff, 0 },
	{ 0x301c, 0xff, 0 },
	{ 0x301a, 0xff, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x300c, 0x61, 0 },
	{ 0x3021, 0x03, 0 },
	{ 0x3011, 0x02, 0 },
	{ 0x6900, 0x0c, 0 },
	{ 0x6901, 0x01, 0 },
	{ 0x3033, 0x08, 0 },
	{ 0x3503, 0x10, 0 },
	{ 0x302d, 0x2f, 0 },
	{ 0x3025, 0x03, 0 },
	{ 0x3003, 0x20, 0 },
	{ 0x3004, 0x03, 0 },
	{ 0x3005, 0x20, 0 },
	{ 0x3006, 0x91, 0 },
	{ 0x3600, 0x74, 0 },
	{ 0x3601, 0x2b, 0 },
	{ 0x3612, 0x00, 0 },
	{ 0x3611, 0x67, 0 },
	{ 0x3633, 0xba, 0 },
	{ 0x3602, 0x2f, 0 },
	{ 0x3603, 0x00, 0 },
	{ 0x3630, 0xa8, 0 },
	{ 0x3631, 0x16, 0 },
	{ 0x3714, 0x10, 0 },
	{ 0x371d, 0x01, 0 },
	{ 0x4300, 0x3a, 0 },
	{ 0x3007, 0x01, 0 },
	{ 0x3024, 0x01, 0 },
	{ 0x3020, 0x0b, 0 },
	{ 0x3702, 0x0d, 0 },
	{ 0x3703, 0x20, 0 },
	{ 0x3704, 0x15, 0 },
	{ 0x3709, 0x28, 0 },
	{ 0x370d, 0x00, 0 },
	{ 0x3712, 0x00, 0 },
	{ 0x3713, 0x20, 0 },
	{ 0x3715, 0x04, 0 },
	{ 0x381d, 0x43, 0 },
	{ 0x6900, 0x01, 0 },
	{ 0x381c, 0xC0, 0 },
	{ 0x3824, 0x10, 0 },
	{ 0x3815, 0x8c, 0 },
	{ 0x3804, 0x05, 0 },
	{ 0x3805, 0x1f, 0 },
	{ 0x3800, 0x00, 0 },
	{ 0x3801, 0x00, 0 },
	{ 0x3806, 0x03, 0 },
	{ 0x3807, 0x01, 0 },
	{ 0x3802, 0x00, 0 },
	{ 0x3803, 0x2e, 0 },
	{ 0x3808, 0x05, 0 },
	{ 0x3809, 0x00, 0 },
	{ 0x380a, 0x02, 0 },
	{ 0x380b, 0xd0, 0 },
	{ 0x380c, 0x06, 0 },
	{ 0x380d, 0xf6, 0 },
	{ 0x380e, 0x03, 0 },
	{ 0x380f, 0x80, 0 },
	{ 0x3811, 0x10, 0 },
	{ 0x381f, 0x0c, 0 },
	{ 0x3621, 0x63, 0 },
	{ 0x5005, 0x08, 0 },
	{ 0x56d5, 0x00, 0 },
	{ 0x56d6, 0x80, 0 },
	{ 0x56d7, 0x00, 0 },
	{ 0x56d8, 0x00, 0 },
	{ 0x56d9, 0x00, 0 },
	{ 0x56da, 0x80, 0 },
	{ 0x56db, 0x00, 0 },
	{ 0x56dc, 0x00, 0 },
	{ 0x56e8, 0x00, 0 },
	{ 0x56e9, 0x7f, 0 },
	{ 0x56ea, 0x00, 0 },
	{ 0x56eb, 0x7f, 0 },
	{ 0x56d0, 0x00, 0 },
	{ 0x5006, 0x24, 0 },
	{ 0x5608, 0x00, 0 },
	{ 0x52d7, 0x06, 0 },
	{ 0x528d, 0x08, 0 },
	{ 0x5293, 0x12, 0 },
	{ 0x52d3, 0x12, 0 },
	{ 0x5288, 0x06, 0 },
	{ 0x5289, 0x20, 0 },
	{ 0x52c8, 0x06, 0 },
	{ 0x52c9, 0x20, 0 },
	{ 0x52cd, 0x04, 0 },
	{ 0x5381, 0x00, 0 },
	{ 0x5382, 0xff, 0 },
	{ 0x5651, 0x00, 0 },
	{ 0x5652, 0x80, 0 },
	{ 0x4605, 0x00, 0 },
	{ 0x4606, 0x07, 0 },
	{ 0x4607, 0x71, 0 },
	{ 0x460a, 0x02, 0 },
	{ 0x460b, 0x70, 0 },
	{ 0x460c, 0x00, 0 },
	{ 0x4620, 0x0e, 0 },
	{ 0x4700, 0x04, 0 },
	{ 0x4701, 0x01, 0 },
	{ 0x4702, 0x00, 0 },
	{ 0x4703, 0x00, 0 },
	{ 0x4704, 0x00, 0 },
	{ 0x4705, 0x00, 0 },
	{ 0x4706, 0x00, 0 },
	{ 0x4707, 0x00, 0 },
	{ 0x4004, 0x08, 0 },
	{ 0x4005, 0x18, 0 },
	{ 0x4001, 0x04, 0 },
	{ 0x4050, 0x20, 0 },
	{ 0x4051, 0x22, 0 },
	{ 0x4057, 0x9c, 0 },
	{ 0x405a, 0x00, 0 },
	{ 0x4202, 0x02, 0 },
	{ 0x3023, 0x10, 0 },
	{ 0x0100, 0x01, 0 },
	{ 0x0100, 0x01, 0 },
	{ 0x6f0e, 0x00, 0 },
	{ 0x6f0f, 0x00, 0 },
	{ 0x460e, 0x08, 0 },
	{ 0x460f, 0x01, 0 },
	{ 0x4610, 0x00, 0 },
	{ 0x4611, 0x01, 0 },
	{ 0x4612, 0x00, 0 },
	{ 0x4613, 0x01, 0 },
	{ 0x4605, 0x00, 0 },
	{ 0x4608, 0x00, 0 },
	{ 0x4609, 0x08, 0 },
	{ 0x6804, 0x00, 0 },
	{ 0x6805, 0x06, 0 },
	{ 0x6806, 0x00, 0 },
	{ 0x5120, 0x00, 0 },
	{ 0x3510, 0x00, 0 },
	{ 0x3504, 0x00, 0 },
	{ 0x6800, 0x00, 0 },
	{ 0x6f0d, 0x00, 0 },
	{ 0x5000, 0xff, 0 },
	{ 0x5001, 0xbf, 0 },
	{ 0x5002, 0xfe, 0 },
	{ 0x503d, 0x00, 0 },
	{ 0xc450, 0x01, 0 },
	{ 0xc452, 0x04, 0 },
	{ 0xc453, 0x00, 0 },
	{ 0xc454, 0x00, 0 },
	{ 0xc455, 0x00, 0 },
	{ 0xc456, 0x00, 0 },
	{ 0xc457, 0x00, 0 },
	{ 0xc458, 0x00, 0 },
	{ 0xc459, 0x00, 0 },
	{ 0xc45b, 0x00, 0 },
	{ 0xc45c, 0x00, 0 },
	{ 0xc45d, 0x00, 0 },
	{ 0xc45e, 0x00, 0 },
	{ 0xc45f, 0x00, 0 },
	{ 0xc460, 0x00, 0 },
	{ 0xc461, 0x01, 0 },
	{ 0xc462, 0x01, 0 },
	{ 0xc464, 0x88, 0 },
	{ 0xc465, 0x00, 0 },
	{ 0xc466, 0x8a, 0 },
	{ 0xc467, 0x00, 0 },
	{ 0xc468, 0x86, 0 },
	{ 0xc469, 0x00, 0 },
	{ 0xc46a, 0x40, 0 },
	{ 0xc46b, 0x50, 0 },
	{ 0xc46c, 0x30, 0 },
	{ 0xc46d, 0x28, 0 },
	{ 0xc46e, 0x60, 0 },
	{ 0xc46f, 0x40, 0 },
	{ 0xc47c, 0x01, 0 },
	{ 0xc47d, 0x38, 0 },
	{ 0xc47e, 0x00, 0 },
	{ 0xc47f, 0x00, 0 },
	{ 0xc480, 0x00, 0 },
	{ 0xc481, 0xff, 0 },
	{ 0xc482, 0x00, 0 },
	{ 0xc483, 0x40, 0 },
	{ 0xc484, 0x00, 0 },
	{ 0xc485, 0x18, 0 },
	{ 0xc486, 0x00, 0 },
	{ 0xc487, 0x18, 0 },
	{ 0xc488, 0x2e, 0 },
	{ 0xc489, 0x80, 0 },
	{ 0xc48a, 0x2e, 0 },
	{ 0xc48b, 0x80, 0 },
	{ 0xc48c, 0x00, 0 },
	{ 0xc48d, 0x04, 0 },
	{ 0xc48e, 0x00, 0 },
	{ 0xc48f, 0x04, 0 },
	{ 0xc490, 0x07, 0 },
	{ 0xc492, 0x20, 0 },
	{ 0xc493, 0x08, 0 },
	{ 0xc498, 0x02, 0 },
	{ 0xc499, 0x00, 0 },
	{ 0xc49a, 0x02, 0 },
	{ 0xc49b, 0x00, 0 },
	{ 0xc49c, 0x02, 0 },
	{ 0xc49d, 0x00, 0 },
	{ 0xc49e, 0x02, 0 },
	{ 0xc49f, 0x60, 0 },
	{ 0xc4a0, 0x04, 0 },
	{ 0xc4a1, 0x00, 0 },
	{ 0xc4a2, 0x06, 0 },
	{ 0xc4a3, 0x00, 0 },
	{ 0xc4a4, 0x00, 0 },
	{ 0xc4a5, 0x10, 0 },
	{ 0xc4a6, 0x00, 0 },
	{ 0xc4a7, 0x40, 0 },
	{ 0xc4a8, 0x00, 0 },
	{ 0xc4a9, 0x80, 0 },
	{ 0xc4aa, 0x0d, 0 },
	{ 0xc4ab, 0x00, 0 },
	{ 0xc4ac, 0x0f, 0 },
	{ 0xc4ad, 0xc0, 0 },
	{ 0xc4b4, 0x01, 0 },
	{ 0xc4b5, 0x01, 0 },
	{ 0xc4b6, 0x00, 0 },
	{ 0xc4b7, 0x01, 0 },
	{ 0xc4b8, 0x00, 0 },
	{ 0xc4b9, 0x01, 0 },
	{ 0xc4ba, 0x01, 0 },
	{ 0xc4bb, 0x00, 0 },
	{ 0xc4be, 0x02, 0 },
	{ 0xc4bf, 0x33, 0 },
	{ 0xc4c8, 0x03, 0 },
	{ 0xc4c9, 0xd0, 0 },
	{ 0xc4ca, 0x0e, 0 },
	{ 0xc4cb, 0x00, 0 },
	{ 0xc4cc, 0x0e, 0 },
	{ 0xc4cd, 0x51, 0 },
	{ 0xc4ce, 0x0e, 0 },
	{ 0xc4cf, 0x51, 0 },
	{ 0xc4d0, 0x04, 0 },
	{ 0xc4d1, 0x80, 0 },
	{ 0xc4e0, 0x04, 0 },
	{ 0xc4e1, 0x02, 0 },
	{ 0xc4e2, 0x01, 0 },
	{ 0xc4e4, 0x10, 0 },
	{ 0xc4e5, 0x20, 0 },
	{ 0xc4e6, 0x30, 0 },
	{ 0xc4e7, 0x40, 0 },
	{ 0xc4e8, 0x50, 0 },
	{ 0xc4e9, 0x60, 0 },
	{ 0xc4ea, 0x70, 0 },
	{ 0xc4eb, 0x80, 0 },
	{ 0xc4ec, 0x90, 0 },
	{ 0xc4ed, 0xa0, 0 },
	{ 0xc4ee, 0xb0, 0 },
	{ 0xc4ef, 0xc0, 0 },
	{ 0xc4f0, 0xd0, 0 },
	{ 0xc4f1, 0xe0, 0 },
	{ 0xc4f2, 0xf0, 0 },
	{ 0xc4f3, 0x80, 0 },
	{ 0xc4f4, 0x00, 0 },
	{ 0xc4f5, 0x20, 0 },
	{ 0xc4f6, 0x02, 0 },
	{ 0xc4f7, 0x00, 0 },
	{ 0xc4f8, 0x04, 0 },
	{ 0xc4f9, 0x0b, 0 },
	{ 0xc4fa, 0x00, 0 },
	{ 0xc4fb, 0x01, 0 },
	{ 0xc4fc, 0x01, 0 },
	{ 0xc4fd, 0x01, 0 },
	{ 0xc4fe, 0x04, 0 },
	{ 0xc4ff, 0x02, 0 },
	{ 0xc500, 0x68, 0 },
	{ 0xc501, 0x74, 0 },
	{ 0xc502, 0x70, 0 },
	{ 0xc503, 0x80, 0 },
	{ 0xc504, 0x05, 0 },
	{ 0xc505, 0x80, 0 },
	{ 0xc506, 0x03, 0 },
	{ 0xc507, 0x80, 0 },
	{ 0xc508, 0x01, 0 },
	{ 0xc509, 0xc0, 0 },
	{ 0xc50a, 0x01, 0 },
	{ 0xc50b, 0xa0, 0 },
	{ 0xc50c, 0x01, 0 },
	{ 0xc50d, 0x2c, 0 },
	{ 0xc50e, 0x01, 0 },
	{ 0xc50f, 0x0a, 0 },
	{ 0xc510, 0x00, 0 },
	{ 0xc511, 0x00, 0 },
	{ 0xc512, 0xe5, 0 },
	{ 0xc513, 0x14, 0 },
	{ 0xc514, 0x04, 0 },
	{ 0xc515, 0x00, 0 },
	{ 0xc518, 0x03, 0 },
	{ 0xc519, 0x48, 0 },
	{ 0xc51a, 0x07, 0 },
	{ 0xc51b, 0x70, 0 },
	{ 0xc2e0, 0x00, 0 },
	{ 0xc2e1, 0x51, 0 },
	{ 0xc2e2, 0x00, 0 },
	{ 0xc2e3, 0xd6, 0 },
	{ 0xc2e4, 0x01, 0 },
	{ 0xc2e5, 0x5e, 0 },
	{ 0xc2e9, 0x01, 0 },
	{ 0xc2ea, 0x7a, 0 },
	{ 0xc2eb, 0x90, 0 },
	{ 0xc2ed, 0x01, 0 },
	{ 0xc2ee, 0x7a, 0 },
	{ 0xc2ef, 0x64, 0 },
	{ 0xc308, 0x00, 0 },
	{ 0xc309, 0x00, 0 },
	{ 0xc30a, 0x00, 0 },
	{ 0xc30c, 0x00, 0 },
	{ 0xc30d, 0x01, 0 },
	{ 0xc30e, 0x00, 0 },
	{ 0xc30f, 0x00, 0 },
	{ 0xc310, 0x01, 0 },
	{ 0xc311, 0x60, 0 },
	{ 0xc312, 0xff, 0 },
	{ 0xc313, 0x08, 0 },
	{ 0xc314, 0x01, 0 },
	{ 0xc315, 0x7f, 0 },
	{ 0xc316, 0xff, 0 },
	{ 0xc317, 0x0b, 0 },
	{ 0xc318, 0x00, 0 },
	{ 0xc319, 0x0c, 0 },
	{ 0xc31a, 0x00, 0 },
	{ 0xc31b, 0xe0, 0 },
	{ 0xc31c, 0x00, 0 },
	{ 0xc31d, 0x14, 0 },
	{ 0xc31e, 0x00, 0 },
	{ 0xc31f, 0xc5, 0 },
	{ 0xc320, 0xff, 0 },
	{ 0xc321, 0x4b, 0 },
	{ 0xc322, 0xff, 0 },
	{ 0xc323, 0xf0, 0 },
	{ 0xc324, 0xff, 0 },
	{ 0xc325, 0xe8, 0 },
	{ 0xc326, 0x00, 0 },
	{ 0xc327, 0x46, 0 },
	{ 0xc328, 0xff, 0 },
	{ 0xc329, 0xd2, 0 },
	{ 0xc32a, 0xff, 0 },
	{ 0xc32b, 0xe4, 0 },
	{ 0xc32c, 0xff, 0 },
	{ 0xc32d, 0xbb, 0 },
	{ 0xc32e, 0x00, 0 },
	{ 0xc32f, 0x61, 0 },
	{ 0xc330, 0xff, 0 },
	{ 0xc331, 0xf9, 0 },
	{ 0xc332, 0x00, 0 },
	{ 0xc333, 0xd9, 0 },
	{ 0xc334, 0x00, 0 },
	{ 0xc335, 0x2e, 0 },
	{ 0xc336, 0x00, 0 },
	{ 0xc337, 0xb1, 0 },
	{ 0xc338, 0xff, 0 },
	{ 0xc339, 0x64, 0 },
	{ 0xc33a, 0xff, 0 },
	{ 0xc33b, 0xeb, 0 },
	{ 0xc33c, 0xff, 0 },
	{ 0xc33d, 0xe8, 0 },
	{ 0xc33e, 0x00, 0 },
	{ 0xc33f, 0x48, 0 },
	{ 0xc340, 0xff, 0 },
	{ 0xc341, 0xd0, 0 },
	{ 0xc342, 0xff, 0 },
	{ 0xc343, 0xed, 0 },
	{ 0xc344, 0xff, 0 },
	{ 0xc345, 0xad, 0 },
	{ 0xc346, 0x00, 0 },
	{ 0xc347, 0x66, 0 },
	{ 0xc348, 0x01, 0 },
	{ 0xc349, 0x00, 0 },
	{ 0x6700, 0x04, 0 },
	{ 0x6701, 0x7b, 0 },
	{ 0x6702, 0xfd, 0 },
	{ 0x6703, 0xf9, 0 },
	{ 0x6704, 0x3d, 0 },
	{ 0x6705, 0x71, 0 },
	{ 0x6706, 0x78, 0 },
	{ 0x6708, 0x05, 0 },
	{ 0x3822, 0x50, 0 },
	{ 0x6f06, 0x6f, 0 },
	{ 0x6f07, 0x00, 0 },
	{ 0x6f0a, 0x6f, 0 },
	{ 0x6f0b, 0x00, 0 },
	{ 0x6f00, 0x03, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x3042, 0xf0, 0 },
	{ 0x301b, 0xf0, 0 },
	{ 0x301c, 0xf0, 0 },
	{ 0x301a, 0xf0, 0 },
	{ 0x0100, 0x00, 0 },
	{ 0x0100, 0x01, 0 },
};

// Updated ADV7481 init Sequence for only CVBS (Analog) Camera
static struct camera_i2c_reg_array adv7481_init_regs[]=
{
	/* Global Settings */
	{ 0xff, 0xff, 5000},
	{ 0x01, 0x76, 0},
	//{ 0x05, 0x4a, 0},
	{ 0x00, 0x30, 000},
	{ 0xf2, 0x01, 0},
	{ 0xf3, 0x4c, 0},
	{ 0xf4, 0x44, 0},
	{ 0xf5, 0x74, 0},
	{ 0xf6, 0x78, 0},
	{ 0xf7, 0x64, 0},
	{ 0xf8, 0x62, 0},
	{ 0xf9, 0xf0, 0},
	{ 0xfa, 0x82, 0},
	{ 0xfb, 0xf2, 0},
	{ 0xfc, 0x90, 0},
	{ 0xfd, 0x94, 0},
};

// Slave 0xE0
static struct camera_i2c_reg_array adv7481_config_regs[]= {
{ 0x0, 0x30, 0},
{ 0xe, 0xff, 0},
};

//Slave 0xF2 SDP Main
static struct camera_i2c_reg_array adv7481_config_sdp_regs[]= {
	{ 0xe, 0x0, 0},
    { 0xf, 0x0, 0},
	{ 0x52, 0xCD, 0},
	{ 0x00, 0x00, 0},
	{ 0xe, 0x80, 0},
	{ 0x9c, 0x0, 0},
	{ 0x9c, 0xff, 0},
	{ 0xe, 0x0, 0},
	{ 0x80, 0x51, 0},
	{ 0x81, 0x51, 0},
	{ 0x82, 0x68, 0},
	{ 0x03, 0x42, 0},
	{ 0x04, 0x07, 0},
	{ 0x13, 0x00, 0},
	{ 0x17, 0x41, 0},
	{ 0x31, 0x12, 0},
};

//check SDP lock/unlock status
// 0xF2
static struct camera_i2c_reg_array adv7481_lock_sdp_RO_MAP_1_regs[]= {
	{ 0xe, 0x2, 0}
};

//check SDP lock/unlock status
// 0xF2
static struct camera_i2c_reg_array adv7481_lock_read_sdp_RO_MAP_1_regs[]= {
	{ 0x49, 0x2, 0}
};


//check SDP lock/unlock status
// 0xF2
static struct camera_i2c_reg_array adv7481_lock_sdp_RO_MAP_regs[]= {
	{ 0xe, 0x1, 0}
};

//check SDP lock/unlock status
// 0xF2
static struct camera_i2c_reg_array adv7481_lock_read_sdp_RO_MAP_regs[]= {
	{ 0x10, 0x1, 0}
};

//Slave 0xE0
static struct camera_i2c_reg_array adv7481_start_2_regs[] = {
{ 0x10, 0xa8, 0}
};

//Slave F2
static struct camera_i2c_reg_array adv7481_sensor_pre_start_regs[] = {
	{ 0xe, 0x01, 0},
};
// Slave 0x90 / 0x94
static struct camera_i2c_reg_array adv7481_sensor_start_regs[] = {
	{ 0x0, 0x81, 0},
	{ 0x0, 0xa1, 0},
	{ 0xdb, 0x10, 0},
	{ 0xd6, 0x7, 0},
	{ 0xc4, 0xa, 0},
	{ 0x71, 0x33, 0},
	{ 0x72, 0x11, 0},
	{ 0xf0, 0x0, 0},
	{ 0x31, 0x82, 0},
	{ 0x1e, 0x40, 0},
	{ 0xda, 0x1, 2000},
	{ 0x0, 0x21, 1000},
	{ 0xc1, 0x2b, 1000},
	{ 0x31, 0x80, 0},
};

// #ifdef ADV7481
	struct i2c_config_data config_data[10];
// #else
// I2C config data where last set is for stream start.
	// struct i2c_config_data config_data[4];
// #endif
#ifdef ADV7481

#define ADV7481_IO_MAP_PAD_CTRL_1_ADDR                                      0x1D
#define ADV7481_IO_MAP_SLAVE_ADDR                                           0xE0
#define ADV7481_IO_MAP_PAD_CTRL_1_PDN_INT1_BMSK                             0x80
#define ADV7481_IO_MAP_INT1_CONFIG_ADDR                                     0x40
#define ADV7481_IO_MAP_INT1_CONFIG_INTRQ_DUR_SEL_SHFT                       6
#define ADV7481_IO_MAP_INT1_CONFIG_INTRQ_OP_SEL_SHFT                        0
#define ADV7481_IO_MAP_DATAPATH_INT_MASKB_ADDR                              0x47
#define ADV7481_IO_MAP_DATAPATH_INT_MASKB_INT_SD_MB1_SHFT                   0
#define ADV7481_SDP_MAIN_MAP_USER_MAP_RW_REG_0E_ADDR                        0x0E
#define AD7V481_SDP_MAIN_MAP_USER_MAP_RW_REG_00_ADDR                        0x00
#define ADV7481_SDP_MAIN_MAP_USER_MAP_RW_REG_0F_ADDR                        0x0F
#define ADV7481_SDP_MAIN_MAP_USER_MAP_RW_REG_51_ADDR                        0x51
#define ADV7481_SDP_MAP_SEL_SDP_MAP_1                                       0x20
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_44_ADDR                     0x44
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_44_SD_UNLOCK_MSKB_BMSK      0x02
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_44_SD_UNLOCK_MSKB_SHFT      1
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_44_SD_LOCK_MSKB_BMSK        0x01
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_44_SD_LOCK_MSKB_SHFT        0
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4B_ADDR                     0x4B
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4B_SD_H_LOCK_CHNG_CLR_BMSK  0x04
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4B_SD_H_LOCK_CHNG_CLR_SHFT  2
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4B_SD_V_LOCK_CHNG_CLR_BMSK  0x02
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4B_SD_V_LOCK_CHNG_CLR_SHFT  1
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4C_ADDR                     0x4C
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4C_SD_H_LOCK_CHNG_MSKB_BMSK 0x04
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4C_SD_H_LOCK_CHNG_MSKB_SHFT 2
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4C_SD_V_LOCK_CHNG_MSKB_BMSK 0x02
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4C_SD_V_LOCK_CHNG_MSKB_SHFT 1
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_42_ADDR                   0x42
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_42_SD_UNLOCK_Q_BMSK       0x02
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_42_SD_UNLOCK_Q_SHFT       1
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_42_SD_LOCK_Q_BMSK         0x01
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_42_SD_LOCK_Q_SHFT         0
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_49_ADDR                   0x49
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_49_SD_H_LOCK_BMSK         0x04
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_49_SD_H_LOCK_SHFT         2
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_49_SD_V_LOCK_BMSK         0x02
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_49_SD_V_LOCK_SHFT         1
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_4A_ADDR                   0x4A
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_4A_SD_H_LOCK_CHNG_Q_BMSK  0x04
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_4A_SD_H_LOCK_CHNG_Q_SHFT  2
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_4A_SD_V_LOCK_CHNG_Q_BMSK  0x02
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_4A_SD_V_LOCK_CHNG_Q_SHFT  1
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_45_ADDR                   0x45
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_45_EVEN_FIELD_BMSK        0x10
#define ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_45_EVEN_FIELD_SHFT        4
#define ADV7481_SDP_MAP_SLAVE_ADDR                                          0xF2
#define ADV7481_SDP_MAP_SEL_SDP_MAIN_MAP                                    0x0
#define ADV7481_SDP_MAP_SEL_ADDR                                            0x0E
#define ADV7481_SDP_MAP_SEL_SDP_RO_MAP_1                                    0x02
#define ADV7481_SDP_MAP_SEL_SDP_RO_MAIN_MAP                                 0x01
#define ADV7481_SDP_RO_MAIN_MAP_USER_MAP_R_REG_10_ADDR                      0x10
#define ADV7481_SDP_RO_MAIN_MAP_USER_MAP_R_REG_10_IN_LOCK_BMSK              0x01
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_43_ADDR                     0x43
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_43_SD_UNLOCK_CLR_BMSK       0x02
#define ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_43_SD_LOCK_CLR_BMSK         0x01
#define ADV7481_IO_MAP_INT_RAW_STATUS_ADDR                                  0x3F
#define ADV7481_IO_MAP_INT_RAW_STATUS_INTRQ_RAW_BMSK                        0x01
#define ADV7481_IO_MAP_DATAPATH_RAW_STATUS_ADDR                             0x43
#define ADV7481_IO_MAP_DATAPATH_RAW_STATUS_INT_SD_RAW_BMSK                  0x01
#define ADV7481_IO_MAP_DATAPATH_RAW_STATUS_ADDR                             0x43
#define ADV7481_IO_MAP_DATAPATH_RAW_STATUS_INT_SD_RAW_BMSK                  0x01
#define ADV7481_IO_MAP_DATAPATH_INT_STATUS_ADDR                             0x44
#define ADV7481_IO_MAP_DATAPATH_INT_CLR_ADDR                                0x45
#define ADV7481_IO_MAP_DATAPATH_INT_CLR_INT_SD_CLR_BMSK                     0x01

int adv7481_intr_enable(void)
{
	int rc;
	int cci_master = 0, queue_id = 1;
	struct camera_i2c_reg_array int_enable[6];
	int idx = 0;
	uint32_t read_val = 0;

	//power up INT1, which should be up by default.
	rc = msm_cci_i2c_read(ADV7481_IO_MAP_PAD_CTRL_1_ADDR,
							1,
							&read_val,
							ADV7481_IO_MAP_SLAVE_ADDR,
							cci_master,
							queue_id,
							1,
							1);

	if (read_val & ADV7481_IO_MAP_PAD_CTRL_1_PDN_INT1_BMSK)
	{
		//power up INT1 if not up by default
		read_val &= ~ADV7481_IO_MAP_PAD_CTRL_1_PDN_INT1_BMSK;

		int_enable[0].delay = 0;
		int_enable[0].reg_addr = ADV7481_IO_MAP_PAD_CTRL_1_ADDR;
		int_enable[0].reg_data = read_val;

		rc = msm_cci_i2c_write(&int_enable[0],
							1,
							ADV7481_IO_MAP_SLAVE_ADDR,
							queue_id,
							0,
							1,
							1,
							0,cci_master);
	}

	//configure INT1 interrupt
	int_enable[0].delay = 0;
	int_enable[0].reg_addr = ADV7481_IO_MAP_INT1_CONFIG_ADDR;
	int_enable[0].reg_data = (0x3 << ADV7481_IO_MAP_INT1_CONFIG_INTRQ_DUR_SEL_SHFT)
							| (0x1 << ADV7481_IO_MAP_INT1_CONFIG_INTRQ_OP_SEL_SHFT);


	rc = msm_cci_i2c_write(&int_enable[0],
						1,
						ADV7481_IO_MAP_SLAVE_ADDR,
						queue_id,
						0,
						1,
						1,
						0,cci_master);

	//unmask INT_SD_ST
	int_enable[0].delay = 0;
	int_enable[0].reg_addr = ADV7481_IO_MAP_DATAPATH_INT_MASKB_ADDR;
	int_enable[0].reg_data = 0x1 << ADV7481_IO_MAP_DATAPATH_INT_MASKB_INT_SD_MB1_SHFT;

	rc = msm_cci_i2c_write(&int_enable[0],
						1,
						ADV7481_IO_MAP_SLAVE_ADDR,
						queue_id,
						0,
						1,
						1,
						0,cci_master);

	//CVBS interrupt:
	//set CVBS lock/unlock interrupts
	//map to SDP MAP 1
	idx = 0;

	int_enable[idx].delay = 0;
	int_enable[idx].reg_addr = ADV7481_SDP_MAIN_MAP_USER_MAP_RW_REG_0E_ADDR;
	int_enable[idx].reg_data = ADV7481_SDP_MAP_SEL_SDP_MAP_1;
	idx++;

	int_enable[idx].delay = 0;
	int_enable[idx].reg_addr = ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_44_ADDR;
	int_enable[idx].reg_data = ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_44_SD_UNLOCK_MSKB_BMSK
						| ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_44_SD_LOCK_MSKB_BMSK;
	idx++;

	//unmask SD_LOCK_MSKB/SD_UNLOCK_MSKB
	int_enable[idx].delay = 0;
	int_enable[idx].reg_addr = ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_44_ADDR;
	int_enable[idx].reg_data = ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_44_SD_UNLOCK_MSKB_BMSK
							| ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_44_SD_LOCK_MSKB_BMSK;
	idx++;

	//unmask sd_h_lock_chng_mask/sd_v_lock_chng_mask
	int_enable[idx].reg_addr = ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4C_ADDR;
	int_enable[idx].reg_data = ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4C_SD_H_LOCK_CHNG_MSKB_BMSK
						| ADV7481_SDP_MAP_1_USER_SUB_MAP_1_RW_REG_4C_SD_V_LOCK_CHNG_MSKB_BMSK;
	int_enable[idx].delay = 0;
	idx++;

	//map to SDP MAIN MAP
	int_enable[idx].reg_addr = ADV7481_SDP_MAIN_MAP_USER_MAP_RW_REG_0E_ADDR;
	int_enable[idx].reg_data = ADV7481_SDP_MAP_SEL_SDP_MAIN_MAP;
	int_enable[idx].delay = 0;
	idx++;

	//enable fsc_lock
	int_enable[idx].reg_addr = ADV7481_SDP_MAIN_MAP_USER_MAP_RW_REG_51_ADDR;
	int_enable[idx].reg_data = 0xb6;
	int_enable[idx].delay = 0;
	idx++;

	rc = msm_cci_i2c_write(&int_enable[0],
						idx,
						ADV7481_SDP_MAP_SLAVE_ADDR,
						queue_id,
						0,
						1,
						1,
						0,cci_master);

	return rc;
}

int adv7481_lock_status(void)
{
	static int lock = 1;
	int rc = 0;
	struct camera_i2c_reg_array reg;
	uint8 sd_lock_sts = 0;
	int cci_master = 0, queue_id = 1;
	uint32_t read_val = 0;
	static uint32_t debounce_count = INT_DEBOUNCE_NUM;
	uint8 sd_lock_q_info = 0;

	//check SDP lock/unlock interrupt

	//map to SDP_RO_MAP_1
	reg.reg_addr = ADV7481_SDP_MAP_SEL_ADDR;
	reg.reg_data = ADV7481_SDP_MAP_SEL_SDP_RO_MAP_1;
	reg.delay = 0;

	rc = msm_cci_i2c_write(&reg,
				1,
				ADV7481_SDP_MAP_SLAVE_ADDR,
				queue_id,
				0,
				1,
				1,
				0,cci_master);

	if (rc != 0)
	{
		rc = -1;
		goto EXIT_FLAG;
	}


	rc = msm_cci_i2c_read(ADV7481_SDP_RO_MAP_1_USER_SUB_MAP_1_R_REG_49_ADDR,
									1,
									&read_val,
									ADV7481_SDP_MAP_SLAVE_ADDR,
									cci_master,
									queue_id,
									1,
									1);

	if (rc != 0)
	{
		rc = -6;
		goto EXIT_FLAG;
	}

	sd_lock_q_info = read_val;



	//map to SDP_RO_MAIN
	reg.reg_addr = ADV7481_SDP_MAP_SEL_ADDR;
	reg.reg_data = ADV7481_SDP_MAP_SEL_SDP_RO_MAIN_MAP;
	reg.delay = 0;

	rc = msm_cci_i2c_write(&reg,
							1,
							ADV7481_SDP_MAP_SLAVE_ADDR,
							queue_id,
							0,
							1,
							1,
							0,cci_master);

	if (rc != 0)
	{
		rc = -5;
		goto EXIT_FLAG;
	}

	rc = msm_cci_i2c_read(ADV7481_SDP_RO_MAIN_MAP_USER_MAP_R_REG_10_ADDR,
									1,
									&read_val,
									ADV7481_SDP_MAP_SLAVE_ADDR,
									cci_master,
									queue_id,
									1,
									1);

	if (rc != 0)
	{
		rc = -6;
		goto EXIT_FLAG;
	}

	sd_lock_sts = read_val;


	if (sd_lock_sts == 0xd && sd_lock_q_info == 0x6)
	{
		debounce_count--;
		if(debounce_count == 0) {
			lock = 1;
			CDBG("Locked\n");
			debounce_count = INT_DEBOUNCE_NUM;
		}
	} else {
		lock = 0;
		CDBG("UnLocked\n");
		debounce_count = INT_DEBOUNCE_NUM;
	}

	// restore SDP register map
	reg.reg_addr = ADV7481_SDP_MAP_SEL_ADDR;
	reg.reg_data = ADV7481_SDP_MAP_SEL_SDP_MAIN_MAP;

	rc = msm_cci_i2c_write(&reg,
						1,
						ADV7481_SDP_MAP_SLAVE_ADDR,
						queue_id,
						0,
						1,
						1,
						0,cci_master);

	return lock;
EXIT_FLAG:
	return rc;

}

#endif


static int get_cam_data_digital(struct i2c_config_data **cam_data)
{
	int number_config_elements = 0;

	*cam_data = &config_data[0];
	// Bridge chip init sequence
	config_data[0].size =
		sizeof(ti960_init_regs) / sizeof(ti960_init_regs[0]);
	config_data[0].i2c_slave_address = BRIDGE_SLAVEADDR;
	config_data[0].i2c_regs = &ti960_init_regs[0];
	config_data[0].i2c_num_bytes_address = 1;
	config_data[0].i2c_num_bytes_data = 1;

	// Support for revision 2 and 3
	config_data[0].i2c_revision_id_val[0] = 0x20;
	config_data[0].i2c_revision_id_val[1] = 0x30;
	config_data[0].i2c_revision_id_num = 2;

	config_data[0].i2c_revision_id_reg = 0x3;
	number_config_elements++;

	// Sensor init sequence
	config_data[1].size =
		sizeof(ov10635_regs) / sizeof(ov10635_regs[0]);
	config_data[1].i2c_slave_address = CAMERA_SLAVEADDR;
	config_data[1].i2c_regs = &ov10635_regs[0];
	config_data[1].i2c_num_bytes_address = 2;
	config_data[1].i2c_num_bytes_data = 1;
	config_data[1].i2c_revision_id_val[0] = 0xa6;
	config_data[1].i2c_revision_id_num = 1;
	config_data[1].i2c_revision_id_reg = 0x300A;
	number_config_elements++;

	// Start streaming on RX port 0
	config_data[2].size = sizeof(ti960_start_regs) / sizeof(ti960_start_regs[0]);
	config_data[2].i2c_slave_address = BRIDGE_SLAVEADDR;
	config_data[2].i2c_regs = &ti960_start_regs[0];
	config_data[2].i2c_num_bytes_address = 1;
	config_data[2].i2c_num_bytes_data = 1;
	// Support for revision 2 and 3
	config_data[2].i2c_revision_id_val[0] = 0x20;
	config_data[2].i2c_revision_id_val[1] = 0x30;
	config_data[2].i2c_revision_id_num = 2;

	config_data[2].i2c_revision_id_reg = 0x3;
	number_config_elements++;

	// Stop streaming on RX port 0
	config_data[3].size = sizeof(ti960_stop_regs) / sizeof(ti960_stop_regs[0]);
	config_data[3].i2c_slave_address = BRIDGE_SLAVEADDR;
	config_data[3].i2c_regs = &ti960_stop_regs[0];
	config_data[3].i2c_num_bytes_address = 1;
	config_data[3].i2c_num_bytes_data = 1;

	// Support for revision 2 and 3
	config_data[3].i2c_revision_id_val[0] = 0x20;
	config_data[3].i2c_revision_id_val[1] = 0x30;
	config_data[3].i2c_revision_id_num = 2;
	config_data[3].i2c_revision_id_reg = 0x3;
	number_config_elements++;

	return number_config_elements;
}

static int get_cam_data_analog(struct i2c_config_data **cam_data)
{
    int number_config_elements = 0;

    *cam_data = &config_data[0];
    // Bridge chip init sequence
    config_data[0].size =
        sizeof(adv7481_init_regs) / sizeof(adv7481_init_regs[0]);
    config_data[0].i2c_slave_address = ADV_BRIDGE_SLAVEADDR;
    config_data[0].i2c_regs = &adv7481_init_regs[0];
    config_data[0].i2c_num_bytes_address = 1;
    config_data[0].i2c_num_bytes_data = 1;

    // Support for revision 2 and 3
    config_data[0].i2c_revision_id_val[0] = 0x21;
    config_data[0].i2c_revision_id_val[1] = 0x21;
    config_data[0].i2c_revision_id_num = 2;

    config_data[0].i2c_revision_id_reg = 0xDF;// ADV7481_IO_MAP_CHIP_REV_ID_1_ADDR
    number_config_elements++;

    config_data[1].size = sizeof(adv7481_config_regs) / sizeof(adv7481_config_regs[0]);
    config_data[1].i2c_slave_address = ADV_BRIDGE_SLAVEADDR;
    config_data[1].i2c_regs = &adv7481_config_regs[0];
    config_data[1].i2c_num_bytes_address = 1;
    config_data[1].i2c_num_bytes_data = 1;
	number_config_elements++;

    config_data[2].size = sizeof(adv7481_config_sdp_regs) / sizeof(adv7481_config_sdp_regs[0]);
    config_data[2].i2c_slave_address = 0xF2;
    config_data[2].i2c_regs = &adv7481_config_sdp_regs[0];
    config_data[2].i2c_num_bytes_address = 1;
    config_data[2].i2c_num_bytes_data = 1;
	number_config_elements++;

    // Start ADV Bridge chip for cvbs camera
    config_data[3].size = sizeof(adv7481_start_2_regs) / sizeof(adv7481_start_2_regs[0]);
    config_data[3].i2c_slave_address = ADV_BRIDGE_SLAVEADDR;
    config_data[3].i2c_regs = &adv7481_start_2_regs[0];
    config_data[3].i2c_num_bytes_address = 1;
    config_data[3].i2c_num_bytes_data = 1;
	number_config_elements++;

	// poll  for lock
    config_data[4].size =
        sizeof(adv7481_lock_sdp_RO_MAP_1_regs) / sizeof(adv7481_lock_sdp_RO_MAP_1_regs[0]);
    config_data[4].i2c_slave_address = 0xF2;
    config_data[4].i2c_regs = &adv7481_lock_sdp_RO_MAP_1_regs[0];
    config_data[4].i2c_num_bytes_address = 1;
    config_data[4].i2c_num_bytes_data = 1;
	number_config_elements++;

	config_data[5].size =
		sizeof(adv7481_lock_read_sdp_RO_MAP_1_regs) / sizeof(adv7481_lock_read_sdp_RO_MAP_1_regs[0]);
	config_data[5].i2c_slave_address = 0xF2;
	config_data[5].i2c_regs = &adv7481_lock_read_sdp_RO_MAP_1_regs[0];
	config_data[5].i2c_num_bytes_address = 1;
	config_data[5].i2c_num_bytes_data = 1;

	// locked value expected.
	config_data[5].i2c_revision_id_val[0] = 0x6;
	config_data[5].i2c_revision_id_val[1] = 0;
	config_data[5].i2c_revision_id_num = 1;
	config_data[5].i2c_revision_id_reg = adv7481_lock_read_sdp_RO_MAP_1_regs[0].reg_addr;
	number_config_elements++;

	config_data[6].size =
        sizeof(adv7481_lock_sdp_RO_MAP_regs) / sizeof(adv7481_lock_sdp_RO_MAP_regs[0]);
    config_data[6].i2c_slave_address = 0xF2;
    config_data[6].i2c_regs = &adv7481_lock_sdp_RO_MAP_regs[0];
    config_data[6].i2c_num_bytes_address = 1;
    config_data[6].i2c_num_bytes_data = 1;
	number_config_elements++;

	config_data[7].size =
		sizeof(adv7481_lock_read_sdp_RO_MAP_regs) / sizeof(adv7481_lock_read_sdp_RO_MAP_regs[0]);
	config_data[7].i2c_slave_address = 0xF2;
	config_data[7].i2c_regs = &adv7481_lock_read_sdp_RO_MAP_regs[0];
	config_data[7].i2c_num_bytes_address = 1;
	config_data[7].i2c_num_bytes_data = 1;

	// locked value expected.
	config_data[7].i2c_revision_id_val[0] = 0xd;
	config_data[7].i2c_revision_id_val[1] = 0;
	config_data[7].i2c_revision_id_num = 1;
	config_data[7].i2c_revision_id_reg = adv7481_lock_read_sdp_RO_MAP_regs[0].reg_addr;

	number_config_elements++;
	config_data[8].size = sizeof(adv7481_sensor_pre_start_regs) / sizeof(adv7481_sensor_pre_start_regs[0]);
	config_data[8].i2c_slave_address = 0xF2;
	config_data[8].i2c_regs = &adv7481_sensor_pre_start_regs[0];
	config_data[8].i2c_num_bytes_address = 1;
	config_data[8].i2c_num_bytes_data = 1;
	number_config_elements++;

    // Start streaming on cvbs port
    config_data[9].size = sizeof(adv7481_sensor_start_regs) / sizeof(adv7481_sensor_start_regs[0]);
    config_data[9].i2c_slave_address = ANALOG_CAMERA_SLAVEADDR;
    config_data[9].i2c_regs = &adv7481_sensor_start_regs[0];
    config_data[9].i2c_num_bytes_address = 1;
    config_data[9].i2c_num_bytes_data = 1;
	number_config_elements++;

	return number_config_elements;
}

int get_cam_data(int csi, struct i2c_config_data **cam_data) {

	int number_config_elements = 0;

	switch(csi)
	{
		case 1:
			number_config_elements = get_cam_data_analog(cam_data);
			break;
		case 2:
			number_config_elements = get_cam_data_digital(cam_data);
			break;
		default:
			break;
	}

	return number_config_elements;
}
