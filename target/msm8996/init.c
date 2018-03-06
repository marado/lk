/* Copyright (c) 2014-2016 The Linux Foundation. All rights reserved.
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
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, nit
 * PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <debug.h>
#include <platform/iomap.h>
#include <platform/irqs.h>
#include <platform/gpio.h>
#include <reg.h>
#include <target.h>
#include <platform.h>
#include <dload_util.h>
#include <uart_dm.h>
#include <mmc.h>
#include <spmi.h>
#include <board.h>
#include <smem.h>
#include <baseband.h>
#include <regulator.h>
#include <dev/keys.h>
#include <pm8x41.h>
#include <pm8x41_hw.h>
#include <crypto5_wrapper.h>
#include <clock.h>
#include <partition_parser.h>
#include <scm.h>
#include <platform/clock.h>
#include <platform/gpio.h>
#include <platform/timer.h>
#include <stdlib.h>
#include <ufs.h>
#include <boot_device.h>
#include <qmp_phy.h>
#include <sdhci_msm.h>
#include <qusb2_phy.h>
#include <secapp_loader.h>
#include <rpmb.h>
#include <rpm-glink.h>
#include <psci.h>
#if ENABLE_WBC
#include <pm_app_smbchg.h>
#endif

#if LONG_PRESS_POWER_ON
#include <shutdown_detect.h>
#endif

#if PON_VIB_SUPPORT
#include <vibrator.h>
#define VIBRATE_TIME 250
#endif

#include <pm_smbchg_usb_chgpth.h>

#include <msm_panel.h>
#include <target/display.h>
#include <target/target_camera.h>
#include <target/target_audio.h>

#define ANIMATED_SPLAH_PARTITION "splash"
#define ANIMATED_SPLASH_BUFFER   0x836a5580
#define ANIMATED_SPLASH_LOOPS    150

#define CE_INSTANCE             1
#define CE_EE                   0
#define CE_FIFO_SIZE            64
#define CE_READ_PIPE            3
#define CE_WRITE_PIPE           2
#define CE_READ_PIPE_LOCK_GRP   0
#define CE_WRITE_PIPE_LOCK_GRP  0
#define CE_ARRAY_SIZE           20

#define PMIC_ARB_CHANNEL_NUM    0
#define PMIC_ARB_OWNER_ID       0

/* Run Early camera for 30 seconds i.e 900 frames incase of no gpio */
#define EARLYCAM_NO_GPIO_FRAME_LIMIT 900

static int early_camera_enabled = 1;
static int early_audio_enabled = 1;
#define SMBCHG_USB_RT_STS 0x21310
#define SMBCHG_DC_RT_STS 0x21410
#define USBIN_UV_RT_STS BIT(0)
#define USBIN_OV_RT_STS BIT(1)
#define DCIN_UV_RT_STS  BIT(0)
#define DCIN_OV_RT_STS  BIT(1)

enum
{
	FUSION_I2S_MTP = 1,
	FUSION_SLIMBUS = 2,
} mtp_subtype;

enum
{
	FUSION_I2S_CDP = 2,
} cdp_subtype;

static uint8_t flash_memory_slot = 0;
static void set_sdc_power_ctrl();
static uint32_t mmc_pwrctl_base[] =
	{ MSM_SDC1_BASE, MSM_SDC2_BASE };

static uint32_t mmc_sdhci_base[] =
	{ MSM_SDC1_SDHCI_BASE, MSM_SDC2_SDHCI_BASE };

static uint32_t  mmc_sdc_pwrctl_irq[] =
	{ SDCC1_PWRCTL_IRQ, SDCC2_PWRCTL_IRQ };

struct mmc_device *dev;
struct ufs_dev ufs_device;

/* function pointer for secondary core entry point*/
void (*cpu_on_ep) (void);

/* early domain initialization */
void earlydomain_init();

/* run all early domain services */
void earlydomain_services();

/* early domain cleanup and exit*/
void earlydomain_exit();

/* is charging supported */
bool target_charging_supported();

/* is charging in progress */
bool target_charging_in_progress();

/* This is for openssl assembly to execute neon optimized code.
 * Refer to lib/openssl/crypto/arm_arch.h file
 * ARMV8_SHA256 indicates support for hardware SHA-256 instructions */
#define ARMV8_SHA256 (1 << 4)

/* Enable openssl ARMv8 implemetation */
unsigned int OPENSSL_armcap_P = ARMV8_SHA256;

void target_early_init(void)
{
#if WITH_DEBUG_UART
	uart_dm_init(8, 0, BLSP2_UART1_BASE);
#endif
}

bool mmc_read_done = false;
bool target_is_mmc_read_done()
{
	return mmc_read_done;
}
/* Return 1 if vol_up pressed */
int target_volume_up()
{
	static uint8_t first_time = 0;
	uint8_t status = 0;
	struct pm8x41_gpio gpio;
	memset(&gpio, 0, sizeof(struct pm8x41_gpio));

	if (!first_time) {
		/* Configure the GPIO */
		gpio.direction = PM_GPIO_DIR_IN;
		gpio.function  = 0;
		gpio.pull      = PM_GPIO_PULL_UP_30;
		gpio.vin_sel   = 2;

		pm8x41_gpio_config(2, &gpio);

		/* Wait for the pmic gpio config to take effect */
		udelay(10000);

		first_time = 1;
	}

	/* Get status of P_GPIO_5 */
	pm8x41_gpio_get(2, &status);

	return !status; /* active low */
}

/* Return 1 if vol_down pressed */
uint32_t target_volume_down()
{
	return pm8x41_resin_status();
}

static void target_keystatus()
{
	keys_init();

	if(target_volume_down())
		keys_post_event(KEY_VOLUMEDOWN, 1);

	if(target_volume_up())
		keys_post_event(KEY_VOLUMEUP, 1);
}

void target_uninit(void)
{
	if (platform_boot_dev_isemmc())
	{
		mmc_put_card_to_sleep(dev);
	}

#if VERIFIED_BOOT || defined(SET_ROT_ONLY)
#if VERIFIED_BOOT
	if (target_get_vb_version() == VB_V2 &&
		is_sec_app_loaded())
#endif
	{
		if (send_milestone_call_to_tz() < 0)
		{
			dprintf(CRITICAL, "Failed to unload App for rpmb\n");
			ASSERT(0);
		}
	}
#endif

#if ENABLE_WBC
	if (board_hardware_id() == HW_PLATFORM_MTP)
		pm_appsbl_set_dcin_suspend(1);
#endif


	if (crypto_initialized())
	{
		crypto_eng_cleanup();
		clock_ce_disable(CE_INSTANCE);
	}

	/* Tear down glink channels */
	rpm_glink_uninit();

#if VERIFIED_BOOT || defined(SET_ROT_ONLY)
#if VERIFIED_BOOT
	if (target_get_vb_version() == VB_V2)
#endif
	{
		if (rpmb_uninit() < 0)
		{
			dprintf(CRITICAL, "RPMB uninit failed\n");
			ASSERT(0);
		}
	}
#endif

}

static void set_sdc_power_ctrl()
{
	uint32_t reg = 0;
	uint8_t clk = 0;
	uint8_t cmd = 0;
	uint8_t dat = 0;

	if (flash_memory_slot == 0x1)
	{
		clk = TLMM_CUR_VAL_10MA;
		cmd = TLMM_CUR_VAL_8MA;
		dat = TLMM_CUR_VAL_8MA;
		reg = SDC1_HDRV_PULL_CTL;
	}
	else if (flash_memory_slot == 0x2)
	{
		clk = TLMM_CUR_VAL_16MA;
		cmd = TLMM_CUR_VAL_10MA;
		dat = TLMM_CUR_VAL_10MA;
		reg = SDC2_HDRV_PULL_CTL;
	}

	/* Drive strength configs for sdc pins */
	struct tlmm_cfgs sdc1_hdrv_cfg[] =
	{
		{ SDC1_CLK_HDRV_CTL_OFF,  clk, TLMM_HDRV_MASK, reg },
		{ SDC1_CMD_HDRV_CTL_OFF,  cmd, TLMM_HDRV_MASK, reg },
		{ SDC1_DATA_HDRV_CTL_OFF, dat, TLMM_HDRV_MASK, reg },
	};

	/* Pull configs for sdc pins */
	struct tlmm_cfgs sdc1_pull_cfg[] =
	{
		{ SDC1_CLK_PULL_CTL_OFF,  TLMM_NO_PULL, TLMM_PULL_MASK, reg },
		{ SDC1_CMD_PULL_CTL_OFF,  TLMM_PULL_UP, TLMM_PULL_MASK, reg },
		{ SDC1_DATA_PULL_CTL_OFF, TLMM_PULL_UP, TLMM_PULL_MASK, reg },
	};

	struct tlmm_cfgs sdc1_rclk_cfg[] =
	{
		{ SDC1_RCLK_PULL_CTL_OFF, TLMM_PULL_DOWN, TLMM_PULL_MASK, reg },
	};

	/* Set the drive strength & pull control values */
	tlmm_set_hdrive_ctrl(sdc1_hdrv_cfg, ARRAY_SIZE(sdc1_hdrv_cfg));
	tlmm_set_pull_ctrl(sdc1_pull_cfg, ARRAY_SIZE(sdc1_pull_cfg));
	tlmm_set_pull_ctrl(sdc1_rclk_cfg, ARRAY_SIZE(sdc1_rclk_cfg));
}

uint32_t target_is_pwrkey_pon_reason()
{
	uint8_t pon_reason = pm8950_get_pon_reason();

	if (pm8x41_get_is_cold_boot() && ((pon_reason == KPDPWR_N) || (pon_reason == (KPDPWR_N|PON1))))
		return 1;
	else if (pon_reason == PON1)
	{
		/* DC charger is present or USB charger is present */
		if (((USBIN_UV_RT_STS | USBIN_OV_RT_STS) & pm8x41_reg_read(SMBCHG_USB_RT_STS)) == 0 ||
			((DCIN_UV_RT_STS | DCIN_OV_RT_STS) & pm8x41_reg_read(SMBCHG_DC_RT_STS)) == 0)
			return 0;
		else
			return 1;
	}
	else
		return 0;
}


void target_sdc_init()
{
	struct mmc_config_data config = {0};

	config.bus_width = DATA_BUS_WIDTH_8BIT;
	config.max_clk_rate = MMC_CLK_192MHZ;
	config.hs400_support = 1;

	/* Try slot 1*/
	flash_memory_slot = 1;
	config.slot = 1;
	config.sdhc_base = mmc_sdhci_base[config.slot - 1];
	config.pwrctl_base = mmc_pwrctl_base[config.slot - 1];
	config.pwr_irq     = mmc_sdc_pwrctl_irq[config.slot - 1];

	/* Set drive strength & pull ctrl values */
	set_sdc_power_ctrl();

	if (!(dev = mmc_init(&config)))
	{
		/* Try slot 2 */
		flash_memory_slot = 2;
		config.slot = 2;
		config.max_clk_rate = MMC_CLK_200MHZ;
		config.sdhc_base = mmc_sdhci_base[config.slot - 1];
		config.pwrctl_base = mmc_pwrctl_base[config.slot - 1];
		config.pwr_irq     = mmc_sdc_pwrctl_irq[config.slot - 1];

		/* Set drive strength & pull ctrl values */
		set_sdc_power_ctrl();

		if (!(dev = mmc_init(&config)))
		{
			dprintf(CRITICAL, "mmc init failed!");
			ASSERT(0);
		}
	}
}

void *target_mmc_device()
{
	if (platform_boot_dev_isemmc())
		return (void *) dev;
	else
		return (void *) &ufs_device;
}

#if ENABLE_EARLY_ETHERNET
void toggle_neutrino(void) {
	struct pm8x41_gpio gpio = {
                .direction = PM_GPIO_DIR_OUT,
                .function = PM_GPIO_FUNC_HIGH,
                .vin_sel = 2,   /* VIN_2 */
                .output_buffer = PM_GPIO_OUT_CMOS,
                .out_strength = PM_GPIO_OUT_DRIVE_LOW,
    };

    pm8x41_gpio_config(13, &gpio);
	pm8x41_gpio_set(13, 1);
	mdelay(10);
	pm8x41_gpio_set(13, 0);
	mdelay(10);
	pm8x41_gpio_set(13, 1);
}
#endif

void target_init(void)
{
	dprintf(INFO, "target_init()\n");

	pmic_info_populate();

	spmi_init(PMIC_ARB_CHANNEL_NUM, PMIC_ARB_OWNER_ID);

	/* Initialize Glink */
	rpm_glink_init();
#if ENABLE_EARLY_ETHERNET
	//enable pmic gpio 13
	toggle_neutrino();
#endif
	target_keystatus();

#if defined(LONG_PRESS_POWER_ON) || defined(PON_VIB_SUPPORT)
	switch(board_hardware_id())
	{
		case HW_PLATFORM_QRD:
#if LONG_PRESS_POWER_ON
			shutdown_detect();
#endif
#if PON_VIB_SUPPORT
			vib_timed_turn_on(VIBRATE_TIME);
#endif
			break;
	}
#endif

	if ((CRYPTO_ENGINE_TYPE_HW == board_ce_type()) && target_use_signed_kernel())
	{
		target_crypto_init_params();
	}

	platform_read_boot_config();

#ifdef MMC_SDHCI_SUPPORT
	if (platform_boot_dev_isemmc())
	{
		target_sdc_init();
	}
#endif
#ifdef UFS_SUPPORT
	if (!platform_boot_dev_isemmc())
	{
		ufs_device.base = UFS_BASE;
		ufs_init(&ufs_device);
	}
#endif

	/* Storage initialization is complete, read the partition table info */
	mmc_read_partition_table(0);

#if ENABLE_WBC
	/* Look for battery voltage and make sure we have enough to bootup
	 * Otherwise initiate battery charging
	 * Charging should happen as early as possible, any other driver
	 * initialization before this should consider the power impact
	 */
	if(target_charging_supported())
	{
		pm_appsbl_chg_check_weak_battery_status(1);
	}
#endif

#if VERIFIED_BOOT || defined(SET_ROT_ONLY)
#if VERIFIED_BOOT
	if (VB_V2 == target_get_vb_version())
#endif
	{
		/* Initialize Qseecom */
		if (qseecom_init() < 0)
		{
			dprintf(CRITICAL, "Failed to initialize qseecom\n");
			ASSERT(0);
		}

		/* Start Qseecom */
		if (qseecom_tz_init() < 0)
		{
			dprintf(CRITICAL, "Failed to start qseecom\n");
			ASSERT(0);
		}

		if (rpmb_init() < 0)
		{
			dprintf(CRITICAL, "RPMB init failed\n");
			ASSERT(0);
		}

		/*
		 * Load the sec app for first time
		 */
		if (load_sec_app() < 0)
		{
			dprintf(CRITICAL, "Failed to load App for verified\n");
#if ENABLE_RECOVERY
			if (!use_backup) {
				if (!set_recovery_cookie())
					reboot_device(0);
			}
			dprintf(CRITICAL, "Failed to set the cookie in misc partition\n");
#endif
			ASSERT(0);
		}
	}
#endif
}

unsigned board_machtype(void)
{
	return LINUX_MACHTYPE_UNKNOWN;
}

/* Detect the target type */
void target_detect(struct board_data *board)
{
	/* This is filled from board.c */
}

static uint8_t splash_override;
/* Returns 1 if target supports continuous splash screen. */
int target_cont_splash_screen()
{
	uint8_t splash_screen = 0;
	if(!splash_override && !target_charging_in_progress()) {
		switch(board_hardware_id())
		{
			case HW_PLATFORM_SURF:
			case HW_PLATFORM_MTP:
			case HW_PLATFORM_FLUID:
			case HW_PLATFORM_QRD:
			case HW_PLATFORM_LIQUID:
			case HW_PLATFORM_DRAGON:
			case HW_PLATFORM_ADP:
				dprintf(SPEW, "Target_cont_splash=1\n");
				splash_screen = 1;
				break;
			default:
				dprintf(SPEW, "Target_cont_splash=0\n");
				splash_screen = 0;
		}
	}
	return splash_screen;
}

/* Returns 1 if target supports animated splash screen. */
int target_animated_splash_screen()
{
	uint8_t animated_splash = 0;
	if(!splash_override && !target_charging_in_progress()) {
		switch(board_hardware_id()) {
			case HW_PLATFORM_ADP:
			case HW_PLATFORM_DRAGON:
				dprintf(SPEW, "Target_animated_splash=1\n");
				// enable animated splash for ADP and Dragonboard
				animated_splash = 1;
				break;
			default:
				dprintf(SPEW, "Target_animated_splash=0\n");
				animated_splash = 0;
		}
	}
	return animated_splash;
}

void target_force_cont_splash_disable(uint8_t override)
{
        splash_override = override;
}

/* Detect the modem type */
void target_baseband_detect(struct board_data *board)
{
	uint32_t platform;
	uint32_t platform_hardware;
	uint32_t platform_subtype;

	platform = board->platform;
	platform_hardware = board->platform_hw;
	platform_subtype = board->platform_subtype;

	if (platform_hardware == HW_PLATFORM_SURF)
	{
		if (platform_subtype == FUSION_I2S_CDP)
			board->baseband = BASEBAND_MDM;
	}
	else if (platform_hardware == HW_PLATFORM_MTP)
	{
		if (platform_subtype == FUSION_I2S_MTP ||
			platform_subtype == FUSION_SLIMBUS)
			board->baseband = BASEBAND_MDM;
	}
	/*
	 * Special case if MDM is not set look for chip info to decide
	 * platform subtype
	 */
	if (board->baseband != BASEBAND_MDM)
	{
		switch(platform) {
		case APQ8096:
		case APQ8096AU:
		case APQ8096SG:
		case APQ8096SGAU:
			board->baseband = BASEBAND_APQ;
			break;
		case MSM8996:
		case MSM8996AU:
		case MSM8996SG:
		case MSM8996SGAU:
		case MSM8996L:
			board->baseband = BASEBAND_MSM;
			break;
		default:
			dprintf(CRITICAL, "Platform type: %u is not supported\n",platform);
			ASSERT(0);
		};
	}
}

unsigned target_baseband()
{
	return board_baseband();
}

void target_serialno(unsigned char *buf)
{
	unsigned int serialno;
	if (target_is_emmc_boot()) {
		serialno = mmc_get_psn();
		snprintf((char *)buf, 13, "%x", serialno);
	}
}

int emmc_recovery_init(void)
{
	return _emmc_recovery_init();
}

void target_usb_phy_reset()
{
	usb30_qmp_phy_reset();
	qusb2_phy_reset();
}

void target_usb_phy_sec_reset()
{
	qusb2_phy_reset();
}

target_usb_iface_t* target_usb30_init()
{
	target_usb_iface_t *t_usb_iface;

	t_usb_iface = calloc(1, sizeof(target_usb_iface_t));
	ASSERT(t_usb_iface);


	/* for SBC we use secondary port */
	if (board_hardware_id() == HW_PLATFORM_SBC)
	{
		/* secondary port have no QMP phy,use only QUSB2 phy that have only reset */
		t_usb_iface->phy_init   = NULL;
		t_usb_iface->phy_reset  = target_usb_phy_sec_reset;
		t_usb_iface->clock_init = clock_usb20_init;
	} else {
		t_usb_iface->phy_init   = usb30_qmp_phy_init;
		t_usb_iface->phy_reset  = target_usb_phy_reset;
		t_usb_iface->clock_init = clock_usb30_init;
	}

	t_usb_iface->vbus_override = 1;

	return t_usb_iface;
}

/* identify the usb controller to be used for the target */
const char * target_usb_controller()
{
	return "dwc";
}

uint32_t target_override_pll()
{
	if (board_soc_version() >= 0x20000)
		return 0;
	else
		return 1;
}

crypto_engine_type board_ce_type(void)
{
#if ENABLE_HW_CRYPTO
	return CRYPTO_ENGINE_TYPE_HW;
#else
	return CRYPTO_ENGINE_TYPE_SW;
#endif
}

/* Set up params for h/w CE. */
void target_crypto_init_params()
{
	struct crypto_init_params ce_params;

	/* Set up base addresses and instance. */
	ce_params.crypto_instance  = CE_INSTANCE;
	ce_params.crypto_base      = MSM_CE_BASE;
	ce_params.bam_base         = MSM_CE_BAM_BASE;

	/* Set up BAM config. */
	ce_params.bam_ee               = CE_EE;
	ce_params.pipes.read_pipe      = CE_READ_PIPE;
	ce_params.pipes.write_pipe     = CE_WRITE_PIPE;
	ce_params.pipes.read_pipe_grp  = CE_READ_PIPE_LOCK_GRP;
	ce_params.pipes.write_pipe_grp = CE_WRITE_PIPE_LOCK_GRP;

	/* Assign buffer sizes. */
	ce_params.num_ce           = CE_ARRAY_SIZE;
	ce_params.read_fifo_size   = CE_FIFO_SIZE;
	ce_params.write_fifo_size  = CE_FIFO_SIZE;

	/* BAM is initialized by TZ for this platform.
	 * Do not do it again as the initialization address space
	 * is locked.
	 */
	ce_params.do_bam_init      = 0;

	crypto_init_params(&ce_params);
}

#if ENABLE_WBC
unsigned target_pause_for_battery_charge(void)
{
	unsigned int target_pause = 0;

	if(target_charging_supported())
	{
		uint8_t pon_reason = pm8x41_get_pon_reason();
		uint8_t is_cold_boot = pm8x41_get_is_cold_boot();
		pm_smbchg_usb_chgpth_pwr_pth_type charger_path = PM_SMBCHG_USB_CHGPTH_PWR_PATH__INVALID;
		dprintf(INFO, "%s : pon_reason is %d cold_boot:%d charger path: %d\n", __func__,
			pon_reason, is_cold_boot, charger_path);
		/* In case of fastboot reboot,adb reboot or if we see the power key
		* pressed we do not want go into charger mode.
		* fastboot reboot is warm boot with PON hard reset bit not set
		* adb reboot is a cold boot with PON hard reset bit set
		*/
		pm_smbchg_get_charger_path(1, &charger_path);
		if (is_cold_boot &&
				(!(pon_reason & HARD_RST)) &&
				(!(pon_reason & KPDPWR_N)) &&
				((pon_reason & PON1)) &&
				((charger_path == PM_SMBCHG_USB_CHGPTH_PWR_PATH__DC_CHARGER) ||
				(charger_path == PM_SMBCHG_USB_CHGPTH_PWR_PATH__USB_CHARGER)))

			target_pause = 1;
	}

	return target_pause;
}

bool target_charging_supported()
{
	switch(board_hardware_id())
	{
		case HW_PLATFORM_MTP:
		case HW_PLATFORM_FLUID:
		case HW_PLATFORM_QRD:
			return 1;
		default:
			/* Charging not supported */
			return 0;
	};
}

bool target_charging_in_progress()
{
	return pm_appsbl_charging_in_progress();
}

#else
unsigned target_pause_for_battery_charge(void)
{
	return 0;
}

bool target_charging_supported()
{
	return 0;
}

bool target_charging_in_progress()
{
	return 0;
}
#endif /*ENABLE_WBC*/

int set_download_mode(enum reboot_reason mode)
{
	int ret = 0;
	ret = scm_dload_mode(mode);

	return ret;
}

void pmic_reset_configure(uint8_t reset_type)
{
	pm8994_reset_configure(reset_type);
}

uint32_t target_get_pmic()
{
	return PMIC_IS_PMI8996;
}

#if EARLYDOMAIN_SUPPORT

/* calls psci to turn on the secondary core */
void enable_secondary_core()
{
    cpu_on_ep = &cpu_on_asm;

    if (psci_cpu_on(platform_get_secondary_cpu_num(), (paddr_t)cpu_on_ep))
    {
        dprintf(CRITICAL, "Failed to turn on secondary CPU: %x\n", platform_get_secondary_cpu_num());
    }
    dprintf (INFO, "LK continue on cpu0\n");
}

/* handles the early domain */
void earlydomain()
{
    /* init and run early domain services*/
    earlydomain_init();

    /* run all early domain services */
    earlydomain_services();

    /* cleanup and exit early domain services*/
    earlydomain_exit();
}

/* early domain initialization */
void earlydomain_init()
{
    dprintf(INFO, "started early domain on secondary CPU\n");
}

/* early domain cleanup and exit */
void earlydomain_exit()
{
    isb();

    /* clean-up */
    arch_disable_cache(UCACHE);

    arch_disable_mmu();

    dsb();
    isb();

    arch_disable_ints();

    /* turn off secondary cpu */
    psci_cpu_off();
}

enum compression_type {
	COMPRESSION_TYPE_NONE = 0,
	COMPRESSION_TYPE_RLE24,
	COMPRESSION_TYPE_MAX
};

typedef struct animated_img_header {
	unsigned char magic[LOGO_IMG_MAGIC_SIZE]; /* "SPLASH!!" */
	uint32_t display_id;
	uint32_t width;
	uint32_t height;
	uint32_t fps;
	uint32_t num_frames;
	uint32_t type;
	uint32_t blocks;
	uint32_t img_size;
	uint32_t offset;
	uint8_t reserved[512-44];
} animated_img_header;

#define NUM_DISPLAYS 3
void **buffers[NUM_DISPLAYS];
struct animated_img_header g_head[NUM_DISPLAYS];

int animated_splash_screen_mmc()
{
	int index = INVALID_PTN;
	unsigned long long ptn = 0;
	struct fbcon_config *fb_display = NULL;
	struct animated_img_header *header;
	uint32_t blocksize, realsize, readsize;
	void *buffer;
	uint32_t i = 0, j = 0;
	void *tmp;
	void *head;
	int ret = 0;

	index = partition_get_index(ANIMATED_SPLAH_PARTITION);
	if (index == 0) {
		dprintf(CRITICAL, "ERROR: splash Partition table not found\n");
		return -1;
	}
	ptn = partition_get_offset(index);
	if (ptn == 0) {
		dprintf(CRITICAL, "ERROR: splash Partition invalid\n");
		return -1;
	}

	mmc_set_lun(partition_get_lun(index));

	blocksize = mmc_get_device_blocksize();
	if (blocksize == 0) {
		dprintf(CRITICAL, "ERROR:splash Partition invalid blocksize\n");
		return -1;
	}
	// Assume it is always for display ID 0
	fb_display = target_display_get_fb(0);

	if (!fb_display) {
		dprintf(CRITICAL, "ERROR: fb config is not allocated\n");
		return -1;
	}

	buffer = (void *)ANIMATED_SPLASH_BUFFER;
	for (j = 0; j < NUM_DISPLAYS; j++)
	{
		head = (void *)&(g_head[j]);
		if (mmc_read(ptn, (uint32_t *)(head), blocksize)) {
			dprintf(CRITICAL, "ERROR: Cannot read splash image header\n");
			return -1;
		}

		header = (animated_img_header *)head;
		if (memcmp(header->magic, LOGO_IMG_MAGIC, 8)) {
			dprintf(CRITICAL, "Invalid magic number in header %s %d\n",
				header->magic, header->height);
			ret = -1;
			goto end;
		}

		if (header->width == 0 || header->height == 0) {
			dprintf(CRITICAL, "Invalid height and width\n");
			ret = -1;
			goto end;
		}

		buffers[j] = (void **)malloc(header->num_frames*sizeof(void *));
		if (buffers[j] == NULL) {
			dprintf(CRITICAL, "Cant alloc mem for ptrs\n");
			ret = -1;
			goto end;
		}
		if ((header->type == COMPRESSION_TYPE_RLE24) && (header->blocks != 0)) {
			dprintf(CRITICAL, "Compressed data not supported\n");
			ret = 0;
			goto end;
		} else {
			if ((header->width > fb_display->width) ||
				(header->height > fb_display->height)) {
				dprintf(CRITICAL, "Logo config greater than fb config. header->width %u"
					" fb->width = %u header->height = %u fb->height = %u\n",
					header->width, fb_display->width, header->height, fb_display->height);
				ret = -1;
				goto end;
			}
			dprintf(INFO, "width:%d height:%d blocks:%d imgsize:%d num_frames:%d\n", header->width,
			header->height, header->blocks, header->img_size,header->num_frames);
			realsize =  header->blocks * blocksize;
			if ((realsize % blocksize) != 0)
				readsize =  ROUNDUP(realsize, blocksize) - blocksize;
			else
				readsize = realsize;
			if (blocksize == LOGO_IMG_HEADER_SIZE) {
				if (mmc_read((ptn + LOGO_IMG_HEADER_SIZE), (uint32_t *)buffer, readsize)) {
					dprintf(CRITICAL, "ERROR: Cannot read splash image from partition 1\n");
					ret = -1;
					goto end;
				}
			} else {
				if (mmc_read(ptn + blocksize , (uint32_t *)buffer, realsize)) {
					dprintf(CRITICAL, "ERROR: Cannot read splash image from partition 2\n");
					ret = -1;
					goto end;
				}
			}
			tmp = buffer;
			for (i = 0; i < header->num_frames; i++) {
				buffers[j][i] = tmp;
				tmp = tmp + header->img_size;
			}

		}
		buffer = tmp;
		ptn += LOGO_IMG_HEADER_SIZE + readsize;
	}
end:
	return ret;
}

#if EARLYCAMERA_NO_GPIO

inline bool get_reverse_camera_gpio() {
	return TRUE;
}

#else

inline bool get_reverse_camera_gpio() {
	/* if gpio == 1, it is ON
	   if gpio == 0, it is OFF */
	return (1 == gpio_get(103));
}

#endif

/* checks if GPIO or equivalent trigger to enable early camera is set to ON
   If this function retuns FALSE, only animated splash may be shown.
   This is also a check to see if early-camera/animated splash can exit*/
bool is_reverse_camera_on() {
	uint32_t trigger_reg = 0;
	trigger_reg = readl_relaxed((void *)MDSS_SCRATCH_REG_2);
	if ((FALSE == get_reverse_camera_gpio()) ||
		(0xF5F5F5F5 == trigger_reg))
		return FALSE; /* trigger to exit */
	else
		return TRUE;
}

int animated_splash() {
	void *disp_ptr, *layer_ptr;
	uint32_t ret = 0, k = 0, j = 0;
	uint32_t frame_cnt[NUM_DISPLAYS];
	struct target_display_update update[NUM_DISPLAYS];
	struct target_layer layer[NUM_DISPLAYS];
	struct target_display * disp;
	struct fbcon_config *fb;
	uint32_t sleep_time;
	uint32_t disp_cnt = target_display_init_count();
	uint32_t reg_value;
	bool camera_on = FALSE;
	bool camera_frame_on = false;
#if EARLYCAMERA_NO_GPIO
	uint32_t frame_count = 0;
#endif

	if (!buffers[0]) {
		dprintf(CRITICAL, "Unexpected error in read\n");
		return 0;
	}
	for (j = 0; j < NUM_DISPLAYS; j ++) {
		frame_cnt[j] = 0;
		disp_ptr = target_display_open(j);
		if (disp_ptr == NULL) {
			dprintf(CRITICAL, "Display open failed\n");
			return -1;
		}
		disp = target_get_display_info(disp_ptr);
		if (disp == NULL){
			dprintf(CRITICAL, "Display info failed\n");
			return -1;
		}
		layer_ptr = target_display_acquire_layer(disp_ptr, "as", kFormatRGB888);
		if (layer_ptr == NULL){
			dprintf(CRITICAL, "Layer acquire failed\n");
			return -1;
		}
		fb = target_display_get_fb(j);

		layer[j].layer = layer_ptr;
		layer[j].z_order = 2;
		update[j].disp = disp_ptr;
		update[j].layer_list = &layer[j];
		update[j].num_layers = 1;
		layer[j].fb = fb;
		sleep_time = 1000 / g_head[j].fps;
		layer[j].width = fb->width;
		layer[j].height = fb->height;
		if (g_head[j].width < fb->width) {
			dprintf(SPEW, "Modify width\n");
			layer[j].width = g_head[j].width;
		}
		if (g_head[j].height < fb->height) {
			dprintf(SPEW, "Modify height\n");
			layer[j].height = g_head[j].height;
		}
	}

	while (1) {
		if(early_audio_enabled == 1) {
			if (early_audio_check_dma_playback())
			{
				early_audio_end();
				early_audio_enabled = 0;
			}
		}

		camera_on = is_reverse_camera_on();

		reg_value = readl_relaxed((void *)MDSS_SCRATCH_REG_1);
		if (0xFEFEFEFE == reg_value) {
			//This value indicates kernel request LK to shutdown immediately
			break;
		}
		else if (0xDEADDEAD == reg_value) {
			// This reg value means kernel is started
			// LK should notify kernel by writing 0xDEADBEEF to
			// MDSS_SCRATCH_REG_1 when it is ready to exit

			if (0 == early_camera_enabled)
				break;
			else if ((1 == early_camera_enabled) &&
					(FALSE == camera_on) && (false == camera_frame_on))
				break;
		}

		for (j = 0; j < disp_cnt; j++) {
			if (j == 0 && early_camera_enabled == 1) {
				if (early_camera_on()) {
					if(layer[j].layer) {
						target_release_layer(&layer[j]);
						layer[j].layer = NULL;
					}
					camera_frame_on = true;
					continue;
				} else {
					if(!layer[j].layer) {
						layer_ptr = target_display_acquire_layer(disp_ptr, "as", kFormatRGB888);
						layer[j].layer = layer_ptr;
						layer[j].z_order = 2;
						camera_frame_on = false;
					}
				}
			}
			layer[j].fb->base = buffers[j][frame_cnt[j]];
			layer[j].fb->format = kFormatRGB888;
			layer[j].fb->bpp = 24;
			ret = target_display_update(&update[j],1, j);
			frame_cnt[j]++;
			if (frame_cnt[j] >= g_head[j].num_frames) {
				frame_cnt[j] = 0;
			}
		}

		if(early_camera_enabled == 1)
			// Rely on camera timing to flip.
			early_camera_flip();
		else
			// assume all displays have the same fps
			mdelay_optimal(sleep_time);
		k++;
#if EARLYCAMERA_NO_GPIO
		frame_count++;
		if (EARLYCAM_NO_GPIO_FRAME_LIMIT < frame_count)
			break;
#endif
	}
	if (early_camera_enabled == 1)
		early_camera_stop();
	for (j = 0; j < disp_cnt; j++) {
		target_release_layer(&layer[j]);
	}

	return ret;
}

/* early domain services */
void earlydomain_services()
{
	int ret = 0;
	int i = 0;
	bool panel_is_selected;

	dprintf(CRITICAL, "earlydomain_services: Waiting for display init to complete\n");

	while((FALSE == target_display_is_init_done()) && (i < 100))
	{
		mdelay_optimal(10); // delay for 10ms
		i++;
	}

	panel_is_selected = target_display_panel_is_selected();

	/* Ensure panel type is selected before touching display scratch register. */
	if (panel_is_selected) {
		dprintf(CRITICAL, "earlydomain_services: Display init done\n");

		/* Notify Kernel that LK is running */
		writel(0xC001CAFE, MDSS_SCRATCH_REG_1);

	} else {
		dprintf(CRITICAL, "earlydomain_services: panel is not selected\n");
	}

	/* starting early domain services */
	if (early_camera_init() == -1) {
		early_camera_enabled = 0;
		dprintf(CRITICAL, "earlydomain_services: Early Camera exit init failed\n");
	} else {
		dprintf(CRITICAL, "earlydomain_services: Early Camera starting\n");
	}

	/* starting early audio */
	if (early_audio_init() == -1) {
		early_audio_enabled = 0;
		dprintf(CRITICAL, "earlydomain_services: Early Audio exit start failed\n");
	} else {
		// Write the first playback period
		early_audio_check_dma_playback();
		dprintf(CRITICAL, "earlydomain_services: Early Audio started\n");
	}

	/*Create Animated splash thread
	if target supports it*/
	if (panel_is_selected && target_animated_splash_screen())
	{
		ret = animated_splash_screen_mmc();
		mmc_read_done = true;
		dprintf(CRITICAL, "earlydomain_services: mmc read done\n");
		if (ret) {
			dprintf(CRITICAL, "Error in reading memory. Skip Animated splash\n");
		} else {
			animated_splash();
		}
	}

	if (early_audio_enabled == 1)
	{
		while(!early_audio_check_dma_playback())
		{
			mdelay_optimal(early_audio_get_sleep_time_ms());
		}
		early_audio_end();
		early_audio_enabled = 0;
	}

	if (panel_is_selected) {
		// Notify Kernel that LK is shutdown
		writel(0xDEADBEEF, MDSS_SCRATCH_REG_1);
	}
}

#else

/* stubs for early domain functions */
void enable_secondary_core() {}
void earlydomain() {}
void earlydomain_init() {}
void earlydomain_services() {}
void earlydomain_exit() {}

#endif /* EARLYDOMAIN_SUPPORT */

int target_update_cmdline(char *cmdline)
{
	uint32_t platform_id = board_platform_id();
	int len = 0;
	if (platform_id == APQ8096SG || platform_id == MSM8996SG)
	{
		strlcpy(cmdline, " fpsimd.fpsimd_settings=0", TARGET_MAX_CMDLNBUF);
		len = strlen (cmdline);

		/* App settings are not required for other than v1.0 SoC */
		if (board_soc_version() > 0x10000) {
			strlcpy(cmdline + len, " app_setting.use_app_setting=0", TARGET_MAX_CMDLNBUF - len);
			len = strlen (cmdline);
		}
	}

	return len;
}

#if _APPEND_CMDLINE
int get_target_boot_params(const char *cmdline, const char *part, char **buf)
{
	int system_ptn_index = -1;
	unsigned int lun = 0;
	char lun_char_base = 'a', lun_char_limit = 'h';

	/*allocate buflen for largest possible string*/
	uint32_t buflen = strlen(" root=/dev/mmcblock0p") + sizeof(int) + 1; /*1 character for null termination*/

	if (!cmdline || !part ) {
		dprintf(CRITICAL, "WARN: Invalid input param\n");
		return -1;
	}

	system_ptn_index = partition_get_index(part);
	if (system_ptn_index == -1)
	{
		dprintf(CRITICAL,"Unable to find partition %s\n",part);
		return -1;
	}

	*buf = (char *)malloc(buflen);
	if(!(*buf)) {
		dprintf(CRITICAL,"Unable to allocate memory for boot params\n");
		return -1;
	}

	/*
	 * check if cmdline contains "root="/"" at the beginning of buffer or
	 * " root="/"ubi.mtd" in the middle of buffer.
	 */
	if ((strncmp(cmdline," root=",strlen(" root=")) == 0) ||
		strstr(cmdline, " root="))
		dprintf(DEBUG, "DEBUG: cmdline has root=\n");
	else
	{
		if (platform_boot_dev_isemmc()) {
			snprintf(*buf, buflen, " root=/dev/mmcblock0p%d",
					system_ptn_index + 1);
		} else {
			lun = partition_get_lun(system_ptn_index);
			if ((lun_char_base + lun) > lun_char_limit) {
				dprintf(CRITICAL, "lun value exceeds limit\n");
				return -1;
			}
			snprintf(*buf, buflen, " root=/dev/sd%c%d",
					lun_char_base + lun,
					partition_get_index_in_lun(part, lun));
		}
	}
	/*in success case buf will be freed in the calling function of this*/
	return 0;
}
#endif
