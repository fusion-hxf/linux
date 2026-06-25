/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Based of tfa9895.c
 * Register definitions taken from tfa98xx kernel driver:
 * Copyright (C) 2014-2020 NXP Semiconductors, All Rights Reserved.
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#define TFA987X_SYS_CTRL0		0x00
#define TFA987X_SYS_CTRL0_PWDN_MSK	BIT(0)
#define TFA987X_SYS_CTRL0_I2CR_MSK	BIT(1)
#define TFA987X_SYS_CTRL0_AMPE_MSK	BIT(3)
#define TFA987X_SYS_CTRL0_DCDC_MSK	BIT(4)

#define TFA987X_SYS_CTRL1		0x01
#define TFA987X_SYS_CTRL1_MANSCONF_MSK	BIT(2)
#define TFA987X_SYS_CTRL1_MANSAOOSC_MSK	BIT(4)

#define TFA987X_SYS_CTRL2		0x02
#define TFA987X_SYS_CTRL2_AUDFS_MSK	GENMASK(3, 0)
#define TFA987X_SYS_CTRL2_FRACTDEL_MSK	GENMASK(10, 5)

#define TFA987X_REV			0x03
#define TFA987X_CLK_GATING_CTRL		0x05

#define TFA987X_TDM_CFG0		0x20
#define TFA987X_TDM_CFG0_FSBCLKS_MSK	GENMASK(15, 12)
#define TFA987X_TDM_CFG1		0x21
#define TFA987X_TDM_CFG1_NSLOTS_MSK	GENMASK(3,  0)
#define TFA987X_TDM_CFG1_SLOTBITS_MSK	GENMASK(8,  4)
#define TFA987X_TDM_CFG2		0x22
#define TFA987X_TDM_CFG2_SWIDTH_MSK	GENMASK(6,  2)
#define TFA987X_TDM_CFG3		0x23
#define TFA987X_TDM_CFG3_SPKE_MSK	BIT(0)
#define TFA987X_TDM_CFG3_DCE_MSK	BIT(1)
#define TFA987X_TDM_CFG3_CSE_MSK	BIT(3)
#define TFA987X_TDM_CFG3_VSE_MSK	BIT(4)
#define TFA987X_TDM_CFG6		0x26
#define TFA987X_TDM_CFG6_SPKS_MSK	GENMASK(3,  0)
#define TFA987X_TDM_CFG6_DCS_MSK	GENMASK(4,  7)
#define TFA987X_TDM_CFG6_CSS_MSK	GENMASK(15, 12)
#define TFA987X_TDM_CFG7		0x27
#define TFA987X_TDM_CFG7_VSS_MSK	GENMASK(3,  0)

#define TFA987X_AUDIO_CTRL		0x51
#define TFA987X_AUDIO_CTRL_BSSS_MSK	BIT(0)
#define TFA9872_AUDIO_CTRL_INTSMUTE_MSK	BIT(1)
#define TFA987X_AUDIO_CTRL_HPFBYP_MSK	BIT(5)
#define TFA987X_AUDIO_CTRL_DPSA_MSK     BIT(7)

#define TFA987X_AMP_CFG			0x52
#define TFA987X_AMP_CFG_CLIPCTRL_MSK	GENMASK(4, 2)
#define TFA987X_AMP_CFG_GAIN_MSK	GENMASK(12, 5)
#define TFA987X_AMP_CFG_SLOPEE_MSK	BIT(13)
#define TFA987X_AMP_CFG_SLOPESET_MSK	BIT(14)

#define TFA987X_KEY1_PWM_CFG		0x58
#define TFA987X_TDM_CFG8		0x61
#define TFA987X_TDM_CFG8_DCG_MSK	GENMASK(5, 2)
#define TFA987X_TDM_CFG8_SPKG_MSK	GENMASK(9, 6)

#define TFA987X_LOW_NOISE_GAIN1		0x62
#define TFA987X_LOW_NOISE_GAIN2		0x63
#define TFA987X_MODE1_DET1		0x64
#define TFA987X_MODE1_DET1_LPM1MODE_MSK	GENMASK(15, 14)
#define TFA987X_MODE1_DETECTOR2		0x65
#define TFA987X_TDM_SRC			0x68
#define TFA987X_CURSENSE_COMP		0x6f
#define TFA987X_DCDC_CTRL0		0x70
#define TFA9872_DCDC_CTRL0_DCVOS_MSK	GENMASK(2,  0)
#define TFA987X_DCDC_CTRL0_MCC_MSK	GENMASK(6,  3)
#define TFA987X_DCDC_CTRL0_DCIE_MSK	BIT(9)
#define TFA987X_DCDC_CTRL1		0x71
#define TFA987X_DCDC_CTRL4		0x74
#define TFA9872_DCDC_CTRL4_DCVOF_MSK	GENMASK(2,  0)
#define TFA987X_DCDC_CTRL4_DCTRIP_MSK	GENMASK(8,  4)
#define TFA987X_DCDC_CTRL5		0x75
#define TFA987X_DCDC_CTRL5_DCTRIP2_MSK	GENMASK(7,  3)
#define TFA987X_DCDC_CTRL6		0x76
#define TFA9874_DCDC_CTRL6_DCVOF_MSK	GENMASK(8,  3)
#define TFA9874_DCDC_CTRL6_DCVOS_MSK	GENMASK(14,  9)

static int tfa987x_digital_mute(struct snd_soc_dai *codec_dai, int mute, int stream)
{
	struct snd_soc_component *component = codec_dai->component;
	int val = mute ? 0 : TFA987X_SYS_CTRL0_AMPE_MSK;

	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	snd_soc_component_update_bits(component, TFA987X_SYS_CTRL0,
						 TFA987X_SYS_CTRL0_AMPE_MSK, val);

	/* spk-vol: 刷机后用 `dmesg | grep spk-vol` 看每次播放起停时的实际增益寄存器值，
	 * 配合 `amixer -c0 cset name='Speaker Volume' N` 验证 AMP_CFG 增益方向/单调性。 */
	{
		unsigned int ampcfg = snd_soc_component_read(component,
							     TFA987X_AMP_CFG);

		dev_info(component->dev,
			 "spk-vol: amp %s AMP_CFG=0x%04x gain(bit5-12)=%lu\n",
			 mute ? "disable" : "enable", ampcfg,
			 FIELD_GET(TFA987X_AMP_CFG_GAIN_MSK, ampcfg));
	}

	return 0;
}

static const struct snd_soc_dapm_widget tfa987x_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Speaker"),
	SND_SOC_DAPM_OUT_DRV_E("PWUP", TFA987X_SYS_CTRL0, 0, 1, NULL, 0, NULL, 0),
};

static const struct snd_soc_dapm_route tfa987x_dapm_routes[] = {
	{"PWUP", NULL, "HiFi Playback"},
	{"Speaker", NULL, "PWUP"},
};

/* 扬声器硬件音量：暴露 AMP_CFG(0x52) 的 GAIN 字段(bit5..12，8 位)为 ALSA 控件，
 * 让 PipeWire/Plasma/UCM 能直接控硬件增益（耳机走 WCD 已有硬件音量，扬声器原本一个都没有，
 * 只能软件衰减）。SOC_SINGLE 只动 bit5..12，不碰同寄存器的 CLIPCTRL/SLOPE 位。
 *
 * 说明/待验证：mainline 这版 TFA 驱动不跑 NXP 私有 DSP，故无法走 DSP 平滑音量；AMP_CFG GAIN
 * 是放大器增益字段(可能为较粗的档位而非 256 级平滑)。即便偏粗也无妨——PipeWire 会用软件音量
 * 补足细分。invert=0 假定「寄存器值越大=增益越大」；刷机后用 `amixer -c0 cset name='Speaker Volume' N`
 * 边扫边听确认方向/单调性（若发现越大越小，改 invert=1）。不用此控件时等于无害空操作。 */
static const struct snd_kcontrol_new tfa987x_snd_controls[] = {
	SOC_SINGLE("Speaker Volume", TFA987X_AMP_CFG, 5, 0xff, 0),
};

static const struct snd_soc_component_driver tfa987x_component = {
	.controls		= tfa987x_snd_controls,
	.num_controls		= ARRAY_SIZE(tfa987x_snd_controls),
	.dapm_widgets		= tfa987x_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tfa987x_dapm_widgets),
	.dapm_routes		= tfa987x_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(tfa987x_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct snd_soc_dai_ops tfa987x_dai_ops = {
	.mute_stream = tfa987x_digital_mute,
};

static struct snd_soc_dai_driver tfa987x_dai = {
	.name = "tfa987x-hifi",
	.playback = {
		.stream_name	= "HiFi Playback",
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
		.rates		= SNDRV_PCM_RATE_48000,
		.rate_min	= 48000,
		.rate_max	= 48000,
		.channels_min	= 1,
		.channels_max	= 2,
	},
	.ops = &tfa987x_dai_ops,
};

static const struct regmap_config tfa987x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xff,
};

static bool tfa987x_setup_dcdc(struct device *dev, struct regmap *rmap, u16 rev)
{
	u32 mcc, dcvof, dcvos, dctrip, dctrip2 = 0;
	int i;
	u32 *dest[] = { &mcc, &dcvof, &dcvos, &dctrip, &dctrip2 };
	const char *props[] = { "max-coil-current",
	      "first-boost-voltage", "second-boost-voltage",
	      "first-boost-trip-lvl", "second-boost-trip-lvl" };

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		int ret;
		ret = of_property_read_u32(dev->of_node, props[i], dest[i]);
		if (ret < 0 && ret != -EINVAL)
			return false;
	}

	if (!FIELD_FIT(TFA987X_DCDC_CTRL0_MCC_MSK, mcc) ||
	    !FIELD_FIT(TFA987X_DCDC_CTRL4_DCTRIP_MSK, dctrip) ||
	    !FIELD_FIT(TFA987X_DCDC_CTRL5_DCTRIP2_MSK, dctrip2))
		return false;

	switch (rev & 0xff) {
	case 0x72:
		if (!FIELD_FIT(TFA9872_DCDC_CTRL4_DCVOF_MSK, dcvof) ||
		    !FIELD_FIT(TFA9872_DCDC_CTRL0_DCVOS_MSK, dcvos))
			return false;

		if (dcvof)
			regmap_update_bits(rmap, TFA987X_DCDC_CTRL4,
					TFA9872_DCDC_CTRL4_DCVOF_MSK,
					FIELD_PREP(TFA9872_DCDC_CTRL4_DCVOF_MSK, dcvof));
		if (dcvos)
			regmap_update_bits(rmap, TFA987X_DCDC_CTRL0,
					TFA9872_DCDC_CTRL0_DCVOS_MSK,
					FIELD_PREP(TFA9872_DCDC_CTRL0_DCVOS_MSK, dcvos));
		break;
	case 0x74:
		if (!FIELD_FIT(TFA9874_DCDC_CTRL6_DCVOF_MSK, dcvof) ||
		    !FIELD_FIT(TFA9874_DCDC_CTRL6_DCVOS_MSK, dcvos))
			return false;

		if (dcvof)
			regmap_update_bits(rmap, TFA987X_DCDC_CTRL6,
					TFA9874_DCDC_CTRL6_DCVOF_MSK,
					FIELD_PREP(TFA9874_DCDC_CTRL6_DCVOF_MSK, dcvof));

		if (dcvos)
			regmap_update_bits(rmap, TFA987X_DCDC_CTRL0,
					TFA9874_DCDC_CTRL6_DCVOS_MSK,
					FIELD_PREP(TFA9874_DCDC_CTRL6_DCVOS_MSK, dcvos));
		break;
	default:
		return false;
	}

	if (dctrip)
		regmap_update_bits(rmap, TFA987X_DCDC_CTRL4,
					 TFA987X_DCDC_CTRL4_DCTRIP_MSK,
					 FIELD_PREP(TFA987X_DCDC_CTRL4_DCTRIP_MSK, dctrip));
	if (dctrip2)
		regmap_update_bits(rmap, TFA987X_DCDC_CTRL5,
					 TFA987X_DCDC_CTRL5_DCTRIP2_MSK,
					 FIELD_PREP(TFA987X_DCDC_CTRL5_DCTRIP2_MSK, dctrip2));
	if (mcc)
		regmap_update_bits(rmap, TFA987X_DCDC_CTRL0,
					 TFA987X_DCDC_CTRL0_MCC_MSK |
					 TFA987X_DCDC_CTRL0_DCIE_MSK,
					 FIELD_PREP(TFA987X_DCDC_CTRL0_MCC_MSK, mcc) |
					 TFA987X_DCDC_CTRL0_DCIE_MSK);
	return true;
}

static int tfa987x_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct regmap *rmap;
	unsigned int rev;
	int ret;

	rmap = devm_regmap_init_i2c(i2c, &tfa987x_regmap_config);
	if (IS_ERR(rmap))
		return PTR_ERR(rmap);

	ret = regmap_read(rmap, TFA987X_REV, &rev);
	if (ret < 0) {
		dev_err(dev, "Failed to read revision register: %d\n", ret);
		return ret;
	}

	switch (rev) {
		case 0x1b72:
		case 0x2b72:
		case 0x3b72:
		case 0x0c74:
			dev_info(dev, "Chip revision: 0x%04x\n", rev);
			break;
		default:
			dev_err(dev, "Unsupported chip revision: 0x%04x\n", rev);
			return -ENODEV;
	}

	/* Perform soft reset */
	regmap_write(rmap, TFA987X_SYS_CTRL0, TFA987X_SYS_CTRL0_I2CR_MSK);

	/* Setup DC-DC Converter if we have configuration */
	regmap_update_bits(rmap, TFA987X_SYS_CTRL0, TFA987X_SYS_CTRL0_DCDC_MSK,
			   FIELD_PREP(TFA987X_SYS_CTRL0_DCDC_MSK,
				   (u16) tfa987x_setup_dcdc(dev, rmap, rev)));

	/* Disable DPSA */
	regmap_update_bits(rmap, TFA987X_AUDIO_CTRL,
				 TFA987X_AUDIO_CTRL_DPSA_MSK, 0);

	/* Setup TDM 16 bit 1 slot config */
	regmap_update_bits(rmap, TFA987X_TDM_CFG0,
				 TFA987X_TDM_CFG0_FSBCLKS_MSK,
				 FIELD_PREP(TFA987X_TDM_CFG0_FSBCLKS_MSK, 0));
	regmap_update_bits(rmap, TFA987X_TDM_CFG1,
				 TFA987X_TDM_CFG1_NSLOTS_MSK |
				 TFA987X_TDM_CFG1_SLOTBITS_MSK,
				 FIELD_PREP(TFA987X_TDM_CFG1_NSLOTS_MSK, 1) |
				 FIELD_PREP(TFA987X_TDM_CFG1_SLOTBITS_MSK, 15));
	regmap_update_bits(rmap, TFA987X_TDM_CFG2,
				 TFA987X_TDM_CFG2_SWIDTH_MSK,
				 FIELD_PREP(TFA987X_TDM_CFG2_SWIDTH_MSK, 15));

	/* No current/voltage sense over TDM */
	regmap_update_bits(rmap, TFA987X_TDM_CFG3,
				 TFA987X_TDM_CFG3_SPKE_MSK |
				 TFA987X_TDM_CFG3_DCE_MSK |
				 TFA987X_TDM_CFG3_CSE_MSK |
				 TFA987X_TDM_CFG3_VSE_MSK,
				 TFA987X_TDM_CFG3_SPKE_MSK);
	regmap_update_bits(rmap, TFA987X_TDM_CFG6,
				 TFA987X_TDM_CFG6_SPKS_MSK,
				 FIELD_PREP(TFA987X_TDM_CFG6_SPKS_MSK, 0));

	if ((rev & 0xff) == 0x72)
		regmap_update_bits(rmap, TFA987X_MODE1_DET1,
				 TFA987X_MODE1_DET1_LPM1MODE_MSK,
				 FIELD_PREP(TFA987X_MODE1_DET1_LPM1MODE_MSK, 1));

	/* Turn on this thing */
	regmap_update_bits(rmap, TFA987X_SYS_CTRL0,
				 TFA987X_SYS_CTRL0_PWDN_MSK, 0);

	regmap_update_bits(rmap, TFA987X_SYS_CTRL1,
				 TFA987X_SYS_CTRL1_MANSCONF_MSK,
				 TFA987X_SYS_CTRL1_MANSCONF_MSK);

	/* spk-vol: 记录上电后默认增益/DCDC，便于评估「Speaker Volume」量程与是否需要 boost。 */
	{
		unsigned int ampcfg = 0, tdm8 = 0, dcdc0 = 0;

		regmap_read(rmap, TFA987X_AMP_CFG, &ampcfg);
		regmap_read(rmap, TFA987X_TDM_CFG8, &tdm8);
		regmap_read(rmap, TFA987X_DCDC_CTRL0, &dcdc0);
		dev_info(dev,
			 "spk-vol: init rev=0x%04x AMP_CFG=0x%04x gain=%lu TDM_CFG8=0x%04x SPKG=%lu DCDC_CTRL0=0x%04x(boost=%s)\n",
			 rev, ampcfg, FIELD_GET(TFA987X_AMP_CFG_GAIN_MSK, ampcfg),
			 tdm8, FIELD_GET(TFA987X_TDM_CFG8_SPKG_MSK, tdm8), dcdc0,
			 (dcdc0 & TFA987X_DCDC_CTRL0_DCIE_MSK) ? "on" : "off");
	}

	return devm_snd_soc_register_component(dev, &tfa987x_component,
						    &tfa987x_dai, 1);
}

static const struct of_device_id tfa987x_of_match[] = {
	{ .compatible = "nxp,tfa9872" },
	{ .compatible = "nxp,tfa9874" },
	{ }
};
MODULE_DEVICE_TABLE(of, tfa987x_of_match);

static struct i2c_driver tfa987x_i2c_driver = {
	.driver = {
		.name = "tfa987x",
		.of_match_table = tfa987x_of_match,
	},
	.probe = tfa987x_i2c_probe,
};
module_i2c_driver(tfa987x_i2c_driver);

MODULE_DESCRIPTION("ASoC NXP Semiconductors TFA9872/TFA9874 driver");
MODULE_LICENSE("GPL v2");
