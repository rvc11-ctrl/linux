// SPDX-License-Identifier: GPL-2.0-only
/*
 * TAS2557 smart amplifier codec driver for downstream-style SDM660 speaker paths.
 *
 * This keeps the board-specific startup sequence from the earlier minimal port,
 * but now also requests the vendor DSP firmware and applies the parsed program /
 * configuration blocks that the downstream driver relies on for actual output.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define DEFAULT_SAMPLE_RATE_48K			48000

#define TAS2557_REG(book, page, reg) ((((unsigned int)(book) * 256 * 128) + \
				      ((unsigned int)(page) * 128)) + (reg))
#define TAS2557_BOOK_ID(reg) ((u8)((reg) / (256 * 128)))
#define TAS2557_PAGE_ID(reg) ((u8)(((reg) % (256 * 128)) / 128))
#define TAS2557_PAGE_REG(reg) ((u8)(((reg) % (256 * 128)) % 128))

#define TAS2557_SW_RESET_REG			TAS2557_REG(0, 0, 1)
#define TAS2557_REV_PGID_REG			TAS2557_REG(0, 0, 3)
#define TAS2557_POWER_CTRL1_REG			TAS2557_REG(0, 0, 4)
#define TAS2557_POWER_CTRL2_REG			TAS2557_REG(0, 0, 5)
#define TAS2557_SPK_CTRL_REG			TAS2557_REG(0, 0, 6)
#define TAS2557_MUTE_REG			TAS2557_REG(0, 0, 7)
#define TAS2557_CRC_CHECKSUM_REG		TAS2557_REG(0, 0, 32)
#define TAS2557_CRC_RESET_REG			TAS2557_REG(0, 0, 33)
#define TAS2557_SAFE_GUARD_REG			TAS2557_REG(0, 0, 37)
#define TAS2557_CLK_ERR_CTRL			TAS2557_REG(0, 0, 44)
#define TAS2557_CLK_ERR_CTRL2			TAS2557_REG(0, 0, 45)
#define TAS2557_CLK_ERR_CTRL3			TAS2557_REG(0, 0, 46)
#define TAS2557_DBOOST_CFG_REG			TAS2557_REG(0, 0, 52)
#define TAS2557_SAR_ADC2_REG			TAS2557_REG(0, 0, 21)

#define TAS2557_ASI1_DAC_FORMAT_REG		TAS2557_REG(0, 1, 1)
#define TAS2557_GPIO1_PIN_REG			TAS2557_REG(0, 1, 61)
#define TAS2557_GPIO2_PIN_REG			TAS2557_REG(0, 1, 62)
#define TAS2557_GPI_PIN_REG			TAS2557_REG(0, 1, 77)

#define TAS2557_SOFT_MUTE_REG			TAS2557_REG(100, 0, 7)

#define TAS2557_PG_VERSION_1P0			0x80
#define TAS2557_PG_VERSION_2P1			0xa0

#define TAS2557_SAFE_GUARD_PATTERN		0x5a
#define TAS2557_WORDLENGTH_MASK			GENMASK(4, 3)
#define TAS2557_FORMAT_MASK			GENMASK(7, 5)

#define TAS2557_FORMAT_I2S			(0x0 << 5)
#define TAS2557_FORMAT_DSP			(0x1 << 5)
#define TAS2557_FORMAT_RIGHT_J			(0x2 << 5)
#define TAS2557_FORMAT_LEFT_J			(0x3 << 5)

#define TAS2557_UDELAY				0xfffffffe

#define TAS2557_FW_NAME			"tas2557_uCDSP.bin"
#define TAS2557_PG1P0_FW_NAME			"tas2557_pg1p0_uCDSP.bin"

#define PPC_DRIVER_CRCCHK			0x00000200
#define PPC_DRIVER_CONFDEV			0x00000300
#define PPC_DRIVER_MTPLLSRC			0x00000400
#define PPC_DRIVER_CFGDEV_NONCRC		0x00000101

#define TAS2557_BLOCK_PLL			0x00
#define TAS2557_BLOCK_PGM_ALL			0x0d
#define TAS2557_BLOCK_PGM_DEV_A			0x01
#define TAS2557_BLOCK_CFG_COEFF_DEV_A		0x03
#define TAS2557_BLOCK_CFG_PRE_DEV_A		0x04
#define TAS2557_BLOCK_CFG_POST			0x05
#define TAS2557_BLOCK_CFG_POST_POWER		0x06

struct tas2557_block {
	u32 type;
	u8 pchk_present;
	u8 pchk;
	u8 ychk_present;
	u8 ychk;
	u32 commands;
	u8 *data;
};

struct tas2557_data_image {
	char name[64];
	char *description;
	u32 blocks;
	struct tas2557_block *block;
};

struct tas2557_pll {
	char name[64];
	char *description;
	struct tas2557_block block;
};

struct tas2557_program {
	char name[64];
	char *description;
	u8 app_mode;
	u16 boost;
	struct tas2557_data_image data;
};

struct tas2557_configuration {
	char name[64];
	char *description;
	u32 devices;
	u32 program;
	u32 pll;
	u32 sampling_rate;
	u8 pll_src;
	u32 pll_src_rate;
	struct tas2557_data_image data;
};

struct tas2557_firmware {
	u32 fw_size;
	u32 checksum;
	u32 ppc_version;
	u32 fw_version;
	u32 driver_version;
	u32 timestamp;
	char ddc_name[64];
	char *description;
	u32 device_family;
	u32 device;
	u32 plls;
	struct tas2557_pll *pll;
	u32 programs;
	struct tas2557_program *program;
	u32 configurations;
	struct tas2557_configuration *configuration;
};

struct tas2557_priv {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct mutex lock;
	struct tas2557_firmware fw;
	u8 current_book;
	u8 current_page;
	u8 i2s_bits;
	u8 pgid;
	u32 current_program;
	u32 current_configuration;
	u32 current_sample_rate;
	bool firmware_loaded;
	bool powered;
};

static const struct regmap_config tas2557_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 127,
	.cache_type = REGCACHE_NONE,
};

static const unsigned int tas2557_default_data[] = {
	TAS2557_SAR_ADC2_REG, 0x05,
	TAS2557_CLK_ERR_CTRL2, 0x21,
	TAS2557_CLK_ERR_CTRL3, 0x21,
	TAS2557_DBOOST_CFG_REG, 0x0b,
	TAS2557_SAFE_GUARD_REG, TAS2557_SAFE_GUARD_PATTERN,
	0xffffffff, 0xffffffff,
};

static const unsigned int tas2557_startup_data[] = {
	TAS2557_GPI_PIN_REG, 0x15,
	TAS2557_GPIO1_PIN_REG, 0x01,
	TAS2557_GPIO2_PIN_REG, 0x01,
	TAS2557_POWER_CTRL2_REG, 0xa0,
	TAS2557_POWER_CTRL2_REG, 0xa3,
	TAS2557_POWER_CTRL1_REG, 0xf8,
	TAS2557_UDELAY, 2000,
	TAS2557_CLK_ERR_CTRL, 0x2b,
	0xffffffff, 0xffffffff,
};

static const unsigned int tas2557_unmute_data[] = {
	TAS2557_MUTE_REG, 0x00,
	TAS2557_SOFT_MUTE_REG, 0x00,
	0xffffffff, 0xffffffff,
};

static const unsigned int tas2557_shutdown_data[] = {
	TAS2557_CLK_ERR_CTRL, 0x00,
	TAS2557_SOFT_MUTE_REG, 0x01,
	TAS2557_UDELAY, 10000,
	TAS2557_MUTE_REG, 0x03,
	TAS2557_POWER_CTRL1_REG, 0x60,
	TAS2557_UDELAY, 2000,
	TAS2557_POWER_CTRL2_REG, 0x00,
	TAS2557_POWER_CTRL1_REG, 0x00,
	TAS2557_GPIO1_PIN_REG, 0x00,
	TAS2557_GPIO2_PIN_REG, 0x00,
	TAS2557_GPI_PIN_REG, 0x00,
	0xffffffff, 0xffffffff,
};

static void tas2557_clear_firmware(struct tas2557_firmware *fw)
{
	u32 index;
	u32 block_index;

	if (!fw)
		return;

	kfree(fw->description);

	for (index = 0; index < fw->plls; index++) {
		kfree(fw->pll[index].description);
		kfree(fw->pll[index].block.data);
	}
	kfree(fw->pll);

	for (index = 0; index < fw->programs; index++) {
		kfree(fw->program[index].description);
		kfree(fw->program[index].data.description);
		for (block_index = 0; block_index < fw->program[index].data.blocks; block_index++)
			kfree(fw->program[index].data.block[block_index].data);
		kfree(fw->program[index].data.block);
	}
	kfree(fw->program);

	for (index = 0; index < fw->configurations; index++) {
		kfree(fw->configuration[index].description);
		kfree(fw->configuration[index].data.description);
		for (block_index = 0; block_index < fw->configuration[index].data.blocks; block_index++)
			kfree(fw->configuration[index].data.block[block_index].data);
		kfree(fw->configuration[index].data.block);
	}
	kfree(fw->configuration);

	memset(fw, 0, sizeof(*fw));
}

static void tas2557_free_firmware_action(void *data)
{
	struct tas2557_priv *priv = data;

	tas2557_clear_firmware(&priv->fw);
}

static unsigned int tas2557_fw_convert_number(const unsigned char *data)
{
	return data[3] + (data[2] << 8) + (data[1] << 16) + (data[0] << 24);
}

static bool tas2557_fw_has_bytes(const unsigned char *data,
				 const unsigned char *end,
				 size_t need)
{
	return data <= end && need <= (size_t)(end - data);
}

static int tas2557_fw_bounded_strlen(const unsigned char *data,
				     const unsigned char *end)
{
	const unsigned char *p;

	for (p = data; p < end; p++) {
		if (!*p)
			return p - data;
	}

	return -EINVAL;
}

static int tas2557_change_book_page(struct tas2557_priv *priv, u8 book, u8 page)
{
	int ret;

	if (priv->current_book == book && priv->current_page == page)
		return 0;

	if (priv->current_book != book) {
		ret = regmap_write(priv->regmap, 0, 0);
		if (ret)
			return ret;
		priv->current_page = 0;

		ret = regmap_write(priv->regmap, 127, book);
		if (ret)
			return ret;
		priv->current_book = book;
	}

	if (priv->current_page != page) {
		ret = regmap_write(priv->regmap, 0, page);
		if (ret)
			return ret;
		priv->current_page = page;
	}

	return 0;
}

static int tas2557_dev_read(struct tas2557_priv *priv, unsigned int reg,
			    unsigned int *value)
{
	int ret;

	ret = tas2557_change_book_page(priv, TAS2557_BOOK_ID(reg),
				       TAS2557_PAGE_ID(reg));
	if (ret)
		return ret;

	return regmap_read(priv->regmap, TAS2557_PAGE_REG(reg), value);
}

static int tas2557_dev_write(struct tas2557_priv *priv, unsigned int reg,
			     unsigned int value)
{
	int ret;

	ret = tas2557_change_book_page(priv, TAS2557_BOOK_ID(reg),
				       TAS2557_PAGE_ID(reg));
	if (ret)
		return ret;

	return regmap_write(priv->regmap, TAS2557_PAGE_REG(reg), value);
}

static int tas2557_dev_bulk_write(struct tas2557_priv *priv, unsigned int reg,
				  const void *data, size_t count)
{
	int ret;

	ret = tas2557_change_book_page(priv, TAS2557_BOOK_ID(reg),
				       TAS2557_PAGE_ID(reg));
	if (ret)
		return ret;

	return regmap_bulk_write(priv->regmap, TAS2557_PAGE_REG(reg), data, count);
}

static int tas2557_dev_update_bits(struct tas2557_priv *priv, unsigned int reg,
				   unsigned int mask,
				   unsigned int value)
{
	int ret;

	ret = tas2557_change_book_page(priv, TAS2557_BOOK_ID(reg),
				       TAS2557_PAGE_ID(reg));
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap, TAS2557_PAGE_REG(reg), mask, value);
}

static int tas2557_dev_load_data(struct tas2557_priv *priv,
				 const unsigned int *data)
{
	unsigned int reg;
	unsigned int val;
	int index;
	int ret = 0;

	for (index = 0; ; index += 2) {
		reg = data[index];
		val = data[index + 1];

		if (reg == 0xffffffff)
			break;
		if (reg == TAS2557_UDELAY) {
			udelay(val);
			continue;
		}

		ret = tas2557_dev_write(priv, reg, val);
		if (ret)
			break;
	}

	return ret;
}

static int tas2557_set_bit_rate(struct tas2557_priv *priv, unsigned int bits)
{
	unsigned int val;

	switch (bits) {
	case 16:
		val = 0x0 << 3;
		break;
	case 20:
		val = 0x1 << 3;
		break;
	case 24:
		val = 0x2 << 3;
		break;
	case 32:
		val = 0x3 << 3;
		break;
	default:
		return -EINVAL;
	}

	priv->i2s_bits = bits;

	return tas2557_dev_update_bits(priv, TAS2557_ASI1_DAC_FORMAT_REG,
				       TAS2557_WORDLENGTH_MASK, val);
}

static int tas2557_set_serial_format(struct tas2557_priv *priv,
				      unsigned int fmt)
{
	unsigned int val;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		val = TAS2557_FORMAT_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		val = TAS2557_FORMAT_DSP;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		val = TAS2557_FORMAT_RIGHT_J;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val = TAS2557_FORMAT_LEFT_J;
		break;
	default:
		return -EINVAL;
	}

	return tas2557_dev_update_bits(priv, TAS2557_ASI1_DAC_FORMAT_REG,
				       TAS2557_FORMAT_MASK, val);
}

static void tas2557_hw_reset(struct tas2557_priv *priv)
{
	if (!priv->reset_gpio)
		return;

	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	usleep_range(2000, 3000);
}

static int tas2557_load_default(struct tas2557_priv *priv)
{
	int ret;

	ret = tas2557_dev_load_data(priv, tas2557_default_data);
	if (ret)
		return ret;

	ret = tas2557_set_serial_format(priv, SND_SOC_DAIFMT_I2S);
	if (ret)
		return ret;

	ret = tas2557_set_bit_rate(priv, priv->i2s_bits ?: 16);
	if (ret)
		return ret;

	return tas2557_dev_update_bits(priv, TAS2557_ASI1_DAC_FORMAT_REG, 0x01, 0x01);
}

static int tas2557_fw_parse_header(struct tas2557_priv *priv,
				   struct tas2557_firmware *fw,
				   unsigned char *data,
				   unsigned int size)
{
	static const unsigned char magic[4] = { 0x35, 0x35, 0x35, 0x32 };
	unsigned char *start = data;
	const unsigned char *end = data + size;
	int len;

	if (!tas2557_fw_has_bytes(data, end, 104))
		return -EINVAL;
	if (memcmp(data, magic, sizeof(magic)))
		return -EINVAL;
	data += 4;

	fw->fw_size = tas2557_fw_convert_number(data);
	data += 4;
	fw->checksum = tas2557_fw_convert_number(data);
	data += 4;
	fw->ppc_version = tas2557_fw_convert_number(data);
	data += 4;
	fw->fw_version = tas2557_fw_convert_number(data);
	data += 4;
	fw->driver_version = tas2557_fw_convert_number(data);
	data += 4;
	fw->timestamp = tas2557_fw_convert_number(data);
	data += 4;

	if (!tas2557_fw_has_bytes(data, end, sizeof(fw->ddc_name)))
		return -EINVAL;
	memcpy(fw->ddc_name, data, sizeof(fw->ddc_name));
	data += sizeof(fw->ddc_name);

	len = tas2557_fw_bounded_strlen(data, end);
	if (len < 0)
		return len;
	if (!tas2557_fw_has_bytes(data, end, len + 1))
		return -EINVAL;
	fw->description = kmemdup(data, len + 1, GFP_KERNEL);
	if (!fw->description)
		return -ENOMEM;
	data += len + 1;

	if (!tas2557_fw_has_bytes(data, end, 8))
		return -EINVAL;
	fw->device_family = tas2557_fw_convert_number(data);
	data += 4;
	fw->device = tas2557_fw_convert_number(data);
	data += 4;

	if (fw->device_family != 0 || fw->device != 2) {
		dev_err(priv->dev, "unsupported firmware target family=%u device=%u\n",
			fw->device_family, fw->device);
		return -EINVAL;
	}

	return data - start;
}

static int tas2557_fw_parse_block(struct tas2557_priv *priv,
				  struct tas2557_firmware *fw,
				  struct tas2557_block *block,
				  unsigned char *data,
				  const unsigned char *end)
{
	unsigned char *start = data;
	u32 bytes;

	if (!tas2557_fw_has_bytes(data, end, 4))
		return -EINVAL;
	block->type = tas2557_fw_convert_number(data);
	data += 4;

	if (fw->driver_version >= PPC_DRIVER_CRCCHK) {
		if (!tas2557_fw_has_bytes(data, end, 4))
			return -EINVAL;
		block->pchk_present = data[0];
		block->pchk = data[1];
		block->ychk_present = data[2];
		block->ychk = data[3];
		data += 4;
	}

	if (!tas2557_fw_has_bytes(data, end, 4))
		return -EINVAL;
	block->commands = tas2557_fw_convert_number(data);
	data += 4;

	if (block->commands > UINT_MAX / 4)
		return -EINVAL;
	bytes = block->commands * 4;
	if (!tas2557_fw_has_bytes(data, end, bytes))
		return -EINVAL;
	block->data = kmemdup(data, bytes, GFP_KERNEL);
	if (!block->data)
		return -ENOMEM;
	data += bytes;

	return data - start;
}

static int tas2557_fw_parse_data(struct tas2557_priv *priv,
				 struct tas2557_firmware *fw,
				 struct tas2557_data_image *image,
				 unsigned char *data,
				 const unsigned char *end)
{
	unsigned char *start = data;
	u32 block_index;
	unsigned int parsed;
	int len;

	if (!tas2557_fw_has_bytes(data, end, sizeof(image->name)))
		return -EINVAL;
	memcpy(image->name, data, sizeof(image->name));
	data += sizeof(image->name);

	len = tas2557_fw_bounded_strlen(data, end);
	if (len < 0)
		return len;
	if (!tas2557_fw_has_bytes(data, end, len + 1))
		return -EINVAL;
	image->description = kmemdup(data, len + 1, GFP_KERNEL);
	if (!image->description)
		return -ENOMEM;
	data += len + 1;

	if (!tas2557_fw_has_bytes(data, end, 2))
		return -EINVAL;
	image->blocks = (data[0] << 8) + data[1];
	data += 2;

	image->block = kcalloc(image->blocks, sizeof(*image->block), GFP_KERNEL);
	if (image->blocks && !image->block)
		return -ENOMEM;

	for (block_index = 0; block_index < image->blocks; block_index++) {
		parsed = tas2557_fw_parse_block(priv, fw, &image->block[block_index],
						data, end);
		if ((int)parsed < 0)
			return (int)parsed;
		data += parsed;
	}

	return data - start;
}

static int tas2557_fw_parse_plls(struct tas2557_priv *priv,
				 struct tas2557_firmware *fw,
				 unsigned char *data,
				 const unsigned char *end)
{
	unsigned char *start = data;
	u32 index;
	unsigned int parsed;
	int len;

	if (!tas2557_fw_has_bytes(data, end, 2))
		return -EINVAL;
	fw->plls = (data[0] << 8) + data[1];
	data += 2;

	fw->pll = kcalloc(fw->plls, sizeof(*fw->pll), GFP_KERNEL);
	if (fw->plls && !fw->pll)
		return -ENOMEM;

	for (index = 0; index < fw->plls; index++) {
		if (!tas2557_fw_has_bytes(data, end, sizeof(fw->pll[index].name)))
			return -EINVAL;
		memcpy(fw->pll[index].name, data, sizeof(fw->pll[index].name));
		data += sizeof(fw->pll[index].name);

		len = tas2557_fw_bounded_strlen(data, end);
		if (len < 0)
			return len;
		if (!tas2557_fw_has_bytes(data, end, len + 1))
			return -EINVAL;
		fw->pll[index].description = kmemdup(data, len + 1, GFP_KERNEL);
		if (!fw->pll[index].description)
			return -ENOMEM;
		data += len + 1;

		parsed = tas2557_fw_parse_block(priv, fw, &fw->pll[index].block, data, end);
		if ((int)parsed < 0)
			return (int)parsed;
		data += parsed;
	}

	return data - start;
}

static int tas2557_fw_parse_programs(struct tas2557_priv *priv,
				     struct tas2557_firmware *fw,
				     unsigned char *data,
				     const unsigned char *end)
{
	unsigned char *start = data;
	u32 index;
	unsigned int parsed;
	int len;

	if (!tas2557_fw_has_bytes(data, end, 2))
		return -EINVAL;
	fw->programs = (data[0] << 8) + data[1];
	data += 2;

	fw->program = kcalloc(fw->programs, sizeof(*fw->program), GFP_KERNEL);
	if (fw->programs && !fw->program)
		return -ENOMEM;

	for (index = 0; index < fw->programs; index++) {
		if (!tas2557_fw_has_bytes(data, end, sizeof(fw->program[index].name)))
			return -EINVAL;
		memcpy(fw->program[index].name, data, sizeof(fw->program[index].name));
		data += sizeof(fw->program[index].name);

		len = tas2557_fw_bounded_strlen(data, end);
		if (len < 0)
			return len;
		if (!tas2557_fw_has_bytes(data, end, len + 1))
			return -EINVAL;
		fw->program[index].description = kmemdup(data, len + 1, GFP_KERNEL);
		if (!fw->program[index].description)
			return -ENOMEM;
		data += len + 1;

		if (!tas2557_fw_has_bytes(data, end, 3))
			return -EINVAL;
		fw->program[index].app_mode = data[0];
		data++;
		fw->program[index].boost = (data[0] << 8) + data[1];
		data += 2;

		parsed = tas2557_fw_parse_data(priv, fw, &fw->program[index].data, data, end);
		if ((int)parsed < 0)
			return (int)parsed;
		data += parsed;
	}

	return data - start;
}

static int tas2557_fw_parse_configurations(struct tas2557_priv *priv,
				   struct tas2557_firmware *fw,
				   unsigned char *data,
				   const unsigned char *end)
{
	unsigned char *start = data;
	u32 index;
	unsigned int parsed;
	int len;

	if (!tas2557_fw_has_bytes(data, end, 2))
		return -EINVAL;
	fw->configurations = (data[0] << 8) + data[1];
	data += 2;

	fw->configuration = kcalloc(fw->configurations, sizeof(*fw->configuration), GFP_KERNEL);
	if (fw->configurations && !fw->configuration)
		return -ENOMEM;

	for (index = 0; index < fw->configurations; index++) {
		if (!tas2557_fw_has_bytes(data, end, sizeof(fw->configuration[index].name)))
			return -EINVAL;
		memcpy(fw->configuration[index].name, data, sizeof(fw->configuration[index].name));
		data += sizeof(fw->configuration[index].name);

		len = tas2557_fw_bounded_strlen(data, end);
		if (len < 0)
			return len;
		if (!tas2557_fw_has_bytes(data, end, len + 1))
			return -EINVAL;
		fw->configuration[index].description = kmemdup(data, len + 1, GFP_KERNEL);
		if (!fw->configuration[index].description)
			return -ENOMEM;
		data += len + 1;

		if ((fw->driver_version >= PPC_DRIVER_CONFDEV) ||
		    ((fw->driver_version >= PPC_DRIVER_CFGDEV_NONCRC) &&
		     (fw->driver_version < PPC_DRIVER_CRCCHK))) {
			if (!tas2557_fw_has_bytes(data, end, 2))
				return -EINVAL;
			fw->configuration[index].devices = (data[0] << 8) + data[1];
			data += 2;
		} else {
			fw->configuration[index].devices = 1;
		}

		if (!tas2557_fw_has_bytes(data, end, 6))
			return -EINVAL;
		fw->configuration[index].program = data[0];
		data++;
		fw->configuration[index].pll = data[0];
		data++;
		fw->configuration[index].sampling_rate = tas2557_fw_convert_number(data);
		data += 4;

		if (fw->driver_version >= PPC_DRIVER_MTPLLSRC) {
			if (!tas2557_fw_has_bytes(data, end, 5))
				return -EINVAL;
			fw->configuration[index].pll_src = data[0];
			data++;
			fw->configuration[index].pll_src_rate = tas2557_fw_convert_number(data);
			data += 4;
		}

		parsed = tas2557_fw_parse_data(priv, fw, &fw->configuration[index].data,
					       data, end);
		if ((int)parsed < 0)
			return (int)parsed;
		data += parsed;
	}

	return data - start;
}

static int tas2557_fw_parse(struct tas2557_priv *priv,
			    struct tas2557_firmware *fw,
			    unsigned char *data,
			    unsigned int size)
{
	const unsigned char *end = data + size;
	int parsed;

	parsed = tas2557_fw_parse_header(priv, fw, data, size);
	if (parsed < 0)
		return parsed;
	data += parsed;
	if (data > end)
		return -EINVAL;

	parsed = tas2557_fw_parse_plls(priv, fw, data, end);
	if (parsed < 0)
		return parsed;
	data += parsed;
	if (data > end)
		return -EINVAL;

	parsed = tas2557_fw_parse_programs(priv, fw, data, end);
	if (parsed < 0)
		return parsed;
	data += parsed;
	if (data > end)
		return -EINVAL;

	parsed = tas2557_fw_parse_configurations(priv, fw, data, end);
	if (parsed < 0)
		return parsed;
	data += parsed;
	if (data > end)
		return -EINVAL;

	return 0;
}

static int tas2557_load_block(struct tas2557_priv *priv,
			      const struct tas2557_block *block)
{
	unsigned int command = 0;
	const unsigned char *data;
	unsigned char book;
	unsigned char page;
	unsigned char offset;
	unsigned int length;
	int ret = 0;

	while (command < block->commands) {
		data = block->data + command * 4;
		book = data[0];
		page = data[1];
		offset = data[2];
		command++;

		if (offset <= 0x7f) {
			ret = tas2557_dev_write(priv, TAS2557_REG(book, page, offset), data[3]);
			if (ret)
				return ret;
			continue;
		}

		if (offset == 0x81) {
			msleep((book << 8) + page);
			continue;
		}

		if (offset != 0x85)
			continue;

		data += 4;
		length = (book << 8) + page;
		book = data[0];
		page = data[1];
		offset = data[2];

		ret = tas2557_dev_bulk_write(priv, TAS2557_REG(book, page, offset),
					     data + 3, length);
		if (ret)
			return ret;

		command++;
		if (length >= 2)
			command += ((length - 2) / 4) + 1;
	}

	return 0;
}

static int tas2557_load_data_type(struct tas2557_priv *priv,
				  const struct tas2557_data_image *image,
				  unsigned int type)
{
	unsigned int index;
	bool matched = false;
	int ret = 0;

	for (index = 0; index < image->blocks; index++) {
		if (image->block[index].type != type)
			continue;

		matched = true;
		ret = tas2557_load_block(priv, &image->block[index]);
		if (ret)
			return ret;
	}

	return matched ? 0 : -ENOENT;
}

static bool tas2557_cfg_is_calibration(const struct tas2557_configuration *cfg)
{
	return strncmp(cfg->name, "calibration_", 12) == 0;
}

static int tas2557_apply_calibration(struct tas2557_priv *priv)
{
	unsigned int index;
	int ret;

	if (!priv->firmware_loaded || priv->current_configuration == U32_MAX)
		return 0;

	for (index = 0; index < priv->fw.configurations; index++) {
		const struct tas2557_configuration *cfg = &priv->fw.configuration[index];

		if (!tas2557_cfg_is_calibration(cfg))
			continue;
		if (cfg->program != priv->current_program)
			continue;
		if (cfg->sampling_rate != priv->current_sample_rate)
			continue;

		ret = tas2557_load_data_type(priv, &cfg->data,
					     TAS2557_BLOCK_CFG_COEFF_DEV_A);
		if (ret && ret != -ENOENT)
			return ret;
	}

	return 0;
}

static int tas2557_pick_configuration(struct tas2557_priv *priv,
				      unsigned int program,
				      int requested_config)
{
	unsigned int index;
	int last = -ENOENT;

	if (requested_config >= 0) {
		if ((unsigned int)requested_config >= priv->fw.configurations)
			return -EINVAL;
		if (priv->fw.configuration[requested_config].program != program)
			return -EINVAL;
		return requested_config;
	}

	for (index = 0; index < priv->fw.configurations; index++) {
		const struct tas2557_configuration *cfg = &priv->fw.configuration[index];

		if (cfg->program != program)
			continue;
		if (tas2557_cfg_is_calibration(cfg))
			continue;
		if (!priv->current_sample_rate ||
		    cfg->sampling_rate == priv->current_sample_rate)
			last = index;
	}

	if (last >= 0)
		return last;

	for (index = 0; index < priv->fw.configurations; index++) {
		if (priv->fw.configuration[index].program == program &&
		    !tas2557_cfg_is_calibration(&priv->fw.configuration[index]))
			return index;
	}

	return -ENOENT;
}

static int tas2557_load_configuration(struct tas2557_priv *priv,
				      int prev_config,
				      int new_config,
				      bool restore_power)
{
	const struct tas2557_configuration *prev = NULL;
	const struct tas2557_configuration *cfg;
	const struct tas2557_pll *pll;
	int ret;

	if ((unsigned int)new_config >= priv->fw.configurations)
		return -EINVAL;

	cfg = &priv->fw.configuration[new_config];
	if ((unsigned int)cfg->program >= priv->fw.programs)
		return -EINVAL;

	if (prev_config >= 0 && (unsigned int)prev_config < priv->fw.configurations)
		prev = &priv->fw.configuration[prev_config];

	if (!prev || prev->pll != cfg->pll) {
		if (cfg->pll >= priv->fw.plls)
			return -EINVAL;
		pll = &priv->fw.pll[cfg->pll];
		ret = tas2557_load_block(priv, &pll->block);
		if (ret)
			return ret;
	}

	ret = tas2557_load_data_type(priv, &cfg->data, TAS2557_BLOCK_CFG_PRE_DEV_A);
	if (ret && ret != -ENOENT)
		return ret;

	ret = tas2557_load_data_type(priv, &cfg->data, TAS2557_BLOCK_CFG_COEFF_DEV_A);
	if (ret)
		return ret;

	ret = tas2557_load_data_type(priv, &cfg->data, TAS2557_BLOCK_CFG_POST);
	if (ret && ret != -ENOENT)
		return ret;

	if (restore_power) {
		ret = tas2557_dev_load_data(priv, tas2557_startup_data);
		if (ret)
			return ret;

		ret = tas2557_load_data_type(priv, &cfg->data, TAS2557_BLOCK_CFG_POST_POWER);
		if (ret && ret != -ENOENT)
			return ret;

		ret = tas2557_dev_load_data(priv, tas2557_unmute_data);
		if (ret)
			return ret;

		ret = tas2557_apply_calibration(priv);
		if (ret)
			return ret;

		priv->powered = true;
	}

	priv->current_configuration = new_config;
	priv->current_sample_rate = cfg->sampling_rate;

	return 0;
}

static int tas2557_set_program(struct tas2557_priv *priv,
			       unsigned int program,
			       int requested_config)
{
	bool was_powered = priv->powered;
	int config;
	int ret;

	if (!priv->firmware_loaded)
		return -EINVAL;
	if (program >= priv->fw.programs)
		return -EINVAL;

	config = tas2557_pick_configuration(priv, program, requested_config);
	if (config < 0)
		return config;

	if (was_powered) {
		ret = tas2557_dev_load_data(priv, tas2557_shutdown_data);
		if (ret)
			return ret;
		priv->powered = false;
	}

	tas2557_hw_reset(priv);
	ret = tas2557_dev_write(priv, TAS2557_SW_RESET_REG, 0x01);
	if (ret)
		return ret;
	usleep_range(1000, 2000);

	ret = tas2557_load_default(priv);
	if (ret)
		return ret;

	ret = tas2557_load_data_type(priv, &priv->fw.program[program].data, TAS2557_BLOCK_PGM_ALL);
	if (ret && ret != -ENOENT)
		return ret;

	ret = tas2557_load_data_type(priv, &priv->fw.program[program].data, TAS2557_BLOCK_PGM_DEV_A);
	if (ret)
		return ret;

	priv->current_program = program;

	return tas2557_load_configuration(priv, -1, config, was_powered);
}

static int tas2557_set_sampling_rate(struct tas2557_priv *priv, unsigned int rate)
{
	unsigned int index;
	int last = -ENOENT;

	if (!priv->firmware_loaded)
		return 0;
	if (rate == priv->current_sample_rate)
		return 0;

	for (index = 0; index < priv->fw.configurations; index++) {
		const struct tas2557_configuration *cfg = &priv->fw.configuration[index];

		if (cfg->program != priv->current_program)
			continue;
		if (tas2557_cfg_is_calibration(cfg))
			continue;
		if (cfg->sampling_rate != rate)
			continue;

		last = index;
	}

	if (last >= 0)
		return tas2557_load_configuration(priv,
			priv->current_configuration == U32_MAX ? -1 :
			(int)priv->current_configuration,
			last, priv->powered);

	dev_warn(priv->dev, "no TAS2557 firmware configuration for %u Hz\n", rate);
	return -EINVAL;
}

static const char *tas2557_select_firmware_name(struct tas2557_priv *priv)
{
	switch (priv->pgid) {
	case TAS2557_PG_VERSION_2P1:
		return TAS2557_FW_NAME;
	case TAS2557_PG_VERSION_1P0:
		return TAS2557_PG1P0_FW_NAME;
	default:
		return NULL;
	}
}

static int tas2557_load_firmware(struct tas2557_priv *priv)
{
	const struct firmware *fw;
	const char *fw_name;
	int ret;

	if (priv->firmware_loaded)
		return 0;

	fw_name = tas2557_select_firmware_name(priv);
	if (!fw_name)
		return -ENOTSUPP;

	ret = request_firmware(&fw, fw_name, priv->dev);
	if (ret)
		return dev_err_probe(priv->dev, ret, "failed to request %s\n", fw_name);

	tas2557_clear_firmware(&priv->fw);
	ret = tas2557_fw_parse(priv, &priv->fw, (unsigned char *)fw->data, fw->size);
	release_firmware(fw);
	if (ret)
		return ret;
	if (!priv->fw.programs || !priv->fw.configurations)
		return -EINVAL;

	priv->firmware_loaded = true;
	ret = tas2557_set_program(priv, 0, -1);
	if (ret)
		return ret;

	dev_info(priv->dev, "loaded TAS2557 firmware with %u programs and %u configurations\n",
		 priv->fw.programs, priv->fw.configurations);

	return 0;
}

static int tas2557_power(struct tas2557_priv *priv, bool enable)
{
	int ret;
	unsigned int value;

	ret = tas2557_dev_read(priv, TAS2557_SAFE_GUARD_REG, &value);
	if (ret)
		return ret;

	if ((value & 0xff) != TAS2557_SAFE_GUARD_PATTERN)
		return -EIO;

	if (enable) {
		if (priv->powered)
			return 0;

		ret = tas2557_load_firmware(priv);
		if (ret)
			return ret;

		ret = tas2557_dev_load_data(priv, tas2557_startup_data);
		if (ret)
			return ret;

		if (priv->current_configuration != U32_MAX) {
			ret = tas2557_load_data_type(priv,
					     &priv->fw.configuration[priv->current_configuration].data,
					     TAS2557_BLOCK_CFG_POST_POWER);
			if (ret && ret != -ENOENT)
				return ret;
		}

		ret = tas2557_dev_load_data(priv, tas2557_unmute_data);
		if (ret)
			return ret;

		ret = tas2557_apply_calibration(priv);
		if (ret)
			return ret;

		priv->powered = true;
		return 0;
	}

	if (!priv->powered)
		return 0;

	ret = tas2557_dev_load_data(priv, tas2557_shutdown_data);
	if (!ret)
		priv->powered = false;

	return ret;
}

static int tas2557_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *component = dai->component;
	struct tas2557_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	mutex_lock(&priv->lock);
	ret = tas2557_power(priv, !mute);
	mutex_unlock(&priv->lock);

	return ret;
}

static int tas2557_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct tas2557_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	mutex_lock(&priv->lock);
	ret = tas2557_load_firmware(priv);
	if (!ret)
		ret = tas2557_set_sampling_rate(priv, params_rate(params));
	if (!ret)
		ret = tas2557_set_bit_rate(priv, snd_pcm_format_width(params_format(params)));
	mutex_unlock(&priv->lock);

	return ret;
}

static int tas2557_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct tas2557_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	mutex_lock(&priv->lock);
	ret = tas2557_set_serial_format(priv, fmt);
	mutex_unlock(&priv->lock);

	return ret;
}

static const struct snd_soc_dapm_widget tas2557_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUT_DRV("ClassD", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route tas2557_audio_map[] = {
	{ "DAC", NULL, "ASI1" },
	{ "ClassD", NULL, "DAC" },
	{ "OUT", NULL, "ClassD" },
};

static const struct snd_soc_component_driver tas2557_component_driver = {
	.dapm_widgets = tas2557_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tas2557_dapm_widgets),
	.dapm_routes = tas2557_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tas2557_audio_map),
	.endianness = 1,
	.use_pmdown_time = 1,
};

static const struct snd_soc_dai_ops tas2557_dai_ops = {
	.hw_params = tas2557_hw_params,
	.set_fmt = tas2557_set_dai_fmt,
	.mute_stream = tas2557_mute_stream,
	.no_capture_mute = 1,
};

#define TAS2557_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			 SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver tas2557_dai_driver = {
	.name = "tas2557 ASI1",
	.playback = {
		.stream_name = "ASI1 Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = TAS2557_FORMATS,
	},
	.ops = &tas2557_dai_ops,
	.symmetric_rate = 1,
};

static int tas2557_i2c_probe(struct i2c_client *client)
{
	struct tas2557_priv *priv;
	unsigned int value;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &client->dev;
	priv->i2s_bits = 16;
	priv->current_sample_rate = DEFAULT_SAMPLE_RATE_48K;
	priv->current_configuration = U32_MAX;
	mutex_init(&priv->lock);

	ret = devm_add_action_or_reset(&client->dev, tas2557_free_firmware_action, priv);
	if (ret)
		return ret;

	priv->regmap = devm_regmap_init_i2c(client, &tas2557_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(priv->regmap),
				     "failed to allocate regmap\n");

	priv->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(priv->reset_gpio),
				     "failed to get reset gpio\n");

	of_property_read_u8(client->dev.of_node, "ti,i2s-bits", &priv->i2s_bits);
	if (!priv->i2s_bits)
		priv->i2s_bits = 16;

	i2c_set_clientdata(client, priv);

	tas2557_hw_reset(priv);

	mutex_lock(&priv->lock);
	ret = tas2557_dev_write(priv, TAS2557_SW_RESET_REG, 0x01);
	if (!ret)
		usleep_range(1000, 2000);
	if (!ret)
		ret = tas2557_dev_read(priv, TAS2557_REV_PGID_REG, &value);
	if (!ret) {
		priv->pgid = value;
		if (value != TAS2557_PG_VERSION_1P0 && value != TAS2557_PG_VERSION_2P1)
			ret = -ENODEV;
	}
	if (!ret)
		ret = tas2557_load_default(priv);
	if (!ret) {
		ret = tas2557_load_firmware(priv);
		if (ret)
			dev_warn(priv->dev, "TAS2557 firmware load deferred: %d\n", ret);
		ret = 0;
	}
	mutex_unlock(&priv->lock);
	if (ret)
		return dev_err_probe(&client->dev, ret, "failed to initialize TAS2557\n");

	return devm_snd_soc_register_component(&client->dev,
					       &tas2557_component_driver,
					       &tas2557_dai_driver, 1);
}

static const struct of_device_id tas2557_of_match[] = {
	{ .compatible = "ti,tas2557" },
	{ }
};
MODULE_DEVICE_TABLE(of, tas2557_of_match);

static const struct i2c_device_id tas2557_i2c_ids[] = {
	{ "tas2557", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas2557_i2c_ids);

static struct i2c_driver tas2557_i2c_driver = {
	.driver = {
		.name = "tas2557",
		.of_match_table = tas2557_of_match,
	},
	.probe = tas2557_i2c_probe,
	.id_table = tas2557_i2c_ids,
};
module_i2c_driver(tas2557_i2c_driver);

MODULE_DESCRIPTION("Firmware-aware TAS2557 smart amplifier codec driver");
MODULE_AUTHOR("GitHub Copilot");
MODULE_LICENSE("GPL");
