/*
 * Copyright (c) 2022 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/w1.h>
#include <zephyr.h>
#include <ztest.h>

#define W1_DUMMY_DEVICE_1 DT_INST(0, test_w1_dummy_device)
#define W1_DUMMY_DEVICE_2 DT_INST(1, test_w1_dummy_device)

#define ZEPHYR_W1_SERIAL DT_INST(0, zephyr_w1_serial)

#if DT_NODE_HAS_STATUS(ZEPHYR_W1_SERIAL, okay)
#define W1_CONTROLLER ZEPHYR_W1_SERIAL
#else
#error Your devicetree has no enabled nodes with a compatible w1-driver
#endif

const struct device *get_w1_controller_dev(void)
{
	const struct device *dev = DEVICE_DT_GET(W1_CONTROLLER);

	zassert_true(device_is_ready(dev), "W1 controller not found");

	return dev;
}

const struct device *get_w1_dummy_device_1(void)
{
	const struct device *dev = DEVICE_DT_GET(W1_DUMMY_DEVICE_1);

	zassert_true(device_is_ready(dev), "W1 dummy device 1 not found");

	return dev;
}

const struct device *get_w1_dummy_device_2(void)
{
	const struct device *dev = DEVICE_DT_GET(W1_DUMMY_DEVICE_2);

	zassert_true(device_is_ready(dev), "W1 dummy device 2 not found");

	return dev;
}

/* test vectors: */
const uint8_t rom_01_bytes[] = { 0x2d, 0x18, 0x08, 0xf5, 0x2d, 0x00, 0x00, 0x67 };
const uint8_t rom_02_bytes[] = { 0x2d, 0x2d, 0xfc, 0xf4, 0x2d, 0x00, 0x00, 0x57 };
const uint8_t rom_03_bytes[] = { 0x2d, 0xa8, 0xdc, 0xf2, 0x2d, 0x00, 0x00, 0xa7 };

const uint64_t rom_01_64 = 0x2d1808f52d000067;
const uint64_t rom_02_64 = 0x2d2dfcf42d000057;
const uint64_t rom_03_64 = 0x2da8dcf22d0000a7;

const struct w1_rom rom_01 = {
	.family = 0x2d,
	.serial = { 0x18, 0x08, 0xf5, 0x2d, 0x00, 0x00 },
	.crc = 0x67,
};
const struct w1_rom rom_02 = {
	.family = 0x2d,
	.serial = { 0x2d, 0xfc, 0xf4, 0x2d, 0x00, 0x00 },
	.crc = 0x57,
};
const struct w1_rom rom_03 = {
	.family = 0x2d,
	.serial = { 0xa8, 0xdc, 0xf2, 0x2d, 0x00, 0x00 },
	.crc = 0xa7,
};

const uint8_t crc16_1_in[11] = { 0x0f, 0x00, 0x00, 0xff, 0xee, 0xdd,
				 0xcc, 0xdd, 0xcc, 0xbb, 0xff };
const uint16_t crc16_1 = 0x60bb;
const uint8_t crc16_2_in[11] = { 0x0f, 0x08, 0x00, 0xaa, 0xbb, 0xcc,
				 0xdd, 0xaa, 0xbb, 0xcc, 0xdd };
const uint16_t crc16_2 = 0x8909;
const uint8_t crc16_3_in[12] = { 0xaa, 0x00, 0x00, 0x07, 0x00, 0x00,
				 0x00, 0xcc, 0xaa, 0xbb, 0xcc, 0xdd };
const uint16_t crc16_3 = 0x5d69;

void test_w1_basic(void)
{
	const struct device *w1_dev = get_w1_controller_dev();
	size_t peripheral_count;

	zassert_equal(w1_lock_bus(w1_dev), 0, "Fail lock 1");
	zassert_equal(w1_lock_bus(w1_dev), 0, "Fail lock 2");
	zassert_equal(w1_unlock_bus(w1_dev), 0, "Fail unlock 1");
	zassert_equal(w1_unlock_bus(w1_dev), 0, "Fail unlock 2");

	peripheral_count = w1_get_peripheral_count(w1_dev);
	zassert_equal(peripheral_count, 2,
		      "peripheral_count does not match dt definitions: %u/2",
		      peripheral_count);
}

void test_w1_crc(void)
{
	uint8_t crc8_result;
	uint16_t crc16_result;

	/* crc8 */
	crc8_result = w1_crc8(rom_01_bytes, 8);
	zassert_equal(crc8_result, 0, "crc1: crc over complete rom not 0");

	crc8_result = w1_crc8(rom_02_bytes, 8);
	zassert_equal(crc8_result, 0, "crc2: crc over complete rom not 0");

	crc8_result = w1_crc8(rom_03_bytes, 7);
	zassert_equal(crc8_result, rom_03_bytes[7], "crc3 does not match");

	/* crc16 */
	crc16_result = w1_crc16(W1_CRC16_SEED, crc16_1_in, sizeof(crc16_1_in));
	zassert_equal(crc16_result, crc16_1, "crc16_1 does not match");

	crc16_result = w1_crc16(W1_CRC16_SEED, crc16_2_in, sizeof(crc16_2_in));
	zassert_equal(crc16_result, crc16_2, "crc16_2 does not match");

	crc16_result = w1_crc16(W1_CRC16_SEED, crc16_3_in, sizeof(crc16_3_in));
	zassert_equal(crc16_result, crc16_3, "crc16_3 does not match");
}

void test_w1_rom(void)
{
	struct w1_rom rom_x;
	uint64_t rom_x_64 = -1;

	rom_x_64 = w1_rom_to_uint64(&rom_01);
	zassert_equal(rom_01_64, rom_x_64,
		      "rom_01_struct converted to uint64 does not match");
	rom_x_64 = w1_rom_to_uint64(&rom_02);
	zassert_equal(rom_02_64, rom_x_64,
		      "rom_02_struct converted to uint64 does not match");

	w1_uint64_to_rom(rom_01_64, &rom_x);
	zassert_mem_equal(&rom_x, &rom_01, sizeof(rom_01),
			  "rom_01_64 converted to rom struct does not match");
	w1_uint64_to_rom(rom_03_64, &rom_x);
	zassert_mem_equal(&rom_x, &rom_03, sizeof(rom_03),
			  "rom_03_64 converted to rom struct does not match");
}

void test_w1_reset_empty(void)
{
	int ret;
	const struct device *w1_dev = get_w1_controller_dev();

	ret = w1_reset_bus(w1_dev);
	zassert_false((ret < 0), "w1_reset failed. Err: %d", ret);
	zassert_equal(ret, 0, "In case no devices are connected should return 0");
}

int found_w1_devices;

void w1_test_search_callback(struct w1_rom found_rom, void *callback_arg)
{
	ARG_UNUSED(found_rom);
	ARG_UNUSED(callback_arg);
	found_w1_devices++;
}

void test_w1_search_empty(void)
{
	int ret;
	const struct device *w1_dev = get_w1_controller_dev();

	ret = w1_search_rom(w1_dev, 0, 0);
	zassert_equal(ret, 0, "In case no devices are connected should return 0");

	ret = w1_search_alarm(w1_dev, 0, 0);
	zassert_equal(ret, 0, "In case no devices are connected should return 0");
}

void test_w1_fire_and_forget(void)
{
	int ret;
	const struct device *w1_dev = get_w1_controller_dev();
	const uint8_t block_send[8] = { 0x0F, 0x0E, 0x0D, 0x0C, 0xC0, 0xD0, 0xE0, 0xF0 };

	ret = w1_write_bit(w1_dev, false);
	zassert_equal(ret, 0, "write_bit: error: %d", ret);

	ret = w1_write_byte(w1_dev, 0x3b);
	zassert_equal(ret, 0, "write_byte: error: %d", ret);

	ret = w1_write_block(w1_dev, block_send, sizeof(block_send));
	zassert_equal(ret, 0, "write_block: error: %d", ret);
}

void test_w1_receive_nothing(void)
{
	int ret;
	const struct device *w1_dev = get_w1_controller_dev();
	uint8_t block_rcv[8] = { 0x0F, 0x0E, 0x0D, 0x0C, 0xC0, 0xD0, 0xE0, 0xF0 };
	const uint8_t block_ref[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	/* on idle bus without sender all received bits should be logical ones */

	ret = w1_read_bit(w1_dev);
	zassert_true((ret >= 0), "read_bit: error: %d", ret);
	zassert_equal(ret, 1, "bit: empty receive should be logical ones");

	ret = w1_read_byte(w1_dev);
	zassert_true((ret >= 0), "read_byte: error: %d", ret);
	zassert_equal(ret, 0xFF, "byte: empty receive should be logical 0xFF");

	ret = w1_read_block(w1_dev, block_rcv, sizeof(block_rcv));
	zassert_equal(ret, 0, "read_block: error: %d", ret);
	zassert_mem_equal(block_rcv, block_ref, sizeof(block_rcv),
			  "block: empty receive should be local all 0xFF");
}

void test_w1_peripheral(void)
{
	int ret;
	struct w1_config cfg_1 = { .rom_id = {} };
	const struct device *w1_dev = get_w1_controller_dev();
	const uint8_t block_send[8] = { 0x0F, 0x0E, 0x0D, 0x0C, 0xC0, 0xD0, 0xE0, 0xF0 };
	uint8_t block_rcv[8] = { 0x00 };

	ret = w1_read_rom(w1_dev, &cfg_1.rom_id);
	zassert_equal(ret, -ENODEV, "read_rom should fail w/o connected dev");

	ret = w1_match_rom(w1_dev, &cfg_1);
	zassert_equal(ret, -ENODEV, "match_rom should fail w/o connected dev");

	ret = w1_resume_command(w1_dev);
	zassert_equal(ret, -ENODEV, "resume command should fail w/o connected dev");

	ret = w1_skip_rom(w1_dev, &cfg_1);
	zassert_equal(ret, -ENODEV, "skip_rom should fail w/o connected dev");

	ret = w1_reset_select(w1_dev, &cfg_1);
	zassert_equal(ret, -ENODEV, "reset_select should fail w/o connected dev");

	ret = w1_write_read(w1_dev, &cfg_1, block_send, 8, block_rcv, 0);
	zassert_equal(ret, -ENODEV, "w1_write_read should fail w/o connected dev");
}
