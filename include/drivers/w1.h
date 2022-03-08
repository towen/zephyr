/*
 * Copyright (c) 2018 Roman Tataurov <diytronic@yandex.ru>
 * Copyright (c) 2022 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public 1-Wire Driver APIs
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_W1_H_
#define ZEPHYR_INCLUDE_DRIVERS_W1_H_

#include <device.h>
#include <sys/crc.h>
#include <zephyr/types.h>
#include <sys/byteorder.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 1-Wire Interface
 * @defgroup w1_interface 1-Wire Interface
 * @ingroup io_interfaces
 * @{
 */

/*
 * Count the number of peripheral devices expected on the bus.
 * This can be used to decide if the bus has a multi-drop topology or
 * only a single device is present.
 * There is a comma after each ordinal (including the last)
 * Hence FOR_EACH adds "+1" once too often which has to be subtracted in the end.
 */
#define F1(x) 1
#define W1_PERIPHERALS_COUNT(node_id) \
		(FOR_EACH(F1, (+), DT_SUPPORTS_DEP_ORDS(node_id)) - 1)
#define W1_INST_PERIPHERALS_COUNT(inst)  \
		(W1_PERIPHERALS_COUNT(DT_DRV_INST(inst)))

/** W1 driver settings types */
enum w1_driver_settings_type {
	W1_SETTING_SPEED,
	W1_SETTING_STRONG_PULLUP,
};

/** holds configuration data common to all w1-controller implementations */
struct w1_controller_config {
	/* Number of connected client devices */
	uint16_t client_count;
};

struct w1_controller_data {
	/* The mutex used by w1_lock_bus and w1_unlock_bus methods */
	struct k_mutex bus_lock;
};

/**  @cond INTERNAL_HIDDEN */
typedef int (*w1_reset_bus_t)(const struct device *dev);
typedef int (*w1_read_bit_t)(const struct device *dev);
typedef int (*w1_write_bit_t)(const struct device *dev, bool bit);
typedef int (*w1_read_byte_t)(const struct device *dev);
typedef int (*w1_write_byte_t)(const struct device *dev, const uint8_t byte);
typedef size_t (*w1_get_peripheral_count_t)(const struct device *dev);
typedef int (*w1_configure_t)(const struct device *dev,
			      enum w1_driver_settings_type type,
			      uint32_t value);

__subsystem struct w1_driver_api {
	w1_reset_bus_t reset_bus;
	w1_read_bit_t read_bit;
	w1_write_bit_t write_bit;
	w1_read_byte_t read_byte;
	w1_write_byte_t write_byte;
	w1_configure_t configure;
};
/** @endcond */

/** @cond INTERNAL_HIDDEN */
__syscall int w1_change_bus_lock(const struct device *dev, bool lock);

static inline int z_impl_w1_change_bus_lock(const struct device *dev, bool lock)
{
	struct w1_controller_data *ctrl_data = dev->data;

	if (lock) {
		return k_mutex_lock(&ctrl_data->bus_lock, K_FOREVER);
	} else {
		return k_mutex_unlock(&ctrl_data->bus_lock);
	}
}
/** @endcond */

/**
 * @brief Lock the 1-wire bus to prevent simultaneous access.
 *
 * This routine locks the bus to prevent simultaneous access from different
 * threads. The calling thread waits until the bus becomes available.
 * A thread is permitted to lock a mutex it has already locked.
 *
 * @param dev  Pointer to the device structure for the driver instance.
 *
 * @return 0 on success, or a negative error code on error.
 */
static inline int w1_lock_bus(const struct device *dev)
{
	return w1_change_bus_lock(dev, true);
}

/**
 * @brief Unlock the 1-wire bus.
 *
 * This routine unlocks the bus to permit access to bus line.
 *
 * @param dev  Pointer to the device structure for the driver instance.
 *
 * @return 0 on success, or a negative error code on error.
 */
static inline int w1_unlock_bus(const struct device *dev)
{
	return w1_change_bus_lock(dev, false);
}

/**
 * @brief 1-Wire data link layer
 * @defgroup w1_data_link 1-Wire data link layer
 * @ingroup w1_interface
 * @{
 */

/**
 * @brief Reset the 1-Wire bus to prepare peripheral devices for communication.
 *
 * This routine resets all 1-Wire bus peripheral devices such that they are
 * ready to receive a command.
 * Connected peripherals answer with a presence pulse once they are ready
 * to receive data.
 *
 * In case the driver supports both standard speed and overdrive speed,
 * the reset routine takes care of sendig either a short or a long reset pulse
 * depending on the current state.
 *
 * @param dev    Pointer to the device structure for the driver instance.
 *
 * @retval < 0 In case of an error
 * @retval 0 If no devices answer with a present pulse.
 * @retval 1  If at least one device answers with a present pulse.
 */
__syscall int w1_reset_bus(const struct device *dev);

static inline int z_impl_w1_reset_bus(const struct device *dev)
{
	const struct w1_driver_api *api = dev->api;

	return api->reset_bus(dev);
}

/**
 * @brief Read a single bit from the 1-Wire device.
 *
 * @param dev  Pointer to the device structure for the driver instance.
 *
 * @return The read bit value on success, or a negative error code on error.
 */
__syscall int w1_read_bit(const struct device *dev);

static inline int z_impl_w1_read_bit(const struct device *dev)
{
	const struct w1_driver_api *api = dev->api;

	return api->read_bit(dev);
}

/**
 * @brief Write a single bit to the 1-Wire device.
 *
 * @param dev  Pointer to the device structure for the driver instance.
 * @param bit  Transmitting bit value 1 or 0.
 *
 * @return a negative error code on error
 */
__syscall int w1_write_bit(const struct device *dev, const bool bit);

static inline int z_impl_w1_write_bit(const struct device *dev, bool bit)
{
	const struct w1_driver_api *api = dev->api;

	return api->write_bit(dev, bit);
}

/**
 * @brief Read a single byte from the 1-Wire device.
 *
 * @param dev  Pointer to the device structure for the driver instance.
 *
 * @return The read byte value on success, or a negative error code on error.
 */
__syscall int w1_read_byte(const struct device *dev);

static inline int z_impl_w1_read_byte(const struct device *dev)
{
	const struct w1_driver_api *api = dev->api;

	return api->read_byte(dev);
}

/**
 * @brief Write a single byte to the 1-Wire device.
 *
 * @param dev   Pointer to the device structure for the driver instance.
 * @param byte  Transmitting byte.
 *
 * @return a negative error code on error
 */
__syscall int w1_write_byte(const struct device *dev, uint8_t byte);

static inline int z_impl_w1_write_byte(const struct device *dev, uint8_t byte)
{
	const struct w1_driver_api *api = dev->api;

	return api->write_byte(dev, byte);
}

/**
 * @brief Read a block of data from the 1-Wire device.
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param buffer  Pointer to receive buffer.
 * @param length  Length of receiving buffer (in bytes).
 *
 * @return a negative error code on error
 */
static inline int w1_read_block(const struct device *dev,
				uint8_t *buffer, size_t length)
{
	int ret;

	for (int i = 0; i < length; ++i) {
		ret = w1_read_byte(dev);
		if (ret < 0) {
			return ret;
		}
		buffer[i] = ret;
	}

	return 0;
}

/**
 * @brief Write a block of data from the 1-Wire device.
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param buffer  Pointer to transmitting buffer.
 * @param length  Length of transmitting buffer (in bytes).
 *
 * @return a negative error code on error
 */
static inline int w1_write_block(const struct device *dev,
				 const uint8_t *buffer, size_t length)
{
	int ret;

	for (int i = 0; i < length; ++i) {
		ret = w1_write_byte(dev, buffer[i]);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

/**
 * @brief Get the number of peripheral devices on the bus.
 *
 * @param dev  Pointer to the device structure for the driver instance.
 *
 * @return Number of connected w1 peripheral device, or
 *         a negative error code on error
 */
__syscall size_t w1_get_peripheral_count(const struct device *dev);

static inline size_t z_impl_w1_get_peripheral_count(const struct device *dev)
{
	const struct w1_controller_config *ctrl_cfg = dev->config;

	return ctrl_cfg->client_count;
}

/**
 * @brief Configure parameters of W1-Master.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param type enum specifying the setting type
 * @param value the settings value
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error, failed to configure device.
 */
__syscall int w1_configure(const struct device *dev,
			   enum w1_driver_settings_type type,
			   uint32_t value);

static inline int z_impl_w1_configure(const struct device *dev,
				      enum w1_driver_settings_type type,
				      uint32_t value)
{
	const struct w1_driver_api *api = dev->api;

	return api->configure(dev, type, value);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
#include <syscalls/w1.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_W1_H_ */
