/*
 * Copyright (c) 2022 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <syscall_handler.h>
#include <drivers/w1.h>

static inline int z_vrfy_w1_reset_bus(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, reset_bus));

	return z_impl_w1_reset_bus((const struct device *)dev);
}
#include <syscalls/w1_reset_bus_mrsh.c>

static inline int z_vrfy_w1_read_bit(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, read_bit));

	return z_impl_w1_read_bit((const struct device *)dev);
}
#include <syscalls/w1_read_bit_mrsh.c>

static inline int z_vrfy_w1_write_bit(const struct device *dev, bool bit)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, write_bit));

	return z_impl_w1_write_bit((const struct device *)dev, bit);
}
#include <syscalls/w1_write_bit_mrsh.c>

static inline int z_vrfy_w1_read_byte(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, read_byte));

	return z_impl_w1_read_byte((const struct device *)dev);
}
#include <syscalls/w1_read_byte_mrsh.c>

static inline int z_vrfy_w1_write_byte(const struct device *dev, uint8_t byte)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, write_byte));

	return z_impl_w1_write_byte((const struct device *)dev, (uint8_t)byte);
}
#include <syscalls/w1_write_byte_mrsh.c>

static inline int z_vrfy_w1_change_bus_lock(const struct device *dev, bool lock)
{
	Z_OOPS(Z_SYSCALL_OBJ(dev, K_OBJ_DRIVER_W1));

	return z_impl_w1_change_bus_lock((const struct device *)dev, lock);
}
#include <syscalls/w1_change_bus_lock_mrsh.c>

static inline int z_vrfy_w1_configure(const struct device *dev,
				      enum w1_driver_settings_type type,
				      uint32_t value)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, configure));

	return z_impl_w1_configure(dev, type, value);
}
#include <syscalls/w1_configure_mrsh.c>

static inline size_t z_vrfy_w1_get_peripheral_count(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_OBJ(dev, K_OBJ_DRIVER_W1));

	return z_impl_w1_get_peripheral_count((const struct device *)dev);
}
#include <syscalls/w1_get_peripheral_count_mrsh.c>

#if CONFIG_W1_NET
static inline int z_vrfy_w1_search_bus(const struct device *dev,
				       uint8_t command, uint8_t family,
				       w1_search_callback_t callback_isr,
				       void *callback_arg)
{
	Z_OOPS(Z_SYSCALL_OBJ(dev, K_OBJ_DRIVER_W1));

	Z_OOPS(Z_SYSCALL_VERIFY_MSG(callback_isr == 0,
				    "callbacks may not be set from user mode"));
	/* callback_arg is not dereferenced, no need to check parameter */

	return z_impl_w1_search_bus((const struct device *)dev,
				    (uint8_t)command, (uint8_t)family,
				    (w1_search_callback_t)callback_isr,
				    (void *)callback_arg);
}

#endif /* CONFIG_W1_NET */

#include <syscalls/w1_search_bus_mrsh.c>
