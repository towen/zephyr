# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_sources_ifdef(CONFIG_USERSPACE w1_handlers.c)

# drivers implementing link functions (read, write, reset)
# none implemented yet
