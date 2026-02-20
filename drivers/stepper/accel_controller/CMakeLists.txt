# Copyright (c) 2026 Navimatix GmbH
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_ZEPHYR_ACCEL_CONTROLLER zephyr_accel_controller.c)
