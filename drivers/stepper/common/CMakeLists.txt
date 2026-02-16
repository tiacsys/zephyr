# SPDX-FileCopyrightText: Copyright (c) 2026 Navimatix GmbH
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories_ifdef(CONFIG_STEPPER_CONTROLLER_EVENT_COMMON include)

zephyr_library_sources_ifdef(CONFIG_STEPPER_CONTROLLER_EVENT_COMMON stepper_controller_event_common.c)
