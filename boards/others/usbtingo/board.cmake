# SPDX-FileCopyrightText: Copyright (c) 2026 Navimatix GmbH
# SPDX-FileCopyrightText: Copyright (c) 2026 TiaC Systems
# SPDX-License-Identifier: Apache-2.0

board_set_flasher(blhost)

board_runner_args(dfu-util "--pid=0483:df11" "--alt=0" "--dfuse")
board_runner_args(blhost "--dev-id-type=usb" "--dev-id=0x1FC9:0x0022")
board_runner_args(blhost "--reset")

include(${ZEPHYR_BASE}/boards/common/dfu-util.board.cmake)
include(${ZEPHYR_BASE}/boards/common/blhost.board.cmake)
