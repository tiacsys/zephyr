#
# SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
# SPDX-License-Identifier: Apache-2.0
#

*** Settings ***
Documentation    Robot testfile to check for existence of commands.
Library          twister_harness.robot_framework.ZephyrLibrary

*** Test Cases ***
Command workflow test
   [Documentation]    Test that all commands exist.
   Get a Device
   Run Device
   Run Command        echo 'my device'
   Close Device
