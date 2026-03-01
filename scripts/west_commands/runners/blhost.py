# SPDX-FileCopyrightText: Copyright (c) 2026 Navimatix GmbH
# SPDX-FileCopyrightText: Copyright (c) 2026 TiaC Systems
# SPDX-License-Identifier: Apache-2.0

'''Runner for flashing with blhost.'''

import logging
import os
import subprocess
from pathlib import Path

from runners.core import MissingProgram, RunnerCaps, ZephyrBinaryRunner


class BLHostBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for blhost.'''

    def __init__(
        self,
        cfg,
        dev_id=None,
        dev_id_type=None,
        erase=False,
        reset=False,
        commander=None,
        tool_opt=None,
    ):
        super().__init__(cfg)
        self.dev_id = dev_id
        self.dev_id_type = dev_id_type
        self.file = cfg.file
        self.file_type = cfg.file_type
        self.hex_name = cfg.hex_file
        self.bin_name = cfg.bin_file
        self.elf_name = cfg.elf_file
        self.erase = bool(erase)
        self.reset = bool(reset)
        self.commander = commander

        self.tool_opt = []
        if tool_opt is not None:
            for opts in [shlex.split(opt) for opt in tool_opt]:
                self.tool_opt += opts

    @classmethod
    def name(cls):
        return 'blhost'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'}, dev_id=True, erase=True, reset=True, tool_opt=True)

    @classmethod
    def dev_id_help(cls) -> str:
        return '''Device identifier. Use it in conjunction with the device identifier
                  type to select which device or instance to target when multiple ones
                  are connected. See SPSDK documentation for 'blhost' commander.'''

    @classmethod
    def do_add_parser(cls, parser):
        parser.add_argument(
            '--dev-id-type',
            choices=['buspal', 'can', 'lpcusbsio', 'port', 'sdio', 'usb', 'plugin'],
            default='usb',
            help='''Device identifier type. Use it to select which interface to target with
                    device identifier. See SPSDK documentation for 'blhost' commander.''',
        )
        parser.add_argument(
            '--commander',
            default='blhost',
            help="SPSDK Commander, default is blhost",
        )

    @classmethod
    def do_create(cls, cfg, args):
        return BLHostBinaryRunner(
            cfg,
            dev_id=args.dev_id,
            dev_id_type=args.dev_id_type,
            erase=args.erase,
            reset=args.reset,
            commander=args.commander,
            tool_opt=args.tool_opt,
        )

    def do_run(self, command, **kwargs):
        try:
            self.commander = os.fspath(Path(self.require(self.commander)).resolve())
        except MissingProgram as err:
            self.logger.error(
                '''You may use `pip install spsdk` to install SPSDK 'blhost' commander.'''
            )
            raise err

        if command == 'flash':
            self.flash(**kwargs)

    def flash(self, **kwargs):
        kwargs = {}
        if not self.logger.isEnabledFor(logging.DEBUG):
            kwargs['stdout'] = subprocess.DEVNULL

        # Determine device type for SPSDK Commander
        if self.dev_id_type and self.dev_id:
            # Build device selection for SPSDK Commander
            device_args = [f'--{self.dev_id_type}', f'{self.dev_id}']
        else:
            raise RuntimeError('Please specify a device type and identifier with the '
                               '-i/--dev-id and --dev-id-type command-line switch.')

        cmd_flash = (
            [self.commander]
            + device_args
            + self.tool_opt
        )

        self.ensure_output('bin')
        self.logger.info(f"Flashing file: {self.cfg.bin_file}")

        if self.erase is True:
            cmd = cmd_flash + ['flash-erase-all']
            self.logger.info(f"Command: {cmd}")
            self.check_call(cmd, **kwargs)

        cmd = cmd_flash + ['flash-image'] + [f'{self.cfg.bin_file}'] + ['erase']
        self.logger.info(f"Command: {cmd}")
        self.check_call(cmd, **kwargs)

        if self.reset is True:
            cmd = cmd_flash + ['reset']
            self.logger.info(f"Command: {cmd}")
            self.check_call(cmd, **kwargs)
