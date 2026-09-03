from ros2cli.command import CommandExtension

import app


class CmdCenterCommand(CommandExtension):
    """Launch the ros2-cmdctr terminal UI."""

    NAME = 'cmd_center'
    EXTENSION_POINT_VERSION = '0.1'

    def add_arguments(self, parser, cli_name, *, argv=None):
        pass

    def main(self, *, parser, args):
        app.main()
        return 0
