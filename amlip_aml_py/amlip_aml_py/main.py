"""Run Aml Main Node."""

import signal

from amlip_node_py.amlip_node_py.node import StatusAmlipNode


def handler(sig_num, curr_stack_frame):
    """Handle SIGINT."""
    pass


def main(args=None):
    """Run amlip_python Reader test."""
    signal.signal(signal.SIGINT, handler)
    print('Starting amlip_aml_py execution.')

    launch_node()

    print('Stopping execution of amlip_aml_cpp gently.')


def launch_node():
    """Create a ReaderNode that will be destroyed when finish function."""
    node = StatusAmlipNode()
    node.spin()

    print('Node running, press ^C to exit.')

    signal.pause()

    node.stop()


if __name__ == '__main__':
    main()
