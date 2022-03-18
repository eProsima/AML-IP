"""Run Aml Main Node."""

import signal

import amlip_node


def handler(sig_num, curr_stack_frame):
    """Handle SIGINT."""
    pass


def main(args=None):
    """Run amlip_python Reader test."""
    signal.signal(signal.SIGINT, handler)
    print('Starting amlip_aml_py execution.')

    launch_reader()

    print('Stopping execution of amlip_aml_cpp gently.')


def launch_reader():
    """Create a ReaderNode that will be destroyed when finish function."""
    reader = amlip_node.ReaderNode('HelloWorld')
    reader.start()

    print('Node running, press ^C to exit.')

    signal.pause()


if __name__ == '__main__':
    main()
