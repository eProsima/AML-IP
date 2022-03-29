"""Run Aml Main Node."""

import signal
import threading

from amlip_node_py.node.StatusAmlipNode import StatusAmlipNode


def handler(sig_num, curr_stack_frame):
    """Handle SIGINT."""
    print('SIGINT received.')
    pass


def main(args=None):
    """Run amlip_python Reader test."""
    signal.signal(signal.SIGINT, handler)

    print('Starting amlip_aml_py execution.')

    # Create thread
    node = StatusAmlipNode(
        lambda status: print(f'Status read from PY: {status}'))
    print(f'Node Status created with id: {node.get_id()}.')

    # Execute spin in another thread
    thread_ = threading.Thread(target=node.spin)
    thread_.start()
    print('Node running, press ^C to exit.')

    # Wait signal
    signal.pause()
    print('Signal received, stopping node.')

    # Stop node
    node.stop()
    thread_.join()
    print('Node stopped correctly.')

    print('Stopping execution of amlip_aml_cpp gently.')


if __name__ == '__main__':
    main()
