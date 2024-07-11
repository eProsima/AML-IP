# Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import signal
import requests

import logging
from py_utils.logging.log_utils import CustomLogger

from amlip_py.node.FiwareNode import FiwareNode

# Domain ID
DOMAIN_ID = 166

# Set up logging
logger = CustomLogger(logger_name='PyTestFiwareNode', log_level=logging.INFO)


def main():
    """Execute main routine."""

    # Create node
    logger.info('Starting Manual Test Fiware Node Py execution. Creating Node...')

    try:
        ip = '192.168.1.51'
        fiware_node = FiwareNode('PyTestFiwareNode', ip, logger)
        logger.info(f'FiwareNode started at ip : {ip}')

        # start the Flask server
        fiware_node.app.run(host=ip, port=1028)

    except requests.exceptions.RequestException as e:
        logger.error(f'Failed to initialize subscriptions: {e}')
    except Exception as e:
        logger.error(f'Failed to start FiwareNode: {e}')

    logger.info('Finishing Manual Test Agent Client Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
