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

"""AML-IP Fiware Node API specification."""

from flask import Flask, request, Response
import requests
import json
import time

import logging
from py_utils.logging.log_utils import CustomLogger

from amlip_py.node.AsyncEdgeNode import AsyncEdgeNode, InferenceListenerLambda
from amlip_py.types.InferenceDataType import InferenceDataType

from amlip_swig import FiwareNode as cpp_FiwareNode


headers_POST = {
    'Content-Type': 'application/json'
}

headers_GET = {
    'Accept': 'application/json'
}


class FiwareNode(cpp_FiwareNode):
    """
    AML-IP Fiware Node.

    """

    def __init__(
            self,
            name: str,
            server_ip: str,
            server_port: int,
            context_broker_ip: str = 'localhost',
            context_broker_port: int = 1026,
            entity_id: str = 'ID_0',
            entity_data: str = 'data',
            entity_solution: str = 'inference',
            domain: int = None,
            logger=CustomLogger(logger_name='FiwareNode', log_level=logging.WARNING)):
        """
        Create a new Fiware Node with a given name.

        Parameters
        ----------
        name : str
            Name of the node.
        server_ip : str
            IP address of the server.
        server_port : int
            Port of the server.
            Defaults to 1028.
        context_broker_ip : str
            IP address of the context broker.
            Defaults to 'localhost'.
        context_broker_port : int
            Port of the context broker.
            Defaults to 1026.
        entity_id : str
            ID of the context broker entity.
            Defaults to 'ID_0'.
        entity_data : str
            Name of the entity data attribute.
            Defaults to 'data'.
        entity_solution : str
            Name of the entity solution attribute.
            Defaults to 'inference'.
        logger : logging.Logger
            Logger object.
            Defaults to a new CustomLogger with log level WARNING.

        """
        #####
        # Parent class constructor
        if domain is None:
            super().__init__(name)
        else:
            super().__init__(name, domain)

        self.server_ip = server_ip
        self.server_port = server_port
        self.context_broker_ip = context_broker_ip
        self.context_broker_port = context_broker_port
        self.id = entity_id
        self.entity_data = entity_data
        self.entity_solution = entity_solution
        self.logger = logger
        self.logger.info(f'Creating FiwareNode with name: {name}')

        self.name = name

        self.edge = AsyncEdgeNode('edge_node'+name,
                                  listener=InferenceListenerLambda(self.inference_received))

        self.app = Flask(__name__)

        self.setup_routes()
        self.init_subscriptions()

    def setup_routes(self):
        """
        Handle requests to the /accumulate route.

        This function will be called for all HTTP methods (GET, POST, PUT, DELETE, PATCH).

        """

        self.app.add_url_rule('/accumulate',
                              view_func=self.accumulate,
                              methods=['GET', 'POST', 'PUT', 'DELETE', 'PATCH'])

    def init_subscriptions(self):
        """
        Initialize subscriptions to notify on all changes to the entity with self.id.

        """

        subscription_data = {
            'description': 'Notify me of all changes',
            'subject': {
                'entities': [
                    {
                        'id': self.id,
                        'type': 'string'
                    }
                ],
                'condition': {
                    'attrs': [
                        # make a notification trigger on changes to the self.entity_data attribute
                        self.entity_data
                    ],
                    'notifyOnMetadataChange': False
                }
            },
            'notification': {
                'http': {
                    'url': f'http://{self.server_ip}:{self.server_port}/accumulate'
                },
                'attrs': []  # all the attributes in the entity
            }
        }

        response = requests.post(
            f'http://{self.context_broker_ip}:{self.context_broker_port}/v2/subscriptions',
            headers=headers_POST,
            data=json.dumps(subscription_data))

        response.raise_for_status()

    def run(self):
        """
        Start the Flask server.

        """

        self.app.run(host=self.server_ip, port=self.server_port)

    def post_data(
            self,
            data):
        """
        Post data to the Fiware context broker.

        Parameters
        ----------
        data : dict
            Data to be posted.

        """

        entity_data = {
            'id': self.id,
            'type': 'string',
            self.entity_data: {
                'value': data,
                'type': 'InferenceDataType'
            },
            self.entity_solution: {
                'value': '',
                'type': 'InferenceSolutionDataType'
            }
        }

        try:
            response = requests.post(
                f'http://{self.context_broker_ip}:{self.context_broker_port}/v2/entities',
                headers=headers_POST,
                data=json.dumps(entity_data))

            response.raise_for_status()
            self.logger.info('Data posted successfully')

        except requests.exceptions.RequestException as e:
            self.logger.warning(f'Failed to post data: {e} ')
            raise e

    def inference_received(
            self,
            inference,
            task_id,
            server_id):
        """
        Handle inference data received from the inference node.

        Parameters
        ----------
        inference : InferenceSolutionDataType
            Inference of the data previously sent.
        task_id : str
            Id of the data which this inference answers.
        server_id : AmlipIdDataType
            Id of the InferenceNode which sent the data.

        """

        self.logger.info(f'Inference received from server: {server_id}\n'
                         f' with id: {task_id}\n'
                         f' inference: {inference.to_string()}')

        metadata = {
            'task_id': {
                'value': task_id,
                'type': 'string'
            },
            'server_id': {
                'value': server_id.to_string(),
                'type': 'string'
            }
        }

        self.patch_inference(inference.to_string(), metadata=metadata)

    def patch_inference(
            self,
            inference,
            metadata={}) -> None:
        """
        Patch inference data to the Fiware context broker.

        Parameters
        ----------
        inference : InferenceSolutionDataType
            Inference of the data previously sent.
        metadata : dict
            Metadata to be included with the inference data.
            Defaults to an empty dictionary.

        """

        data = {
            self.entity_solution: {
                'value': inference,
                'type': 'InferenceSolutionDataType',
                'metadata': metadata
            }
        }

        try:
            response = requests.patch(
                f'http://{self.context_broker_ip}:{self.context_broker_port}/' +
                f'v2/entities/{self.id}/attrs',
                headers=headers_POST, data=json.dumps(data))

            response.raise_for_status()
            self.logger.info('Inference data posted successfully')
        except requests.exceptions.RequestException as e:
            self.logger.warning(f'Failed to patch inference data: {e}')
            raise e

    def get_inference(
            self,
            timeout=0) -> dict:
        """
        Get inference data from the Fiware context broker.

        Parameters
        ----------
        timeout : int
            Timeout in seconds for the request.
            Defaults to 0 (no timeout).

        Returns
        -------
        inference : dict
            Inference data retrieved from the context broker.

        """

        start_time = time.time()
        inference = {}

        while True:
            try:
                response = requests.get(
                    f'http://{self.context_broker_ip}:{self.context_broker_port}/' +
                    f'v2/entities/{self.id}?attrs={self.entity_solution}',
                    headers=headers_GET)

                response.raise_for_status()

                inference = response.json()

                if inference[self.entity_solution]['value']:
                    self.logger.info('Inference data retrieved successfully')
                    break
                else:
                    self.logger.info('Empty inference data, retrying...')

            except requests.exceptions.HTTPError as err:
                self.logger.warning(f'HTTP error occurred: {err}')
                inference = json.dumps({'Error': '{}'.format(err)})
                break  # Exit loop on HTTP error
            except requests.exceptions.RequestException as err:
                self.logger.warning(f'Request exception occurred: {err}')
                inference = json.dumps({'Error': '{}'.format(err)})
                break  # Exit loop on request exception

            if timeout > 0 and (time.time() - start_time) > timeout:
                self.logger.warning('Timeout reached while waiting for inference data')
                inference = json.dumps({'Error': 'Timeout reached' +
                                        ' while waiting for inference data'})
                break

        time.sleep(1)  # Sleep for a second before retrying

        return inference

    def accumulate(self):
        """
        Handle /accumulate route to save request content.

        """

        try:
            self.record_request(request)
            if self.send_continue(request):
                self.logger.info('Sending 100-continue response')
                return Response(status=100)
            else:
                self.logger.info('Sending 200 OK response')
                return Response(status=200)
        except Exception as e:
            self.logger.warning(f'Error handling request: {e}')
            return Response(status=500)

    def record_request(self, request):
        """
        Common function used by several route methods to save request content.

        Parameters
        ----------
        request : flask.request
            The Flask request object containing request details.

        """

        # Summary
        request_summary = self.get_request_summary(request)

        # Separator
        s = '=======================================\n'

        # Data
        raw_data = self.get_request_data(request)
        data = json.dumps(raw_data, indent=4, sort_keys=True)

        if raw_data:
            inference_data = InferenceDataType(raw_data['data'][0][self.entity_data]['value'])
            self.edge.request_inference(inference_data)

            # TODO: add task_id to the entity metadata
            # (not supported by the context broker yet?
            # https://fiware-orion.readthedocs.io/en/2.4.0/user/metadata/index.html)

            # task_id = self.edge.request_inference(inference_data)
            # data = {
            #     'metadata': {
            #         'task_id': {
            #             'value': task_id,
            #             'type': 'string'
            #         }
            #     }
            # }

            # response = requests.patch(
            #     f'http://{context_broker_ip}:{context_broker_port}/v2/entities/{id}/attrs/data/',
            #     headers=headers_POST, data=json.dumps(data))

            # response.raise_for_status()

        self.logger.info(request_summary+s+data+s)

    def get_request_summary(self, request) -> str:
        """
        Construct a summary of the request including method, URL, headers, and query parameters.

        Parameters
        ----------
        request : flask.request
            The Flask request object.

        Returns
        -------
        summary : str
            A formatted string summarizing the request details.

        """

        summary = f'{request.method} {request.scheme}://{request.host}{request.path}'
        if request.args:
            summary += '?' + '&'.join([f'{k}={request.args[k]}' for k in request.args])
        summary += '\n'

        for header_name in self.sort_headers(request.headers.keys()):
            summary += f'{header_name}: {request.headers[header_name]}\n'

        return summary

    def get_request_data(self, request):
        """"
        Extract and process payload from the request data.

        Parameters
        ----------
        request : flask.request
            The Flask request object.

        Returns
        -------
        formatted_payload : dict
            The parsed JSON payload if available, else None.

        """

        if request.data:
            try:
                data = json.loads(request.data)
                return data
            except ValueError as e:
                self.logger.error(f'Error parsing JSON payload: {e}')

        return None

    @staticmethod
    def send_continue(request):
        """
        Inspect request header in order to look if we have to continue or not

        """

        for h in request.headers.keys():
            if ((h == 'Expect') and (request.headers[h] == '100-continue')):
                return True

        return False

    @staticmethod
    def dict_raise_on_duplicates(ordered_pairs):
        """
        Reject duplicate keys.

        """

        d = {}
        for k, v in ordered_pairs:
            if k in d:
                raise ValueError(f'duplicate key: {(k,)}')
            else:
                d[k] = v
        return d

    @staticmethod
    def sort_headers(headers):
        """
        Sort headers in a predefined order.

        """

        sorted = []
        headers_order = [
            'Fiware-Servicepath',
            'Content-Length',
            'X-Auth-Token',
            'User-Agent',
            'Ngsiv2-Attrsformat',
            'Host',
            'Accept',
            'Fiware-Service',
            'Content-Type',
            'Fiware-Correlator',
        ]

        # headers is a generator object, not exactly a list (i.e. it doesn't have remove method)
        headers_list = list(headers)

        sorted_headers = [h for h in headers_order if h in headers_list]
        remaining_headers = [h for h in headers_list if h not in headers_order]

        sorted = sorted_headers + remaining_headers

        return sorted
