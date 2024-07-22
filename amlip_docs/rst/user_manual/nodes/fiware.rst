.. include:: ../../exports/alias.include

.. _user_manual_nodes_fiware:

###########
Fiware Node
###########

This kind of node performs interaction with a `FIWARE context broker <https://fiware-orion.readthedocs.io/en/master/>`__ for handling inference data.
It provides mechanisms to read data from a context broker entity and request inference of this data from an :ref:`user_manual_nodes_inference`.
It also post the received inference to the context broker entity.
To facilitate the inference requests, the Fiware Node includes an :ref:`user_manual_nodes_edge`.

Steps
-----

* Create a new :code:`FiwareNode` object with at least a :code:`name`, a :code:`server_ip` and a :code:`server_port`.
* Start the Flask server.

.. tabs::

    .. tab:: Python

        .. code-block:: python

            server_ip = '192.168.1.1'
            server_port = 1028

            # Create a new Fiware Node
            node = FiwareNode(name='My_Fiware_Node',
                            server_ip=server_ip,
                            server_port=server_port,
                            context_broker_ip='localhost',
                            context_broker_port=1026,
                            entity_id='ID_entity',
                            entity_data='data',
                            entity_solution='inference',
                            domain=0,
                            logger=CustomLogger(logger_name='FiwareNode', log_level=logging.WARNING))

            # Start the Flask server
            fiware_node.run()
