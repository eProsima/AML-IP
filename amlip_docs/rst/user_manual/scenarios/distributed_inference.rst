.. include:: ../../exports/alias.include

.. _user_manual_scenarios_distributed_inference:

##############################
Distributed Inference Scenario
##############################

This :term:`Scenario` involves the action of distributing a large amount of *Data* to remote
nodes to perform their inferences, in order to parallelize them and do not block any other important actions that may require to run in the same device.
By performing this action, the system ensures seamless execution of multiple tasks, optimizing overall performance and resource utilization.
It uses the :ref:`protocols_dds_multiservice` to efficiently publish and distribute the data across remote nodes, ensuring a streamlined and effective process.

The inference is performed in a :ref:`user_manual_nodes_inference` and sent to an :ref:`user_manual_nodes_edge`.

.. figure:: /rst/figures/distributed_inference_scenario.png
    :align: center
    :width: 75%

.. _user_manual_datatype_inference:

Inference Data Type
===================

The **Inference** Data Type represents a partial data-set.
Internally, the data sent from an *Edge Node* to an *Inference Node* are treated as byte arrays of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.
From Python API, the way to interact with it is by `str` and `bytes` type.

.. _user_manual_datatype_inference_solution:

Inference Solution Data Type
============================

The **Inference Solution** Data Type represents the inference of the data sent by the *Edge Node*.
The **Inference Solution** sent from an *Inference Node* to an *Edge Node* is treated as a bytes array of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.
From Python API, the way to interact with it is by `str` and `bytes` type.

.. note::

    There is no real data type here, the data format inside is whatever the user wants it to be.
