.. include:: ../../exports/alias.include

.. _user_manual_scenarios_collaborative_learning:

###############################
Collaborative Learning Scenario
###############################

In this :term:`Scenario`, *Model Manager Receiver* and *Model Manager Sender* nodes working on the same problem share their locally obtained models with each other, without having to share the private datasets with which they were trained on.
This intends to lead towards a more complex and accurate model.
It leverages the :ref:`protocols_dds_rpc` communication protocol/paradigm in order to exchange all required information (model requests/replies) in an efficient way.

The Model Manager Sender Nodes publish :ref:`user_manual_scenarios_collaborative_learning_statistics` 's while the Model Manager Receiver Nodes listen to them.
When a :ref:`user_manual_nodes_model_receiver` is interested in a model based on its :ref:`user_manual_scenarios_collaborative_learning_statistics`, it sends a :ref:`user_manual_scenarios_collaborative_learning_model_request` request to the Model Manager Sender Node that sent the :ref:`user_manual_scenarios_collaborative_learning_statistics`.
The :ref:`user_manual_nodes_model_sender` will respond with a :ref:`user_manual_scenarios_collaborative_learning_model_reply`.

.. figure:: /rst/figures/scenarios/collaborative_learning_scenario.png
    :align: center
    :width: 80%

.. _user_manual_scenarios_collaborative_learning_model_request:

Model Request Data Type
=======================

The **Model Request** Data Type represents a problem request.
Internally, *requests* sent from a *Model Manager Receiver Node* to a *Model Manager Sender Node* are treated as byte arrays of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.

.. note::

    A more specific Data Type will be implemented in future releases for efficiency improvements.


.. _user_manual_scenarios_collaborative_learning_model_reply:

Model Reply Data Type
=====================

The **Model Reply** Data Type represents a problem reply with the requested model.
The *replies* sent from a *Model Manager Sender Node* to a *Model Manager Receiver Node* are treated as a byte array of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.

.. note::

    A more specific Data Type will be implemented in future releases for efficiency improvements.


.. _user_manual_scenarios_collaborative_learning_statistics:


Model Statistics Data Type
==========================

The **Statistics** Data Type represents the statistics of models, such as their number of parameters or the datasets they were trained on.
The *messages* sent from a *Model Manager Sender Node* to a *Model Manager Receiver Node* are treated as a byte array of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.

.. note::

    A more specific Data Type will be implemented in future releases for efficiency improvements.
