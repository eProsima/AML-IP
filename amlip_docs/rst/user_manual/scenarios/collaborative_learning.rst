.. include:: ../../exports/alias.include

.. _user_manual_scenarios_collaborative_learning:

###############################
Collaborative Learning Scenario
###############################

This :term:`Scenario` performs the action of  work on the same problem independently in remote nodes,
using different training datasets, in order to share the knowledge learned with other nodes and
do not block any other important actions that may require to run in the same device.
This creates a more complex and accurate model.
It uses the :ref:`user_manual_dds_rpc` communication to publish those problems in an efficient way.

.. figure:: /rst/figures/collaborative_learning_scenario.png
    :align: center

.. _user_manual_scenarios_collaborative_learning_model:

Model Data Type
===============

The **Model** Data Type represents a problem request.
Internally, *requests* sent from a *Model Manager Receiver Node* to a *Model Manager Sender Node* are treated as byte arrays of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.

.. note::

    A more specific Data Type will be implemented in future releases for efficiency improvements.


.. _user_manual_scenarios_collaborative_learning_solution:

Model Solution Data Type
========================

The **Solution** Data Type represents a problem reply with a more complex and accurate model.
The *replies* sent from a *Model Manager Sender Node* to a *Model Manager Receiver Node* are treated as a bytes array of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.

.. note::

    A more specific Data Type will be implemented in future releases for efficiency improvements.


.. _user_manual_scenarios_collaborative_learning_statistics:


Model Statistics Data Type
==========================

The **Statistics** Data Type represents the statistics of a model, such as the weights or the dataset.
The *messages* sent from a *Model Manager Sender Node* to a *Model Manager Receiver Node* are treated as a bytes array of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.

.. note::

    A more specific Data Type will be implemented in future releases for efficiency improvements.
