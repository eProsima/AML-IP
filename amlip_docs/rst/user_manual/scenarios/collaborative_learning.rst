.. include:: ../../exports/alias.include

.. _user_manual_scenarios_collaborative_learning:

###############################
Collaborative Learning Scenario
###############################

In this :term:`Scenario`, multiple nodes working on the same problem share their locally obtained models with each other, without the need to share the private datasets with which they were trained.
This cooperation intends to lead towards the requested model.
It leverages the :ref:`user_manual_dds_rpc` communication protocol/paradigm in order to exchange all required information (model requests/replies) in an efficient way.

.. figure:: /rst/figures/collaborative_learning_scenario.png
    :align: center
    :width: 75%

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

The **Solution** Data Type represents a problem reply with the requested model.
The *replies* sent from a *Model Manager Sender Node* to a *Model Manager Receiver Node* are treated as a bytes array of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.

.. note::

    A more specific Data Type will be implemented in future releases for efficiency improvements.


.. _user_manual_scenarios_collaborative_learning_statistics:


Model Statistics Data Type
==========================

The **Statistics** Data Type represents the statistics of models, such as the weights or the dataset.
The *messages* sent from a *Model Manager Sender Node* to a *Model Manager Receiver Node* are treated as a bytes array of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.

.. note::

    A more specific Data Type will be implemented in future releases for efficiency improvements.
