.. include:: ../../exports/alias.include

.. _user_manual_scenarios_workload_distribution:

##############################
Workload Distribution Scenario
##############################

This :term:`Scenario` performs the action of distributing a high computational effort *Task* in remote
nodes, in order to parallelize the task and do not block any other important actions that may require
to run in the same device.
It uses the :ref:`user_manual_dds_multiservice` communication to publish those tasks in an efficient way.

The *Task* distributed is the training data-set of an |aml| model.
This model is stored in a :ref:`user_manual_nodes_main` and the training data-set is divided in different *Jobs*,
that are sent along with states of the model to :ref:`Computing Nodes <user_manual_nodes_computing>`
in order to perform this training in parallel,
reducing the workload in the *Main Node* host, that may require to perform other actions at the same time.


.. _user_manual_scenarios_workload_distribution_job:

Job Data Type
=============

The **Job** Data Type represents a partial data-set and a model state.
Internally, *Jobs* sent from a *Main Node* to a *Computing Node* are treated as byte array of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.
From Python API, the only way to interact with it is by `str` type.

.. note::

    A more specific Data Type will be implemented in future releases for efficiency improvements.


.. _user_manual_scenarios_workload_distribution_solution:

Job Solution Data Type
======================

The **Solution** Data Type represents an *Atomization* or new model state.
The **Solution** sent from a *Computing Node* to a *Main Node* is treated as a bytes array of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.
From Python API, the only way to interact with it is by `str` type.

.. note::

    A more specific Data Type will be implemented in future releases for efficiency improvements.
