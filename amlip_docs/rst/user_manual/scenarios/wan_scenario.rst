.. include:: ../../exports/alias.include

.. _user_manual_scenarios_wan:

############
WAN Scenario
############

.. todo

.. _user_manual_scenarios_wan_inference:

Inference Data Type
===================

The **Inference** Data Type represents a partial data-set.
Internally, *Inferences* sent from an *Edge Node* to an *Inference Node* are treated as byte arrays of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.
From Python API, the only way to interact with it is by `str` and `bytes` type.

.. _user_manual_scenarios_wan_inference_solution:

Inference Solution Data Type
======================

The **Inference Solution** Data Type represents....
The **Inference Solution** sent from a *Inference Node* to a *Edge Node* is treated as a bytes array of arbitrary size.
So far, the interaction with this class could be done from a :code:`void*`, a byte array or a string.
From Python API, the only way to interact with it is by `str` and `bytes` type.
