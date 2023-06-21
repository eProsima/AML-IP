.. include:: ../../exports/alias.include

.. _user_manual_data_inference:

#########
Inference
#########

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
