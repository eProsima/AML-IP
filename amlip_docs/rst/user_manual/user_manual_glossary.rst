.. include:: ../exports/alias.include

.. _user_manual_glossary:

############
Nomenclature
############


AML-IP nomenclature
===================

.. glossary::

    Action
        Each of the steps in which any calculation or communication process is divided.

    Scenario
        Set of :term:`Actions <Action>` that performs a whole,
        independent and self-contained behavior inside an |aml| network.
        More information in :ref:`User Manual section <user_manual_scenarios>`.

    Node
        Independent and stand-alone piece of software that performs different :term:`Actions <Action>`.
        Each Node belongs to one and only one :term:`Scenario`.
        More information in :ref:`User Manual section <user_manual_nodes>`.


AML nomenclature
================

    Atomization
        Specific state of an |aml| model.


DDS nomenclature
================

.. glossary::

    Publish
        To send a data or message to all the entities in the network that have :term:`Subscribe` to the same
        :term:`Topic` in which it is being publish.

    Subscribe
        To connect to a specific :term:`Topic` and awaits for messages :term:`Publish` in such topic.

    Topic
        An abstract channel of communication that connects *Publishes* that :term:`Publish`
        and *Subscribers* that :term:`Subscribe`.

    Endpoint
        Individual Entity that can :term:`Subscribe` or :term:`Publish` in a specific :term:`Topic`.

    QoS
        **Quality of Service**. Configurations of :term:`Topic` and :term:`Endpoint` that allow to specify the
        communication behavior.
        This allows to create *reliable* or *best-effort* communication channels,
        to determine the life of a data sent, to set internal configurations, etc.
