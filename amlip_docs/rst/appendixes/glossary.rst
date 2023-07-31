.. include:: ../exports/alias.include

.. _glossary:

########
Glossary
########

.. glossary::

    API
        **Application Programming Interface**

    CI
        **Continuous Integration**

    OOP
        **Object Oriented Programming**

    OS
        **Operating System**

AML-IP
======

.. glossary::

    Action
        Each of the steps in which any calculation or communication process is divided.

    AML-IP
        **Algebraic Machine Learning - Integrating Platform**

    Scenario
        Set of :term:`Actions <Action>` that performs a whole, independent and self-contained behavior inside an |aml| network.
        More information in :ref:`User Manual section <user_manual_scenarios>`.

    Node
        Independent and stand-alone piece of software that performs different :term:`Actions <Action>`.
        Each Node belongs to one and only one :term:`Scenario`.
        More information in :ref:`User Manual section <user_manual_nodes>`.

AML
===

.. glossary::

    AML
        **Algebraic Machine Learning**

    Atomization
        Specific state of an |aml| model.

    ML
        **Machine Learning**

DDS
===

.. glossary::

    DDS
        **Data Distribution Service** protocol.
        Specification: `<https://www.omg.org/spec/DDS/>`_.
        More information in :ref:`User Manual section <technologies_dds>`.

    Domain Id
        Virtual partition for DDS networks.

    Endpoint
        Individual Entity that can :term:`Subscribe` or :term:`Publish` in a specific :term:`Topic`.

    Publish
        To send a data or message to all the entities in the network subscribed to the same :term:`Topic` in which the data is being published.

    RTPS
        **Real-time Publish-Subscribe** protocol `<https://www.omg.org/spec/DDSI-RTPS/>`_.

    Subscribe
        To connect to a specific :term:`Topic` and to receive messages published in such topic.

    Topic
        An abstract channel of communication that connects *Publishers* that :term:`Publish` and *Subscribers* that :term:`Subscribe`.

    OMG
        **Object Management Group** `<https://www.omg.org/>`_.

    QoS
        **Quality of Service**. Configurations of :term:`Topic` and :term:`Endpoint` that allow to specify the communication behavior.
        This allows to create *reliable* or *best-effort* communication channels, to determine the life of a data sent, to set internal configurations, etc.

Networking
==========

.. glossary::

    IP
        * **Internet Protocol**

    LAN
        **Local Area Network**


    NAT
        **Network Address Translation**:
        Typically an internet router multiplexes all the traffic through
        a public IP to several private IPs.
        Usually, the machines under the router network cannot be accessed from the outside unless a Port is forwarded
        in the router configuration, or if such host has previously started a TCP communication with the message source.

    P2P
        **Peer to Peer**

    TCP
        **Transmission Control Protocol**

    UDP
        **User Datagram Protocol**

    URL
        **Uniform Resource Locator**

    WAN
        **Wide Area Network**
