.. include:: ../../exports/alias.include

##################
Internal Protocols
##################

This page briefly describes the protocols developed on top of DDS that enable AML-IP node communications as well as the deployment of the different :ref:`scenarios <user_manual_scenarios>` presented in the user manual of this documentation.

DDS Topics
----------

All the |amlip| Topics in the DDS |amlip| network have a previous name mangling.
So an |amlip| topic named , :code:`some_topic` would actually be named :code:`amlip::some_topic` in the underneath DDS network.

.. _protocols_dds_multiservice:

MultiService over DDS
----------------------

A new communication protocol based on :term:`DDS` has been designed in order to fulfill the necessity of distributing a task in a network.
The idea is a Service protocol based on Client-Server communication where multiple servers could be available at the same time in the same network.
This protocol creates an auto-regulated orchestration method where a task could be distributed to **one and only one** server that is publicly available, and each server receives no more than one task at a time.

.. _protocols_dds_rpc:

RPC over DDS
------------

The Remote Procedure Call based on :term:`DDS` has been implemented to meet the need to distribute requests and replies across a network.

Unlike how it is implemented in ROS 2 (with Fast DDS as middleware), topic mangling is used for the communication between the servers and clients.

The following diagram illustrates the flow of the implementation:

.. figure:: /rst/figures/rpc_diagram.png
    :align: center
    :width: 100%

.. _protocols_dds_properties:

DDS Entities properties
-----------------------

Every |amlip| entity within the |amlip| network is associated with predefined properties that encompass the entity's identification and metadata.

* The :code:`fastdds.application.metadata` property is a :code:`JSON` object that provides detailed information about the entity:

    * Internal: Specifies the name of the node.
    * Entity: expound the DDS entity.
    * Topic: define the topic name.

* The :code:`fastdds.application.id` property uniquely identifies the DDS application to which the entity belongs, in this case :code:`AML_IP`.

For a practical illustration, consider a `Writer` in a `TestNode` publishing on the `/test` topic.
The corresponding C++ code snippet for configuring the DataWriter QoS properties is as follows:

.. code-block:: cpp

    nlohmann::json property_value;

    property_value["Internal"] = "TestNode";
    property_value["Entity"] = "Writer";
    property_value["Topic"] = "/test";

    eprosima::fastdds::dds::DataWriterQos qos_request_availability_writer_ = default_request_availability_writer_qos_();

    qos_request_availability_writer_.properties().properties().emplace_back("fastdds.application.metadata",
            property_value.dump(), true);

    qos_request_availability_writer_.properties().properties().emplace_back("fastdds.application.id",
            "AML_IP", true);

To retrieve the QoS, the following code can be used:

.. code-block:: cpp

    const std::string* application_id =
                eprosima::fastrtps::rtps::PropertyPolicyHelper::find_property(
            datareader_locked->get_qos().properties(), "fastdds.application.id");
