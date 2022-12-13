.. include:: ../../exports/alias.include

.. |status| replace:: *Status*

.. _user_manual_nodes_main:

#########
Main Node
#########

This kind of Node perform the active (client) action of :ref:`user_manual_scenarios_workload_distribution`.
This node is able to send different *Jobs* serialized as :ref:`user_manual_scenarios_workload_distribution_job` and it
receives a Solution once the task has been executed as :ref:`user_manual_scenarios_workload_distribution_solution`.

.. warning::

    In the current release, the use of a Main node must be synchronous.
    This means that once a job is sent, the thread must wait for the solution to arrive before sending another task.
    In future release asynchronous methods will be available.


Example of Usage
================

This node kind does require **active** interaction with the user to perform its action.
User can use method :code:`request_job_solution` to send a new *Job*.
The thread calling this method will wait until the whole process has finished and the *Solution* has arrived from
the *Computing Node* in charge of this *Job*.
By destroying the node every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Main Node creating an object of such class with a name.
* Create a new :code:`JobDataType` from an array of bytes.
* Send a new *Job* synchronously and wait for the solution by calling :code:`request_job_solution`.

.. tabs::

    .. tab:: C++

        .. code-block:: cpp

            // Create a new Main Node
            auto node = eprosima::amlip::MainNode("My_Main_Node");

            // Create a new job to be executed remotely
            auto new_job = eprosima::amlip::JobDataType("Some Job as byte array serialized from a string");

            // Send a Job to a remote Computing and waits for the answer
            // This could be called with an id as well, and it will return the server id that send the solution
            auto solution = node.request_job_solution(new_job);

    .. tab:: Python

        .. code-block:: python

            # Create a new Main Node
            node = MainNode("My_Main_Node")

            # Create a new job to be executed remotely
            new_job = JobDataType("Some Job as byte array serialized from a string")

            # Send a Job to a remote Computing and waits for the answer
            solution, server_id = node.request_job_solution(new_job)
