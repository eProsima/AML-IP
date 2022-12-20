.. include:: ../../exports/alias.include

.. |status| replace:: *Status*

.. _user_manual_nodes_computing:

##############
Computing Node
##############

This kind of Node performs the passive (server) action of :ref:`user_manual_scenarios_workload_distribution`.
This node waits for a *Job* serialized as :ref:`user_manual_scenarios_workload_distribution_job`, and once received it performs a calculation (implemented by the user) whose output is the solution as :ref:`user_manual_scenarios_workload_distribution_solution`.

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

* Instantiate the Computing Node creating an object of such class with a name.
* Create a new :code:`JobDataType` from an array of bytes.
* Send a new *Job* synchronously and wait for the solution by calling :code:`request_job_solution`.

.. tabs::

    .. tab:: C++

        .. code-block:: cpp

            // Create a new Computing Node
            auto node = eprosima::amlip::ComputingNode("My_Computing_Node");

            // Create a callback to process a job and return a solution
            auto process_solution = []( const eprosima::amlip::types::JobDataType& ){
                eprosima::amlip::types::JobSolutionDataType solution;
                // Do some code that calculates the solution
                return solution;
            };

            // Wait for 1 task from any client and answer it with process_solution callback
            node.process_job(process_solution);

    .. tab:: Python

        .. code-block:: python

            # Create a new Computing Node
            node = ComputingNode("My_Computing_Node")

            def process_solution():
                JobSolutionDataType solution;
                # Do some code that calculates the solution
                return solution

            # Wait for 1 task from any client and answer it with process_solution callback
            node.process_job(callback=process_solution)
