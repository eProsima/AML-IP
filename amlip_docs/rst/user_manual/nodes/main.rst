.. include:: ../../exports/alias.include

.. |status| replace:: *Status*

.. _user_manual_nodes_main:

#########
Main Node
#########

This kind of Node performs the active (client) action of :ref:`user_manual_scenarios_workload_distribution`.
This node is able to send different *Jobs* serialized as :ref:`user_manual_scenarios_workload_distribution_job` and it
receives a Solution once the task has been executed as :ref:`user_manual_scenarios_workload_distribution_solution`.

***********
Synchronous
***********

This node kind does require **active** interaction with the user to perform its action.
This means that once a job is sent, the thread must wait for the solution to arrive before sending another task.
Users can use method :code:`request_job_solution` to send a new *Job*.
The thread calling this method will wait until the whole process has finished and the *Solution* has arrived from the *Computing Node* in charge of this *Job*.
By destroying the node every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Main Node creating an object of this class with a name.
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

************
Asynchronous
************

Users can use method :code:`request_job_solution` to send a new *Job*.
Due to being asynchronous, multiple requests can be sent without waiting for the previous one to finish. The solution will be sent back to the user through the listener.
By destroying the node every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Asynchronous Main Node creating an object of this class with a name, a listener or callback and a domain.
* Create a new :code:`JobDataType` from an array of bytes.
* Send a new *Job* asynchronously and wait for the solution by calling :code:`request_job_solution`.
* Wait for the solution.

.. tabs::

    .. tab:: Python

        .. code-block:: python

            def solution_received(
                    solution,
                    task_id,
                    server_id):
                print(f'Solution received from server: {server_id}\n'
                    f' with id: {task_id}\n'
                    f' solution: {solution.to_string()}')

            def main():
                # Create a new Async Main Node
                node = AsyncMainNode(
                    'MyAsyncMainNode',
                    callback=solution_received,
                    domain=100)

                # Create new data to be executed remotely
                data_str = '<Job Data In Py String Async [LAMBDA]>'
                job_data = JobDataType(data_str)

                # Send data to a remote Computing Node and waits for the solution
                task_id = main_node.request_job_solution(job_data)

                # User must wait to receive solution.
                # Out of scope, the node will be destroyed,
                # and thus the solution will not arrive.
