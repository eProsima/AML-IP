.. include:: ../exports/alias.include

.. _release_notes:

.. comment the include of forthcoming when new info is added

.. .. include:: forthcoming_version.rst

##############
Version v0.2.0
##############

This release adds new **features**:

* Implement asynchronous request model in ``ModelManagerReceiver``.
* Add ``fastdds.application.id`` property to participants and endpoints.
* Add ``fastdds.application.metadata`` property to participants and endpoints.
* Add ``MainNode`` costructor with ``domain`` parameter in Python bindings.
* Add Python bindings for Agent Nodes.
* Rename ``agent_tool`` package to ``amlip_agent``.

This includes the following **Bugfixes**:

* Update ``AsyncComputingNode`` and ``AsyncInferenceNode`` to only stop if the current state is running and only run if the current state is stopped.
* ASAN (Address Sanitizer) fixes.
* Fix allowlist namespacing in ``AgentNode``.
* Call change status in ``AsyncComputingNode``.

This release includes the following **CI improvements**:

* Migrate CI actions to use eProsima-CI.
* Include branch environment variables in CI configuration.

This release add new **Documentation features**:

* Add instructions to build the Docker image.
* Add ``Agent Tool`` section.
* Add ``Enabling Technologies`` and ``Internal Protocols`` sections.

This release includes the following *Dependencies Update*:

.. list-table::
    :header-rows: 1

    *   -
        - Repository
        - Old Version
        - New Version
    *   - Foonathan Memory Vendor
        - `eProsima/foonathan_memory_vendor <https://github.com/eProsima/foonathan_memory_vendor>`_
        - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`_
        - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`_
    *   - Fast CDR
        - `eProsima/Fast-CDR <https://github.com/eProsima/Fast-CDR>`_
        - `v2.1.2 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.1.2>`_
        - `v2.2.1 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.2.1>`_
    *   - Fast DDS
        - `eProsima/Fast-DDS <https://github.com/eProsima/Fast-DDS>`_
        - `v2.13.1 <https://github.com/eProsima/Fast-DDS/releases/tag/v2.13.1>`_
        - `v2.14.0 <https://github.com/eProsima/Fast-DDS/releases/tag/v2.14.0>`_
    *   - Dev Utils
        - `eProsima/dev-utils <https://github.com/eProsima/dev-utils>`_
        - `v0.5.0 <https://github.com/eProsima/dev-utils/releases/tag/v0.5.0>`_
        - `v0.6.0 <https://github.com/eProsima/dev-utils/releases/tag/v0.6.0>`_
    *   - DDS Pipe
        - `eProsima/DDS-Pipe <https://github.com/eProsima/DDS-Pipe.git>`_
        - `v0.3.0 <https://github.com/eProsima/DDS-Pipe/releases/tag/v0.3.0>`__
        - `v0.4.0 <https://github.com/eProsima/DDS-Pipe/releases/tag/v0.4.0>`__
    *   - DDS Router
        - `eProsima/DDS-Router <https://github.com/eProsima/DDS-Router.git>`_
        - `v2.1.0 <https://github.com/eProsima/DDS-Router/releases/tag/v2.1.0>`__
        - `v2.2.0 <https://github.com/eProsima/DDS-Router/releases/tag/v2.2.0>`__


#################
Previous Versions
#################

.. include:: previous_versions/v0.1.0.rst
