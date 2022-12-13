.. include:: ../../../exports/alias.include
.. include:: ../../../exports/roles.include

.. _cmake_options:

#############
CMake options
#############

|eamlip| provides numerous CMake options for changing the behavior and configuration of
|amlip|.
These options allow the developer to enable/disable certain |amlip| settings by defining these options to
``ON``/``OFF`` at the CMake execution, or set the required path to certain dependencies.

.. warning::
    These options are only for advance developers who installed |eamlip| from sources.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Possible values
        - Default

    *   - :class:`CMAKE_BUILD_TYPE`
        - CMake optimization build type.
        - ``Release`` |br|
          ``Debug``
        - ``Release``

    *   - :class:`BUILD_ALL`
        - Build all |amlip| sub-packages. |br|
          Setting to ``ON`` sets to ``ON`` |br|
          :class:`BUILD_TOOL`, :class:`BUILD_LIBRARY`, and |br|
          :class:`BUILD_DOCS`.
        - ``OFF`` |br|
          ``ON``
        - ``OFF``

    *   - :class:`BUILD_LIBRARY`
        - Build the |amlip| libraries sub-packages. |br|
          It is set to ``ON`` if :class:`BUILD_ALL` is set to ``ON``.
        - ``OFF`` |br|
          ``ON``
        - ``ON``

    *   - :class:`BUILD_TOOL`
        - Build the |amlip| tools sub-packages. |br|
          It is set to ``ON`` if :class:`BUILD_ALL` is set to ``ON``.
        - ``OFF`` |br|
          ``ON``
        - ``ON``

    *   - :class:`BUILD_DOCS`
        - Build the |amlip| documentation sub-packages. |br|
          It is set to ``ON`` if :class:`BUILD_ALL` is set to ``ON``.
        - ``OFF`` |br|
          ``ON``
        - ``OFF``

    *   - :class:`BUILD_TESTS`
        - Build the |amlip| application and documentation |br|
          tests. Setting :class:`BUILD_TESTS` to ``ON`` sets |br|
          :class:`BUILD_ALL`, :class:`BUILD_LIBRARY_TESTS`, |br|
          :class:`BUILD_TOOL_TESTS`, and :class:`BUILD_DOCS_TESTS` |br|
          to ``ON``.
        - ``OFF`` |br|
          ``ON``
        - ``OFF``

    *   - :class:`BUILD_LIBRARY_TESTS`
        - Build the |amlip| library tests. It is |br|
          set to ``ON`` if :class:`BUILD_TESTS` is set to ``ON``. |br|
        - ``OFF`` |br|
          ``ON``
        - ``OFF``

    *   - :class:`BUILD_TOOL_TESTS`
        - Build the |amlip| application tests. It is |br|
          set to ``ON`` if :class:`BUILD_TESTS` is set to ``ON``. |br|
        - ``OFF`` |br|
          ``ON``
        - ``OFF``

    *   - :class:`BUILD_DOCS_TESTS`
        - Build the |amlip| documentation tests. It is |br|
          set to ``ON`` if :class:`BUILD_TESTS` is set to ``ON``. |br|
        - ``OFF`` |br|
          ``ON``
        - ``OFF``

    *   - :class:`BUILD_MANUAL_TESTS`
        - Build the |amlip| manual tests.
        - ``OFF`` |br|
          ``ON``
        - ``OFF``

    *   - :class:`LOG_INFO`
        - Activate |amlip| execution logs. It is |br|
          set to ``ON`` if :class:`CMAKE_BUILD_TYPE` is set |br|
          to ``Debug``.
        - ``OFF`` |br|
          ``ON``
        - ``ON`` if ``Debug`` |br|
          ``OFF`` otherwise

    *   - :class:`ASAN_BUILD`
        - Activate address sanitizer build.
        - ``OFF`` |br|
          ``ON``
        - ``OFF``

    *   - :class:`TSAN_BUILD`
        - Activate thread sanitizer build.
        - ``OFF`` |br|
          ``ON``
        - ``OFF``
