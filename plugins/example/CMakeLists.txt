# PLUGIN

set(PLUGIN_NAME "plugin-example")
set(PLUGIN_DESCRIPTION "Example Plugin")

option(PLUGIN_EXAMPLE "${PLUGIN_DESCRIPTION}" ON)
add_feature_info("${PLUGIN_NAME}" PLUGIN_EXAMPLE "${PLUGIN_DESCRIPTION}")

if (PLUGIN_EXAMPLE)

    # Property quid

    add_library(prop_quid SHARED
            property-quid.cpp
        )
    add_dependencies(prop_quid ${SWITCHER_LIBRARY})

    # Method quid

    add_library(meth_quid SHARED
            method-quid.cpp
        )
    add_dependencies(meth_quid ${SWITCHER_LIBRARY})

    # Signal quid

    add_library(signal_quid SHARED
            signal-quid.cpp
            )
    add_dependencies(signal_quid ${SWITCHER_LIBRARY})

    # Custom Save

    add_library(custom_save SHARED
        custom-save-plugin.cpp
        )

    # Shmdata Quid

    add_library(shmdata_quid SHARED
        shmdata-quid.cpp
        )

    # Dynamic Writer Quid

    add_library(shmdata_dynamic_writer SHARED
        shmdata-dynamic-writer.cpp
        )

    # Dynamic Reader Quid

    add_library(shmdata_dynamic_reader SHARED
        shmdata-dynamic-reader.cpp
        )

    # GST quid
    
    add_library(gst_quid SHARED
        gst-quid.cpp
        )

    # TEST

    add_executable(check_property_plugin check_property_plugin.cpp)
    add_test(check_property_plugin check_property_plugin)

    add_executable(check_method_plugin check_method_plugin.cpp)
    add_test(check_method_plugin check_method_plugin)

    add_executable(check_custom_save check_custom_save_plugin.cpp)
    add_test(check_custom_save check_custom_save)

    add_executable(check_signals_plugin check_signals_plugin.cpp)
    add_test(check_signals_plugin check_signals_plugin)

    add_executable(check_gst_plugin check_gst_plugin.cpp)
    add_test(check_gst_plugin check_gst_plugin)

    add_executable(check_shmdata_quid check_shmdata_quid.cpp)
    add_test(check_shmdata_quid check_shmdata_quid)

    add_executable(check_dynamic_writer_quid check_dynamic_writer_quid.cpp)
    add_test(check_dynamic_writer_quid check_dynamic_writer_quid)

    add_executable(check_dynamic_reader_quid check_dynamic_reader_quid.cpp)
    add_test(check_dynamic_reader_quid check_dynamic_reader_quid)

    # INSTALL

    set(EXAMPLE_QUIDS
      custom_save
      gst_quid
      meth_quid
      prop_quid
      shmdata_dynamic_reader
      shmdata_dynamic_writer
      shmdata_quid
      signal_quid
      )    

    install(TARGETS
      ${EXAMPLE_QUIDS}
      LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins
      COMPONENT lib
      )

    install(TARGETS
      ${EXAMPLE_QUIDS}
      LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins
      COMPONENT dev
      )

    install(TARGETS
      ${EXAMPLE_QUIDS}
      LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins
      COMPONENT dbg
      )

endif ()
