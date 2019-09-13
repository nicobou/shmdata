# PLUGIN

set(PLUGIN_NAME "plugin-executor")
set(PLUGIN_DESCRIPTION "Executor Plugin")

option(PLUGIN_EXECUTOR "${PLUGIN_DESCRIPTION}" TRUE)
add_feature_info("${PLUGIN_NAME}" PLUGIN_EXECUTOR "${PLUGIN_DESCRIPTION}")

if (PLUGIN_EXECUTOR)

    add_definitions(-Wno-error=missing-field-initializers)
    
    add_library(executor SHARED
        executor.cpp
        )

    add_dependencies(executor ${SWITCHER_LIBRARY})

    # TEST

    add_executable(check_executor check_executor.cpp)
    add_test(check_executor check_executor)

    add_executable(check_executor_caps check_executor_caps.cpp)
    add_test(check_executor_caps check_executor_caps)

    # INSTALL

    install(TARGETS executor LIBRARY DESTINATION ${SWITCHER_LIBRARY}/plugins)

endif ()
