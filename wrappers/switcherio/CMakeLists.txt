cmake_minimum_required(VERSION 3.16)

# Search for Python3
find_package(Python3 3.8 COMPONENTS Interpreter Development REQUIRED)

# cmake option for the enabling of Switcherio 
option(WITH_SWITCHERIO "Switcherio Wrapper" ${WITH_PYTHON}) # default is ON if Python3_FOUND
add_feature_info("wrapper-switcherio" WITH_SWITCHERIO "Switcher-io Wrapper")

if (WITH_SWITCHERIO)
  if(Python3_FOUND)
    add_test(NAME pyquid_switcherio COMMAND ${Python3_EXECUTABLE} -m unittest WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif() # python3 found
endif() # WITH_SWITCHERIO

