cmake_minimum_required(VERSION 3.16)

# Search for Python3
find_package(Python3 3.8 COMPONENTS Interpreter Development REQUIRED)

if(Python3_FOUND)
  # install dependencies
  execute_process(COMMAND ${Python3_EXECUTABLE} -m pip install -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt -q)
  # run test
  add_test(NAME pyquid_switcherio COMMAND ${Python3_EXECUTABLE} -m unittest WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

