C API
=====

The C API is a port of the C++. It is accessible through inclusion of the following header files:
* Use a Shmdata writer: shmdata/cwriter.h
* Use a Shmdata follower: \ref shmdata/cfollower.h
* Integrate Shmdata logging into your logging system: \ref shmdata/cfollower.h

The \ref tests/check-c-wrapper.cpp provides an example through the writing and following of a single shmdata from a same process.