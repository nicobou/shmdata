C++ API
=======

The C++ API is the Shmdata native API. It is moslty composed of a two main classes: the \ref shmdata::Writer and the \ref shmdata::Follower.

Shmdata types are described with a string containing a key-value list, following the GStreamer
syntax for caps. Shmdata comes with a shmdata::Type class that helps parsing and generation such
type description.

Here follows examples taken from the Shmdata test suite:
* \ref tests/check-writer-follower.cpp : a shmdata::Writer and a shmdata::Follower are instanciated and communicates from the same process 
* \ref tests/check-shmdata.cpp : two writing methods are illustrated (copy of buffer and direct access to the shmdata memory)
* \ref tests/check-type-parser.cpp : use of shmdata::Type
