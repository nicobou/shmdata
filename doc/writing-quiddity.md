Writing Plugins   
=======

Some example quiddities are available in the Switcher repository. Each example illustrates a particular switcher feature:
* Properties: they can also be instantiated dynamically from inside a quiddity. Properties can be of several types (integer, string, colour, ...) and are used in order to configure and/or get information about the state of a quiddity. A user can register to a property change. See [hpp](../plugins/example/property-quid.hpp) and [cpp](../plugins/example/property-quid.cpp).  
* InfoTree: a data  structure associated with instances of quiddities that facilitate (de)serialization of data in a tree. It is used extensively during interaction with [Scenic](https://gitlab.com/sat-metalab/scenic). Many quiddities expose shmdata information through their InfoTree. See [property-quid](../plugins/example/property-quid.cpp) and [signal-quid](../plugins/example/signal-quid.cpp)
* Signals: a signal allows a quiddity to notify a switcher user of a particular event. See [hpp](../plugins/example/signal-quid.hpp) and [cpp](../plugins/example/signal-quid.cpp).
* Methods: they can be instantiated dynamically by quiddities. While quiddities instantiate specific methods, some implements widely used methods (as _connect_ for Shmdata). See [hpp](../plugins/example/method-quid.hpp) and [cpp](../plugins/example/method-quid.cpp).
* Custom save: Switcher provides saving and loading of all instantiated quiddities. The state of a quiddity is defined by the values of the properties. If property saving is not enough in order to define the state of your quiddity, you can handle the saving process and add your data in the switcher save file. See [hpp](../plugins/example/custom-save-plugin.hpp) and [cpp](../plugins/example/custom-save-plugin.cpp).
