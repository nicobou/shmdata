Repository directory structure
=======
```bash
.
├── cmake        # additional cmake files
├── doc          # contains markdown files describing switcher
├── dockerfiles  # files for running switcher in docker
├── plugins      # implementation of quiddities as plugin
├── scripts      # scripts automation of switcher and shmdata release
├── src          # implementation of 'switcher' and 'switcher-ctrl' commands
├── switcher     # switcher's code
├── tests        # switcher automated tests. Other tests can be found in python wrapper and the plugin directory 
├── tools        # command line tools installed by switcher
└── wrappers     # switcher wrapper to python3
```

Switcher's directory structure
=======

```bash
switcher/
├── configuration  # switcher's configuration file
├── gst            # switcher internal handling of GStreamer
├── logger         # generic `spdlog` macros`
├── infotree       # information tree 
├── quiddities     # several internal quiddities that are embedded into the switcher library
├── quiddity       # switcher's quiddity
│   ├── bundle     # create new quiddities from a description of a shmdata pipeline composed of several other quiddities
│   ├── method     # quiddity's methods
│   ├── property   # quiddity's properties
│   └── signal     # quiddity's signals
├── session        # switcher's session manager
├── shmdata        # switcher internal handling of shmdata
│   └── caps       # switcher internal handling of shmdata caps
└── utils          # switcher utils, including file, net and several cpp idioms
```

Writing a Quiddity (as a plugin)   
=======

The preferred way of adding features to switcher is to write a dedicated quiddity. Many plugins are available in the Switcher repository, but you can write a quiddity in your own repository using with the installed switcher pkg-config `switcher.pc`. Some example quiddities are available in the Switcher repository. Some examples illustrate the following feature:
* Properties: can be of several types (integer, string, selection, colour, ...). They provide setter and getter and define the state of a quiddity during save and load of switcher session. A user can register to a property update. See [hpp](../plugins/example/property-quid.hpp) and [cpp](../plugins/example/property-quid.cpp).
* InfoTree: a data  structure associated with instances of quiddities that facilitate (de)serialization of data in a tree. It is used extensively during interaction with [Scenic](https://gitlab.com/sat-mtl/tools/scenic/scenic). Many quiddities expose shmdata information through their InfoTree. See [property-quid](../plugins/example/property-quid.cpp) and [signal-quid](../plugins/example/signal-quid.cpp)
* Signals: allow a quiddity to notify a switcher user of a particular event, like tree updates. See [hpp](../plugins/example/signal-quid.hpp) and [cpp](../plugins/example/signal-quid.cpp).
* Methods: allow a user to invoke a method from a quiddity. See [hpp](../plugins/example/method-quid.hpp) and [cpp](../plugins/example/method-quid.cpp).
* Custom save: Switcher provides saving and loading of all instantiated quiddities. The state of a quiddity is defined by the values of the properties. If property saving is not enough in order to define the state of your quiddity, you can handle the saving process and add your data in the switcher save file. See [hpp](../plugins/example/custom-save-plugin.hpp) and [cpp](../plugins/example/custom-save-plugin.cpp).
