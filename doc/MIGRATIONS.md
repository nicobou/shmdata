This file lists changes to apply when migrating from a switcher version to a new one.

From switcher version 3.1.14 to version 3.2.0
--------------------------------------------

The `pyquid.Session` API have changed slightly. 

  - Existing read method have been renamed load. New load method loads a session file in Switcher state just like read used to do.
  - Read method now is now returning session file content instead of loading its state.
  - New write method allows to write a session file content to disk without loading its state.

From switcher version 3.1.12 to version 3.2.0
--------------------------------------------

The `pyquid.Quiddity` constructor has changed! The use of:

```py
quid = Quiddity(swithcher=sw,
                kind='dummy',
                nickname='myquid')
assert isinstance(quid, Quiddity) is True
```

has been **removed**, we recommend to use the `pyquid.Switcher` API instead:

```py
quid = sw.create(kind='dummy',
                 nickname='myquid')
assert isinstance(quid, Quiddity) is True
```

From switcher version 3.0.4 to version 3.1.0
--------------------------------------------

The logger has been refactored, and is now based on spdlog. This affects switcher core and how Quiddities report logs:

* `warning` is now `LOGGER_WARN`
* `error` is now `LOGGER_ERROR`
* `debug` is now `LOGGER_DEBUG`
* `info` is now `LOGGER_INFO`
* `message` is now `LOGGER_INFO`
* The log messages are now using the libfmt format: https://fmt.dev/latest/index.html
* all the `LOGGER_` are global functions that takes the logger handler as argument

For instance, the following line:
```
error("error while setting up interprocess communication. Error: %", strerror(errno));
```
Is now refactored as:
```
LOGGER_ERROR(logger_,
             "error while setting up interprocess communication. Error: {}",
             strerror(errno));
```


From switcher version 2.3.0 to version 3.0.0
--------------------------------------------

In C++ and Python API: addition of a name for Shmdata attached to Quiddity and refactoring the Shmdata connection methods:

* Connect and disconnect methods has been removed from the Quiddity and must be replaced by the Quiddity Claw system. Doxygen documentation for the Claw system has been added.
* All the tests have been ported to the new system
* Two Python examples are provided in `wrappers/python/examples/`:  `shmdata-connection.py` and `shmdata-dynamic-claws`
* One C++ example is provided: `plugins/example/check_shmdata_quid.cpp`
* An example Quiddity illustrates the use of the Claw system when writing a Quiddity: `plugins/example/shmdata-quid.cpp`

Other changes in C++ and Python API:

* pyQuiddity sends Infotree object in signal callbacks instead of its JSON serialization. This affects `pyinfotree.get` and `pyQuiddity::subscribe_to_signal`
* Category and tags are removed from the Quiddity description since the new Claw system allows for more precise description of Quiddity Shmdata connection capabilities

Quiddity related keyword `class` and `type` have been harmonized and refactored as `kind`:

* `switcher-ctrl` and `switcher` command lines have renamed arguments: `--list-classes` renamed into `--list-kinds`, `--classes-doc` into `--kinds-doc`, `--class-doc` into `--kind-doc`
* `swquid-info` command changed the argument `-c`, `--class-doc` into `-k`, `--kind-doc`
* Quiddity factory documentation has its key `classes` replaced with `kinds` and `class` with `kind`
* Quiddity infotree now describes its kind with the "kind" key instead of "class"
* C++ API has reworded method names, with `kind` replacing `class` and `kinds` replacing `classes`
* Python API changed some methods arguments called `type` into `kind`: `pyquiddity` constructor and `pyswitch::create`
* PyQuiddity has `get_type` renamed into `get_kind`
* PySwitch has some methods renaming: `list_classes` into `list_kinds`, `classes_doc` into `kinds_doc` and `class_doc` into `kind_doc`

From switcher version 2.2.6 to version 2.2.8
---------------------------------------------

* In the python API, `name` argument has been renamed in `nickname` for Switcher `create` method and for the Quiddity constructor

From switcher version 2.1.38 to version 2.2.0
---------------------------------------------

Quiddity names have been replaced by id. Quiddity naming is still possible to the use of Quiddity nicknames.

Some methods from the C++ API has been refactored:

* In Quiddity, `quiddity::get_name` is replaced by either the `quiddity::get_nickname` method or the newly introduced `quiddity::get_id`.
* In Quiddity container, `get_name_from_caps` is renamed into `get_nickname_from_caps`
* In Quiddity container, signatures of `create`, `remove` and `get_quiddity` methods now take as argument the Quiddity id instead of the Quiddity name

In the python API:

* Quiddity `get_name` has been removed. `get_nickname` is still available
* Quiddity `get_names` has been renamed into `get_nicknames`
* Qrox has been removed from the python API, now a Switcher directly creates a Quiddity object
