This file lists changes to apply when migrating from a switcher version to a new one.

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


