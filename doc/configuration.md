Configuration
=======

Switcher has the ability to load a configuration from a file in [JSON](https://www.json.org/json-en.html) format. 

Following the [XDG Base Directory Specification](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html), a global configuration file  `global.json` for all instances of [Switcher](https://gitlab.com/sat-metalab/switcher) might be placed under the `$HOME/.config/switcher` directory.

Configuration keys permit to change the default behavior of [Switcher](https://gitlab.com/sat-metalab/switcher) built-in `Quiddities` and define [Custom Bundles](writing-bundles.md).

At runtime, the custom configuration is passed to their respective quids upon instantiation.

> Note that this is *not* a way to set defaults for the *properties* of each quiddity, but rather for specific *keys* used by each quiddities in their own way.
> At the moment, only [Custom Bundles](writing-bundles.md), `jackserver`, `jacksink`, `jacksrc`, `glfwin` and `pjsip` have predefined configuration *keys*.

A sample configuration [dummy-switcher.json](doc/dummy-switcher.json) exists to demonstrate all the possible *keys* used by [Switcher](https://gitlab.com/sat-metalab/switcher) quiddities, as well as some bundle examples.
