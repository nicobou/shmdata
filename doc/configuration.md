
Configuration
=======

Switcher has the ability to load a configuration from a file in [JSON](https://www.json.org/json-en.html) format. 

Following the [XDG Base Directory Specification](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html), a global configuration file  `global.json` for all instances of [Switcher](https://gitlab.com/sat-metalab/switcher) might be placed under the `$HOME/.config/switcher` directory.

Configuration keys permit to change the default behavior of [Switcher](https://gitlab.com/sat-metalab/switcher) built-in `Quiddities` and define [Custom Bundles](writing-bundles.md).

At runtime, the custom configuration is passed to their respective quids upon instantiation.

> Note that this is *not* a way to set defaults for the *properties* of each quiddity, but rather for specific *keys* used by each quiddities in their own way.
> At the moment, only [Custom Bundles](writing-bundles.md), `jackserver`, `jacksink`, `jacksrc`, `glfwin` and `pjsip` have predefined configuration *keys*.

A sample configuration [dummy-switcher.json](doc/dummy-switcher.json) exists to demonstrate all the possible *keys* used by [Switcher](https://gitlab.com/sat-metalab/switcher) quiddities, as well as some bundle examples.

---

# Available configuration keys

<details open>
  <summary><h2>shm</h2></summary>
  <details>
    <summary><h3>directory</h3></summary>

Specifies the folder in which to save shmdata sockets, by default this points to the [$XDG_RUNTIME_DIR](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html).

> [$XDG_RUNTIME_DIR](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html) defines the base directory relative to which user-specific non-essential runtime files and other file objects (such as sockets, named pipes, ...) should be stored. The directory MUST be owned by the user, and he MUST be the only one having read and write access to it. Its Unix access mode MUST be 0700.

Note that if switcher is running as `root`, the `shm.directory` falls back to the `/tmp` folder.

  </details>
  <details>
    <summary><h3>prefix</h3></summary>

Specifies the prefix to use for shmdata socket file names when saving them, by default `switcher_` is used.

  </details>
</details>

