
Configuration
=======

Switcher has the ability to load a configuration from a file in [JSON](https://www.json.org/json-en.html) format. 

Following the [XDG Base Directory Specification](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html), a global configuration file  `global.json` for all instances of [Switcher](https://gitlab.com/nicobou/switcher) might be placed under the `$HOME/.config/switcher` directory.

Configuration keys permit to change the default behavior of [Switcher](https://gitlab.com/nicobou/switcher) built-in `Quiddities` and define [Custom Bundles](writing-bundles.md).

At runtime, the custom configuration is passed to their respective quiddities upon instantiation.

> Note that this is *not* a way to set defaults for the *properties* of each quiddity, but rather for specific *keys* used by each quiddities in their own way.
> At the moment, only [Custom Bundles](writing-bundles.md), `jackserver`, `jacksink`, `jacksrc`, `glfwin` and `pjsip` have predefined configuration *keys*.

A sample configuration [dummy-switcher.json](doc/dummy-switcher.json) exists to demonstrate all the possible *keys* used by [Switcher](https://gitlab.com/nicobou/switcher) quiddities, as well as some bundle examples.

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

<details open>
  <summary><h2>logs</h2></summary>
  <details>
    <summary><h3>filepath</h3></summary>

Specifies the filepath to where logs should be written, by default this points to `~/.local/state/switcher/logs/switcher.log` in the [$XDG_STATE_HOME](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html).

> The [$XDG_STATE_HOME](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html) contains state data that should persist between (application) restarts, but that is not important or portable enough to the user that it should be stored in [$XDG_DATA_HOME](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html). It may contain:
> - actions history (logs, history, recently used files, …)
> - current state of the application that can be reused on a restart (view, layout, open files, undo history, …)

The following format is used to write logs to this filepath: `DATE|INSTANCE_NAME|PID|THREAD_ID|LOG_LEVEL: MESSAGE|SOURCE_BASENAME:SOURCE_LINE_NUMBER`

> Note that if [$XDG_STATE_HOME](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html) environment variable is not defined, the `logs.filepath` falls back to `~/.local/state/switcher/logs/switcher.log`
> In case [Switcher](https://gitlab.com/nicobou/switcher) is running as `root`, the `logs.filepath` falls back to `/var/log/switcher/switcher.log`

  </details>
  <details>
    <summary><h3>max_files</h3></summary>

Specifies the maximum number of files that should be rotated when logging.

This should be an integer greater than `0` otherwise `3` files are rotated by default.

  </details>

  <details>
    <summary><h3>max_size</h3></summary>

Specifies the maximum size (MB) of a file before it is rotated when logging.

This should be an integer greater than `0` otherwise `100` is used by default.

  </details>

  <details>
    <summary><h3>log_level</h3></summary>

Specifies the log level to use.

This can be any of: `trace`, `debug`, `info`, `warn`, `err`, `critical`, `off`.

> Note that if an unrecognized *log level* is used, logging will be disabled as if it was `off`.
> Also the `debug` option of [Switcher](https://gitlab.com/nicobou/switcher) takes priority over this setting and would enforce a `debug` log level (except if the log level is lower than `debug` such as `trace`).

  </details>
</details>
