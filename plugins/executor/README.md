# Executor plugin

The Executor quiddity can be used for launching custom programs.

## Usage

Using the `command_line` argument, the user can specify an executable program (and its arguments) that will launch when the quiddity is started. The command line can be written the same way it would be written in a standard shell, as long as the specified program is in the system's PATH (if it isn't, an absolute path to the executable can also be specified.) The quiddity will also automatically perform word expansion (using wordexp). It is the user's responsibility to correctly quote arguments containing whitespaces. `command_line` is mandatory.

The `periodic` property can be used to make the process periodic: when set to `true`, the process will be launched again as soon as the first child process finishes, until the user manually stops the quiddity.

If desired, the user can also set the `autostart` property to `true`; this will cause the specified command line to automatically execute when a shmdata is connected to the Executor quiddity. Likewise, the program will automatically stop on disconnection.

Once the executed process finishes (either normally or because of an error), the quiddity will automatically become 'stopped'. (ie. the `started` property will become `false`). To execute the specified program again, simply put the quiddity back on. The only exception is when the `periodic` property is set to `true`: the program will start anew automatically and the quiddity's `started` property will stay `true`.

The `whitelist_caps` property needs to be set explicitly in order to allow connections with the Executor as a writer quiddity. It should enumerate all compatible caps in order to connect the Executor and shmdatas. By default, the `can_sink_caps` method will **accept** all connections if the `whitelist_caps` property is empty.

## Special strings

A maximum of three shmdatas can be connected to Executor at any time: 1 audio shmdata, 1 video shmdata and 1 'other' shmdata (MIDI, OSC, etc.).

The user can dynamically use the connected shmpaths values in the `command_line` parameter using special strings. When starting the quiddity (either manually or automatically via the `autostart` parameter), the quiddity will automatically replace every occurrence of those special strings in the `command_line` parameter with the actual shmpaths connected to Executor.

Special strings are:

* `_shmpath_audio_` (replaced by the connected audio shmdata on execution, if there is one.)
* `_shmpath_video_` (replaced by the connected raw video shmdata on execution, if there is one.)
* `_shmpath_` (replaced by the connected data shmdata (OSC, MIDI, and other) on execution, if there is one.)

### Example

If the `/tmp/switcher_default_1_video` and `/tmp/switcher_default_8_audio` shmdatas are connected to Executor, then the following `command_line` parameter:

```/usr/local/bin/shmdata2ndi -v _shmpath_video_ -a _shmpath_audio_ -n ndi_test```

will become the following once the quiddity is started:

```/usr/local/bin/shmdata2ndi -v /tmp/switcher_default_1_video -a /tmp/switcher_default_8_audio -n ndi_test```

## Process stdout/stderr

The child process's `stdout` and `stderr` will be captured only **when the process is terminated**. The entirety of the process's `stdout` and `stderr` (ie. every line that was written to those outputs during the child process's lifespan) will be parsed and grafted in the Executor's InfoTree, under `output.stdout` and `output.stderr`. The process's return code will also be grafted under `output.return_code`.

If the process is run again (either by starting the quiddity manually or automatically through the `periodic` property), the existing `output.stdout`, `output.stderr` and `output.return_code` values will be overwritten by new values when the process terminates. **This means that these entries represent only the results of the last execution**.

### Special characters escaping

In order to respect JSON's formatting rules, special characters present in `stdout` and `stderr` (such as `\n` and `\r`) will be escaped with a backslash before being grafted in the infotree. (for example, `\n` will become `\\n`)
