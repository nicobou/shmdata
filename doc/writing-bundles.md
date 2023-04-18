Writing custom quiddity bundles
=======

Bundles offer a great way to save and reuse quiddity settings.

To create a custom bundle, simply add an object identified by a custom type as a key to the [Switcher configuration file](configuration.md) within the `bundles` object. The type of the custom bundle will then be used by [Switcher](https://gitlab.com/nicobou/switcher) upon instantiation of a bundle quiddity.

Each specification of a custom bundle has mandatory configuration keys which are `pipeline` to combine quiddities and `doc` to describe the bundle and define its behavior in [Switcher](https://gitlab.com/nicobou/switcher).

The [dummy-switcher.json](dummy-switcher.json) provides some bundle specification examples.

## 📦 Bundle specifications

Inside a [Switcher configuration file](configuration.md), bundles can be added within an object identified by the `bundles` key. 

A standard bundle specification might looks like this:

```json
"DummyVideo" : {
    "pipeline" : "v4l2src name=HDMI _shmw _device=0 _add_to_start ! videoconvert name=converterHDMI _no_prop ! nvenc name=encoderHDMI _shmw & .HDMI ! glfwin name=windowHDMI",
    "doc" : {
        "long_name" : "Dummy video bundle",
        "category" : "video",
        "tags" : "writer",
        "description" : "A video bundle for testing purposes"
    }
}
```

A custom type (e.g. `DummyVideo`) is required for [Switcher](https://gitlab.com/nicobou/switcher) to create a `Quiddity Bundle`.  

Instances of this bundle will be created using this type (ex. `switcher-ctrl -C DummyVideo dummy`). No whitespaces or underscores allowed.

All fields are mandatory.

* [pipeline](#-pipeline-field): This is where the actual quiddity pipeline will be specified.
* [doc](#-documentation-field): This is where bundle metadata is defined.
  * `long_name`: Full name of the bundle. This name will be displayed when using `switcher-ctrl -K` for example.
  * `category`: Category of the bundle. See [Bundle categories](#categories) below
  * `tags`: Tags will specify the bundle's roles. See [Bundle tags](#tags) below
  * `description`: Short description for the bundle's functionnality. Will be displayed when using `switcher-ctrl -K`.

If the bundle specification is correct, it will be available for creation in Switcher, using `switcher-ctrl -C`.

## 🏷 Pipeline field

The `pipeline` field is the heart of the bundle. This is where the combination of `quiddities` is specified.

To add a quiddity, simply add its `type` (i.e. `audiotestsrc`) followed by the mandatory `name` property.

> Note that the `name` property is used by [Switcher](https://gitlab.com/nicobou/switcher) to name a quiddity instance. This is used, among other things, to manage [shmdata](https://gitlab.com/nicobou/shmdata) socket file names under the `/tmp` directory.

According to the existing properties for a type of quiddity, any of these can be specified using a `property=value` convention and separated by a space. Refer to the infotree of a specific quiddity to get a detailed list of its properties.

For example, the following pipeline will add a `v4l2src` quiddity with its `device` property set to 1 and it's `pixel_format` property set to 4:

```json
"pipeline": "v4l2src name=VideoCapture device=1 pixel_format=4"
```

### Piping

A pipe is a form of redirection used in the `pipeline` of a bundle to connect the `output` of a quiddity to the `input` of another one, simply use `!` to chain them together.

Connecting quiddities permits data to be transferred between them continuously rather than having to pass it through other less direct means. Pipes are `unidirectional`, which means that *data flows from left to right through the pipeline*. 

It is at the user's discretion to ensure that two quiddities are connectable (i.e. that the shmdata socket `caps` are compatible).

Building on the previous example, to add a `videoconvert` quiddity that will take the output of `v4l2src` as input:

```json
"pipeline": "v4l2src name=VideoCapture device=1 pixel_format=4 ! videoconvert name=Converter"
```

The `input` and `output` streams of quiddities can be chained together in this way to create long and complex pipelines.

### Joining

Two or more quiddities can also be created without chaining their `input` and `output` streams, simply use `&` to create both of them.

```json
"pipeline": "executor name=Executor autostart=false periodic=false & watcher name=Watcher create_directory=true"
```

In the example above, `executor` and `watcher` quiddities are created together and exist in the same bundle. However, the output of `executor` will not be piped to the input of `watcher`. 

> Note that some other property values for `executor` and `watcher` quiddities have been set.

### Pipeline tags

Some tags can be added in the pipeline to add some specific behaviours to a quiddity. Each tag applies only to a particular quiddity in the pipeline. 

A `pipeline tag` must be positioned after the `name` property of a quiddity, but before any piping or joining operators (it may be placed, before, after or in between any other property of a quiddity in any order). A quiddity may be marked with multiple tags. 

It is made up from a `tag name` prepended by a `tag modifier` (either `<` or `_`, see the `<noprop` tag and the [Include / Exclude properties](#include-exclude-properties) section)

Five different tags exist that can be found in the **reference table** below:

| Tag name | Description |
| ---      | ---         |
| `<shmr`  | Defines the pipeline's shmdata reader, aka the entry point for shmdata in the pipeline.<br/>A maximum of one reader may be specified in a pipeline. Shmdatas connected to the bundle will be forwarded to the quiddity marked with this tag. |
| `<shmw` | Define the pipeline's shmdata writers, aka quiddities that will produce (write) shmdata in the pipeline.<br/><br/>A quiddity marked with this tag will see its shmdata exposed by the bundle, so that it can be used by other quiddities. If a quiddity creates shmdata but is not marked with this tag, then its shmdata will NOT be accessible outside of the bundle. There is no minimum or maximum number of shmdata writers in a pipeline. |
| `<add_to_start` | Quiddities marked with this tag will be bound to the bundle's `started` property. If at least one `<add_to_start` property is present in the pipeline, a global `started` property will be created for the bundle.<br/><br/>When this property is set to `true`, every quiddity in the pipeline marked with this tag will also have their `started` property set to `true`. Inversely, when the global `started` is set to `false`, every quiddity marked with this tag will be set to `started=false`.<br/><br/>Each quiddity marked with this tag will also have their individual `started` property removed: their `started` state will be entirely dependant on the `started` property state of the bundle. |
| `<top_level` | By default, each quiddity's properties will have a `parent` metadata. By default, the value of this `parent` field will be the `name` of the quiddity to which it belongs (ex: Considering a `v4l2src` named "VideoTest", the parent of `v4l2src`'s `pixel_format` property will be `VideoTest`).<br/><br/>However, when adding the `<top_level` tag for a quiddity, each of its properties will have an empty `parent` field. Each property will be "top level" (ie. they don't have a parent quiddity above them).<br/><br/>This tag is useful for [Scenic](https://gitlab.com/sat-mtl/tools/scenic/scenic), since the `parent` field affects the way a quiddity's properties will be displayed in [Scenic](https://gitlab.com/sat-mtl/tools/scenic/scenic). But aside from that, this tag does nothing else in [Switcher](https://gitlab.com/sat-mtl/tools/switcher). |
| `<no_prop` | This tag will **exclude** all properties of a quiddity. Excluded properties will not show up in [Scenic](https://gitlab.com/sat-mtl/tools/scenic/scenic) and will not be listed when running `switcher-ctrl -t`. In addition, excluded properties cannot be modified and their values cannot be displayed in any way. In short, excluded properties will *disappear*. This tag is closely linked with the underscore (`_`) tag modifier. |

<details>
  <summary>📚 Examples</summary>

Considering the following pipeline:

```json
"pipeline" : "v4l2src name=Capture <shmw device=0 pixel_format=4 <add_to_start ! nvenc name=Encoder preset=0 codec=0 <shmw",
```

This pipeline specifies that:

* The bundle will expose two shmdatas (one from the `v4l2src`, one from the `nvenc` quiddity).
* The v4l2src `started` property will be bound to the bundle's `started` property.

Another example:

```json
"pipeline" : "glfwin name=Window title=Monitor <top_level <shmr width=400 height=225 position_x=0 position_y=0"
```

The pipeline specifies that:

* The glfwin quiddity will read shmdata, and is the entry point for shmdata in the pipeline.
* Glfwin's properties will be top-level, and will have no parent.

In this pipeline:

```json
"pipeline" : "executor name=Exec <no_prop <shmr command_line=somecommand _autostart=false periodic=false"
```

* The `executor` quiddity will read shmdata, and is the entry point for [shmdata](https://gitlab.com/nicobou/shmdata) in the pipeline.
* All properties of `executor` are excluded, except for `autostart`. However, `periodic` will still be set to `false` and `command_line` will be set to `somecommand` before being excluded.

</details>

### Include / Exclude properties

Properties of a quiddity can be **excluded** individually by prefixing the property with an underscore (`_`).

Properties prefixed with an underscore will have the specified value applied (if any), before being excluded.

However, if the quiddity is marked with the `<no_prop` tag, the meaning of the underscore changes: it is used to **include** properties instead of excluding them (all properties of the quiddity would be excluded except those prefixed with an underscore). 

In short:

* `<no_prop` tag is **absent**: the underscore is used to exclude individual properties.
* `<no_prop` tag is **present**: the underscore is used to include individual properties (unmarked properties would be excluded).

See the `<no_prop` tag in the [Pipeline Tags Reference Table](#pipeline-tags) for an explanation on excluding a property.

<details>
  <summary>📚 Example</summary>

```json
"pipeline" : "ladspa name=Delay <shmr <no_prop plugins=Delayorama _first-delay=0.0 _delay-range=0.0001 delay-change=0.2 ! jacksink name=Jack _auto_connect=true do_format_conversion=true _do_rate_conversion"
```

In the above pipeline:

* All of `ladspa`'s properties are excluded, except `first-delay` and `delay-range`.
* Only the `auto_connect` and `do_rate_conversion` properties of `jacksink` are excluded.
* Notice that no value is assigned to `do_rate_conversion`. This means that the default value for `do_rate_conversion` will be applied before exclusion.

</details>

### `Group` property

The `group` property can be added to any quiddity. It is optional; its purpose is to override the `parent` value of each of the quiddity's properties with its value. The value of `group` is arbitrary. The effects are similar to the `<top_level`: it affects the way [Scenic](https://gitlab.com/sat-mtl/telepresence/scenic) displays properties, but it is not of much use in [Switcher](https://gitlab.com/nicobou/switcher).

### Escaping

Double quotes `"` can be escaped using the backslash (`\`). Whitespaces can be escaped by prefixing them with two backslashes. 
However, whitespaces contained in (escaped) quotes are automatically escaped (no backslashes are required).

As an example:

```json
 "pipeline" : "executor name=Test <shmr command_line=shmdata2ndi\\ -v\\ _shmpath_video_\\ -a\\ _shmpath_audio_\\ -n autostart=false"
```

is equivalent to:

```json
 "pipeline" : "executor name=Test <shmr command_line=\"shmdata2ndi -v _shmpath_video_ -a _shmpath_audio_ -n autostart=false\""
```

The value of `command_line` will be `shmdata2ndi -v _shmpath_video_ -a _shmpath_audio_ -n`.

For more information, see the [Executor](../plugins/executor/README.md) quiddity.

## 🏷 Documentation field

The `doc` field helps to describe the bundle and define its behavior in [Switcher](https://gitlab.com/nicobou/switcher).

Both `long_name` and `description` field values are arbitrary.

### Categories

A bundle's category is used to specify the type of shmdata and/or data that the bundle will produce or handle. It can also be used for other means, such as filtering quiddities or giving hints regarding to the quiddity's intended use.

Some categories are already predefined:

* `video`
* `audio`
* `audio/video`
* `file`
* `control`
* `midi`
* `network`
* `monitoring`
* `vrpn`
* `time`
* `utils`
* `other`
* `test`

It is recommended to try and use one of the categories above, but its value can be arbitrary.

### Tags

Tags define the bundle's roles and how it will behave in [Switcher](https://gitlab.com/nicobou/switcher). One or more of the following tags may be specified:

* `reader`: Bundle will act as a destination (it reads shmdata)
* `writer`: Bundle will act as a source (it writes shmdata)

Additional tags exist, that are specific to a certain kind of usage, either in [Switcher](https://gitlab.com/nicobou/switcher) or in [Scenic](https://gitlab.com/sat-mtl/tools/scenic/scenic):

* `occasional-writer`: Bundle will occasionally write shmdata
* `device`: Bundle is attached to a device (ex: a MIDI keyboard)
* `hid`: Bundle is attached to a HID device (ex: a computer mouse)
* `dummy`: Bundle is a dummy bundle

## Placeholders

Documentation on placeholders to come soon.
