# Writing quiddity bundles

Bundles offer a way to define preconfigured quiddity pipelines. These pipelines can then be instantiated in Switcher just like normal quiddities. Every parameter from every quiddity in the pipeline can be accessed as usual, and methods from each quiddity in the bundle can also be invoked normally (using `switcher-ctrl`, for example). Bundles are a great way to save and reuse quiddity configurations.

By default, bundles must be specified in a JSON file named `switcher.json` and saved to `$HOME/.scenic/`. The [dummy-switcher.json](dummy-switcher.json) provides bundle examples.

## Bundle specifications

Inside `switcher.json`, bundles must be described in the bundle branch. A standard bundle description looks like this:

```json
"DummyVideo" : {
    "pipeline" : "v4l2src name=HDMI _shmw _device=0 _add_to_start ! videoconvert name=converterHDMI _no_prop ! nvenc name=encoderHDMI _shmw & .HDMI ! glfwin name=windowHDMI",
    "doc" : {
        "long_name" : "Dummy video bundle",
        "category" : "video",
        "tags" : "writer",
        "description" : "A dummy video bundle for testing purposes"
    }
}
```

All fields are mandatory.

* Name (`DummyVideo`): Every bundle needs a name. Instances of this bundle will be created using this name (ex. `switcher-ctrl -C DummyVideo dummy`). No whitespaces or underscores allowed.
* `pipeline`: This is where the actual quiddity pipeline will be specified. See [Pipeline description](#pipeline-description).
* `doc`: This is where bundle metadata is defined.
  * `long_name`: Full name of the bundle. This name will be displayed when using `switcher-ctrl -K` for example.
  * `category`: Category of the bundle. See [Bundle categories](#bundle-categories) below
  * `tags`: Tags will specify the bundle's roles. See [Bundle tags](#bundle-tags) below
  * `description`: Short description fo the bundle's functionnality. Will be displayed when using `switcher-ctrl -K`.

If the bundle specification is correct, it will be available for creation in Switcher, using `switcher-ctrl -C`.

## Pipeline description

The pipeline field is the heart of the bundle. This is where the quiddities that compose the bundle are specified.

To add a quiddity in the pipeline, simply add its name. Following the quiddity's name, it is possible to specify default values for one or many of the quiddity's properties, using the `property=value` format. Each property/value pair must be separated by a whitespace. For example, the following pipeline will add a `v4l2src` quiddity with its `device` property set to 1 and it's `pixel_format` property set to 4:

```json
"pipeline": "v4l2src name=VideoCapture device=1 pixel_format=4"
```

Notice the `name` property right next to `v4l2src`. This property is mandatory, and is used to name the instance of the v4l2src quiddity that will be created.

Refer to a quiddity's infotree to get a complete list of its properties.

### Piping

To pipe the output of the v4l2src quiddity into the input of another quiddity, simply use `!` to join them together. It is up to you to ensure that this connection is actually possible (specifically, that the shmdata caps are compatible).

Following on the previous example, to add a `videoconvert` quiddity that will take the `v4l2src`'s output as an input:

```json
"pipeline": "v4l2src name=VideoCapture device=1 pixel_format=4 ! videoconvert name=Converter"
```

You can chain quiddities this way to create long and complex pipelines.

### Joining

It is possible to create a bundle with two or more quiddities without having to chain their outputs and inputs, by using `&`.

```json
"pipeline": "executor name=Executor autostart=false periodic=false & watcher name=Watcher create_directory=true"
```

In the above example, `executor` and `watcher` are created together and exist inside the same bundle. However, `executor`'s output will not be piped to `watcher`'s inputs. Notice also that default values have been set for some of `executor`'s and `watcher`'s properties.

### Pipeline tags

Some tags can be added in the pipeline to add some specific behaviours to a quiddity. Each tag applies only to a specific quiddity in the pipeline. In order for a tag to be applied to a specific quiddity, it must be positioned after the quiddity's name, but before any piping or joining operator (it may be placed, before, after or in between a quiddity's properties, in any order). A quiddity may be marked with multiple tags. Five different tags exist:

#### `<shmr`

Defines the pipeline's shmdata reader, aka the entry point for shmdata in the pipeline.

A maximum of one reader may be specified in a pipeline. Shmdatas connected to the bundle will be forwarded to the quiddity marked with this tag.

#### `<shmw`

Define the pipeline's shmdata writers, aka quiddities that will produce (write) shmdata in the pipeline.

A quiddity marked with this tag will see its shmdata exposed by the bundle, so that it can be used by other quiddities. If a quiddity creates shmdata but is not marked with this tag, then its shmdata will NOT be accessible outside of the bundle. There is no minimum or maximum number of shmdata writers in a pipeline.

#### `<add_to_start`

Quiddities marked with this tag will be bound to the bundle's `started` property. If at least one `<add_to_start` property is present in the pipeline, a global `started` property will be created for the bundle.

When this property is set to `true`, every quiddity in the pipeline marked with this tag will also have their `started` property set to `true`. Inversely, when the global `started` is set to `false`, every quiddity marked with this tag will be set to `started=false`.

Each quiddity marked with this tag will also have their individual `started` property removed: their `started` state will be entirely dependant on the bundle's `started` state.

#### `<top_level`

By default, each quiddity's properties will have a `parent` metadata. By default, the value of this `parent` field will be the `name` of the quiddity to which it belongs (ex: Considering a `v4l2src` named "VideoTest", the parent of `v4l2src`'s `pixel_format` property will be `VideoTest`).

However, when adding the `<top_level` tag for a quiddity, each of its properties will have an empty `parent` field. Each property will be "top level" (ie. they don't have a parent quiddity above them).

This tag is useful for Scenic, since the `parent` field affects the way a quiddity's properties will be displayed in Scenic. But aside from that, this tag does nothing else in Switcher.

#### `<no_prop`

This tag will blacklist all of a quiddity's properties. Blacklisted properties will not show up in Scenic and will not be listed when running `switcher-ctrl -t`. In addition, blacklisted properties cannot be modified and their values cannot be displayed in any way. In short, blacklisted properties will 'disappear'. This tag is closely linked with the underscore (`_`) modifier.

#### Tags example

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

* The `executor` quiddity will read shmdata, and is the entry point for shmdata in the pipeline.
* All of `executor`'s properties are blacklisted, except for `autostart`. However, `periodic` will still be set to `false` and `command_line` will be set to `somecommand` before being blacklisted.

### Property blacklist / whitelist

Quiddity properties can be blacklisted individualy (see the [`<no_prop` tag](#<no_prop) for an explanation on blacklisting) by prefixing the property's name with an underscore. Properties prefixed with an underscore will get their specified value applied (if there is one), before being blacklisted.

However, if the quiddity is marked with the `<no_prop` tag, the underscore's role changes: it is used to whitelist properties instead of blacklisting them. All of the quiddity's properties will be blacklisted, except for those prefixed with the underscore.

In short:

* `<no_prop` tag is absent: underscore is used to blacklist individual properties
* `<no_prop` tag is present: underscore is used to prevent a property from being blacklisted (whitelisting)

Example:

```json
"pipeline" : "ladspa name=Delay <shmr <no_prop plugins=Delayorama _first-delay=0.0 _delay-range=0.0001 delay-change=0.2 ! jacksink name=Jack _auto_connect=true do_format_conversion=true _do_rate_conversion"
```

In the above pipeline:

* All of `ladspa`'s properties are blacklisted, except `first-delay` and `delay-range`.
* Only the `auto_connect` and `do_rate_conversion` properties of `jacksink` are blacklisted.
* Notice that no value is assigned to `do_rate_conversion`. This means that `do_rate_conversion`'s default value will be applied before it is blacklisted.

### `Group` property

The `group` property can be added to any quiddity. It is optional; its purpose is to override the `parent` value of each of the quiddity's properties with its value. The value of `group` is arbitrary. The effects are similar to the [`<top_level` tag](#<top_level): it affects the way Scenic displays properties, but it is not of much use in Switcher.

### Escaping

Quotes `"` can be escaped using the backslash (`\`). Whitespaces can be escaped by prefixing them with two backslashes. However, whitespaces contained in (escaped) quotes are automatically escaped (no backslashes are required).

Example:

```json
 "pipeline" : "executor name=Test <shmr command_line=shmdata2ndi\\ -v\\ _shmpath_video_\\ -a\\ _shmpath_audio_\\ -n autostart=false"
```

The value of `command_line` will be `shmdata2ndi -v _shmpath_video_ -a _shmpath_audio_ -n`.

## Bundle categories

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

## Bundle tags

Tags define the bundle's roles and how it will behave in Switcher. One or more of the following tags may be specified:

* `reader`: Bundle will act as a destination (it reads shmdata)
* `writer`: Bundle will act as a source (it writes shmdata)

Additional tags exist, that are specific to a certain kind of usage, either in Switcher or in Scenic:

* `occasional-writer`: Bundle will occasionally write shmdata
* `device`: Bundle is attached to a device (ex: a MIDI keyboard)
* `hid`: Bundle is attached to a HID device (ex: a computer mouse)
* `dummy`: Bundle is a dummy bundle

## Placeholders

Documentation on placeholders to come soon.
