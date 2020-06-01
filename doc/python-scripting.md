Python3 scripting   
=======

Switcher library can be used with Python by importing the ```pyquid``` module available in this repository.


## First steps

### Module documentation

Documentation can be obtained by invoking the help command in the python interpreter:

```bash
python3 -c "import pyquid; help (pyquid)"
```

### Examples

See the example files [here](../wrappers/python/examples/).


## Getting to know the module

`pyquid` allows for interaction with Switcher's services called "quiddities". This module gives access to the quiddities in two ways:

* through a `Qrox` object, which is a handle to the quiddity and allows for getting general information about it like its name, id, or get a `Quiddity` object from it.
* through a `Quiddity` object which gives control over the wrapped quiddity inside Switcher, like setting/getting properties, invoking methods, specifying its configuration and getting its `InfoTree`.

Apart from the quiddities, the `pyquid` module allows for creating two more objects:

* `Switcher` objects which wrap an instance of Switcher, and which allows for creating quiddities and gettings `Qrox` for them.
* `InfoTree` object which is a key-value structure and is used to store the state and configuration of Switcher and its quiddities.

In the following example we will go through the steps to create a `Switcher` object, create quiddities using it, getting information about their capabilities and setting their properties. Let's first open a Python interpreter and then:

```python
import pyquid

sw = pyquid.Switcher('a_good_name')
```

We don't know what we can do for now, so let's ask it:

```python
help(sw)
```

This will print a lot of stuff, but here are some interesting parts:

```
class Switcher(builtins.object)
 |  Switcher objects.
 |  Entry point for creating, removing and accessing quiddities.
 |  
 |  Methods defined here:
...
 |  class_doc(...)
 |      Get a JSON documentation of a given classes.
 |      Arguments: (class)
 |      Returns: JSON-formated documentation of the class.
 |  
 |  classes_doc(...)
 |      Get a JSON documentation of all classes.
 |      Arguments: (None)
 |      Returns: JSON-formated documentation of all classes available.
 |  
 |  create(...)
 |      Create a quiddity. The name and the config are optional.The config (an InfoTree) overrides the switcher configuration file
 |      Arguments: (type, name, config)
 |      Returns: a handle to the created quiddity (pyquid.Qrox), or None
...
 |  list_classes(...) |      Get the list of available type of quiddity.
 |      Arguments: (None)
 |      Returns: A list a of class name.
 |  
 |  list_quids(...)
 |      Get the list of instanciated quiddities.
 |      Arguments: (None)
 |      Returns: A list a of quiddities.
...
```

So it looks like we can get a list of the available classes (or quiddities), get some documentation about them and create one given its class name. Let's list the available classes:

```python
sw.list_classes()
```

You will get a list looking more or less like the following, depending on the available classes:

```
['OSCsink', 'OSCsrc', 'SOAPcontrolServer', 'audioenc', 'audiotestsrc', 'avplayer', 'avrec', 'cropper', 'custom-save', 'decoder', 'dummy', 'dummysink', 'emptyquid', 'executor', 'extshmsrc', 'filesrc', 'glfwin', 'httpsdpdec', 'jackserver', 'jacksink', 'jacksrc', 'ladspa', 'ltcdiff', 'ltcsource', 'ltctojack', 'method', 'midisink', 'midisrc', 'nvenc', 'protocol-mapper', 'pulsesink', 'pulsesrc', 'resample', 'rtmp', 'shmdelay', 'signal', 'sip', 'systemusage', 'timelapse', 'urisrc', 'v4l2src', 'videnc', 'videoconvert', 'videosnapshot', 'videotestsrc', 'vncclientsrc', 'vrpnsink', 'vrpnsrc', 'watcher']
```

Note that to get information about a class, you can use the `swquid-info` command line tool:

```bash
swquid-info --short-list --properties videotestsrc
swquid-info --short-list --methods videotestsrc
```

Alright, we have a lot of choices here. As we want to test the module, we will continue with a simple service. We will create a quiddity which generates video patterns and shows the generated shmdata in a window.

```python
video_test_qrox = sw.create('videotestsrc')
video_test_quid = video_test_qrox.quid()
```

By default this quiddity is stopped. To start it you have to set the `started` property to `True`:

```python
video_test_quid.set('started', True)
#Check that the video is indeed started
video_test_quid.get('started')
```

A shmdata socket has not been created. If you followed closely the previous commands you should check the data flow with the following command line:

```bash
sdflow /tmp/switcher_a_good_name_1_video
```

Let's now create a window. In Switcher this is done using the `glfwin` quiddity:

```python
win_qrox = sw.create('glfwin')
win_quid = win_qrox.quid()
```

If you have read Switcher's documentation you should know that data sharing between the quiddities is done through the shmdata library. To connect the window quiddity to the video test pattern, we need to give the former the socket path to the shmdata generated by the `videotestsrc` quiddity:

```python
video_shm_path = video_test_quid.make_shmpath('video')
win_quid.invoke('connect', [video_shm_path])
```

You should now see the video pattern inside the window!

Note that the parameter given to `video_test_quid.make_shmpath` is the suffix of the shmdata path, which is generated automatically by the quiddity when started. To get a list of the shmdatas generated by a quiddity you have to get the information from the quiddity's `InfoTree`:

```python
print(video_test_quid.get_info_tree_as_json('.shmdata'))
```

You should get something looking like this:

```
{
  "writer" : {
    "suffix" : "video", "/tmp/switcher_a_good_name_1_video" : {
      "caps" : "video/x-raw, format=(string)I420, width=(int)1920, height=(int)1080, framerate=(fraction)30/1, multiview-mode=(string)mono, pixel-aspect-ratio=(fraction)1/1, interlace-mode=(string)progressive",                                                              
      "category" : "video",                                                                                                             
      "stat" : {
        "byte_rate" : 93312000.0, "rate" : 30.0
      }
    }
  }
}
```

More generally, you can get all the capabilities of a quiddity by asking for its whole `InfoTree`:

```python
print(video_test_quid.get_info_tree_as_json('.'))
```

We can now clean everything up:

```
win_quid.invoke('disconnect', [video_shm_path])
sw.remove(win_qrox.id())
sw.remove(video_test_qrox.id())
```

To go deeper inside `pyquid`, go through the examples mentioned [earlier](#Examples).
