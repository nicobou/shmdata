NEWS
====
Here you will find a high level list of new features and bugfixes for each releases. 

switcher 3.0.0 (2021-10-04)
---------------------------
This is an official release in the 3.0 stable series.

New features:
* ✨ InfoTree API: Usage improvements
* ✨ add Claw classes and integrate in Quiddity
* ✨ add can_do methods to claw and connect_quid
* ✨ add can_sink_caps to claw
* ✨ add connection API for shmdata reader Quiddity
* ✨ add dynamic shmdata in claw
* ✨ add get_copy to infoTree
* ✨ add repeat_array_indexes option in infoTree json serialization
* ✨ add signals for dynamic connection spec
* ✨ add try-connect and get-con-spec to switcher-ctrl, with updated doc
* ✨ get optional parsing error message when deserializing InfoTree
* ✨ parsing ConnectionSpec
* ✨ python wrapper for Claw

Bug fixes:
* 🎨 reorder python example files
* 🐛 fix CI
* 🐛 fix assert not handled in python text number 11
* 🐛 fix infotree foreach breaking the tree
* 🐛 fix shmshot (wrong use of python condition variable)

Improvements:
* 👷 add color to compiler output
* 📝 formating in Ids doxygen

Documentation:
* 📝 update MIGRATIONS.md for Switcher 3.0.0 release

Breaking change:
* 🐛 Fix the SIP plugin along with the `get_info` method for `pyquid`
* 💥 pyquid sends Infotree in signals rather than a json serialization of it
* 💥 removed connect methods on quiddities
* 💥 use kind keyword instead of either class or type for string description of quiddity types
* 🔥 remove category and tag


switcher 2.3.0 (2021-09-20)
---------------------------
This is an official release in the 2.3 stable series.

Documentation:
* 📝 Add MIGRATIONS.md
* 📝 Update author script
* 📝 Update configuration and bundle documentation

New features:
* ✨ Add Session API to Python wrapper
* ✨ Add IP address for each interface in systemusage's InfoTree
* ✨ Add print switcher version to swquid-info
* ✨ Add possibility to construct Quiddity object in python wrapper

Bug fixes:
* 🐛 Fix webrtc deadlock
* 🐛 Fix `pyquid.Switcher.get_quid` for Quiddity initialization
* 📝 Fix and improve doc
* 🐛 Fix defaul SIP port is applied before the one requested in configuration


Breaking changes:
* 🔥 Remove the syphon plugin
* 🔥 Remove Ubuntu 18.04 related files
* 💥 For consistency with the CPP API, refactor Quiddity.init and create in pyquid with name arg reworded as nickname
* add PyErr in pyquid.Quiddity

Improvements:
* 🚨 Reduce noise for pyquid at compile time

switcher 2.2.6 (2021-05-17)
---------------------------
This is an official release in the 2.2 stable series.

Improvements:
* ⬆️  upgrade dependency to nvidia driver version 460
* 📝 add gitlab issues template, code of conduct and RFC process

switcher 2.2.4 (2021-05-03)
---------------------------
This is an official release in the 2.2 stable series.

* ✨ Add hardware support for Nvidia Jetson

switcher 2.2.2 (2021-04-19)
---------------------------
This is an official release in the 2.2 stable series.

New feature:
* ✨ default plugin path is scanned recursively

Improvement:
* 🎨 install nvidia plugin in its own folder

switcher 2.2.0 (2021-04-08)
---------------------------
This is an official release in the 2.2 stable series.

Breaking changes, API refactoring:
* 💥 pyswitch on-quiddity-created and on-quiddity-removed callbacks give quiddity id instead of quiddity name
* 💥 removed pyqrox, pyswitch creates now directly a pyquiddity
* 💥 removed get_qrox_from_nickname
* 💥 get_name moved to get_id (Quiddity class), get_name moved to get_nickname and get_qrox_from_name moved to get_qrox_from_nickname (python)

Improvement:
* ⚡️ improved use of reference and const

switcher 2.1.38 (2021-04-08)
---------------------------
This is an official release in the 2.1 stable series.

* 🐛 Fix shmpath collision in PJSIP

switcher 2.1.36 (2021-03-23)
---------------------------
This is an official release in the 2.1 stable series.

New feature:
* ✨ Add allowlist_caps property in shmdelay

Improvements:
* ✅ Improved pyquid subscribe to property test
* 💬 Upgrade nvenc property descriptions
* 📄 Removed extra license

switcher 2.1.34 (2021-03-08)
---------------------------
This is an official release in the 2.1 stable series.

New Feature:
* Add a Ubuntu packaging script

Improvements:
* ✅ Improved python tests about signal subscription
* 🎨 Remove property refresh in videotestsrc
* ✨ Add can-sink-caps method to SIP quiddity

Bug Fixes:
* 💚 Fixed docker deploy CI for scenic
* 📝 Fixed docstring of Pyquid set_nickname
* 🐛 Fixed decodebin-to-shmdata issue with non-interleaved audio
* 🐛 Fixed jacksink quiddity segfault with non interleaved audio
* 🐛 Fixed VRPN not using the submodule

switcher 2.1.32 (2021-02-22)
---------------------------
This is an official release in the 2.1 stable series.

New features:
* ✨ Add subscribe and unsubscribe PySwitch methods
* ✨ Update PythonAPI to include load_bundles

Improvement:
* 👷 Replaced GLFW, ImGUI and PJSIP downloads with submodules
* ♻️  Get_quiddity_id returns a qid_t

Bug Fixes:
* 🐛 Fix make package_source_test to include non-root directories named build
* 🐛 Fix release script to get submodules
* 🐛 Fix call with shmdata with no switcher-name and quiddity-id in caps
* 🐛 Fix call with nvenc

switcher 2.1.30 (2021-02-09)
---------------------------
This is an official release in the 2.1 stable series.

New feature:
* ✨ Make the RTMP quiddity startable

Improvement:
* ✨ Allow whitespaces in nicknames sent to SIP

Bug fixes:
* 🐛 Fixed shmdata2jack not attaching to the jack dummy server
* 🐛 Fixed release script options
* 🐛 Fixed xvfb installation for CI tests

switcher 2.1.28 (2020-12-03)
---------------------------
This is an official release in the 2.1 stable series.

Bug fix:
* 🐛 fix ladspa quiddity issue limiting to mono input ladspa plugins

New features:
* ✨ Add server_name & client_name props in jacksink
* ✨ Add server_name property in jacksrc

Improvements
* 🎨 Rename jack-client-name property to client_name
* ➕ Add back swh-plugins dependency

switcher 2.1.26 (2020-11-16)
---------------------------
This is an official release in the 2.1 stable series.

Packaging and CI:
* 💚 fix check_switcher_ctrl with package_source_test
* 💚 fix pulsesrc test blocked when no pulseaudio server is running
* 👷 add Debian package build in CI with ubuntu 20.04
* 📦 generate Debian package with make package

Improvement:
* ♻️ Give IDs to shmpaths created by SIP quiddity

switcher 2.1.24 (2020-11-03)
---------------------------
This is an official release in the 2.1 stable series.

New feature:
* ✨ add support for SWITCHER_PLUGIN_PATH environment variable

Improvement:
* ⬇️  use dlopen instead of g_module for loading quiddity plugins

Bug fixes:
* 🐛 fix package_source_test when switcher is not already installed
* 🐛 fix segfault in avplayer when started with empty folder
* 👷 use dependency files for CI and install instructions
* 💚 fix glfw test frequent segfault in CI

switcher 2.1.22 (2020-10-05)
---------------------------
This is an official release in the 2.1 stable series.

* 💚 fix test-package (CI stage)

switcher 2.1.20 (2020-09-22)
---------------------------
This is an official release in the 2.1 stable series.

* ✨ Add quiddity and Switcher info in shmdata caps
* 🍱 updating switcher logo and adding graphic charter
* 📝 update dependencies in instalaltion doc (ubuntu 20.04)
* ✅ test for the webrtc quiddity

switcher 2.1.18 (2020-09-08)
---------------------------
This is an official release in the 2.1 stable series.

* 🎨 Refactored the simple server to be a class in the WebRTC Quiddity

switcher 2.1.16 (2020-08-07)
---------------------------
This is an official release in the 2.1 stable series.

* 📝 update instruction for build with ubuntu 20.04
* 🐛 fix filesrc quiddity blocking when set with a non existing file
* 🥅 catching errors related to method signatures in pyquiddity

switcher 2.1.14 (2020-07-14)
---------------------------
This is an official release in the 2.1 stable series.

* 🐛 Fix invalid quiddity pointer segfault

switcher 2.1.12 (2020-06-29)
---------------------------
This is an official release in the 2.1 stable series.

Bug fix:
* 🐛 compilation error with escape_json on arm

switcher 2.1.10 (2020-06-16)
---------------------------
This is an official release in the 2.1 stable series.

Bug fixes:
* 🐛 Fixed PJCall not handling correctly external shmdatas
* 🐛 fix swcam-display segfaut in 20.04

Dependency upgrades:
* ⬆️ upgrade glfw to version 3.3.2
* ⬆️ using Ubuntu 20.04 instead of 19.10 in CI for coverage

Improvements:
* ♻️ Add enums for various SIP statuses
* ♻️ use lambda instead of method pointer in ThreadedWrapper

switcher 2.1.8 (2020-06-01)
---------------------------
This is an official release in the 2.1 stable series.

* 📝 Improved Python wrapper documentation

switcher 2.1.6 (2020-05-19)
---------------------------
This is an official release in the 2.1 stable series.

Bug fixes:
* 🐛 fix issue with ladspa plugin with ca_FR locale
* 🐛 fix pyquid_quid_infotree_stress test

Improvements:
* 👽 Update PJSIP sendto function call to sendto2
* ✨ Expose SIP method to hang up all calls

switcher 2.1.4 (2020-05-06)
---------------------------
This is an official release in the 2.1 stable series.

New features:
* ✨ Add webrtc quiddity with example signaling server and web client

Bug fixes:
* 🐛 Graft only connections on attach_shmdata
* 🐛 Fixed release script not filling NEWS.md

New dependency related to the webrtc quiddity:
* ➕ Added libsoup for websocket communication
* ➕ Added gsteamer-webrtc-1.0


switcher 2.1.0 (2020-03-26)
---------------------------
This is an official release in the 2.1 stable series.

Documentation:
* Add an example of quiddity using gst

New features:
* ✨ Add dynamic bundle loading
* ✨ add cfor_each_in_array for infotree
* ✨ add safe-bool-log util
* ✨ adding shmshot tool for image capture of video shmdata

Bug fixes:
* 🐛 Fix a segfault when a glfwin is destroyed
* 🐛 Fix parsing of bundle parameters with dashes
* 🐛 Fixed CI fail on develop and master branch
* 🐛 Fixed an issue with dependencies detection through pkg-config
* 🐛 Return false when the pipeline's state can't be changed
* 🐛 fix jacksink not reconnecting to its autoconnect destination
* 🐛 improving glibmainloop concurrency strategy
* 🐛 urisrc unregisters uridecodebin signals before destruction of pipeline

Continuous integration:
* ✅ Allowed for GLFW plugin to be tested by the CI
* ✅ Changed Ubuntu version for coverage check
* ✅ added tests for tools

switcher 2.0.2 (2020-02-06)
---------------------------
This is an official release in the 2.0 stable series.

New features:
* ✨✅ adding videosnapshot quiddity

Bug fixes:
* 🐛  Fix deinterlace crash in v4l2src when pixel format is not raw video
* 💚 add missing dependency related to ubuntu 20:04 image update

Typos:
* ✏️  Fix typo in NEWS.md
* ✏️ sochet meant socket

switcher 2.0.0 (2020-01-31)
---------------------------
This is an official release in the 2.0 stable series.

Breaking changes:
* 💥 fileutils namespace
* 💥 any in switcher namespace
* 💥 namespace stringutils
* 💥 renaming logger classes and files
* 💥 renaming infotree files
* 💥 shmdata namespace
* 💥 quiddity namespace
* 💥 quiddities namespace
* 💥 log namespace
* 💥 gst namespace
* 💥 infotree namespace
* 💥 subfolder in switcher sources
* 💥 init_startable in now private, a StartableQuiddity must call the appropriate parent constructor
* 💥 Rename pixel_format property of videoconvert

New Features: 
* ✨✅ adding for_each_in_array in InfoTree
* ✨ Add deinterlacer in v4l2src
* ✨ Add video properties auto-detect
* ✨ Add jack-server quiddity
* ✨ Add swquid-info, a command line informator for quiddities
* ✨ Add swcam-display tool
* ✨ Add graft Infotree by value
* ✨ Add get_name, get_type, set & get nickname in pyquiddity
* ✨ Add autostart property to midisink
* ✨ Add autostart property to OSCsink
* ✨ Add restart_on_change property to Executor
* ✨ Add do-lost in rtp-session
* ✨ Add video cropper quiddity

Bug fixes:
* 🐛 Fix SIP transmissin of shmdata created by NDI2Shmdata
* 🐛 Fix in shmdata-to-jack destruction
* 🐛 Fix play pause action in filesrc
* 🐛 Fix map midi value to property
* 🐛 Light refactor of OSCsrc
* 🐛 Fix handling of shmdata created by SIP when calling contact
* 🐛 Refactor gst-decodebin and fix erratic behavior when decoder connects to itself
* 🐛 Fix race condition with gstream pipeline play/pause
* 🐛 Explicitly capture invite_session in pjcall
* 🐛 Add a destructor for Watcher

Analytics:
* 📈 coverage in CI

Docs:
* 📝 Add GPLv3 badge in README.md
* 📝 Fix pipeline status in README.md
* 📝 Updated code structure documentation
* 📝 Updated InfoTree exemple links in writing-quiddity.md

Tests:
* ✅ Add test for switcher commands
* ✅ Add switcher log and quiddity configuration to quiddity-basic-test

switcher 1.1.2 (2019-09-17)
---------------------------
This is an official release in the 1.1 stable series.

New features:
* CI now pushes image into registry for develop and master branches + updated docker documentation
* Add quiddity (Watcher) for watching directories
* Add Executor quiddity
* addition of shmdata2jack, a simple command line tool.
* mention NDI2shmdata in README.md
* Add documentation on writing bundles
* Add run-switcher-in-docker in documentation
* Add missing caps specifier in JackToShmdata
* pjsip upgraded to 2.9
* Add argument to init ShmdataFollower tree on server connect
* Add do not convert rate and format in jacksink in order to get more than 63 channels
* Add stun turn port configuration in doc about bash scripting a sip call
* pyquid is installed in python lib path instead of cmake prefix
* removed nvdec quiddity
* Make nvdec (GStreamer element) compatible with decodebin and shmdata

Bug fixes:
* Fix: Resolve build problems from Scenic images
* Handle escaped whitespaces in bundle parser
* Fix signal handling problems in Executor
* Fix fullscreen windows not displaying in right monitor
* Fix GLFW fullscreen windows from minimizing automatically
* Escape special characters for JSON grafting
* Fix segfault when decorated glfwin is destroyed
* Fix fullscreen glfwin bug
* Fix dysfunctional image loading in glfwvideo
* adding Docker (desktop host and Raspi) file and documentation for shmdata, switcher & scenic in docker with support nvidia GPU encoding.
* less spelling mistakes
* adding informations about writing quiddities
* fix nvenc build on ubuntu 19.04
* fix race condition during GST Pipe destruction
* fix compilation issues with Ubuntu 19.04
* fix ltfdiff test sometimes crashing
* fix gsp-pipeliner leaking bus watch
* fixes in shmdata to jack
* do not notify reader stats
* fix set sip related server port from uris
* fix set sip server port from registration uri
* register to the appropriate quiddity in infotree stress python test
* fix inconsistent control of gst pipeline play/pause states
* document how to set remote sip server port from command line
* fix append of remote port in sip registration url
* fix compilation issue with pyquiddity.cpp
* use asyncio in pyquid infotree stress test
* fix possible deadlocks in pyquiddity
* typo in qrox header
* fix glfwin sometimes segfault
* glfwin handle dynamic change in caps without disconnecting shmdata
* avoid passing nullptr as a string in uridecodebin
* infotree signal stress test and pyquid multi-threading improvement
* pyquid: use save and restore thread instead of not thread safe PyGIL
* more asserts in pyquid signal file
* pixel converter destruct gst pipeline before elements
* gstpipe set gst pipeline to null from the destructor and unref from gmainloop
* gstpipe kills gst pipeline from the main loop
* fix sometimes filesrc crash at deletion (members order in quiddity-container)
* reverse gst-element ref removal from previous commit
* adding unsubscribe to signal in python exemple
* avoid segfaut due to simultameous g_signal_handler_disconnect and signal notification in gst-subscriber
* information tree serialization is mutexed now
* Add documentation for building NVDEC
* Make nvdec compatible with decodebin and shmdata
* Fix log messages in PJCall
* Removed the validation on overlay_additional_font and overlay_config properties
* fix member orders in videotestsrc
* adding python3 in CI and switcher dependencies
* Handle gstreamer.primary_priority configuration as an int
* Drop corrupted frames when decoding H264 streams
* Allow a Gstreamer plugin's rank to be set using the switcher.json file

switcher 1.1.0 (2018-12-13)
---------------------------
This is an official release in the 1.1 stable series.

New features:
* adding switcher logo
* dynamic configuration at quiddity creation that override switcher static configuration
* jack quiddities takes a dynamic configuration for selection of jack server name (see python example #8)
* adding invoke_async to pyquiddity

Bug fixes:
* check for availability of misleading indentation flag with c++ compiler
* check for availability of deprecated register for a jack header
* ltc plugin checks for availability of deprecated register for a jack header
* uint replaced by unsigned int
* fix clang compilation (but need to disable SOAP plugin)

switcher 1.0.0 (2018-09-19)
---------------------------
This is an official release in the 1.0 stable series.

Bug fixes:
* fix jacksink infinite loop
* saving shm connection is now based on reader saving name and suffix of writer quiddity if available. raw_shmdata path are still possibly saved if no information about the writer are available
* adding a test for shmdata transfert between bundles
* get header installed in a switcher version specific folder
* c++ version generated in pkgconfig
* fix Urisrc will sometimes crash on delete
* fix config not loaded excepted for bundles
* fix compilation for ubuntu 18.04

New Features:
* switcher python wrapper
* improved documentation
* new filesrc quiddity with play/pause/loop with basic test.
* adding two properties for jacksink: i) connect all to first channel and ii) connect only first channel
* httpsdpdec uses now a prop instead of a method for uri
* add log options to sip plugin, config option to python wrapper, and example python script
* adding initial doc for protocol mapper
* InfoTree copy
* get version and default plugin path from methods in switcher
* Makes it so videoconvert uses multi-threading if available (GStreamer version >= 1.12).
* add a resampling quiddity based on libsamplerate
* provide plugin path with pkgconfig

switcher 0.8.68 (2018-04-06)
---------------------------
This is an official release in the 0.8 stable series.

* Adding sip-call.md
* Add using-osc-quiddities.md
* Disable libwebrtc for pjsip because we don't use it and it prevents switcher from being built on ARM (no SSE2 support, duh!)

switcher 0.8.66 (2018-02-21)
---------------------------
This is an official release in the 0.8 stable series.

* fix the nvenc bug causing crash when adding/removing nvenc quiddities

switcher 0.8.64 (2017-11-29)
---------------------------
This is an official release in the 0.8 stable series.

* actually deactivated buggy lock win aspect ratio in glfw

switcher 0.8.62 (2017-11-27)
---------------------------
This is an official release in the 0.8 stable series.

* deactivated buggy lock win aspect ratio in glfw

switcher 0.8.60 (2017-10-27)
---------------------------
This is an official release in the 0.8 stable series.

Bug fixes:
* Fix member ordering issue with pjsip
* pjsip built in release mode with extra checks (assert does not quit anymore)
* Fix sdp issue with opus audio
* Extra check for pjsip ids
* Fix media-label containing some char results in stream not sent
* Change iv4l2 io-mode when using force framerate option to avoid being stuck.

New Features:
* Refactoring log system Renaming manager_impl_ into qcontainer_ QuiddityConfiguration for creation of quiddities Remove registry by quiddity name Init method removed from quiddities
* Ubuntu 17.04
* Timecode "package" and shmdelay documentation.
* Adding support for timecode input to compute delay automatically.
* Shmdata generic delay with manual delay.
* Timecode generation or reading timecode from source file.

switcher 0.8.58 (2017-08-14)
---------------------------
This is an official release in the 0.8 stable series.

Bug fixes:
* glfw win aspect ratio
* fix nicknames not saved
* fix audio property group not initialized into gst-audio-codec
* fixed selection properties not being able to be set correctly after selection refactor

New Features:
* shmdata AVPlayer (still seek/sync issues)
* signaling on-nicknamed
* rename quiddity-manager into switcher and quiddity manager impl into quiddity container
* make it so the periodic task starts at regular intervals instead of waiting a fixed period of time
* refactoring signals using native c++ instead of gobject
* centralize quiddities doc management outside of quiddity class

switcher 0.8.56 (2017-07-20)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* nickname for quid
* using quid nickname in sip calls
* Multichannel audio/video shmdata recorder.
* adding jacksink max_number_of_channels config
* adding window aspect ratio locking in glfwvideo
* reorder glfw groups
* adding max_number_of_channels config for jacksrc
* select selections by name is now available

Bug fixes:
* consistent property naming among jacksrc and jacksink
* refactoring quiddity manager: no more commands and simpler save and load
* prefer droping frame than accumulating in GLFWIN
* set drop-on-latency at true for rtpbin in rtp-session2
* fix audiotestsrc sometime crashing during check_startable
* fix midisrc always expose shmwriter
* Fixed protocol mapper CMakeList.
* fix SIP plugin with OSX

switcher 0.8.54 (2017-04-12)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* ENABLE_GPL is now available as a compilation option, see README for more details.

Bug fixes:
* OSX compilation.
* Save default instead of IP address when SIP DNS server is default.
* Fixes enabled/disabled state and message for width and height of videotestsrc.
* Force pixel-aspect-ratio in timelapse to fix deadlock when not providing one.
* Copy/paste error in audio shmdata subscriber pruning callback in RTMP quiddity.
* Fixes hardcoded channel mask for jacksrc shmdata.

switcher 0.8.52 (2017-03-31)
---------------------------
This is an official release in the 0.8 stable series.

Bug fixes:
* fix rtmp does not seem to connect to shmdata
* fix can-sink-cap brocken in scenic wyth extshm
* init tree with shmpath in extshmwriter
* update dependencies for OSX
* fix logs not displayed with switcher

switcher 0.8.50 (2017-03-17)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* RTMP quiddity compatible with multiple streaming applications (Youtube/Twitch tested).
* Defaults glfw window overlay to bottom center and 20 pt font size.
* Adding tree grafted and pruned to signal quid example.
* Remove visual disconnection message because it is very often misleading (until we have the reinvite feature).
* Synchronize SIP account de-registration.

Bug fixes:
* Fix race condition with shmdata subscribers in pjsip.
* Put the glClear back awaiting cleanup of the rendering loop because it caused issues with capture cards visualizations.
* Fixed fragmnent shader (caused intel/amd issues). Removed useless glClear causing flickering on slow computers.
* Fix sip label collision.
* Fix some incoming_call_ related crash when caller hangs up.

switcher 0.8.48 (2017-03-06)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* Generic property to protocol mapper. Two first protocols: OSC/Curl. Removed redundant http-get-service quiddity.
* Increase default time interval for the shmdata update process. Keep it shorter (1s) for discrete quiddities like osc/midi.
* LTC to jack transport quiddity.

Bug fixes:
* Only define default update interval for shmdata stats in one place and fix the displayed byte rate and rate.
* Fix GL dependencies in the install instructions
* Fixed appearance of glfw overlay.
* Close SIP transport when destroying pjsip threaded wrapper. Avoids having the sip port attached after closing switcher.
* Fixed and reorganized doxygen build.
* reinitialize jack_input when starting jack-to-shmdata

switcher 0.8.46 (2017-02-17)
---------------------------
This is an official release in the 0.8 stable series.

Bug fixes:
* update coding.md
* fix leaking extshmsrc

switcher 0.8.44 (2017-02-03)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* Added support for multichannel in ladspa plugin.

Bug fixes:
* Check weak ptr to shared ptr validity in pjsip.
* Various fixes in imgui and glfw.
* Setting of fullscreen property was desynchronized due to its processing in render loop.
* Fix jack trying to connect ports with quiddity name instead of jack client name.

switcher 0.8.42 (2017-01-20)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* Creating a quiddity for each incoming SIP stream is now the default behaviour.

Bug fixes:
* fix issue with unsubscribe signals of a quiddity
* Remove useless lock that could cause a deadlock.
* Put all geometry modifications in render task to avoid race conditions.
* Added a timeout to SIP account registration to avoid potential deadlock.
* Fixed wrong buddy id lookup in SIP plugin.
* Forward manager configuration to the bundle manager.
* Process position and size of glfwin in render loop and optimize property setters in case we set them without changing their value.
* Fixed deadlock during removal of exposed SIP quiddity.
* Checks validity of OpenGL buffers before deleting them. Doing otherwise could crash at destruction in some early error cases (e.g no display then the OpenGL API is not loaded and calling glDelete* would crash.)

switcher 0.8.40 (2017-01-04)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* New option in SIP configuration to automatically create a raw shmdata quiddity for incoming streams.
* New notification system in QuiddityContainer to track removal/creation of quiddities. SIP Plugin uses it instead of old legacy CreateRemoveSpy quiddity.
* Remove CreateRemoveSpy quiddity and all its hooks in QuiddityContainer_Impl.
* Port to nvidia SDK 7.1. Needs driver version 375.20 or newer.

Bug fixes:
* Fixed quiddity renaming helper, wrong string was returned.
* Fix midi not shown when receiving from sip.
* Fix member order issue at destruction in bundle class.
* Self destruct the bundle if one of its quiddity is destroyed.

switcher 0.8.38 (2016-12-14)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* New quiddity implementing LADSPA plugins.
* New whitelist mechanism when using no_prop option in bundle definition. reviewers: nouillot, edurand
* Added "group" keyword in bundle description to customize the name of the group of properties of each quiddity.

Bug fixes:
* Fixed nvenc test after bundle pipeline syntax change.
* Use docker image for CI to speed it up.
* Fixed a potential race condition between gst::Pipeliner main loop and its destructor.
* MIDI unit test does not fail if /dev/snd cannot be found.
* Fixed deadlock when deleting a badly created glfwin. Also, overlay is not mandatory now, if something wrong happens during overlay initialization, it will only be hidden.
* Unify OSC quiddity naming.
* Change git path to gitlab for release script.
* Forward graft/prune events in bundles for custom branches (e.g: focused event in glfwin).
* Keep bundle parameters ordered so that dependent properties are declared in the right order. reviewers: nbouillot, edurand
* Fixed properties ordering in bundle for grouping to avoid losing properties in the inspector.
* Added a safety in bundle property mirroring.

switcher 0.8.36 (2016-11-25)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* New bundle option to have a quiddity's properties at top level of the inspector. review: nbouillot
* Dummy sink and nvenc decode test with a bundle,

Bug fixes:
* Various fixes in nvenc, glfw and shmdata-to-jack.
* Bundle properties were not mirrored when creating dynamically after the creation of the bundle quiddity.
* Fixed build type for glfw and a memory leak in ImGui context creation.
* Graphical overlay in glfwin.
* gcc 6.4 on ubuntun 16.10 fixes.
* Removing hardcoded shmdata initial sizes.
* Fix segfault for receiver when cleaning a call.
* Fix issues with SIP when loading file (blacklisting several SIP methods).
* Optionally lower case accounts and TURN user.
* v4l2src USB devices are pushed at the end of the device list.

switcher 0.8.34 (2016-11-11)
---------------------------
This is an official release in the 0.8 stable series.

New Features:
* Add possibility of setting  desired target bitrate for NVENC.
* Allows blacklisting of param in a bundle without specifying its value.
* VRPN Properties Added support for exposing VRPN analog and button channels as properties in the source as well as creating custom analog and button properties to be exposed in the sink.

Bug Fixes:
* Synchronize shmdata version in switcher in release script.
* When saving history, only filter the quiddity creation step for quiddities_at_reset instead of ignoring them completely.
* Json parser was failing when parsing a null-value node. Also refactored deserializer to make it more consistent with json-glib API.
* Bundle internal manager is scaning same plugin dirs as the bundle manager.
* Add required dependency 'libssl-dev' to install cmd
* Property replace correctly notifies UI.
* Fixed race condition with ports_to_connect_ in shmdata-to-jack.
* Fixed missing property fields in tree when replacing the property.
* Fixed empty configuration tree issue.
* Fixed cmake build in release mode.
* Fixed parent property not working with bundles.
* Fixed underscores in bundle names causes shmdata path issues.
* Fixed v4l2 standard framerates not disabled when started.
* Fixed bundle does not disconnect shmdata.

switcher 0.8.32 (2016-10-28)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* Persistence when loading scenarii with pulse sources.
* Cmake build system. 
* Fixed Mac build.
* Bundle: filter property per quiddit.y
* Startable bundle: quiddities can have their started property exposed at the bundle level.
* From bundle pipeline description, optionally hide all properties of a given quiddity.

switcher 0.8.30 (2016-10-21)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* GLFW window. 
* Bundle implementation, forwarding properties from contained quiddities check if quid names are unique in bundle force name specification for each quid in a bundle bundle connects and disconnects shmdatas among contained quiddities.
* vrpn pluginm, source and sink.

Bug fixes:
* Fixed a crash when creating/removing sources from the system and disabled device selection when started.
* various fixes in shmdata-to-jack,
* videoconvert cannot connect to itself,
* adding jackd as dependency in INSTALL.md,

switcher 0.8.28 (2016-10-03)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* Using shmdata version 1.3,
* Adding empty quiddity for saving independant user tree,
* Custom framerates in v4l2src and videotestsrc,
* Videotestsrc and v4l2src do not compress,
* Add a mechanism to force the set callback of a property,
* Adding a message in infotree that tells why a property has been disabled,
* Sdd color property type,
* V4l2src is saving devices either by port, or by device id,
* Per quiddity custom state saving,
* Update INSTALL.md with additional Nvidia details,
* Added a mechanism to add a notification callback to a gstreamer property,
* Notify the information tree when a gtk window gets the focus,
* Porting from GTK2 to GTK3.

Bug fixes:
* Fixed version numbering when minor or major version update,
* Fix missing spaces in INSTALL.md,
* Fix gtk3 warning message during configure,
* Fix default video encoder cannot handle full HD,
* Fix quid issue with long names,
* Fix SIP remote contact added even if call is refused,
* Clear all artifacts when stopping or disconnecting the source from a gtk window or when rotating the image,
* Clang doesn't accept to friend "class" something defined as a struct. Fixed OSX build,
* Move include to fix build on OSX.

switcher 0.8.26 (2016-09-12)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* Shmdata access rate is notified in the quiddity information tree, along with byte_rate.
* Make DNS in SIP configurable and get the system one by default instead of a hardcoded value.
* Audiotestsrc quiddity revamping : added properties (sample rate, channels number, audio format), modified gstreamer pipeline lifecycle.
* Cleaned up UGstElem lifecycle (better refcount management) and fixed PBag::replace so it doesn't reset the index of the selection.
* Adding more properties to videotestsrc and templating selection.
* Removing more codec option from gst-video-codec and gst-audio-codec.
* Added generic methods to get gstreamer elements caps values in gst::utils.
* Adding SIP whitelist.
* Using nvenc 7.
* Only record the shmdata connections state instead of each connect/disconnect command when saving a session.

Bug fixes
* Fix stun/turn from configuration file is deadlocking.
* SIP quiddity notifies when configuration is applied at initialization.
* Fix bug regarding multiple simultaneous nvenc sessions crashing, re-enabled nvenc test, code simplication.

switcher 0.8.24 (2016-08-31)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* URI/URL image player.

Bug fixes:
* Fixed shmdata size computation and disconnection method.
* Clarified timeout warning for SIP registration and reduced invite timeout.
* Catch gstreamer errors in URI player to avoid deadlocks.
* Removing hardcorded nvidia driver version in release script.
* Adding audio multichannel error message for issue with encoder.

switcher 0.8.22 (2016-08-17)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* H265 encoding/decoding with NVENC/NVDEC.
* gst-audio-codec cleanup

Bug fixes:
* Added mutexes for safety and fixed a crash in jack_on_process.
* Fixed shmdata lifecycle in nvdec plugin to fix a race condition.
* No initialization of CUVIDDECODECREATEINFO because it contains elements withtout default constructor.

switcher 0.8.20 (2016-08-05)
---------------------------
This is an official release in the 0.8 stable series.

* fix jacksrc cannot connect to jacksink

switcher 0.8.18 (2016-08-04)
---------------------------
This is an official release in the 0.8 stable series.

* save file: save quiddities and property values and history is saving only invocations,
* alphanum, space and '-' are only allowed chars in quiddity names, others are replaced by '-',
* saving and loading user_data tree with history,
* fix OSX compilation,
* fix sometime wrong property (Selection) value when generated from GStreamer element enum property,
* fix property notification stops after property replace,
* fix 2 nvenc stress test,
* fix audio encoder default value,
* fix property documentation not updated with replace,
* fix gtkwin xevent to shmdata,
* fix a type mismatch in the sip_port property.

switcher 0.8.16 (2016-07-20)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* Implementation of hardware decoding with nvcuvid.
* Port to NVIDIA Codec SDK 6 and build cleanup.
* SIP quiddity exposes decompressed streams (default) or compressed streams,

Bug fixes:
* fix SIP status cannot go from offline to something else
* fix issues with video converter quid (making gst-pixel-format-converter RAII)
* fix nvenc test
* notifying user that no more nvenc quiddity can be created,
* increasing number of simultaneous media in SIP/SDP,

switcher 0.8.14 (2016-07-06)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* Fixed stun negotiation for SIP. NAT traversal works now
* c++14 support
* Posture_Source: added new filtering parameters (bilateral filter and hole filling)

Bug fixes:
* prevent self call through SIP
* fix sometimes failing check_rtp_session
* reduce registration timeout to 3 seconds when server name is wrong
* message to user when SIP port cannot be bound plus other messages
* add pixel-aspect-ratio to caps in posture_source.cpp
* fix v4l2src issue when error from the GStreamer pipeline

switcher 0.8.12 (2016-06-21)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* Generic decoder quiddity
* PostureScan3DGPU: added support for multicore compression,
* Reflecting modifications to PostureVision to support RS cameras,

Bug fixes:
* Fixed regex for version and set git merge strategy to 'theirs' in release script.
* Fix bug with threaded wrapper sync task
* jack fixes and jack server is starting if not running,
* using pjsip 2.5.1 instead of 2.2.1
* Created a script to parse and print translatable error messages from switcher to scenic.
* Use g_message to give information to the interface from switcher (temporary solution awaiting quiddity manager refactoring).
* Fix SIGSEGV and SIGABRT errors happening during property changes (race conditions). 
* Hang up incoming call after outgoing call in case of self-call, otherwise shmdata would not be disconnected properly when hanging up.
* X events capture is now optional for gtk window.
* Disable nvenc options when an encoding session is ongoing.
* Manage error case in property subscription mechanism.

switcher 0.8.10 (2016-06-08)
---------------------------
This is an official release in the 0.8 stable series.

New features:
* GTK geometry saving
* Added per-quiddity config
* Per-quiddity user tree
* Posture: Added a compress option to Scan3DGPU
* Add autoconnect feature to jack source and output.
* Reformatted code base with clang-format.
* untested syphonsrc port to shmdata1
* Posture: added PostureColorizeGL

Bug fixes:
* do not call contact if already calling it 
* Release script, usable by shmdata and switcher.
* fix nvenc 2Kx2K transmission with SIP
* adding a more complete test for nvenc plugin
* fix h264 in gst codec
* sip hang up is hanging up outgoing call or incoming call, but not both in the same invocation
* pjsip do not assert if cannot answer to call
* Fixed SIP connections graft/prune and empty array serialization for the tree
* Fixed an issue where only the last shmdata reader was updated in the info tree.
* Destroy "logger" quiddity last to keep proper formatting until the end.
* fix systemusage period property
* Fixed deadlocks and crashes when reloading SIP scenarii.
* Switch from hybrid sync/async messaging system in gst-pipeliner to using gstreamer watch mechanism.
* errors are fatal and crash the app (SIGTRAP), changed to a warning with error in the message.
* fix nvenc max number of simultaneous encode sessions reached
* Fixed a crash on disconnection of a source during a SIP call and an accidental creation of a reader key.
* fix video converter crash for large video,
* fix new call from already calling contact,
* fix load history file including create returning null
* Precommit hook now proposes to do the formatting automatically on the changes. 
* Added PostureMeshSerialization quiddity
* Update video caps on-the-fly when selecting them in Scenic.
* fix switcher-ctrl, renaming get_factory_capabilities in get_classes,
* disabling soap set_port after first invocation
* v4l2 shmdata writer is initialized with a larger value in order to avoid automatic resize for the first frame.
* v4l2src uses video-encoded suffix when the device produces non raw video
* GCC 5.3 is more restrictive than 5.1 and is used in Ubuntu 16.04. Soap needed an additional warning exception to build properly.
* fix fullscreen button latency with gtkwin
* fix gtk win properties when no video is connected
* fix gtk window shortcuts
* fix jack-to-shmdata error when deleted
* fix issue in shmdata-to-jack
* fix issue when property not found with soap
* fix missing arg issue with switcher-ctrl
* make timelapse multi shmreader
* num files and dir and nome parameters for timelapse
* max files in timelapse
* timelapse params can be changed live
* width and height in timelapse
* timelapse notifies last image in a read only property
* fix default video encoding is ugly on ubuntu 15.10
* fix seg when trying to register with network and without STUN/TURN
* fix v4l2 trying to compress video when camera is producing compressed video
* fix SIP cannot send to multiple destinations
* fix SIP call issue when no media add to SDP
* fix gtk property issues
* fix nvenc bug when start/stop input video
* fix lost property values for video-codec when start stop
* fix v4l2src capture configuration lost after start/stop
* fix gtk dtor bug
* fix some timelapse internals
* fix distcheck with posture
* PostureScan3DGPU: added preliminary support for mesh refinement
* PostureScan3DGPU: updated some limit values
* Posture: now checking in PostureSource and PostureScan3DGPU that the calibration file is alright
* more changes trying to fix posture transmission
* (pjsip) ports only determined by ICE
* removing several FIXME in pj-sip plugins
* removing some FIXMEs
* some simplification of pjsip plugin code
* fix Stack smashing detected when using stun with 4 shmdata
* fix race condition in rtpsender
* fix generated quid names collision after loading
* fix issue with saving/loading
* fix SIP shmdata byte_rate monitoring
* fix sip compilation with g++ 5.2
* fix missing string with g_warnings
* fix conflict with posture plugin
* fix duplicate byte rate info for shmdata writer in httpsdpdec
* Improved posture plugin threading
* removing old sockfd fix for OSX
* fix pruning with clang
* activating syphon for being ported to shmdata1
* fix check startable
* fix race condition with GstSubscriber
* no wait for state when asking PLAYING or PAUSED state
* disabling GTK on OSX
* add missing include for errno
* fix clang issue with gchar vs. string
* removing curl dependency
* removed hard coded switcher version in switcher-nodejs

switcher 0.8.8 (2016-04-04)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* video timelapse for multiple video shmdata
* option to number files or not
* max number of file for timelapse
* width and height in timelapse
* timelapse notifies last file written

Bugs fixed:

* default video encoding ugly on ubuntu 15.10
* segfault when trying to register with network and without STUN/TURN
* v4l2 trying to compress video when camera is producing compressed video
* SIP cannot send to multiple destinations
* SIP call issue when no media add to SDP
* gtk property issues
* nvenc bug when start/stop input video
* lost property values for video-codec when start stop
* v4l2src capture configuration lost after start/stop
* gtk dtor bug

switcher 0.8.6 (2016-03-21)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* property system refactored
* posture GPU
* nvenc support
* STUN/TURN support
* video timelapse

Bugs fixed:

* too many to be listed here

switcher 0.8.4 (2015-08-18)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* none

Bugs fixed:

* remove dependency to curl (not working with launchpad ppa)

switcher 0.8.2 (2015-08-17)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* none

Bugs fixed:

* switcher plugin not loaded from nodejs addon

switcher 0.8.0 (2015-08-17)
---------------------------
This is an official release in the 0.8 stable series. 

New features:

* SIP
* VNC (compatible with gtkwin)
* jack (resampling + autoconnection)
* audio encoder
* more option with v4l2 capture
* ported to gstreamer 1.0
* ported to shmdata 1.0
* using internal information tree

Bugs fixed:

* too many to be listed here

switcher 0.6.2 (2014-08-04)
---------------------------
This is an official release in the 0.6 stable series. 

New features:

* none
 
Bugs fixed:

* nodejs not loading plugins

switcher 0.6.0 (2014-08-04)
---------------------------
This is an official release in the 0.6 stable series. 

New features:

* information tree
* syphon (OSX)
* improved per-quiddity shmdata dynamic description
* shmdata-any support
* system usage quiddity
* SIP
* posture plugins
 
Bugs fixed:

* too many for being listed here 

switcher 0.4.6 (2014-03-07)
---------------------------
This is an official release in the 0.4 stable series. 

New features:

* none

Bugs fixed:

* node-switcher is better integrated with nodejs

switcher 0.4.4 (2014-03-05)
---------------------------
This is an official release in the 0.4 stable series. 

New features:

* adding node-switchon addons in sources

Bugs fixed:

* fix segfault in soap-ctrl-client
* make node-switcher encoding strings in utf8 instead of ascii

switcher 0.4.2 (2014-02-27)
---------------------------
This is an official release in the 0.4 stable series. 

New features:

* title property for gtkvideosink
* adding videoflip, video balance and gamma to gtk video sink

Bugs fixed:

* too many for being listed here

switcher 0.4.0 (2014-01-28)
---------------------------
This is an official release in the 0.4 stable series. 

New features:

* many video encoders are available with video-sources 
* dynamic presence of properties
* property mapper
* supporting plugins
* gtk video sink with fullscreen
* portmidi plugins
* pulse plugins
* v4l2 plugins
* dictionnary
* osc controler
* several internal features for developing quiddities
* no need to invoke set_runtime anymore

Bugs fixed:

* too many for being listed here

switcher 0.2.2 (2013-06-20)
---------------------------
This is an official release in the 0.2 stable series. 

New features:

* none

Bugs fixed:

* gsoap packaging

switcher 0.2.0 (2013-06-19)
---------------------------

This is the first official release in the 0.2 stable series. 

New features:

* signals
* save/load command history
* soap client

Bugs fixed:

* too many to listed here

switcher 0.1.2 (2013-04-26)
---------------------------
This is the first developer snapshot of switcher.

