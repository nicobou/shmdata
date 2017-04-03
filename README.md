switcher
========

[![build status](https://gitlab.com/sat-metalab/switcher/badges/master/build.svg)](https://gitlab.com/sat-metalab/switcher/commits/master)

[switcher](https://github.com/sat-metalab/switcher) provides audio, video and data signals routing, switching, processing and mixing. It was created at the Society for Arts and Technology (SAT) to give to artists a powerful tool for telepresence in contexts of live arts and new media installations.

See instructions for [installing](doc/INSTALL.md).

See instructions for [coding](doc/CODING.md).

Configuration
------

It is possible to configure the default behaviour of switcher quiddities for your specific instance of Scenic. Such configurations should go in a default file named `switcher.json` and saved to `$HOME/.scenic/switcher.json`.

This file will be read automatically and parsed in all the quiddities making use of it. It is important to understand that this is *not* a way of setting default values for the *properties* but rather a way to define keys used by each quiddity in its own way. As of now, only the bundles, glfwin and pjsip have custom configuration keys.

This file is also the place where bundles will be defined (documentation to come soon). The example file [dummy-switcher.json](doc/dummy-switcher.json) shows all the possible keys used by switcher quiddities, along with an example bundle.

License
-------
switcher is released under the terms of the GNU GPL v3 or above.
libswitcher is released under the terms of the GNU LGPL v2 or above.
switcher plugins licenses are per plugin, see plugin folders.

In addition, the developers of the switcher hereby grants permission for non-GPL compatible GStreamer plugins to be used and distributed together with switcher. This permission is above and beyond the permissions granted by the GPL license by which switcher is covered. If you modify this code, you may extend this exception to your version of the code, but you are not obligated to do so. If you do not wish to do so, delete this exception statement from your version.

Some optional parts of switcher are licensed under the GNU General Public License
version 2 or later (GPL v2+). None of these parts are used by default, you have to explicitly pass -DENABLE\_GPL=ON to cmake to activate them. In this case, libswitcher's license changes to GPL v2+.

Specifically, the GPL parts of libswitcher are:
switcher-pjsip
switcher-v4l2
switcher-gsoap
switcher-ctrl
 
Sponsors
--------
This project is made possible thanks to the Society for Arts and Technologies. [SAT](http://www.sat.qc.ca/) and to the Ministère du Développement économique, de l'Innovation et de l'Exportation du Québec (MDEIE).

