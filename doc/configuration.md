Configuration
=======

It is possible to configure the default behaviour of switcher quiddities for your specific instance of Scenic. Such configurations should go in a default file named `switcher.json` and saved to `$HOME/.scenic/switcher.json`.

A specific configuration is forwarded to related quiddities at their creation. It is important to understand that this is *not* a way of setting default values for the *properties* but rather a way to define keys used by each quiddity in its own way. As of now, only the bundles, glfwin and pjsip have custom configuration keys.

This file is also the place where bundles will be defined (documentation to come soon). The example file [dummy-switcher.json](doc/dummy-switcher.json) shows all the possible keys used by switcher quiddities, along with an example bundle.
