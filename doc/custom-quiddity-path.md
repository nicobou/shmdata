# Switcher quiddity plugins 

Most Quiddities are compiled as shared object files and loaded by switcher during instantiation. When the switcher library is loaded, plugins are loaded from the default plugin path, where switcher installation copies these shared object files. These files are usually located at `/usr/local/switcher-<major>.<minor>/plugins/`, where major and minor  depends on the switcher version. For instance, with switcher version 2.1.23, major is 2 and minor is 1.

# Loading quiddity plugins from custom path

Custom path can be given to switcher using the `SWITCHER_PLUGIN_PATH` environment variable. Multiple paths can be specified, and must be separated with a `:`. Here follows some examples:

```bash
# tell switcher to scan directory and sub-directories for switcher plugins
SWITCHER_PLUGIN_PATH=/tmp/my_switcher_plugins/ switcher

# give several sub-directories
SWITCHER_PLUGIN_PATH=/tmp/first_folder/:/tmp/second_folder/ switcher

# This work also with your python scripts
SWITCHER_PLUGIN_PATH=/tmp/my_switcher_plugins/ python3 ./my_switcher_script.py
```
