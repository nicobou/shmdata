bindir = $(prefix)/lib/node_modules/node-switcher/@LIBSWITCHER_API_VERSION@/

bin_PROGRAMS = \
    build/Release/switcher_addon.node

build/Release/switcher_addon.node:
	node-gyp configure && node-gyp build

install-exec-local:
	node-gyp install

clean-local:
	node-gyp clean