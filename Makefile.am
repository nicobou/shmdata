SUBDIRS= \
    switcher \
    plugins \
    tests 

if SWITCHERBUILD
SUBDIRS += \
    src
endif

if NODEADDONBUILD
SUBDIRS += \
    nodejs
endif

EXTRA_DIST=autogen.sh

# pkg-config file:
DISTCLEANFILES = \
    doxymain.h \
    switcher-@LIBSWITCHER_API_VERSION@.pc \
    switcher-@LIBSWITCHER_API_VERSION@.tar.gz 

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = \
    switcher-@LIBSWITCHER_API_VERSION@.pc 

# Documentation:
html-local: htmldoc
htmldoc:
	@if [ -z `which doxygen` ]; then echo "No Doxygen!"; else doxygen; fi
doc: htmldoc

if HAVE_DOXYGEN
doxyfile.stamp: doxyfile.in
	$(DOXYGEN) doxyfile
	echo `date -R` > doxyfile.stamp

CLEANFILES = doxyfile.stamp

all-local: doxyfile.stamp

clean-local:
	rm -rf html

endif 

