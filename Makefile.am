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

html-local: htmldoc
htmldoc:
	@if [ -z `which doxygen` ]; then echo "No Doxygen!"; else doxygen; fi

# Documentation:

if HAVE_DOXYGEN

doxyfile.stamp:
	echo "/*! \mainpage Switcher for audio, video, data and network streams " > doxymain.h
	echo " * \verbatim" >> doxymain.h
	cat $(srcdir)/README >> doxymain.h
	echo " \endverbatim" >> doxymain.h
	echo " */" >> doxymain.h
	$(DOXYGEN) doxyfile
	echo `date -R` > doxyfile.stamp

CLEANFILES = doxyfile.stamp

all-local: doxyfile.stamp

clean-local:
	rm -rf html

endif 

