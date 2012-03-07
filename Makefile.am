# "foreign" means not GNU, and is common for avoiding boring messages
# about files organized differently from what GNU expects

ACLOCAL_AMFLAGS = -I m4
AUTOMAKE_OPTIONS = foreign

SUBDIRS = \
    shmdata \
    examples 

EXTRA_DIST = \
    LICENSE \
    README

# pkg-config file:
DISTCLEANFILES = libshmdata.pc
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libshmdata-@LIBSHMDATA_API_VERSION@.pc

html-local: htmldoc
htmldoc:
	@if [ -z `which doxygen` ]; then echo "No Doxygen!"; else doxygen; fi

# Documentation:

if HAVE_DOXYGEN

doxyfile.stamp:
	$(DOXYGEN) doxyfile
	echo `date -R` > doxyfile.stamp

CLEANFILES = doxyfile.stamp

all-local: doxyfile.stamp

clean-local:
	rm -rf html

endif 


