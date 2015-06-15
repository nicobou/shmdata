# "foreign" means not GNU, and is common for avoiding messages
# about files organized differently from what GNU expects

#ACLOCAL_AMFLAGS = -I m4
AUTOMAKE_OPTIONS = foreign

SUBDIRS = \
    shmdata \
    utils \
    tests \
    wrappers

if BUILD_GST
SUBDIRS += gst
endif 

EXTRA_DIST = \
    LICENSE \
    README

# pkg-config file:
DISTCLEANFILES = \
    doxymain.h \
    shmdata-@LIBSHMDATA_API_VERSION@.pc \
    shmdata-@LIBSHMDATA_API_VERSION@.tar.gz 

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = \
    shmdata-@LIBSHMDATA_API_VERSION@.pc  

html-local: htmldoc
htmldoc:
	@if [ -z `which doxygen` ]; then echo "No Doxygen!"; else doxygen; fi

# Documentation:

if HAVE_DOXYGEN

doxyfile.stamp:
	echo "/*! \mainpage Library for sharing data flows between processes" > doxymain.h
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


