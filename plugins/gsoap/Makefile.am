AM_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(W_NO_ERROR_UNUSED_PARAMETER) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) 

AM_LIBS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_CFLAGS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -module \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libsoapserver.la \
    libsoapclient.la 

#libsoapserver info
libsoapserver_la_SOURCES = \
    soap-ctrl-server.cpp \
    webservices/soapC.cpp \
    webservices/soapcontrolProxy.cpp \
    webservices/soapcontrolService.cpp 

libsoapserver_la_include_HEADERS = \
    ctrl-webservice-definition.h \
    soap-ctrl-server.h 

libsoapserver_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libsoapserver_la_LDFLAGS = \
    $(GSOAP_LIBS)

#libsoapclient info
libsoapclient_la_SOURCES = \
    soap-ctrl-client.cpp \
    webservices/soapC.cpp \
    webservices/soapcontrolProxy.cpp 

libsoapclient_la_include_HEADERS = \
    soap-ctrl-client.h

libsoapclient_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libsoapclient_la_LDFLAGS = \
    $(GSOAP_LIBS) 

####### webservice code generation into the "webservices" directory
####### note: soapcpp2 -q <namespace> -d <directory-where-goes-generated-cpp>
####### note: webservices/stdsoap2.cpp comes from gsoap sources and 
#######       is required here for linking. Maybe a better solution exists
#######       but did not find anything else. See:
#######       http://stackoverflow.com/questions/6340524/gsoap-linking-error

BUILT_SOURCES = \
    webservices/soapC.cpp \
    webservices/soapcontrolProxy.cpp \
    webservices/soapcontrolService.cpp

CLEANFILES = \
    webservices/*

DISTCLEANFILES = \
    webservices/*

####### On OSX, gsoap is not installed in /usr (if installed through MacPorts)
if HAVE_OSX
gsoap_INCLUDE = /opt/local/share/gsoap/import/
else
gsoap_INCLUDE = /usr/share/gsoap/import/
endif

webservices/soapC.cpp: Makefile
	mkdir -p webservices
	soapcpp2 -I $(gsoap_INCLUDE) -i -d webservices $(srcdir)/ctrl-webservice-definition.h

$(srcdir)/webservices/soapcontrolProxy.cpp: Makefile
	mkdir -p webservices
	soapcpp2 -I $(gsoap_INCLUDE) -i -d webservices $(srcdir)/ctrl-webservice-definition.h

$(srcdir)/webservices/soapcontrolService.cpp: Makefile
	mkdir -p webservices
	soapcpp2 -I $(gsoap_INCLUDE) -i  -d webservices $(srcdir)/ctrl-webservice-definition.h


## unit tests
AM_CFLAGS = \
    -I$(top_srcdir)

AM_CXXFLAGS =  \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) 

AM_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

check_PROGRAMS = \
    check_gsoap_control

check_gsoap_control_SOURCES = \
    check_gsoap_control.cpp

TESTS = $(check_PROGRAMS)
