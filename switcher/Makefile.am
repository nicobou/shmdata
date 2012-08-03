AM_CXXFLAGS = \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    -I$(top_srcdir)/switcher/webservices 

AM_LIBS = \
    $(GSOAP_LIBS) \
    $(GST_LIBS) 

lib_LTLIBRARIES = \
    libswitcher-@LIBSWITCHER_API_VERSION@.la

libswitcher_@LIBSWITCHER_API_VERSION@_la_SOURCES = \
    audio-sink.cpp \
    audio-source.cpp \
    base-entity-manager.cpp \
    base-entity.cpp \
    base-sink.cpp \
    base-source.cpp \
    connector.cpp \
    controller.cpp \
    ctrl-server.cpp \
    hash-table.cpp \
    method.cpp \
    property.cpp \
    runtime.cpp \
    segment.cpp \
    video-sink.cpp \
    video-source.cpp \
    video-test-source.cpp \
    webservices/soapC.cpp \
    webservices/soapcontrolProxy.cpp \
    webservices/soapcontrolService.cpp

libswitcher_@LIBSWITCHER_API_VERSION@_la_LDFLAGS = \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    -version-info $(LIBSWITCHER_SO_VERSION)

libswitcher_@LIBSWITCHER_API_VERSION@_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libswitcher_@LIBSWITCHER_API_VERSION@_la_include_HEADERS = \
    audio-sink.h \
    audio-source.h \
    base-entity-manager.h \
    base-entity.h \
    base-sink.h \
    base-source.h \
    connector.h \
    controller.h \
    creator.h \
    ctrl-server.h \
    ctrl-webservice-definition.h \
    hash-table.h \
    method.h \
    property.h \
    runtime.h \
    segment.h \
    video-sink.h \
    video-source.h \
    video-test-source.h

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

webservices/soapC.cpp: ctrl-webservice-definition.h webservices
	soapcpp2 -I /usr/share/gsoap/import/ -i -d webservices ctrl-webservice-definition.h

webservices/soapcontrolProxy.cpp: ctrl-webservice-definition.h webservices
	soapcpp2 -I /usr/share/gsoap/import/ -i -d webservices ctrl-webservice-definition.h

webservices/soapcontrolService.cpp: ctrl-webservice-definition.h webservices
	soapcpp2 -I /usr/share/gsoap/import/ -i  -d webservices ctrl-webservice-definition.h

webservices:
	mkdir -p webservices

####### end webservice code generation 

# cleaning generated files for webservices
clean-local:
	-rm -f webservices/*

