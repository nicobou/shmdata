AM_CXXFLAGS = \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir)/switcher/webservices

AM_LIBS = \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS)

lib_LTLIBRARIES = \
    libswitcher-@LIBSWITCHER_API_VERSION@.la

libswitcher_@LIBSWITCHER_API_VERSION@_la_SOURCES = \
    aac.cpp \
    audio-sink.cpp \
    audio-source.cpp \
    audio-test-source.cpp \
    aravis-genicam.cpp \
    decodebin2.cpp \
    quiddity-command.cpp \
    quiddity-documentation.cpp \
    quiddity-life-manager.cpp \
    quiddity-manager.cpp \
    quiddity-manager-wrapper.cpp \
    quiddity.cpp \
    base-sink.cpp \
    base-source.cpp \
    soap-ctrl-server.cpp \
    gconf-audio-sink.cpp \
    gconf-audio-source.cpp \
    gconf-video-sink.cpp \
    gconf-video-source.cpp \
    gst-element-cleaner.cpp \
    gst-utils.cpp \
    h264.cpp \
    http-sdp.cpp \
    json-builder.cpp \
    method.cpp \
    property.cpp \
    pulse-sink.cpp \
    rtp-destination.cpp \
    rtp-session.cpp \
    runtime.cpp \
    segment.cpp \
    shmdata-reader.cpp \
    shmdata-writer.cpp \
    udpsink.cpp \
    uridecodebin.cpp \
    video-sink.cpp \
    video-source.cpp \
    video-test-source.cpp \
    webservices/soapC.cpp \
    webservices/soapcontrolProxy.cpp \
    webservices/soapcontrolService.cpp \
    xvimagesink.cpp

libswitcher_@LIBSWITCHER_API_VERSION@_la_LDFLAGS = \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS) \
    -version-info $(LIBSWITCHER_SO_VERSION)

libswitcher_@LIBSWITCHER_API_VERSION@_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libswitcher_@LIBSWITCHER_API_VERSION@_la_include_HEADERS = \
    aac.h \
    abstract-factory.h \
    abstract-factory_spec.h \
    audio-test-source.h \
    audio-sink.h \
    audio-source.h \
    aravis-genicam.h \
    decodebin2.h \
    quiddity-command.h \
    quiddity-documentation.h \
    quiddity-life-manager.h \
    quiddity-manager.h \
    quiddity-manager-wrapper.h \
    quiddity.h \
    base-sink.h \
    base-source.h \
    creator.h \
    soap-ctrl-server.h \
    ctrl-webservice-definition.h \
    gconf-audio-sink.h \
    gconf-audio-source.h \
    gconf-video-sink.h \
    gconf-video-source.h \
    gst-element-cleaner.h \
    gst-utils.h \
    h264.h \
    http-sdp.h \
    hash-table.h \
    hash-table_spec.h \
    json-builder.h \
    method.h \
    property.h \
    pulse-sink.h \
    rtp-destination.h \
    rtp-session.h \
    runtime.h \
    segment.h \
    shmdata-reader.h \
    shmdata-writer.h \
    string-map.h \
    string-map_spec.h \
    udpsink.h \
    uridecodebin.h \
    video-sink.h \
    video-source.h \
    video-test-source.h \
    xvimagesink.h

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

####### On OSX, gsoap is not installed in /usr (if installed through MacPorts)
if HAVE_OSX
gsoap_INCLUDE = /opt/local/share/gsoap/import/
else
gsoap_INCLUDE = /usr/share/gsoap/import/
endif

webservices/soapC.cpp: ctrl-webservice-definition.h webservices
	soapcpp2 -I $(gsoap_INCLUDE) -i -d webservices ctrl-webservice-definition.h

webservices/soapcontrolProxy.cpp: ctrl-webservice-definition.h webservices
	soapcpp2 -I $(gsoap_INCLUDE) -i -d webservices ctrl-webservice-definition.h

webservices/soapcontrolService.cpp: ctrl-webservice-definition.h webservices
	soapcpp2 -I $(gsoap_INCLUDE) -i  -d webservices ctrl-webservice-definition.h

webservices:
	mkdir -p webservices

####### end webservice code generation 

# cleaning generated files for webservices
clean-local:
	-rm -f webservices/*

