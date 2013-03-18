AM_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir)/switcher/webservices

AM_LIBS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(SHMDATA_LIBS)

lib_LTLIBRARIES = \
    libswitcher-@LIBSWITCHER_API_VERSION@.la

libswitcher_@LIBSWITCHER_API_VERSION@_la_SOURCES = \
    aac.cpp \
    aravis-genicam.cpp \
    audio-sink.cpp \
    audio-source.cpp \
    audio-test-source.cpp \
    base-sink.cpp \
    base-source.cpp \
    decodebin2.cpp \
    deinterleave.cpp \
    file-sdp.cpp \
    gobject-custom-property.cpp \
    gobject-wrapper.cpp \
    gst-element-cleaner.cpp \
    gst-parse-to-bin-src.cpp \
    gst-utils.cpp \
    h264.cpp \
    http-sdp.cpp \
    json-builder.cpp \
    method.cpp \
    property.cpp \
    pulse-sink.cpp \
    quiddity-command.cpp \
    quiddity-documentation.cpp \
    quiddity-life-manager.cpp \
    quiddity-manager-wrapper.cpp \
    quiddity-manager.cpp \
    quiddity.cpp \
    rtp-destination.cpp \
    rtp-session.cpp \
    runtime.cpp \
    segment.cpp \
    shmdata-reader.cpp \
    shmdata-writer.cpp \
    soap-ctrl-server.cpp \
    udpsink.cpp \
    uridecodebin.cpp \
    uris.cpp \
    video-rate.cpp \
    video-sink.cpp \
    video-source.cpp \
    video-test-source.cpp \
    vorbis.cpp \
    webservices/soapC.cpp \
    webservices/soapcontrolProxy.cpp \
    webservices/soapcontrolService.cpp \
    xvimagesink.cpp

libswitcher_@LIBSWITCHER_API_VERSION@_la_LDFLAGS = \
    $(GLIB_LIBS) \
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
    aravis-genicam.h \
    audio-sink.h \
    audio-source.h \
    audio-test-source.h \
    base-sink.h \
    base-source.h \
    creator.h \
    ctrl-webservice-definition.h \
    decodebin2.h \
    deinterleave.h \
    file-sdp.h \
    gobject-custom-property.h \
    gobject-wrapper.h \
    gst-element-cleaner.h \
    gst-parse-to-bin-src.h \
    gst-utils.h \
    h264.h \
    hash-table.h \
    hash-table_spec.h \
    http-sdp.h \
    json-builder.h \
    method.h \
    property.h \
    pulse-sink.h \
    quiddity-command.h \
    quiddity-documentation.h \
    quiddity-life-manager.h \
    quiddity-manager-wrapper.h \
    quiddity-manager.h \
    quiddity.h \
    rtp-destination.h \
    rtp-session.h \
    runtime.h \
    segment.h \
    shmdata-reader.h \
    shmdata-writer.h \
    soap-ctrl-server.h \
    string-map.h \
    string-map_spec.h \
    udpsink.h \
    uridecodebin.h \
    uris.h \
    video-rate.h \
    video-sink.h \
    video-source.h \
    video-test-source.h \
    vorbis.h \
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

