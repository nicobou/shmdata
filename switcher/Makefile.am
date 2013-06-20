AM_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(LIBLO_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir)/switcher/webservices

AM_LIBS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(LIBLO_LIBS) \
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
    custom-property-helper.cpp \
    create-remove-spy.cpp \
    decodebin2.cpp \
    deinterleave.cpp \
    fake-shmdata-writer.cpp \
    file-sdp.cpp \
    fakesink.cpp \
    gobject-custom-property.cpp \
    gobject-wrapper.cpp \
    gst-element-cleaner.cpp \
    gst-parse-to-bin-src.cpp \
    gst-video-parse-to-bin-src.cpp \
    gst-utils.cpp \
    h264.cpp \
    http-sdp.cpp \
    http-sdp-dec.cpp \
    json-builder.cpp \
    logger.cpp \
    method.cpp \
    osc-ctrl-server.cpp \
    property.cpp \
    pulse-sink.cpp \
    quiddity-command.cpp \
    quiddity-documentation.cpp \
    quiddity-manager-wrapper.cpp \
    quiddity-manager.cpp \
    quiddity-manager-impl.cpp \
    quiddity-property-subscriber.cpp \
    quiddity-signal-subscriber.cpp \
    quiddity.cpp \
    rtp-destination.cpp \
    rtp-session.cpp \
    runtime.cpp \
    segment.cpp \
    shmdata-from-gdp-file.cpp \
    shmdata-reader.cpp \
    shmdata-to-file.cpp \
    shmdata-writer.cpp \
    signal-string.cpp \
    soap-ctrl-client.cpp \
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
    $(LIBLO_LIBS) \
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
    create-remove-spy.h \
    creator.h \
    ctrl-webservice-definition.h \
    custom-property-helper.h \
    decodebin2.h \
    deinterleave.h \
    fake-shmdata-writer.h \
    fakesink.h \
    file-sdp.h \
    gobject-custom-property.h \
    gobject-wrapper.h \
    gst-element-cleaner.h \
    gst-parse-to-bin-src.h \
    gst-video-parse-to-bin-src.h \
    gst-utils.h \
    h264.h \
    hash-table.h \
    hash-table_spec.h \
    http-sdp.h \
    http-sdp-dec.h \
    json-builder.h \
    logger.h \
    method.h \
    osc-ctrl-server.h \
    property.h \
    pulse-sink.h \
    quiddity-command.h \
    quiddity-documentation.h \
    quiddity-manager-wrapper.h \
    quiddity-manager.h \
    quiddity-manager-impl.h \
    quiddity-property-subscriber.h \
    quiddity-signal-subscriber.h \
    quiddity.h \
    rtp-destination.h \
    rtp-session.h \
    runtime.h \
    segment.h \
    shmdata-from-gdp-file.h \
    shmdata-reader.h \
    shmdata-to-file.h \
    shmdata-writer.h \
    signal-string.h \
    soap-ctrl-client.h \
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
