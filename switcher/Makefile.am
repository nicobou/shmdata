AM_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GST_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(PYTHON_CFLAGS) \
    $(SHMDATA_ANY_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) 

AM_LIBS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(PYTHON_LIBS) \
    $(SHMDATA_ANY_LIBS) \
    $(SHMDATA_LIBS)

lib_LTLIBRARIES = \
    libswitcher-@LIBSWITCHER_API_VERSION@.la

libswitcher_@LIBSWITCHER_API_VERSION@_la_SOURCES = \
    aravis-genicam.cpp \
    audio-source.cpp \
    audio-test-source.cpp \
    single-pad-gst-sink.cpp \
    categorizable.cpp \
    counter-map.cpp \
    custom-property-helper.cpp \
    create-remove-spy.cpp \
    decodebin2.cpp \
    decodebin-to-shmdata.cpp \
    deinterleave.cpp \
    fake-shmdata-writer.cpp \
    file-sdp.cpp \
    fakesink.cpp \
    gobject-custom-property.cpp \
    gobject-wrapper.cpp \
    gpipe.cpp \
    gst-element-cleaner.cpp \
    gst-parse-to-bin-src.cpp \
    gst-video-parse-to-bin-src.cpp \
    gst-utils.cpp \
    http-sdp.cpp \
    http-sdp-dec.cpp \
    information-tree.cpp \
    information-tree-basic-serializer.cpp \
    information-tree-json.cpp \
    jack-audio-source.cpp \
    jack-sink.cpp \
    json-builder.cpp \
    logger.cpp \
    method.cpp \
    on-caps.cpp \
    plugin-loader.cpp \
    property.cpp \
    property-mapper.cpp \
    quiddity-basic-test.cpp \
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
    sdp-utils.cpp \
    segment.cpp \
    shmdata-any-writer.cpp \
    shmdata-any-reader.cpp \
    shmdata-from-gdp-file.cpp \
    shmdata-reader.cpp \
    shmdata-to-file.cpp \
    shmdata-writer.cpp \
    signal-string.cpp \
    startable-quiddity.cpp \
    string-dictionary.cpp \
    udpsink.cpp \
    unique-gst-element.cpp \
    uridecodebin.cpp \
    video-source.cpp \
    video-test-source.cpp \
    xvimagesink.cpp

libswitcher_@LIBSWITCHER_API_VERSION@_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(PYTHON_LIBS) \
    $(SHMDATA_ANY_LIBS) \
    $(SHMDATA_LIBS) \
    -version-info $(LIBSWITCHER_SO_VERSION)

libswitcher_@LIBSWITCHER_API_VERSION@_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libswitcher_@LIBSWITCHER_API_VERSION@_la_include_HEADERS = \
    abstract-factory.h \
    abstract-factory_spec.h \
    any.h \
    aravis-genicam.h \
    audio-source.h \
    audio-test-source.h \
    single-pad-gst-sink.h \
    categorizable.h \
    clock.h \
    counter-map.h \
    create-remove-spy.h \
    creator.h \
    custom-property-helper.h \
    decodebin2.h \
    decodebin-to-shmdata.h \
    deinterleave.h \
    fake-shmdata-writer.h \
    fakesink.h \
    file-sdp.h \
    gobject-custom-property.h \
    gobject-wrapper.h \
    gpipe.h \
    gst-element-cleaner.h \
    gst-parse-to-bin-src.h \
    gst-video-parse-to-bin-src.h \
    gst-utils.h \
    hash-table.h \
    hash-table_spec.h \
    http-sdp.h \
    http-sdp-dec.h \
    information-tree.h \
    information-tree-basic-serializer.h \
    information-tree-json.h \
    jack-audio-source.h \
    jack-sink.h \
    json-builder.h \
    logger.h \
    method.h \
    on-caps.h \
    plugin-loader.h \
    property.h \
    property-mapper.h \
    quiddity-basic-test.h \
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
    scope-exit.h \
    sdp-utils.h \
    segment.h \
    shmdata-any-writer.h \
    shmdata-any-reader.h \
    shmdata-from-gdp-file.h \
    shmdata-reader.h \
    shmdata-to-file.h \
    shmdata-writer.h \
    signal-string.h \
    startable-quiddity.h \
    string-dictionary.h \
    udpsink.h \
    unique-gst-element.h \
    uridecodebin.h \
    video-source.h \
    video-test-source.h \
    xvimagesink.h
