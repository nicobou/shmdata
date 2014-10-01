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
    glibmainloop.cpp \
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
    abstract-factory.hpp \
    abstract-factory_spec.hpp \
    any.hpp \
    aravis-genicam.hpp \
    audio-source.hpp \
    audio-test-source.hpp \
    single-pad-gst-sink.hpp \
    categorizable.hpp \
    clock.hpp \
    const-methods-invoker.hpp \
    counter-map.hpp \
    create-remove-spy.hpp \
    creator.hpp \
    custom-property-helper.hpp \
    decodebin2.hpp \
    decodebin-to-shmdata.hpp \
    deinterleave.hpp \
    fake-shmdata-writer.hpp \
    fakesink.hpp \
    file-sdp.hpp \
    glibmainloop.hpp \
    gobject-custom-property.hpp \
    gobject-wrapper.hpp \
    gpipe.hpp \
    gst-element-cleaner.hpp \
    gst-parse-to-bin-src.hpp \
    gst-video-parse-to-bin-src.hpp \
    gst-utils.hpp \
    hash-table.hpp \
    hash-table_spec.hpp \
    http-sdp.hpp \
    http-sdp-dec.hpp \
    information-tree.hpp \
    information-tree-basic-serializer.hpp \
    information-tree-json.hpp \
    jack-audio-source.hpp \
    jack-sink.hpp \
    json-builder.hpp \
    logger.hpp \
    method.hpp \
    on-caps.hpp \
    plugin-loader.hpp \
    property.hpp \
    property-mapper.hpp \
    quiddity-basic-test.hpp \
    quiddity-command.hpp \
    quiddity-documentation.hpp \
    quiddity-manager-wrapper.hpp \
    quiddity-manager.hpp \
    quiddity-manager-impl.hpp \
    quiddity-property-subscriber.hpp \
    quiddity-signal-subscriber.hpp \
    quiddity.hpp \
    rtp-destination.hpp \
    rtp-session.hpp \
    scope-exit.hpp \
    sdp-utils.hpp \
    segment.hpp \
    shmdata-any-writer.hpp \
    shmdata-any-reader.hpp \
    shmdata-from-gdp-file.hpp \
    shmdata-reader.hpp \
    shmdata-to-file.hpp \
    shmdata-writer.hpp \
    signal-string.hpp \
    startable-quiddity.hpp \
    string-dictionary.hpp \
    udpsink.hpp \
    unique-gst-element.hpp \
    uridecodebin.hpp \
    video-source.hpp \
    video-test-source.hpp \
    xvimagesink.hpp
