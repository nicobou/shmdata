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
    audio-source.cpp \
    audio-test-source.cpp \
    categorizable.cpp \
    counter-map.cpp \
    create-remove-spy.cpp \
    custom-property-helper.cpp \
    decodebin-to-shmdata.cpp \
    decodebin2.cpp \
    default-video-format.cpp \
    fake-shmdata-writer.cpp \
    fakesink.cpp \
    g-source-wrapper.cpp \
    glibmainloop.cpp \
    gobject-custom-property.cpp \
    gobject-wrapper.cpp \
    gst-element-cleaner.cpp \
    gst-parse-to-bin-src.cpp \
    gst-pipe.cpp \
    gst-pipeliner.cpp \
    gst-utils.cpp \
    gst-video-parse-to-bin-src.cpp \
    http-sdp-dec.cpp \
    information-tree-basic-serializer.cpp \
    information-tree-json.cpp \
    information-tree.cpp \
    jack-audio-source.cpp \
    json-builder.cpp \
    logger.cpp \
    method.cpp \
    on-caps.cpp \
    plugin-loader.cpp \
    port-checker.cpp \
    property-mapper.cpp \
    property.cpp \
    quiddity-basic-test.cpp \
    quiddity-command.cpp \
    quiddity-documentation.cpp \
    quiddity-manager-impl.cpp \
    quiddity-manager-wrapper.cpp \
    quiddity-manager.cpp \
    quiddity-property-subscriber.cpp \
    quiddity-signal-subscriber.cpp \
    quiddity.cpp \
    rtp-destination.cpp \
    rtp-session.cpp \
    safe-bool-idiom.cpp \
    sdp-utils.cpp \
    segment.cpp \
    shmdata-any-reader.cpp \
    shmdata-any-writer.cpp \
    shmdata-from-gdp-file.cpp \
    shmdata-reader.cpp \
    shmdata-to-file.cpp \
    shmdata-writer.cpp \
    signal-string.cpp \
    single-pad-gst-sink.cpp \
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
    audio-source.hpp \
    audio-test-source.hpp \
    categorizable.hpp \
    clock.hpp \
    counter-map.hpp \
    create-remove-spy.hpp \
    creator.hpp \
    custom-property-helper.hpp \
    decodebin-to-shmdata.hpp \
    decodebin2.hpp \
    default-video-format.hpp \
    fake-shmdata-writer.hpp \
    fakesink.hpp \
    g-source-wrapper.hpp \
    glibmainloop.hpp \
    gobject-custom-property.hpp \
    gobject-wrapper.hpp \
    gst-element-cleaner.hpp \
    gst-parse-to-bin-src.hpp \
    gst-pipe.hpp \
    gst-pipeliner.hpp \
    gst-utils.hpp \
    gst-video-parse-to-bin-src.hpp \
    http-sdp-dec.hpp \
    information-tree-basic-serializer.hpp \
    information-tree-json.hpp \
    information-tree.hpp \
    jack-audio-source.hpp \
    json-builder.hpp \
    logger.hpp \
    make-consultable.hpp \
    method.hpp \
    on-caps.hpp \
    plugin-loader.hpp \
    port-checker.hpp \
    property-mapper.hpp \
    property.hpp \
    quiddity-basic-test.hpp \
    quiddity-command.hpp \
    quiddity-documentation.hpp \
    quiddity-manager-impl.hpp \
    quiddity-manager-wrapper.hpp \
    quiddity-manager.hpp \
    quiddity-property-subscriber.hpp \
    quiddity-signal-subscriber.hpp \
    quiddity.hpp \
    rtp-destination.hpp \
    rtp-session.hpp \
    safe-bool-idiom.hpp \
    scope-exit.hpp \
    sdp-utils.hpp \
    segment.hpp \
    shmdata-any-reader.hpp \
    shmdata-any-writer.hpp \
    shmdata-from-gdp-file.hpp \
    shmdata-reader.hpp \
    shmdata-to-file.hpp \
    shmdata-writer.hpp \
    signal-string.hpp \
    single-pad-gst-sink.hpp \
    startable-quiddity.hpp \
    std2.hpp \
    string-dictionary.hpp \
    udpsink.hpp \
    unique-gst-element.hpp \
    uridecodebin.hpp \
    video-source.hpp \
    video-test-source.hpp \
    xvimagesink.hpp
