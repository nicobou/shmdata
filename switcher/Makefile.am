AM_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GST_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(PYTHON_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) 

AM_LIBS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(PYTHON_LIBS) \
    $(SHMDATA_LIBS)

lib_LTLIBRARIES = \
    libswitcher-@LIBSWITCHER_API_VERSION@.la

libswitcher_@LIBSWITCHER_API_VERSION@_la_SOURCES = \
    audio-test-source.cpp \
    categorizable.cpp \
    counter-map.cpp \
    create-remove-spy.cpp \
    custom-property-helper.cpp \
    decodebin-to-shmdata.cpp \
    external-shmdata-writer.cpp \
    g-source-wrapper.cpp \
    glibmainloop.cpp \
    gobject-custom-property.cpp \
    gobject-wrapper.cpp \
    gst-audio-codec.cpp \
    gst-audio-encoder.cpp \
    gst-pipe.cpp \
    gst-pipeliner.cpp \
    gst-pixel-format-converter.cpp \
    gst-shmdata-subscriber.cpp \
    gst-utils.cpp \
    gst-video-codec.cpp \
    gst-video-converter.cpp \
    gst-video-encoder.cpp \
    http-sdp-dec.cpp \
    information-tree-basic-serializer.cpp \
    information-tree-json.cpp \
    information-tree.cpp \
    json-builder.cpp \
    logger.cpp \
    method.cpp \
    net-utils.cpp \
    periodic-task.cpp \
    plugin-loader.cpp \
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
    shmdata-connector.cpp \
    shmdata-follower.cpp \
    shmdata-utils.cpp \
    shmdata-writer.cpp \
    signal-string.cpp \
    startable-quiddity.cpp \
    string-dictionary.cpp \
    string-utils.cpp \
    unique-gst-element.cpp \
    uridecodebin.cpp \
    video-test-source.cpp

libswitcher_@LIBSWITCHER_API_VERSION@_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(PYTHON_LIBS) \
    $(SHMDATA_LIBS) \
    -version-info $(LIBSWITCHER_SO_VERSION)

libswitcher_@LIBSWITCHER_API_VERSION@_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libswitcher_@LIBSWITCHER_API_VERSION@_la_include_HEADERS = \
    abstract-factory.hpp \
    abstract-factory_spec.hpp \
    any.hpp \
    audio-test-source.hpp \
    categorizable.hpp \
    counter-map.hpp \
    create-remove-spy.hpp \
    creator.hpp \
    custom-property-helper.hpp \
    decodebin-to-shmdata.hpp \
    external-shmdata-writer.hpp \
    g-source-wrapper.hpp \
    glibmainloop.hpp \
    gobject-custom-property.hpp \
    gobject-wrapper.hpp \
    gst-audio-codec.hpp \
    gst-audio-encoder.hpp \
    gst-pipe.hpp \
    gst-pipeliner.hpp \
    gst-pixel-format-converter.hpp \
    gst-shmdata-subscriber.hpp \
    gst-utils.hpp \
    gst-video-codec.hpp \
    gst-video-converter.hpp \
    gst-video-encoder.hpp \
    http-sdp-dec.hpp \
    information-tree-basic-serializer.hpp \
    information-tree-json.hpp \
    information-tree.hpp \
    json-builder.hpp \
    logger.hpp \
    make-consultable.hpp \
    method.hpp \
    net-utils.hpp \
    periodic-task.hpp \
    plugin-loader.hpp \
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
    shmdata-connector.hpp \
    shmdata-glib-logger.hpp \
    shmdata-follower.hpp \
    shmdata-utils.hpp \
    shmdata-writer.hpp \
    signal-string.hpp \
    startable-quiddity.hpp \
    std2.hpp \
    string-dictionary.hpp \
    string-utils.hpp \
    unique-gst-element.hpp \
    uridecodebin.hpp \
    video-test-source.hpp
