BUILD_DIR = glfw-3.2.1/install_dir

pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libglfwvideo.la

#libglfwvideo info
libglfwvideo_la_SOURCES = \
    glad.c \
    glfw-renderer.cpp \
    glfwvideo.cpp

libglfwvideo_la_include_HEADERS = \
    glfw-renderer.hpp \
    glfwvideo.hpp

BUILT_SOURCES = $(BUILD_DIR)/src/glfw_config.h

$(BUILD_DIR)/src/glfw_config.h: Makefile
	tar -xvzf glfw3_3.2.1.orig.tar.gz     \
    && mkdir -p $(BUILD_DIR) \
    && cd $(BUILD_DIR)       \
    && cmake .. -DBUILD_SHARED_LIBS=TRUE && make -j && sudo make install && cd ../..

clean-local:
	-rm -rf glfw-3.2.1

distclean-local:
	-rm -rf glfw-3.2.1

libglfwvideo_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libglfwvideo_la_LDFLAGS = \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(SHMDATA_LIBS) \
    $(JSONGLIB_LIBS) \
    -avoid-version \
    -module \
    -lGL \
    -ldl \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@ \
    `pkg-config --cflags $(BUILD_DIR)/src/glfw3.pc`

libglfwvideo_la_CXXFLAGS = \
    $(GST_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) \
    `pkg-config --libs $(BUILD_DIR)/src/glfw3.pc`

#### Functional tests
AM_CFLAGS = \
    -I$(top_srcdir)

AM_CXXFLAGS =  \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    -I$(top_srcdir)

AM_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(SHMDATA_LIBS) \
    $(JSONGLIB_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la \
    `pkg-config --libs $(BUILD_DIR)/src/glfw3.pc`

check_PROGRAMS = \
    check_glfw_video

check_glfw_video_SOURCES = \
    check_glfw_video.cpp

TESTS = $(check_PROGRAMS)
