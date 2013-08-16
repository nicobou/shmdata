dnl required version of autoconf
AC_PREREQ([2.53])

dnl package name and package version
AC_INIT([switcher],[0.2.3])

dnl required version of gstreamer and gst-plugins-base
GST_REQUIRED=0.10.36

dnl AC_CONFIG_SRCDIR([src/main.c])
AC_CONFIG_HEADERS([config.h])

dnl required version of automake
AM_INIT_AUTOMAKE([1.10])

dnl enable mainainer mode by default
dnl AM_MAINTAINER_MODE([enable])

dnl sets host_* variables
AC_CANONICAL_HOST

# Define these substitions here to keep all version information in one place.
# For information on how to properly maintain the library version information,
# refer to the libtool manual, section "Updating library version information":
# http://www.gnu.org/software/libtool/manual/html_node/Updating-version-info.html
# see http://sourceware.org/autobook/autobook/autobook_91.html
# format is [current:revision:age]
# libtool versioning is different than the project's versioning. It's for the ABI and API.
# It's in the form "current:revision:age" where:
#  * current: Increment when the interface changes. (and set revision to 0)
#  * revision: Increment when the implementation changes, but not the interface.
#  * age: Backward-compatible with how many interface numbers.
AC_SUBST([LIBSWITCHER_SO_VERSION], [0:0:0])

# The version of the API. Should be major.minor and not change until not backward-compatible
AC_SUBST([LIBSWITCHER_API_VERSION], [0.2])
AC_DEFINE(LIBSWITCHER_API_VERSION, ["0.2"], [API version])

dnl check for tools (compiler etc.)
AC_PROG_CXX
AM_PROG_CC_C_O
AC_PROG_INSTALL
AC_PROG_LIBTOOL

dnl required version of libtool
LT_PREREQ([2.2.6])
LT_INIT


dnl error out if we can't find pkg-config
AC_CHECK_PROG(HAVE_PKGCONFIG, pkg-config, [ ], [
  AC_MSG_ERROR([You need to have pkg-config installed or set the PATH.])
])

case "$host" in *-darwin* | *-macos10*)
  LIBTOOL="glibtool"
  CXXFLAGS+=" -D__Darwin -stdlib=libc++"
  CXX="clang++"
  LDFLAGS+=" -undefined dynamic_lookup -Wl,-headerpad_max_install_names"
  if test -d /opt/local ; then
    CXXFLAGS+=" -I/usr/local/include -I/opt/local/include"
    #LDFLAGS+=" -L/usr/local/lib -L/opt/local/lib"
  elif test -d /sw ; then
    CXXFLAGS+=" -I/sw/include"
    #LDFLAGS+=" -L/sw/lib"
  fi
  
  AM_CONDITIONAL([HAVE_OSX], true)
  AC_DEFINE(HAVE_OSX,[1],[Apple Mac OS X operating system detected])
  
  # OSX does not include /usr/local/lib/pkgconfig path, so we export it here
  # (for everyone, just in case)
  PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
  export PKG_CONFIG_PATH
  
  LDFLAGS+=" -fPIC -L/usr/local/lib -L/usr/local/lib64"
  ;;
*)
  AM_CONDITIONAL([HAVE_OSX], false)
  AC_DEFINE(HAVE_OSX,[0],[Apple Mac OS X operating system not detected])
  ;;
esac

# -- testing GStreamer SDK, if found, then building against --
AM_CONDITIONAL(HAVE_GST_SDK, false)
 # linux
 AC_CHECK_FILE([/opt/gstreamer-sdk/lib/pkgconfig/gstreamer-0.10.pc], 
                [AC_MSG_WARN([building against Linux GStreamer SDK.]) 
                 AM_CONDITIONAL(HAVE_GST_SDK, true)
                 PKG_CONFIG_PATH=/opt/gstreamer-sdk/lib/pkgconfig:$PKG_CONFIG_PATH
                 export PKG_CONFIG_PATH],
                [AC_MSG_WARN([Linux GStreamer SDK not found])])

 # osx
AC_CHECK_FILE([/Library/Frameworks/GStreamer.framework/Libraries/pkgconfig], 
              [AC_MSG_WARN([building against OSX GStreamer SDK.]) 
               AM_CONDITIONAL(HAVE_GST_SDK, true)
               PKG_CONFIG_PATH=/Library/Frameworks/GStreamer.framework/Libraries/pkgconfig:$PKG_CONFIG_PATH
               export PKG_CONFIG_PATH],
              [AC_MSG_WARN([OSX GStreamer SDK not found])])



dnl =======================================================================
dnl Check for the required version of GStreamer core (and gst-plugins-base)
dnl
dnl This will export GST_CFLAGS and GST_LIBS variables for use in Makefile.am
dnl
dnl =======================================================================

PKG_CHECK_MODULES(GST, [
    gstreamer-0.10              >= $GST_REQUIRED
    gstreamer-base-0.10         >= $GST_REQUIRED
    gstreamer-controller-0.10   >= $GST_REQUIRED
    gstreamer-sdp-0.10          >= $GST_REQUIRED
    gstreamer-interfaces-0.10   >= $GST_REQUIRED
  ], [
    AC_SUBST(GST_CFLAGS)
    AC_SUBST(GST_LIBS)
  ], [
  AC_MSG_ERROR([
    Can't find the following GStreamer development packages:

      gstreamer-0.10              >= $GST_REQUIRED
      gstreamer-base-0.10         >= $GST_REQUIRED
      gstreamer-controller-0.10   >= $GST_REQUIRED
      gstreamer-sdp-0.10          >= $GST_REQUIRED
      gstreamer-interfaces-0.10   >= $GST_REQUIRED

    Please make sure you have the necessary GStreamer-0.10
    development headers installed.

    On debian/Ubuntu systems you will probably need to install the
    'libgstreamer0.10-dev' and 'libgstreamer-plugins-base0.10-dev' packages.

    On RPM-based systems you will probably need to install the
    'gstreamer-devel-0.10' package.
  ])
])

PKG_CHECK_MODULES(GLIB, glib-2.0, have_glib=true, have_glib=false)
if test "x${have_glib}" = "xfalse" ; then
    AC_MSG_ERROR([missing package:  libglib2.0-dev  ])
fi

PKG_CHECK_MODULES(GMODULE, gmodule-2.0, have_gmodule=true, have_gmodule=false)
if test "x${have_gmodule}" = "xfalse" ; then
    AC_MSG_ERROR([missing package:  libglib2.0-dev  ])
fi


PKG_CHECK_MODULES(SHMDATA, shmdata-0.8, have_shmdata=true, have_shmdata=false)
if test "x${have_shmdata}" = "xfalse" ; then
    AC_MSG_ERROR([Missing libshmdata-0.8-dev])
fi

PKG_CHECK_MODULES(GSOAP, gsoap++, have_gsoap=true, have_gsoap=false)
if test "x${have_gsoap}" = "xfalse" ; then
    GSOAPBUILD=disabled
    AC_MSG_WARN([Missing gsoap package])
else
    GSOAPBUILD=enabled
    AC_CONFIG_FILES([plugins/gsoap/Makefile])
fi
AM_CONDITIONAL(GSOAPBUILD, test "${GSOAPBUILD}" = "enabled")

PKG_CHECK_MODULES(LIBLO, liblo, have_liblo=true, have_liblo=false)
if test "x${have_liblo}" = "xfalse" ; then
    OSCBUILD=disabled
    AC_MSG_WARN([Missing liblo-dev package, switcher will not be built])
else
    OSCBUILD=enabled
    AC_CONFIG_FILES([plugins/osc/Makefile])
fi
AM_CONDITIONAL(OSCBUILD, test "${OSCBUILD}" = "enabled")

PKG_CHECK_MODULES(PULSE, libpulse-mainloop-glib, have_libpulse=true, have_libpulse=false)
if test "x${have_libpulse}" = "xfalse" ; then
    PULSEBUILD=disabled
    AC_MSG_WARN([Missing libpulse-dev])
else
    PULSEBUILD=enabled
    AC_CONFIG_FILES([plugins/pulse/Makefile])
fi
AM_CONDITIONAL(PULSEBUILD, test "${PULSEBUILD}" = "enabled")

PKG_CHECK_MODULES(JSONGLIB, json-glib-1.0, have_jsonglib=true, have_jsonglib=false)
if test "x${have_jsonglib}" = "xfalse" ; then
    AC_MSG_ERROR([Missing libjson-glib-dev package])
fi

# Check for 
AC_CHECK_HEADER(linux/videodev2.h,
                V4L2BUILD=enabled,
                AC_MSG_WARN([v4l2 capture plugin will not be built (missing videodev2.h from package linux-libc-dev)]))
AM_CONDITIONAL(V4L2BUILD, test "${V4L2BUILD}" = "enabled")
if test "x${V4L2BUILD}" = "xenabled";
  then AC_CONFIG_FILES([plugins/v4l2/Makefile])
else
  V4L2BUILD=disabled
fi

PKG_CHECK_MODULES(GTK, gtk+-2.0, have_gtk=true, have_gtk=false)
if test "x${have_gtk}" = "xfalse" ; then
    GTKBUILD=disabled
    AC_MSG_WARN([Missing libgtk2.0-dev package, gtk fullscreen video will not be built])
else
    GTKBUILD=enabled
    AC_CONFIG_FILES([plugins/gtk/Makefile])
fi
AM_CONDITIONAL(GTKBUILD, test "${GTKBUILD}" = "enabled")

AC_MSG_CHECKING([to see if compiler understands -std=c++0x])
save_CXXFLAGS="$CXXFLAGS"
CXXFLAGS="$CFLAGS $CXXFLAGS -std=c++0x"
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
  CXXFLAGS="$CFLAGS $CXXFLAGS -std=c++0x"
  AC_MSG_RESULT([yes])
], [
  AC_MSG_RESULT([no])
  AC_MSG_ERROR([compiler does not understand -std=c++0x])
])


dnl check if compiler understands -Wall (if yes, add -Wall to GST_CFLAGS)
AC_MSG_CHECKING([to see if compiler understands -Wall])
save_CFLAGS="$CFLAGS"
CFLAGS="$CFLAGS -Wall"
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
  GST_CFLAGS="$GST_CFLAGS -Wall"
  AC_MSG_RESULT([yes])
], [
  AC_MSG_RESULT([no])
])
                                          
AC_CHECK_PROGS([DOXYGEN], [doxygen])
if test -z "$DOXYGEN";
    then AC_MSG_WARN([Doxygen not found - continuing without Doxygen support])
fi

AM_CONDITIONAL([HAVE_DOXYGEN], [test -n "$DOXYGEN"])

dnl testing if switcher can be built and add it to CONFIG_FILES if building
SWITCHERBUILD=enabled
if test "${OSCBUILD}" = "disabled"; 
  then SWITCHERBUILD=disabled
fi
if test "${SOAPBUILD}" = "disabled";
  then SWITCHERBUILD=disabled
fi
if test "${SWITCHERBUILD}" = "enabled";
  then AC_CONFIG_FILES([src/Makefile])
fi
AM_CONDITIONAL(SWITCHERBUILD, test "${SWITCHERBUILD}" = "enabled")
                            
dnl =======================================================================
dnl Finally, create Makefiles in all directories
dnl =======================================================================

AC_CONFIG_FILES([
Makefile
doxyfile
switcher-${LIBSWITCHER_API_VERSION}.pc:switcher.pc.in
switcher/Makefile
tests/Makefile
plugins/Makefile
plugins/example/Makefile
plugins/avcodecs/Makefile
])
AC_OUTPUT

[echo ""]
[echo "-----------------------------------"]
[echo ""]
[echo "OPTIONAL PLUGINS"]
[echo ""]
[echo "V4L2 capture               ${V4L2BUILD}"]
[echo "GTK fullscreen video       ${GTKBUILD}"]
[echo "Pulse audio                ${PULSEBUILD}"]
[echo ""]
[echo "-----------------------------------"]
[echo ""]
[echo "SWITCHER REQUIRED PLUGINS"]
[echo ""]
[echo "OSC controler plugin       ${OSCBUILD}"]
[echo "GSOAP controler plugin     ${GSOAPBUILD}"]
[echo ""]
[echo "-----------------------------------"]
[echo ""]
[echo "SWITCHER BUILD STATUS"]
[echo ""]
[echo "libswitcher                enabled"]
[echo "switcher                   ${SWITCHERBUILD}"]
[echo ""]
[echo "-----------------------------------"]
[echo ""]
