AC_PREREQ([2.61])
AC_INIT(shmdata, [0.8.11], [http://codered.sat.qc.ca/redmine/projects/libshmdata/issues], [shmdata], [http://codered.sat.qc.ca/redmine/projects/libshmdata])
#AC_CONFIG_MACRO_DIR([m4])
AC_MSG_NOTICE([Configuring libshmdata])
AM_INIT_AUTOMAKE([subdir-objects -Wno-portability])
AC_CONFIG_HEADERS([config.h])
AC_CANONICAL_HOST # fills $host variable

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
AC_SUBST([LIBSHMDATA_SO_VERSION], [4:0:0])

# The version of the API. Should be major.minor and not change until not backward-compatible
AC_SUBST([LIBSHMDATA_API_VERSION], [0.8])

# -- checks for programs --
AC_PROG_CXX
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_CHECK_PROGS([DOXYGEN], [doxygen])
if test -z "$DOXYGEN";
    then AC_MSG_WARN([Doxygen not found - continuing without Doxygen support])
fi

AM_CONDITIONAL([HAVE_DOXYGEN], [test -n "$DOXYGEN"])


# -- checks for header files --
AC_CHECK_HEADERS([string.h math.h signal.h stdio.h stdlib.h unistd.h])

# OSX and MINGW
AM_CONDITIONAL(MINGW, false)
AM_CONDITIONAL(HAVE_OSX, false)

case "${build_os}" in
*mingw* | *cygwin*)
  mingw32_support="yes"
  AC_CHECK_HEADER(windows.h)
  AM_CONDITIONAL(MINGW, true)
  LDFLAGS="$LDFLAGS -no-undefined"
  ;;
*darwin* | *rhapsody* | *macosx*)
  AC_ISC_POSIX
  AM_CONDITIONAL(HAVE_OSX, true)
  AC_DEFINE(HAVE_OSX,[1],[Apple Mac OS X operating system detected])
  ;;
*)
  AC_ISC_POSIX
  ;;
esac

# -- special cases --
case "$host" in *-darwin* | *-macos10*)
  LIBTOOL="glibtool"
  CXXFLAGS+=" -D__Darwin"
  LDFLAGS+=" -undefined dynamic_lookup -Wl,-headerpad_max_install_names"
  if test -d /opt/local ; then
    CXXFLAGS+=" -I/usr/local/include -I/opt/local/include"
    #LDFLAGS+=" -L/usr/local/lib -L/opt/local/lib"
  elif test -d /sw ; then
    CXXFLAGS+=" -I/sw/include"
    #LDFLAGS+=" -L/sw/lib"
  fi
  # OSX does not include /usr/local/lib/pkgconfig path, so we need to
  # add it. 
  PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
  export PKG_CONFIG_PATH
  LDFLAGS+=" -fPIC -L/usr/local/lib -L/usr/local/lib64"
  ;;
esac

AC_ARG_ENABLE([universal],
    AS_HELP_STRING([--enable-universal], [Enables a universal i386 / x86_64 build on OSX]),
    [universal_build=$enableval],
    [universal_build=no])

if test "x$universal_build" = xyes; then
    CXXFLAGS+=" -arch i386 -arch x86_64"
    LDFLAGS+=" -arch i386 -arch x86_64"
fi

# -- compilation flags --
CXXFLAGS+=" -g3 -ggdb -Wall -Wextra -Wfatal-errors -Wno-unused-parameter"
CFLAGS+=" -g3 -ggdb -Wall -Wextra -Wfatal-errors -Wno-unused-parameter"

 # -- testing GStreamer SDK, if found, then building against --
AM_CONDITIONAL(HAVE_GST_SDK, false)
 # linux
 AC_CHECK_FILE([/opt/gstreamer-sdk/lib/pkgconfig/gstreamer-0.10.pc], 
                [AC_MSG_WARN([building against Linux GStreamer SDK.]) 
                 AM_CONDITIONAL(HAVE_GST_SDK, true)
                 PKG_CONFIG_PATH=/opt/gstreamer-sdk/lib/pkgconfig:$PKG_CONFIG_PATH
                 export PKG_CONFIG_PATH
                 SHMDATA_GST_PLUGIN_PATH=/opt/gstreamer-sdk/lib],
                [AC_MSG_WARN([Linux GStreamer SDK not found])])

 # osx
AC_CHECK_FILE([/Library/Frameworks/GStreamer.framework/Libraries/pkgconfig], 
              [AC_MSG_WARN([building against OSX GStreamer SDK.]) 
               AM_CONDITIONAL(HAVE_GST_SDK, true)
               PKG_CONFIG_PATH=/Library/Frameworks/GStreamer.framework/Libraries/pkgconfig:$PKG_CONFIG_PATH
               export PKG_CONFIG_PATH
               SHMDATA_GST_PLUGIN_PATH=/Library/Frameworks/GStreamer.framework/Libraries/gstreamer-0.10],
              [AC_MSG_WARN([OSX GStreamer SDK not found])])

# --define where to install gstreamer elements built -- 
if test "x${prefix}" = "x$HOME"; then
  SHMDATA_GST_PLUGIN_PATH="\$(HOME)/.gstreamer-0.10/plugins"
  FLAT_SHMDATA_GST_PLUGIN_PATH=\"$HOME/.gstreamer-0.10/plugins\"
else
  if test "x${SHMDATA_GST_PLUGIN_PATH}" = "x" ; then
    SHMDATA_GST_PLUGIN_PATH="\$(libdir)/gstreamer-0.10"
  fi

  dnl set install paths.
  if test "x$prefix" = "xNONE" ; then
    prefix="/usr/local"
  fi
  if test "x$exec_prefix" = "xNONE" ; then
    exec_prefix=$prefix
  fi
  LIBDIR=`eval echo $libdir`
  LIBDIR=`eval echo $LIBDIR`
  FLAT_SHMDATA_GST_PLUGIN_PATH=\"$LIBDIR/gstreamer-0.10\"
fi

[echo "SHMDATA_GST_PLUGIN_PATH: ${SHMDATA_GST_PLUGIN_PATH}"]

AC_SUBST(SHMDATA_GST_PLUGIN_PATH)
AC_DEFINE_UNQUOTED([SHMDATA_GST_PLUGIN_PATH],[$FLAT_SHMDATA_GST_PLUGIN_PATH],
                   [GStreamer plugin path for element built])

BUILDDIR=`pwd`
SHMDATA_SHM_GST_PLUGIN_BUILD_PATH=\"$BUILDDIR/gst/shm/.libs\"
AC_DEFINE_UNQUOTED([SHMDATA_SHM_GST_PLUGIN_BUILD_PATH],[$SHMDATA_SHM_GST_PLUGIN_BUILD_PATH],
                   [Path where the shm elements have been built])

# -- ( mandatory libraries) --
#Trying to set GST_CFLAGS and GST_LIBS using pkg-config
PKG_CHECK_MODULES(GST, gstreamer-0.10, have_gstreamer=true, have_gstreamer=false)
if test "x${have_gstreamer}" = "xfalse" ; then
    AC_MSG_ERROR([missing package: libgstreamer0.10-dev])
fi

#for building shmsrc and shmsink elements
PKG_CHECK_MODULES(GST_BASE, gstreamer-base-0.10, have_gstreamer_base=true, have_gstreamer_base=false)
if test "x${have_gstreamer_base}" = "xfalse" ; then
  AC_MSG_ERROR([missing package: libgstreamer0.10-dev])
fi

#Trying to set GSTAPP using pkg-config
PKG_CHECK_MODULES(GSTAPP, gstreamer-app-0.10, have_gstreamer_app=true, have_gstreamer_app=false)
if test "x${have_gstreamer_app}" = "xfalse" ; then
    AC_MSG_ERROR([missing package:  libgstreamer-plugins-base0.10-dev  ])
fi

#Trying to set GIO using pkg-config
PKG_CHECK_MODULES(GIO, gio-2.0, have_gio=true, have_gio=false)
if test "x${have_gio}" = "xfalse" ; then
    AC_MSG_ERROR([missing package:  libglib2.0-dev  ])
fi

# -- (openscenegraph library) --
# Trying to set OPENSCENEGRAPH using pkg-config
PKG_CHECK_MODULES(OPENSCENEGRAPH, openscenegraph, have_openscenegraph=true, have_openscenegraph=false)
if test "x${have_openscenegraph}" = "xfalse" ; then
    AC_MSG_WARN([***** missing optional package:  libopenscenegraph-dev ***** ])
    AM_CONDITIONAL(HAVE_OPENSCENEGRAPH, false)
else
    AM_CONDITIONAL(HAVE_OPENSCENEGRAPH, true)
    AC_CONFIG_FILES([shmdata-osg-${LIBSHMDATA_API_VERSION}.pc:shmdata-osg.pc.in])
fi

# Check for Puredata
AC_CHECK_HEADER(m_pd.h,
                PUREDATA=y,
                AC_MSG_WARN([Puredata header not found.]))
AM_CONDITIONAL(PUREDATAFOUND, test "${PUREDATA}" = "y")

if test "${PUREDATA}" = "y" ; then
   #checking libsamplerate0-dev
   PKG_CHECK_MODULES(LIBSAMPLERATE, samplerate, have_samplerate=true, have_samplerate=false)
   if test "x${have_samplerate}" = "xfalse" ; then
       AC_MSG_ERROR([building puredata external enabled, but missing package: libsamplerate0-dev])
   fi
fi

# Override the template file name of the generated .pc file, so that there
# is no need to rename the template file when the API version changes.
AC_CONFIG_FILES([
        Makefile 
        shmdata/Makefile
        examples/Makefile
        tests/Makefile
        doxyfile
        shmdata-${LIBSHMDATA_API_VERSION}.pc:shmdata.pc.in
        shmdata-any-${LIBSHMDATA_API_VERSION}.pc:shmdata-any.pc.in
        plugins/Makefile
        plugins/puredata/Makefile
        gst/Makefile
        gst/shm/Makefile
        ])

AC_OUTPUT

