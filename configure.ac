AC_PREREQ([2.61])
AC_INIT(shmdata, [1.0.1], [http://codered.sat.qc.ca/redmine/projects/libshmdata/issues], [shmdata], [http://codered.sat.qc.ca/redmine/projects/libshmdata])
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
AC_SUBST([LIBSHMDATA_SO_VERSION], [5:0:0])

# The version of the API. Should be major.minor and not change until not backward-compatible
AC_SUBST([LIBSHMDATA_API_VERSION], [1.0])

# -- checks for programs --
AC_PROG_CXX
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_MSG_CHECKING([to see if compiler understands -std=c++11])
CXXFLAGS="$CXXFLAGS -std=c++11"
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
  AC_MSG_RESULT([yes])
], [
  AC_MSG_RESULT([no])
  AC_MSG_ERROR([compiler does not understand -std=c++11])
])


AC_CHECK_PROGS([DOXYGEN], [doxygen])
if test -z "$DOXYGEN";
    then AC_MSG_WARN([Doxygen not found - continuing without Doxygen support])
fi

AM_CONDITIONAL([HAVE_DOXYGEN], [test -n "$DOXYGEN"])


# -- checks for header files --
AC_CHECK_HEADERS([sys/types.h sys/ipc.h sys/shm.h])

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

dnl  # -- testing GStreamer SDK, if found, then building against --
dnl AM_CONDITIONAL(HAVE_GST_SDK, false)
dnl  # linux
dnl  AC_CHECK_FILE([/opt/gstreamer-sdk/lib/pkgconfig/gstreamer-0.10.pc], 
dnl                 [AC_MSG_WARN([building against Linux GStreamer SDK.]) 
dnl                  AM_CONDITIONAL(HAVE_GST_SDK, true)
dnl                  PKG_CONFIG_PATH=/opt/gstreamer-sdk/lib/pkgconfig:$PKG_CONFIG_PATH
dnl                  export PKG_CONFIG_PATH
dnl                  SHMDATA_GST_PLUGIN_PATH=/opt/gstreamer-sdk/lib],
dnl                 [AC_MSG_WARN([Linux GStreamer SDK not found])])

dnl  # osx
dnl AC_CHECK_FILE([/Library/Frameworks/GStreamer.framework/Libraries/pkgconfig], 
dnl               [AC_MSG_WARN([building against OSX GStreamer SDK.]) 
dnl                AM_CONDITIONAL(HAVE_GST_SDK, true)
dnl                PKG_CONFIG_PATH=/Library/Frameworks/GStreamer.framework/Libraries/pkgconfig:$PKG_CONFIG_PATH
dnl                export PKG_CONFIG_PATH
dnl                SHMDATA_GST_PLUGIN_PATH=/Library/Frameworks/GStreamer.framework/Libraries/gstreamer-0.10],
dnl               [AC_MSG_WARN([OSX GStreamer SDK not found])])

dnl # --define where to install gstreamer elements built -- 
dnl if test "x${prefix}" = "x$HOME"; then
dnl   SHMDATA_GST_PLUGIN_PATH="\$(HOME)/.gstreamer-0.10/plugins"
dnl   FLAT_SHMDATA_GST_PLUGIN_PATH=\"$HOME/.gstreamer-0.10/plugins\"
dnl else
dnl   if test "x${SHMDATA_GST_PLUGIN_PATH}" = "x" ; then
dnl     SHMDATA_GST_PLUGIN_PATH="\$(libdir)/gstreamer-0.10"
dnl   fi

dnl   dnl set install paths.
dnl   if test "x$prefix" = "xNONE" ; then
dnl     prefix="/usr/local"
dnl   fi
dnl   if test "x$exec_prefix" = "xNONE" ; then
dnl     exec_prefix=$prefix
dnl   fi
dnl   LIBDIR=`eval echo $libdir`
dnl   LIBDIR=`eval echo $LIBDIR`
dnl   FLAT_SHMDATA_GST_PLUGIN_PATH=\"$LIBDIR/gstreamer-0.10\"
dnl fi

dnl [echo "SHMDATA_GST_PLUGIN_PATH: ${SHMDATA_GST_PLUGIN_PATH}"]

dnl AC_SUBST(SHMDATA_GST_PLUGIN_PATH)
dnl AC_DEFINE_UNQUOTED([SHMDATA_GST_PLUGIN_PATH],[$FLAT_SHMDATA_GST_PLUGIN_PATH],
dnl                    [GStreamer plugin path for element built])

dnl BUILDDIR=`pwd`
dnl SHMDATA_SHM_GST_PLUGIN_BUILD_PATH=\"$BUILDDIR/gst/shm/.libs\"
dnl AC_DEFINE_UNQUOTED([SHMDATA_SHM_GST_PLUGIN_BUILD_PATH],[$SHMDATA_SHM_GST_PLUGIN_BUILD_PATH],
dnl                    [Path where the shm elements have been built])

dnl # -- ( mandatory libraries) --
dnl #Trying to set GST_CFLAGS and GST_LIBS using pkg-config
dnl PKG_CHECK_MODULES(GST, gstreamer-0.10, have_gstreamer=true, have_gstreamer=false)
dnl if test "x${have_gstreamer}" = "xfalse" ; then
dnl     AC_MSG_ERROR([missing package: libgstreamer0.10-dev])
dnl fi

dnl #for building shmsrc and shmsink elements
dnl PKG_CHECK_MODULES(GST_BASE, gstreamer-base-0.10, have_gstreamer_base=true, have_gstreamer_base=false)
dnl if test "x${have_gstreamer_base}" = "xfalse" ; then
dnl   AC_MSG_ERROR([missing package: libgstreamer0.10-dev])
dnl fi

dnl #Trying to set GSTAPP using pkg-config
dnl PKG_CHECK_MODULES(GSTAPP, gstreamer-app-0.10, have_gstreamer_app=true, have_gstreamer_app=false)
dnl if test "x${have_gstreamer_app}" = "xfalse" ; then
dnl     AC_MSG_ERROR([missing package:  libgstreamer-plugins-base0.10-dev  ])
dnl fi

#Trying to set GIO using pkg-config
dnl PKG_CHECK_MODULES(GIO, gio-2.0, have_gio=true, have_gio=false)
dnl if test "x${have_gio}" = "xfalse" ; then
dnl     AC_MSG_ERROR([missing package:  libglib2.0-dev  ])
dnl fi

dnl # Check for Puredata
dnl AC_CHECK_HEADER(m_pd.h,
dnl                 PUREDATA=y,
dnl                 AC_MSG_WARN([Puredata header not found.]))
dnl AM_CONDITIONAL(PUREDATAFOUND, test "${PUREDATA}" = "y")

dnl if test "${PUREDATA}" = "y" ; then
dnl    #checking libsamplerate0-dev
dnl    PKG_CHECK_MODULES(LIBSAMPLERATE, samplerate, have_samplerate=true, have_samplerate=false)
dnl    if test "x${have_samplerate}" = "xfalse" ; then
dnl        AC_MSG_ERROR([building puredata external enabled, but missing package: libsamplerate0-dev])
dnl    fi
dnl fi

dnl # Check for Python to build a wrapper
dnl PKG_CHECK_MODULES(PYTHON3, python3,
dnl     AM_CONDITIONAL(HAVE_PYTHON3, true),
dnl     AM_CONDITIONAL(HAVE_PYTHON3, false))

# Override the template file name of the generated .pc file, so that there
# is no need to rename the template file when the API version changes.
dnl AC_CONFIG_FILES([
dnl         Makefile 
dnl         shmdata/Makefile
dnl         examples/Makefile
dnl         tests/Makefile
dnl         doxyfile
dnl         shmdata-${LIBSHMDATA_API_VERSION}.pc:shmdata.pc.in
dnl         shmdata-any-${LIBSHMDATA_API_VERSION}.pc:shmdata-any.pc.in
dnl         plugins/Makefile
dnl         plugins/puredata/Makefile
dnl         gst/Makefile
dnl         gst/shm/Makefile
dnl         wrappers/Makefile
dnl         wrappers/python/Makefile
dnl         ])

AC_CONFIG_FILES([
        Makefile 
        shmdata/Makefile
        tests/Makefile
        doxyfile
        shmdata-${LIBSHMDATA_API_VERSION}.pc:shmdata.pc.in
        ])

AC_OUTPUT

