AC_PREREQ([2.61])
AC_INIT(shmdata, [1.0.2], [http://codered.sat.qc.ca/redmine/projects/libshmdata/issues], [shmdata], [http://codered.sat.qc.ca/redmine/projects/libshmdata])
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

# gstreamer
dnl required versions of gstreamer and plugins-base
GST_REQUIRED=1.0.0
GSTPB_REQUIRED=1.0.0
BUILD_GST=enabled
dnl give error and exit if we don't have pkgconfig
AC_CHECK_PROG(HAVE_PKGCONFIG, pkg-config, [ ], [
  AC_MSG_WARN([You need to have pkg-config installed!])
  BUILD_GST=disabled
])
dnl Check for the required version of GStreamer core (and gst-plugins-base)
dnl This will export GST_CFLAGS and GST_LIBS variables for use in Makefile.am
dnl
PKG_CHECK_MODULES(GST, [
  gstreamer-1.0 >= $GST_REQUIRED
  gstreamer-base-1.0 >= $GST_REQUIRED
  gstreamer-controller-1.0 >= $GST_REQUIRED
], [
  AC_SUBST(GST_CFLAGS)
  AC_SUBST(GST_LIBS)
], [
  AC_MSG_WARN([
      You need to install or upgrade the GStreamer development
      packages on your system. On debian-based systems these are
      libgstreamer1.0-dev and libgstreamer-plugins-base1.0-dev.
      on RPM-based systems gstreamer1.0-devel, libgstreamer1.0-devel
      or similar. The minimum version required is $GST_REQUIRED.
  ])
  BUILD_GST=disabled
])

AM_CONDITIONAL(BUILD_GST, test "x${BUILD_GST}" = "xenabled")

dnl set the plugindir where plugins should be installed (for src/Makefile.am)
if test "x${prefix}" = "x$HOME"; then
  GST_PLUGIN_DIR="$HOME/.gstreamer-1.0/plugins"
else
  GST_PLUGIN_DIR="\$(libdir)/gstreamer-1.0"
fi
AC_SUBST(GST_PLUGIN_DIR)

dnl set proper LDFLAGS for plugins
GST_PLUGIN_LDFLAGS='-module -avoid-version -export-symbols-regex [_]*\(gst_\|Gst\|GST_\).*'
AC_SUBST(GST_PLUGIN_LDFLAGS)


# Check for Python to build a wrapper
AC_CHECK_PROG(HAVE_PYTHON3,
  python3-config,
  [ ],
  [AC_MSG_WARN([Python3.4 not detected, module will not be built])])
AM_CONDITIONAL(HAVE_PYTHON3, test "x${HAVE_PYTHON3}", = "xtrue")

# Override the template file name of the generated .pc file, so that there
# is no need to rename the template file when the API version changes.
dnl AC_CONFIG_FILES([
dnl         gst/Makefile
dnl         gst/shm/Makefile
dnl         ])

AC_CONFIG_FILES([
        Makefile 
        shmdata/Makefile
        gst/Makefile
        tests/Makefile
        utils/Makefile
        wrappers/Makefile
        wrappers/python/Makefile
        doxyfile
        shmdata-${LIBSHMDATA_API_VERSION}.pc:shmdata.pc.in
        ])

AC_OUTPUT

