AC_PREREQ([2.61])
AC_INIT(shmdata, [0.1.2], [http://codered.sat.qc.ca/redmine/projects/libshmdata/issues], [shmdata], [http://codered.sat.qc.ca/redmine/projects/libshmdata])
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
AC_SUBST([LIBSHMDATA_SO_VERSION], [0:0:0])

# The version of the API. Should be major.minor and not change until not backward-compatible
AC_SUBST([LIBSHMDATA_API_VERSION], [0.1])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_CHECK_PROGS([DOXYGEN], [doxygen])
if test -z "$DOXYGEN";
    then AC_MSG_WARN([Doxygen not found - continuing without Doxygen support])
fi

AM_CONDITIONAL([HAVE_DOXYGEN], [test -n "$DOXYGEN"])
# AM_COND_IF([HAVE_DOXYGEN], [AC_CONFIG_FILES([doxyfile])])


# Checks for header files.
AC_CHECK_HEADERS([string.h math.h signal.h stdio.h stdlib.h unistd.h])

# Special cases
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
  # OSX does not include /usr/local/lib/pkgconfig path, so we export it here
  # (for everyone, just in case)
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

CXXFLAGS+=" -g3 -ggdb -Wall -Wextra -Wfatal-errors -Wno-unused-parameter"
CFLAGS+=" -g3 -ggdb -Wall -Wextra -Wfatal-errors -Wno-unused-parameter"

# -- ( mandatory libraries) --
#Trying to set GST_CFLAGS GST_LIBS using pkg-config
PKG_CHECK_MODULES(GST, gstreamer-0.10, have_gstreamer=true, have_gstreamer=false)
if test "x${have_gstreamer}" = "xfalse" ; then
    AC_MSG_ERROR([missing package: libgstreamer0.10-dev])
fi

#Trying to set GSTAPP using pkg-config
PKG_CHECK_MODULES(GSTAPP, gstreamer-app-0.10, have_gstreamer_app=true, have_gstreamer_app=false)
if test "x${have_gstreamer_app}" = "xfalse" ; then
    AC_MSG_ERROR([missing package:  libgstreamer-plugins-app0.10-dev  ])
fi

#Trying to set GIO using pkg-config
PKG_CHECK_MODULES(GIO, gio-2.0, have_gio=true, have_gio=false)
if test "x${have_gio}" = "xfalse" ; then
    AC_MSG_ERROR([missing package:  gio-2.0  ])
fi

# -- (clutter example) -- 
#Trying to set CLUTTER using pkg-config
PKG_CHECK_MODULES(CLUTTER, clutter-1.0, have_clutter=true, have_clutter=false)
if test "x${have_clutter}" = "xfalse" ; then
    AC_MSG_WARN([missing package:  clutter-1.0  ])
fi

#Trying to set GLEW using pkg-config
PKG_CHECK_MODULES(GLEW, glew, have_glew=true, have_glew=false)
if test "x${have_glew}" = "xfalse" ; then
    AC_MSG_WARN([***** missing optional package:  libglew headers ***** ])
    AM_CONDITIONAL(BUILD_CLUTTER_EXAMPLE, false)
else
    if test "x${have_clutter}" = "xtrue" ; then
        AM_CONDITIONAL(BUILD_CLUTTER_EXAMPLE, true)
    else
        AM_CONDITIONAL(BUILD_CLUTTER_EXAMPLE, false)
    fi
fi

# -- (openscenegraph example) --
# Trying to set OPENSCENEGRAPH using pkg-config
PKG_CHECK_MODULES(OPENSCENEGRAPH, openscenegraph, have_openscenegraph=true, have_openscenegraph=false)
if test "x${have_openscenegraph}" = "xfalse" ; then
    AC_MSG_WARN([***** missing optional package:  openscenegraph ***** ])
    AM_CONDITIONAL(HAVE_OPENSCENEGRAPH, false)
else
    AM_CONDITIONAL(HAVE_OPENSCENEGRAPH, true)
fi

# -- (point cloud library example) --
# Trying to set PCL using pkg-config
PKG_CHECK_MODULES(PCL, pcl_visualization-1.4, have_pcl=true, have_pcl=false)
if test "x${have_pcl}" = "xfalse" ; then
    AC_MSG_WARN([***** missing optional package:  pcl_visualization-1.4 ***** ])
    AM_CONDITIONAL(HAVE_PCL, false)
else
    AM_CONDITIONAL(HAVE_PCL, true)
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
        shmdata-osg-${LIBSHMDATA_API_VERSION}.pc:shmdata-osg.pc.in
        ])

AC_OUTPUT

