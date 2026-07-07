AC_PREREQ([2.69])
AC_INIT([temphum2],[2.0.0])
AM_INIT_AUTOMAKE
: ${CFLAGS="-g -Os -Wall -nostdlib -mlongcalls"} ${LDFLAGS="-specs=nano.specs"}
AC_PROG_CC
AC_PROG_CC_STDC
AC_DEFUN([AC_PROG_AR], [AC_CHECK_TOOL(AR, ar, :)])
AC_PROG_AR
AC_PROG_RANLIB

AC_ARG_WITH([binaries], AS_HELP_STRING([--without-binaries], [Do not generate .bin images (esptool.py required).]))
AM_CONDITIONAL([WITH_BINARIES], [test x$with_binaries != xno])
AM_COND_IF([WITH_BINARIES],[ AC_MSG_NOTICE([generate .bin images]) ])

dnl ===========================================================================
dnl Detect the version of esp32basic installed
dnl ===========================================================================

AC_ARG_WITH([e32bdir],
        [AS_HELP_STRING([--with-e32bdir=PATH], [specify the esp32basic installation directory])],
        [E32BDIR="$withval"], [E32BDIR=""])

if test ! "x$E32BDIR" = "x"; then
        dnl Make sure that the directory exists
        if test "x$E32BDIR" = "xyes"; then
                AC_MSG_ERROR([you must specify a parameter to --with-e32bdir, e.g. --with-e32bdir=/path/to])
        else
                if test -d "$E32BDIR"; then
                        AC_MSG_RESULT([Using user-specified esp32basic directory: $E32BDIR])

                        dnl Add the include directory to E32B_CPPFLAGS
                        E32B_CPPFLAGS="-I$E32BDIR/include"
                        E32B_LDFLAGS="-L$E32BDIR/lib"
                else
                        AC_MSG_ERROR([the --with-e32bdir directory "$E32BDIR" cannot be found])
                fi
        fi
fi


dnl Check that we can find the main.h header file
CPPFLAGS_SAVE="$CPPFLAGS"
CPPFLAGS="$E32B_CPPFLAGS"
AC_CHECK_HEADER([main.h], [], [AC_MSG_ERROR([could not find main.h - you may need to specify the directory of an esp32basic installation using --with-projdir])])
CPPFLAGS="$CPPFLAGS_SAVE"

AC_SUBST([E32B_CPPFLAGS])
AC_SUBST([E32B_LDFLAGS])
AC_SUBST([E32BDIR])

dnl Ensure we can link against libeps32basic
LIBS_SAVE="$LIBS"
LIBS="$E32B_LDFLAGS"
AC_CHECK_LIB([esp32basic], [lockmgr_init],
        [],
        [AC_MSG_ERROR([could not find libesp32basic - you may need to specify the directory of an esp32basic installation using --with-e32bdir])],
        [])
LIBS="$LIBS_SAVE"


AC_CONFIG_FILES([
  Makefile
  src/Makefile
])

AC_OUTPUT
