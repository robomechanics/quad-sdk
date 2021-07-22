prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_FULL_LIBDIR@
includedir=@CMAKE_INSTALL_FULL_INCLUDEDIR@

Name: RBDL
Description: Rigid Body Dynamics Library
URL: https://rbdl.github.io/
Version: @RBDL_VERSION@
Requires: eigen3
Conflicts:
Libs: -L${libdir} -lrbdl -Wl,-rpath ${libdir}
Libs.private:
Cflags: -I${includedir}
