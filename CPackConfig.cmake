set(CPACK_PACKAGE_VENDOR "PICASO 3D")
set(CPACK_PACKAGE_CONTACT "Andrey Isupov <info@picaso-3d.ru>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Picaso XCore Engine")
set(CPACK_PACKAGE_VERSION "01.01.00")
set(CPACK_GENERATOR "DEB")
if(NOT DEFINED CPACK_DEBIAN_PACKAGE_ARCHITECTURE)
  execute_process(COMMAND dpkg --print-architecture OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}-${CPACK_PACKAGE_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

set(DEB_DEPENDS
    "arcus (>= 15.05.90)"
    "protobuf (>= 3.0.0)"
    "libstdc++6 (>= 4.9.0)"
    "libgcc1 (>= 4.9.0)"
)
string(REPLACE ";" ", " DEB_DEPENDS "${DEB_DEPENDS}")
set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEB_DEPENDS})

include(CPack)