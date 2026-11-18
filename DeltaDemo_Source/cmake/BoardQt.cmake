# Board Qt libraries are ARM64; moc/rcc must run on the Windows build host.
set(DEMO_HOST_QT "D:/Qt/5.15.2/msvc2019_64" CACHE PATH "Windows Qt 5.15 tools")
set(Qt5Core_VERSION_MAJOR 5)
set(Qt5Core_VERSION_MINOR 15)
set(Qt5Core_VERSION_STRING 5.15.3)
foreach(module Core Gui Widgets Network)
  add_library(Qt5::${module} SHARED IMPORTED)
  set_target_properties(Qt5::${module} PROPERTIES
    IMPORTED_LOCATION "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libQt5${module}.so"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu/qt5;${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu/qt5/Qt${module}"
    INTERFACE_QT_MAJOR_VERSION 5)
  string(TOUPPER "${module}" upper)
  set_property(TARGET Qt5::${module} PROPERTY INTERFACE_COMPILE_DEFINITIONS "QT_${upper}_LIB;QT_NO_DEBUG")
endforeach()
set_property(TARGET Qt5::Gui PROPERTY INTERFACE_LINK_LIBRARIES Qt5::Core)
set_property(TARGET Qt5::Widgets PROPERTY INTERFACE_LINK_LIBRARIES Qt5::Gui)
set_property(TARGET Qt5::Network PROPERTY INTERFACE_LINK_LIBRARIES Qt5::Core)
foreach(tool moc rcc)
  add_executable(Qt5::${tool} IMPORTED)
  set_property(TARGET Qt5::${tool} PROPERTY IMPORTED_LOCATION "${DEMO_HOST_QT}/bin/${tool}.exe")
endforeach()
