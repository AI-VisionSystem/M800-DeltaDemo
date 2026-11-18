QT += core widgets network
CONFIG += c++17
TARGET = DeltaDemo
TEMPLATE = app
DEFINES += QT_DEPRECATED_WARNINGS
INCLUDEPATH += $$PWD/src
SOURCES += src/main.cpp src/Settings.cpp src/MainWindow.cpp
HEADERS += src/Settings.h src/RobotController.h src/MainWindow.h src/DeltaFeedback.h
RESOURCES += resources/app.qrc
win32: RC_ICONS = resources/app.ico
CONFIG(preview) {
    SOURCES += src/PreviewController.cpp
} else {
    DEFINES += DEMO_WITH_SDK=1
    SOURCES += src/RobotController.cpp
    win32 {
        INCLUDEPATH += $$PWD/sdk/windows/include
        LIBS += $$quote($$PWD/sdk/windows/lib/sldmv.lib)
    } else {
        INCLUDEPATH += $$PWD/sdk/rk3588/include
        !exists($$PWD/sdk/rk3588/lib/libsldml.so): error("Missing M800 ARM64 SDK; see sdk/README.md")
        LIBS += -L$$PWD/sdk/rk3588/lib -lsldml
        QMAKE_RPATHDIR += \$$ORIGIN
    }
}
win32 {
    DESTDIR = $$PWD/../DeltaDemo_Runtime/Windows
    QMAKE_CXXFLAGS += /utf-8 /wd4819 /wd4828
    !CONFIG(preview): QMAKE_POST_LINK += $$QMAKE_COPY $$shell_path($$PWD/sdk/windows/lib/sldmv.dll) $$shell_path($$DESTDIR/sldmv.dll)
} else {
    DESTDIR = $$PWD/../DeltaDemo_Runtime/RK3588
    !CONFIG(preview): QMAKE_POST_LINK += $$QMAKE_COPY $$shell_path($$PWD/sdk/rk3588/lib/libsldml.so) $$shell_path($$DESTDIR/libsldml.so)
}
