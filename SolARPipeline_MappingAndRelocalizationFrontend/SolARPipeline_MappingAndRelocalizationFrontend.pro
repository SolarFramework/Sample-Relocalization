## remove Qt dependencies
QT       -= core gui
CONFIG -= app_bundle qt

QMAKE_PROJECT_DEPTH = 0

## global defintions : target lib name, version
INSTALLSUBDIR = SolARBuild
TARGET = SolARPipelineMappingAndRelocalizationFrontend
FRAMEWORK = $${TARGET}
VERSION=1.0.0

DEFINES += MYVERSION=$${VERSION}
DEFINES += TEMPLATE_LIBRARY
CONFIG += c++1z

include(findremakenrules.pri)

CONFIG(debug,debug|release) {
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    DEFINES += _NDEBUG=1
    DEFINES += NDEBUG=1
}

DEPENDENCIESCONFIG = sharedlib
DEPENDENCIESCONFIG += install_recurse

## Configuration for Visual Studio to install binaries and dependencies. Work also for QT Creator by replacing QMAKE_INSTALL
PROJECTCONFIG = QTVS

#NOTE : CONFIG as staticlib or sharedlib, DEPENDENCIESCONFIG as staticlib or sharedlib and PROJECTDEPLOYDIR MUST BE DEFINED BEFORE templatelibbundle.pri inclusion
include ($$shell_quote($$shell_path($${QMAKE_REMAKEN_RULES_ROOT}/templatelibconfig.pri)))  # Shell_quote & shell_path required for visual on windows

## DEFINES FOR MSVC/INTEL C++ compilers
msvc {
DEFINES += "_BCOM_SHARED=__declspec(dllexport)"
}

INCLUDEPATH += interfaces/

SOURCES += \
    src/SolARMappingAndRelocalizationFrontendPipeline.cpp \
    src/SolARMappingAndRelocalizationFrontendPipeline_main.cpp

HEADERS += \
    interfaces/SolARMappingAndRelocalizationFrontendPipeline.h

unix {
    # Avoids adding install steps manually. To be commented to have a better control over them.
    QMAKE_POST_LINK += "make install"
    QMAKE_CXXFLAGS += -Wignored-qualifiers
}

linux {
    QMAKE_LFLAGS += -ldl
}

win32 {

    DEFINES += WIN64 UNICODE _UNICODE
    QMAKE_COMPILER_DEFINES += _WIN64
    QMAKE_CXXFLAGS += -wd4250 -wd4251 -wd4244 -wd4275
    QMAKE_CXXFLAGS_DEBUG += /Od
    QMAKE_CXXFLAGS_RELEASE += /O2
}

header_files.path = $${PROJECTDEPLOYDIR}/interfaces
header_files.files = $$files($${PWD}/interfaces/*.h*)

xpcf_xml_files.path = $${USERHOMEFOLDER}/.xpcf/SolAR
xpcf_xml_files.files=$$files($${PWD}/xpcf*.xml)

INSTALLS += header_files
INSTALLS += xpcf_xml_files

OTHER_FILES += \
    packagedependencies.txt

DISTFILES += \
    LICENSE \
    README.md \
    bcom-SolARPipelineMapping.pc.in \
    *.xml

#NOTE : Must be placed at the end of the .pro
include ($$shell_quote($$shell_path($${QMAKE_REMAKEN_RULES_ROOT}/remaken_install_target.pri)))) # Shell_quote & shell_path required for visual on windows

