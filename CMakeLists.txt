#Copyright (c) 2022 Ultimaker B.V.
#Copyright (c) 2022 PICASO 3D
#PicasoXCore is released under the terms of the AGPLv3 or higher

cmake_policy(SET CMP0091 NEW)  # For MSVC flags, will be ignored on non-Windows OS's
cmake_minimum_required(VERSION 3.20)
project(PicasoXCore)
find_package(standardprojectsettings REQUIRED)  # imports the cmake module https://github.com/Ultimaker/conan-ultimaker-index/recipes/standardprojectsettings
AssureOutOfSourceBuilds()

set(PICASO_XCORE_VERSION "default" CACHE STRING "Version name of PicasoXCore")
option(ENABLE_ARCUS "Enable support for ARCUS" ON)
option(ENABLE_TESTING "Build with unit tests" OFF)
option(EXTENSIVE_WARNINGS "Build with all warnings" ON)
option(ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS "Enable more optimization flags" ON)
option(USE_SYSTEM_LIBS "Use the system libraries if available" OFF)

if (NOT APPLE)
    option(ENABLE_OPENMP "Use OpenMP for parallel code" ON)
endif ()

# Create Protobuf files if Arcus is used
if (ENABLE_ARCUS)
    message(STATUS "Building with Arcus")

    find_package(arcus REQUIRED)
    find_package(protobuf REQUIRED)
    protobuf_generate_cpp(engine_PB_SRCS engine_PB_HEADERS Cura.proto)
endif ()

### Compiling PicasoXCore ###
# First compile all of PicasoXCore as library, allowing this to be re-used for tests.

set(engine_SRCS # Except main.cpp.
        src/Application.cpp
        src/bridge.cpp
        src/ConicalOverhang.cpp
        src/ExtruderTrain.cpp
        src/FffGcodeWriter.cpp
        src/FffPolygonGenerator.cpp
        src/FffProcessor.cpp
        src/gcodeExport.cpp
        src/GCodePathConfig.cpp
        src/infill.cpp
        src/InsetOrderOptimizer.cpp
        src/layerPart.cpp
        src/LayerPlan.cpp
        src/LayerPlanBuffer.cpp
        src/mesh.cpp
        src/MeshGroup.cpp
        src/Mold.cpp
        src/multiVolumes.cpp
        src/PathOrderPath.cpp
        src/Preheat.cpp
        src/PrimeTower.cpp
        src/raft.cpp
        src/Scene.cpp
        src/SkeletalTrapezoidation.cpp
        src/SkeletalTrapezoidationGraph.cpp
        src/skin.cpp
        src/SkirtBrim.cpp
        src/SupportInfillPart.cpp
        src/Slice.cpp
        src/sliceDataStorage.cpp
        src/slicer.cpp
        src/support.cpp
        src/timeEstimate.cpp
        src/TopSurface.cpp
        src/TreeModelVolumes.cpp
        src/TreeSupport.cpp
        src/WallsComputation.cpp
        src/Weaver.cpp
        src/Wireframe2gcode.cpp
        src/WallToolPaths.cpp

        src/BeadingStrategy/BeadingStrategy.cpp
        src/BeadingStrategy/BeadingStrategyFactory.cpp
        src/BeadingStrategy/DistributedBeadingStrategy.cpp
        src/BeadingStrategy/LimitedBeadingStrategy.cpp
        src/BeadingStrategy/RedistributeBeadingStrategy.cpp
        src/BeadingStrategy/WideningBeadingStrategy.cpp
        src/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp

        src/communication/ArcusCommunication.cpp
        src/communication/ArcusCommunicationPrivate.cpp
        src/communication/CommandLine.cpp
        src/communication/Listener.cpp

        src/infill/ImageBasedDensityProvider.cpp
        src/infill/NoZigZagConnectorProcessor.cpp
        src/infill/ZigzagConnectorProcessor.cpp
        src/infill/LightningDistanceField.cpp
        src/infill/LightningGenerator.cpp
        src/infill/LightningLayer.cpp
        src/infill/LightningTreeNode.cpp
        src/infill/SierpinskiFill.cpp
        src/infill/SierpinskiFillProvider.cpp
        src/infill/SubDivCube.cpp
        src/infill/GyroidInfill.cpp
        src/infill/PicasoHoneyCombInfill.cpp

        src/pathPlanning/Comb.cpp
        src/pathPlanning/GCodePath.cpp
        src/pathPlanning/LinePolygonsCrossings.cpp
        src/pathPlanning/NozzleTempInsert.cpp
        src/pathPlanning/TimeMaterialEstimates.cpp

        src/progress/Progress.cpp
        src/progress/ProgressStageEstimator.cpp

        src/settings/AdaptiveLayerHeights.cpp
        src/settings/FlowTempGraph.cpp
        src/settings/PathConfigStorage.cpp
        src/settings/Settings.cpp
        src/settings/ZSeamConfig.cpp

        src/utils/AABB.cpp
        src/utils/AABB3D.cpp
        src/utils/Date.cpp
        src/utils/ExtrusionJunction.cpp
        src/utils/ExtrusionLine.cpp
        src/utils/ExtrusionSegment.cpp
        src/utils/FMatrix4x3.cpp
        src/utils/gettime.cpp
        src/utils/LinearAlg2D.cpp
        src/utils/ListPolyIt.cpp
        src/utils/MinimumSpanningTree.cpp
        src/utils/md5.cpp
        src/utils/Point3.cpp
        src/utils/PolygonConnector.cpp
        src/utils/PolygonsPointIndex.cpp
        src/utils/PolygonsSegmentIndex.cpp
        src/utils/polygonUtils.cpp
        src/utils/polygon.cpp
        src/utils/PolylineStitcher.cpp
        src/utils/ProximityPointLink.cpp
        src/utils/Simplify.cpp
        src/utils/SVG.cpp
        src/utils/socket.cpp
        src/utils/SquareGrid.cpp
        src/utils/ThreadPool.cpp
        src/utils/ToolpathVisualizer.cpp
        src/utils/VoronoiUtils.cpp
        )

add_library(_PicasoXCore STATIC ${engine_SRCS} ${engine_PB_SRCS})
use_threads(_PicasoXCore)

target_include_directories(_PicasoXCore
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # Include Cura.pb.h
        )
target_compile_definitions(_PicasoXCore
        PUBLIC
        $<$<BOOL:${ENABLE_ARCUS}>:ARCUS>
        $<$<BOOL:${ENABLE_TESTING}>:BUILD_TESTS>
        PRIVATE
        VERSION=\"${PICASO_XCORE_VERSION}\"
        $<$<BOOL:${WIN32}>:NOMINMAX>
        $<$<CONFIG:Debug>:ASSERT_INSANE_OUTPUT>
        $<$<CONFIG:Debug>:USE_CPU_TIME>
        $<$<CONFIG:Debug>:DEBUG>
        $<$<CONFIG:RelWithDebInfo>:ASSERT_INSANE_OUTPUT>
        $<$<CONFIG:RelWithDebInfo>:USE_CPU_TIME>
        $<$<CONFIG:RelWithDebInfo>:DEBUG>
        )

enable_sanitizers(_PicasoXCore)

if (${EXTENSIVE_WARNINGS})
    set_project_warnings(_PicasoXCore)
endif ()

if (ENABLE_OPENMP)
    find_package(OpenMP REQUIRED)
    target_link_libraries(_PicasoXCore PUBLIC OpenMP::OpenMP_CXX)
endif ()

if (ENABLE_ARCUS)
    target_link_libraries(_PicasoXCore PRIVATE arcus::arcus protobuf::libprotobuf)
endif ()

find_package(clipper REQUIRED)
find_package(RapidJSON REQUIRED)
find_package(stb REQUIRED)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(range-v3 REQUIRED)

if (ENABLE_TESTING)
    find_package(GTest REQUIRED)
endif ()

target_link_libraries(_PicasoXCore
        PUBLIC
        spdlog::spdlog
        PRIVATE
        range-v3::range-v3
        fmt::fmt
        clipper::clipper
        rapidjson
        stb::stb
        boost::boost
        $<$<BOOL:${ENABLE_TESTING}>:GTest::gtest>)

if (NOT WIN32)
    add_executable(PicasoXCore src/main.cpp) # Then compile main.cpp as separate executable, and link the library to it.
else ()
    message(STATUS "Using windres")
    set(RES_FILES "PicasoXCore.rc")
    ENABLE_LANGUAGE(RC)
    if (NOT MSVC)
        SET(CMAKE_RC_COMPILER_INIT windres)
        SET(CMAKE_RC_COMPILE_OBJECT
                "<CMAKE_RC_COMPILER> <FLAGS> -O coff <DEFINES> -i <SOURCE> -o <OBJECT>"
                )
    endif ()
    add_executable(PicasoXCore src/main.cpp ${RES_FILES}) # ..., but don't forget the glitter!
endif (NOT WIN32)

# Create the executable
target_link_libraries(PicasoXCore PRIVATE _PicasoXCore)
target_compile_definitions(PicasoXCore PRIVATE VERSION=\"${PICASO_XCORE_VERSION}\")

# Compiling the test environment.
if (ENABLE_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif ()
