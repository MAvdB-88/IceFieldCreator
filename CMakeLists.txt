cmake_minimum_required(VERSION 2.8)
project(IceFieldCreator)
set(PROJECT_NAME IceFieldCreator)

set(OpenCV_DIR "C:/opencv/opencv/build")

find_package(OpenCV)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(C:/rapidjson-master/include)
include_directories(C:/boost_1_67_0)

add_executable(IceFieldCreator 
src/IceFieldCreator.cpp
src/IceFieldCreator.h
src/CollisionDetector.cpp
src/CollisionDetector.h
src/DynamicAabbTree.cpp
src/DynamicAabbTree.h
src/Orientation.h
src/Transform.h
src/Utils.h
src/Utils.cpp
src/Contact.cpp
src/Contact.h
src/InputData.h
src/Main.cpp
src/BodyVec.cpp
src/BodyVec.h
src/Body.cpp
src/Body.h
src/Solver.cpp
src/Solver.h
src/UserInput.h
src/CommandLineReader.cpp
src/CommandLineReader.h
src/ErrorLogger.cpp
src/ErrorLogger.h
src/SolverConstraint.h
src/Contact.cpp
src/Contact.h
src/Reader.cpp
src/Reader.h
src/Writer.cpp
src/Writer.h
src/Shape.cpp
src/Shape.h
src/Vector2.h
src/AABB.cpp
src/AABB.h
src/GUI.cpp
src/GUI.h
src/DesktopResolution.cpp
src/DesktopResolution.h)

target_link_libraries(IceFieldCreator ${OpenCV_LIBS} )
