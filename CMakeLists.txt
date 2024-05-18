cmake_minimum_required(VERSION 3.22)
project(space)

set(CMAKE_CXX_STANDARD 20)

find_package(SDL2 REQUIRED)
find_package(SDL2_image REQUIRED)
find_package(SDL2_mixer REQUIRED)
find_package(SDL2_ttf REQUIRED)

add_executable(space
        main.cpp
        Math/Vec2.hpp
        Entity/GameEntity.hpp
        Entity/GameEntity.cpp
        IGameStage.hpp
        SAP.hpp
        DataFormat.hpp
        DataFormat.cpp
        SAP.cpp
        Math/Polygon.hpp
        Math/Polygon.cpp
        Entity/Laser.hpp
        AssetManager.cpp
        AssetManager.hpp
        Entity/PlayerShip.hpp
        AI/ContextSteering.hpp
        AI/FlowField.hpp
        Math/Helper.hpp
        Entity/Enemy.hpp
        Math/Helper.cpp
        Entity/Meteor.hpp
        Entity/PlayerShip.cpp
        Entity/Enemy.cpp
        Entity/Meteor.cpp
        Entity/Laser.cpp
        AI/ContextSteering.cpp
        AI/FlowField.cpp
        Screen/IScreen.hpp
        Screen/GameScreen.cpp
        Screen/GameScreen.hpp)
target_link_libraries(space PRIVATE SDL2::SDL2 SDL2_image::SDL2_image SDL2_mixer::SDL2_mixer SDL2_ttf::SDL2_ttf)