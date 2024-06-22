cmake_minimum_required(VERSION 3.22)
project(space)

set(CMAKE_CXX_STANDARD 20)

find_package(SDL2 REQUIRED)
find_package(SDL2_image REQUIRED)
find_package(SDL2_mixer REQUIRED)
find_package(SDL2_ttf REQUIRED)

find_package(SDL2 REQUIRED CONFIG COMPONENTS SDL2main)

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
        Screen/GameStageScreen.cpp
        Screen/GameStageScreen.hpp
        Screen/MainScreen.cpp
        Screen/MainScreen.hpp
        Map.hpp
        Map.cpp
        TextRenderer.cpp
        TextRenderer.hpp
        Screen/GameOverScreen.cpp
        Screen/GameOverScreen.hpp
        SDLHelper.hpp
        SDLHelper.cpp
        UI/View.cpp
        UI/View.hpp
        UI/Button.cpp
        UI/Button.hpp
        UI/Column.cpp
        UI/Column.hpp
        UI/TextInput.cpp
        UI/TextInput.hpp
        UI/Label.cpp
        UI/Label.hpp
        Screen/GamePauseScreen.cpp
        Screen/GamePauseScreen.hpp
        Screen/GameScreen.cpp
        Screen/GameScreen.hpp
        Entity/PowerUpHealth.cpp
        Entity/PowerUpHealth.hpp
        ScoreListManager.hpp
        ScoreListManager.cpp
        UI/LRLabel.cpp
        UI/LRLabel.hpp
        Screen/ScoreListScreen.cpp
        Screen/ScoreListScreen.hpp
        Entity/Ship.cpp
        Entity/Ship.hpp
        Particle/Particle.hpp
        Particle/Particle.cpp
        Particle/Explosion.cpp
        Particle/Explosion.hpp
)

if(TARGET SDL2::SDL2main)
    target_link_libraries(space PRIVATE SDL2::SDL2main)
endif()

target_link_libraries(space PRIVATE SDL2::SDL2 SDL2_image::SDL2_image SDL2_mixer::SDL2_mixer SDL2_ttf::SDL2_ttf)