#ifndef SPACE_STARTGAMESCREEN_HPP
#define SPACE_STARTGAMESCREEN_HPP

#include "../UI/Button.hpp"
#include "../UI/Column.hpp"
#include "../UI/HorizontalSlide.hpp"
#include "../UI/Label.hpp"
#include "../UI/Panel.hpp"
#include "../UI/Row.hpp"
#include "IScreen.hpp"
#include "../IGameStage.hpp"

#include <array>

class StartGameScreen : public IScreen {
public:
  enum Event { Back, Start };

private:
  std::function<void(StartGameScreen*, Event)> mEventHandler;
  HorizontalSlide mHorizontalSlide;
  Column mColumn, mFirstColumn, mSecondColumn;
  Panel mFirstPanel{&mFirstColumn}, mSecondPanel{&mSecondColumn};
  Label mTitle{"Start Game", 24};

  Label mColorLabel{"Color"};
  Row mColorsRow{0};
  std::array<std::string, 4> mColorsKeys{"red", "green", "blue", "orange"};
  std::array<Button, 4> mColorsButton{Button{"Red", {.topLeft = 10}},
                                      Button{"Green"}, Button{"Blue"},
                                      Button{"Orange", {.bottomRight = 10}}};

  Label mShipLabel{"Ship"};
  Row mShipsRow{0};
  std::array<Button, 3> mShipsButton{Button{"", {.topLeft = 10}}, Button{""},
                                     Button{"", {.bottomRight = 10}}};

  Row mFirstRow{10};
  Button mBackToMainButton{"Back", {.topLeft = 10}};
  Button mNextButton{"Next", {.bottomRight = 10}};

  Label mModeLabel{"Enemy Mode"};
  std::array<Button, 3> mModeButton{
      Button{"Laser Only", {.topLeft = 10}}, Button{"Missile Only"},
      Button{"Laser with Missile", {.bottomRight = 10}}};

  Row mSecondRow{10};
  Button mBackTo0Button{"Back", {.topLeft = 10}};
  Button mStartButton{"Start", {.bottomRight = 10}};

public:
  std::string selectedColor{mColorsKeys[0]};
  int selectedShip = 1;
  int selectedMode = 0;

  StartGameScreen(std::function<void(StartGameScreen*, Event)> eventHandler);
  void onSizeChanged(const Vec2 &size) override;
  void onSDLEvent(const SDL_Event &event) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer *renderer) override;
  void onPostDraw() override;

  GameParams getGameParams();
};

#endif // SPACE_STARTGAMESCREEN_HPP
