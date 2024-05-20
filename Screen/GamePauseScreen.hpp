#ifndef SPACE_GAMEPAUSESCREEN_HPP
#define SPACE_GAMEPAUSESCREEN_HPP

#include "IScreen.hpp"
#include "../UI/Column.hpp"
#include "../UI/Button.hpp"

class GamePauseScreen : public IScreen {
  Button mResumeButton{"Resume"};
  Button mQuitButton{"Quit"};
  Column mColumn;
public:
  GamePauseScreen();

  void onSizeChanged(const Vec2& size) override;
  void onSDLEvent(const SDL_Event& event) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer* renderer) override;
  void onPostDraw() override;
};

#endif // SPACE_GAMEPAUSESCREEN_HPP
