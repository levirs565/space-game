#ifndef SPACE_GAMEPAUSESCREEN_HPP
#define SPACE_GAMEPAUSESCREEN_HPP

#include <functional>
#include "IScreen.hpp"
#include "../UI/Column.hpp"
#include "../UI/Button.hpp"

class GamePauseScreen : public IScreen {
public:
  enum class Event {
    Resume,
    Quit
  };
private:
  Button mResumeButton{"Resume"};
  Button mQuitButton{"Quit"};
  Column mColumn;
  std::function<void(Event)> mCallback;
  Vec2 mFillSize;
public:
  GamePauseScreen(std::function<void(Event)> callback);

  void onSizeChanged(const Vec2& size) override;
  void onSDLEvent(const SDL_Event& event) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer* renderer) override;
  void onPostDraw() override;
};

#endif // SPACE_GAMEPAUSESCREEN_HPP
