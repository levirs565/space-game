#ifndef SPACE_MAINSCREEN_HPP
#define SPACE_MAINSCREEN_HPP

#include "IScreen.hpp"
#include <SDL.h>
#include <SDL_ttf.h>
#include <array>
#include <functional>
#include <string>
#include <map>
#include "../UI/Button.hpp"
#include "../UI/Column.hpp"
#include "../UI/Label.hpp"
#include "../UI/LRLabel.hpp"
#include <array>

class MainScreen : public IScreen {
public:
  enum class Event { Exit, Start, ScoreList, About };

private:
  Vec2 mSize;
  Label mTitle {"Space Game"};
  Button mStartButton{"Start"};
  Button mExitButton{"Exit"};
  Button mScoreListButton{"Score List"};
  Button mAboutButton{"About"};
  Column mColumn;
  std::map<View*, Event> mEventMap = {
      {&mStartButton, Event::Start},
      {&mScoreListButton, Event::ScoreList},
      {&mExitButton, Event::Exit},
    {&mAboutButton, Event::About}
  };

  std::function<void(Event)> mEventHandler;

public:
  explicit MainScreen(std::function<void(Event)> eventHandler);

  void onSizeChanged(const Vec2 &size) override;
  void onSDLEvent(const SDL_Event &event) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer *renderer) override;
  void onPostDraw() override;
};

#endif // SPACE_MAINSCREEN_HPP
