#ifndef SPACE_SCORELISTSCREEN_HPP
#define SPACE_SCORELISTSCREEN_HPP

#include "IScreen.hpp"
#include "../UI/Column.hpp"
#include "../UI/LRLabel.hpp"
#include "../UI/Button.hpp"
#include <array>
#include <functional>

class ScoreListScreen : public IScreen {
  Label mTitle{"Scores"};
  std::array<LRLabel, 10> mScoreLabel;
  Button mBackButton{"Back"};
  Column mColumn;
  Vec2 mSize;
public:
  enum class Event {
    Back
  };
private:
  std::function<void(Event)> mCallback;
public:
  ScoreListScreen(std::function<void(Event)> callback);

  void onSizeChanged(const Vec2& size) override;
  void onSDLEvent(const SDL_Event& event) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer* renderer) override;
  void onPostDraw() override;
};

#endif // SPACE_SCORELISTSCREEN_HPP
