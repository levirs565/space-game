#ifndef SPACE_GAMEOVERSCREEN_HPP
#define SPACE_GAMEOVERSCREEN_HPP

#include "../TextRenderer.hpp"
#include "../UI/Button.hpp"
#include "../UI/Column.hpp"
#include "../UI/Label.hpp"
#include "../UI/TextInput.hpp"
#include "IScreen.hpp"
#include <functional>

class GameOverScreen : public IScreen {
  Label mGameOverLabel{"Game Over"};
  Label mScoreLabel{"Score: "};
  Label mMessageLabel{"Enter  Your  Name"};
  TextInput mNameInput;
  Button mButton{"Save"};
  Column mColumn;
  Vec2 mFillSize;
  int mScore;
public:
  enum class Event { Quit };
  std::function<void(Event)> mCallback;

public:
  GameOverScreen(std::function<void(Event)> callback);

  void setScore(int score);
  void onSizeChanged(const Vec2 &size) override;
  void onSDLEvent(const SDL_Event &event) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer *renderer) override;
  void onPostDraw() override;
};

#endif // SPACE_GAMEOVERSCREEN_HPP
