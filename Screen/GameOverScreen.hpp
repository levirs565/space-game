#ifndef SPACE_GAMEOVERSCREEN_HPP
#define SPACE_GAMEOVERSCREEN_HPP

#include "IScreen.hpp"
#include "../TextRenderer.hpp"
#include "../UI/Column.hpp"
#include "../UI/TextInput.hpp"
#include "../UI/Button.hpp"
#include "../UI/Label.hpp"

class GameOverScreen : public IScreen {
  Label mGameOverLabel{"Game Over"};
  Label mMessageLabel{"Enter  Your  Name"};
  TextInput mNameInput;
  Button mButton{"OK"};
  Column mColumn;
public:
  GameOverScreen();

  void onSizeChanged(const Vec2& size) override;
  void onSDLEvent(const SDL_Event& event) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer* renderer) override;
  void onPostDraw() override;
};

#endif // SPACE_GAMEOVERSCREEN_HPP
