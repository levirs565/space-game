#include "GameOverScreen.hpp"
#include "../AssetManager.hpp"
#include "../SDLHelper.hpp"

GameOverScreen::GameOverScreen() {
  mColumn.viewList.push_back(&mNameInput);
  mColumn.viewList.push_back(&mButton);
}

void GameOverScreen::onSizeChanged(const Vec2 &size) {
  mColumn.layout(size);
}

void GameOverScreen::onSDLEvent(const SDL_Event &event) {
  if (event.type == SDL_TEXTINPUT) {
    mNameInput.setText(mNameInput.getText() + std::string(event.text.text));
  }
  if (event.type == SDL_KEYDOWN && event.key.keysym.scancode == SDL_SCANCODE_BACKSPACE) {
    const std::string& text = mNameInput.getText();
    if (!text.empty()) {
      mNameInput.setText(text.substr(0, text.length() - 1));
    }
  }
}
void GameOverScreen::onUpdate() {
  mColumn.update();
}
void GameOverScreen::onDraw(SDL_Renderer *renderer) {
  mColumn.draw(renderer);
}
void GameOverScreen::onPostDraw() {}
