#include "GameOverScreen.hpp"
#include "../AssetManager.hpp"
#include "../SDLHelper.hpp"

GameOverScreen::GameOverScreen(std::function<void(Event)> callback): mCallback(std::move(callback)) {
  mColumn.viewList.push_back(&mGameOverLabel);
  mColumn.viewList.push_back(&mMessageLabel);
  mColumn.viewList.push_back(&mNameInput);
  mColumn.viewList.push_back(&mButton);
}

void GameOverScreen::onSizeChanged(const Vec2 &size) {
  mColumn.layout(size);
  mFillSize = size;
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
  if (event.type == SDL_MOUSEBUTTONDOWN &&
      event.button.button == SDL_BUTTON_LEFT) {
    View *clickedView = mColumn.findByPoint({event.button.x, event.button.y});
    if (clickedView == &mButton)
      mCallback(Event::Quit);
  }
}
void GameOverScreen::onUpdate() {
  mColumn.update();
}
void GameOverScreen::onDraw(SDL_Renderer *renderer) {
  SDL_Rect rect{.x = 0, .y = 0, .w = int(mFillSize.x), .h = int(mFillSize.y)};
  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(renderer, 7, 6, 7, 200);
  SDL_RenderFillRect(renderer, &rect);
  mColumn.draw(renderer);
}
void GameOverScreen::onPostDraw() {}
