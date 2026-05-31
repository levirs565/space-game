#include "GamePauseScreen.hpp"

GamePauseScreen::GamePauseScreen(std::function<void(Event)> callback)
    : mCallback(std::move(callback)) {
  mColumn.viewList.push_back(&mResumeButton);
  mColumn.viewList.push_back(&mQuitButton);

  mResumeButton.onClickHandler = [this](Button * button) {
    mCallback(Event::Resume);
    return true;
  };

  mQuitButton.onClickHandler = [this](Button * button) {
    mCallback(Event::Quit);
    return true;
  };
}
void GamePauseScreen::onSizeChanged(const Vec2 &size) {
  mColumn.layout(size);
  mFillSize = size;
}
void GamePauseScreen::onSDLEvent(const SDL_Event &event) {
  mColumn.handleSDLEvent(event);
}
void GamePauseScreen::onUpdate() { mColumn.update(); }
void GamePauseScreen::onDraw(SDL_Renderer *renderer) {
  SDL_FRect rect{.x = 0, .y = 0, .w = float(mFillSize.x), .h = float(mFillSize.y)};
  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(renderer, 7, 6, 7, 200);
  SDL_RenderFillRect(renderer, &rect);
  mColumn.draw(renderer);
}
void GamePauseScreen::onPostDraw() {}
