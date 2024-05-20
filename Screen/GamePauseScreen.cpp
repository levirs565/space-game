#include "GamePauseScreen.hpp"

GamePauseScreen::GamePauseScreen(std::function<void(Event)> callback)
    : mCallback(std::move(callback)) {
  mColumn.viewList.push_back(&mResumeButton);
  mColumn.viewList.push_back(&mQuitButton);
}
void GamePauseScreen::onSizeChanged(const Vec2 &size) {
  mColumn.layout(size);
  mFillSize = size;
}
void GamePauseScreen::onSDLEvent(const SDL_Event &event) {
  if (event.type == SDL_MOUSEBUTTONDOWN &&
      event.button.button == SDL_BUTTON_LEFT) {
    View *clickedView = mColumn.findByPoint({event.button.x, event.button.y});
    if (clickedView == &mResumeButton)
      mCallback(Event::Resume);
    else if (clickedView == &mQuitButton)
      mCallback(Event::Quit);
  }
}
void GamePauseScreen::onUpdate() { mColumn.update(); }
void GamePauseScreen::onDraw(SDL_Renderer *renderer) {
  SDL_Rect rect{.x = 0, .y = 0, .w = int(mFillSize.x), .h = int(mFillSize.y)};
  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(renderer, 7, 6, 7, 200);
  SDL_RenderFillRect(renderer, &rect);
  mColumn.draw(renderer);
}
void GamePauseScreen::onPostDraw() {}
