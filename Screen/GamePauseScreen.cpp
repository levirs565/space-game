#include "GamePauseScreen.hpp"

GamePauseScreen::GamePauseScreen() {
  mColumn.viewList.push_back(&mResumeButton);
  mColumn.viewList.push_back(&mQuitButton);
}
void GamePauseScreen::onSizeChanged(const Vec2 &size) { mColumn.layout(size); }
void GamePauseScreen::onSDLEvent(const SDL_Event &event) {}
void GamePauseScreen::onUpdate() { mColumn.update(); }
void GamePauseScreen::onDraw(SDL_Renderer *renderer) { mColumn.draw(renderer); }
void GamePauseScreen::onPostDraw() {}
