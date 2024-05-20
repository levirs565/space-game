#include "GameScreen.hpp"

void GameScreen::onSizeChanged(const Vec2 &size) {
  mStageScreen.onSizeChanged(size);
  mPauseScreen.onSizeChanged(size);
}
void GameScreen::onSDLEvent(const SDL_Event &event) {
  if (event.type == SDL_KEYDOWN) {
    if (event.key.keysym.scancode == SDL_SCANCODE_ESCAPE) {
      mIsPause = !mIsPause;
    }
  }

  if (!mIsPause)
    mStageScreen.onSDLEvent(event);
  else
    mPauseScreen.onSDLEvent(event);
}
void GameScreen::onUpdate() {
  if (!mIsPause)
    mStageScreen.onUpdate();
  else
    mPauseScreen.onUpdate();
}
void GameScreen::onDraw(SDL_Renderer *renderer) {
  mStageScreen.onDraw(renderer);
  if (mIsPause)
    mPauseScreen.onDraw(renderer);
}
void GameScreen::onPostDraw() {
  if (!mIsPause)
    mStageScreen.onPostDraw();
  else
    mPauseScreen.onPostDraw();
}

GameScreen::GameScreen(std::function<void(Event)> callback)
    : mCallback(std::move(callback)),
      mPauseScreen([this](GamePauseScreen::Event event) {
        if (event == GamePauseScreen::Event::Resume)
          mIsPause = false;
        else if (event == GamePauseScreen::Event::Quit)
          mCallback(Event::Quit);
      }) {}
