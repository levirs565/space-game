#include "GameOverScreen.hpp"
#include "../AssetManager.hpp"
#include "../SDLHelper.hpp"
#include "../ScoreListManager.hpp"

GameOverScreen::GameOverScreen(SDL_Window *window,
                               std::function<void(Event)> callback)
    : mWindow(window), mCallback(std::move(callback)) {
  mColumn.viewList.push_back(&mGameOverLabel);
  mColumn.viewList.push_back(&mScoreLabel);
  mColumn.viewList.push_back(&mMessageLabel);
  mColumn.viewList.push_back(&mNameInput);
  mColumn.viewList.push_back(&mButton);
  SDL_StartTextInput(mWindow);
}
GameOverScreen::~GameOverScreen() { SDL_StopTextInput(mWindow); }

void GameOverScreen::onSizeChanged(const Vec2 &size) {
  mColumn.layout(size);
  mFillSize = size;
}

void GameOverScreen::onSDLEvent(const SDL_Event &event) {
  if (event.type == SDL_EVENT_TEXT_INPUT) {
    mNameInput.setText(mNameInput.getText() + std::string(event.text.text));
  }
  if (event.type == SDL_EVENT_KEY_DOWN &&
      event.key.scancode == SDL_SCANCODE_BACKSPACE) {
    const std::string &text = mNameInput.getText();
    if (!text.empty()) {
      mNameInput.setText(text.substr(0, text.length() - 1));
    }
  }
  if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN &&
      event.button.button == SDL_BUTTON_LEFT) {
    View *clickedView = mColumn.findByPoint({event.button.x, event.button.y});
    if (clickedView == &mButton) {
      std::string name = mNameInput.getText();
      if (name.empty())
        name = "<Anonymous>";
      ScoreListManager::addScore(name, mScore);
      mCallback(Event::Quit);
    }
  }
}
void GameOverScreen::onUpdate() { mColumn.update(); }
void GameOverScreen::onDraw(SDL_Renderer *renderer) {
  SDL_FRect rect{
      .x = 0, .y = 0, .w = float(mFillSize.x), .h = float(mFillSize.y)};
  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(renderer, 7, 6, 7, 200);
  SDL_RenderFillRect(renderer, &rect);
  mColumn.draw(renderer);
}
void GameOverScreen::onPostDraw() {}

void GameOverScreen::setScore(int score) {
  mScore = score;
  mScoreLabel.setText("Score: " + std::to_string(score));
  mColumn.layout(mFillSize);
}
