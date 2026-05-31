#include "MainScreen.hpp"
#include <utility>

MainScreen::MainScreen(std::function<void(Event)> eventHandler)
    : mEventHandler(std::move(eventHandler)) {
  mColumn.viewList.push_back(&mTitle);
  mColumn.viewList.push_back(&mStartButton);
  mColumn.viewList.push_back(&mScoreListButton);
  mColumn.viewList.push_back(&mSettingsButton);
  mColumn.viewList.push_back(&mAboutButton);
  mColumn.viewList.push_back(&mExitButton);

  mStartButton.onClickHandler = [this](Button * button) {
    mEventHandler(Event::Start);
    return true;
  };

  mScoreListButton.onClickHandler = [this](Button * button) {
    mEventHandler(Event::ScoreList);
    return true;
  };

  mSettingsButton.onClickHandler = [this](Button * button) {
    mEventHandler(Event::Settings);
    return true;
  };

  mAboutButton.onClickHandler = [this](Button * button) {
    mEventHandler(Event::About);
    return true;
  };

  mExitButton.onClickHandler = [this](Button * button) {
    mEventHandler(Event::Exit);
    return true;
  };
}

void MainScreen::onSizeChanged(const Vec2 &size) {
  mSize = size;
  mColumn.layout(size);
}
void MainScreen::onSDLEvent(const SDL_Event &event) {
  mColumn.handleSDLEvent(event);
}

void MainScreen::onUpdate() {
  mColumn.update();
}

void MainScreen::onDraw(SDL_Renderer *renderer) {
  mColumn.draw(renderer);
}
void MainScreen::onPostDraw() {}
