#include "MainScreen.hpp"
#include <utility>

MainScreen::MainScreen(std::function<void(Event)> eventHandler)
    : mEventHandler(std::move(eventHandler)) {
  mColumn.viewList.push_back(&mTitle);
  mColumn.viewList.push_back(&mStartButton);
  mColumn.viewList.push_back(&mScoreListButton);
  mColumn.viewList.push_back(&mAboutButton);
  mColumn.viewList.push_back(&mExitButton);
}

void MainScreen::onSizeChanged(const Vec2 &size) {
  mSize = size;
  mColumn.layout(size);
}
void MainScreen::onSDLEvent(const SDL_Event &event) {
  if (event.type == SDL_MOUSEBUTTONDOWN &&
      event.button.button == SDL_BUTTON_LEFT) {
    View* clickedView = mColumn.findByPoint({event.button.x, event.button.y});
    if (clickedView != nullptr) {
      if (mEventMap.contains(clickedView))
        mEventHandler(mEventMap[clickedView]);
    }
  }
}

void MainScreen::onUpdate() {
  mColumn.update();
}

void MainScreen::onDraw(SDL_Renderer *renderer) {
  mColumn.draw(renderer);
}
void MainScreen::onPostDraw() {}
