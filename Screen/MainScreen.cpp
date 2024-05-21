#include "MainScreen.hpp"
#include <utility>

MainScreen::MainScreen(std::function<void(Event)> eventHandler)
    : mEventHandler(std::move(eventHandler)) {
  mColumn.viewList.push_back(&mTitle);

  mNamaArray[0].setText("Levi Rizki S", "123230127");
  mNamaArray[1].setText("Faiz Muhammad A", "123230128");
  mNamaArray[2].setText("Raymond Agung R", "123230129");
  mNamaArray[3].setText("Amanda Latifah", "123230138");
  mNamaArray[4].setText("Isyraf Fajar A", "123230140");

  for (LRLabel& label : mNamaArray) {
    mColumn.viewList.push_back(&label);
  }
  
  mColumn.viewList.push_back(&mStartButton);
  mColumn.viewList.push_back(&mScoreListButton);
  mColumn.viewList.push_back(&mExitButton);
}

void MainScreen::onSizeChanged(const Vec2 &size) {
  mSize = size;
  mColumn.layout(size);
  for (LRLabel& label : mNamaArray) {
    label.layout();
  }
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
