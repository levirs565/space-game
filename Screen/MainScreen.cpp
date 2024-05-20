#include "MainScreen.hpp"
#include <utility>

MainScreen::MainScreen(std::function<void(Event)> eventHandler)
    : mEventHandler(std::move(eventHandler)) {
  mColumn.viewList.push_back(&mStartButton);
  mColumn.viewList.push_back(&mExitButton);
}

void MainScreen::onSizeChanged(const Vec2 &size) {
  mSize = size;
  mColumn.layout(size);
}
void MainScreen::onSDLEvent(const SDL_Event &event) {
  if (event.type == SDL_MOUSEBUTTONDOWN &&
      event.button.button == SDL_BUTTON_LEFT) {
    SDL_Point point{event.button.x, event.button.y};
    for (View* view : mColumn.viewList) {
      if (view->isPointInside(point)) {
        if (mEventMap.contains(view))
          mEventHandler(mEventMap[view]);
        break;
      }
    }
  }
}

void MainScreen::onUpdate() {
  SDL_Point mouse;
  SDL_GetMouseState(&mouse.x, &mouse.y);

  for (View* view : mColumn.viewList) {
    auto button = dynamic_cast<Button*>(view);
    if (button == nullptr) continue;

    button->setFocus(button->isPointInside(mouse));
  }

  mColumn.update();
}

void MainScreen::onDraw(SDL_Renderer *renderer) {
  mColumn.draw(renderer);
}
void MainScreen::onPostDraw() {}
