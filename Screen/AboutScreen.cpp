#include "AboutScreen.hpp"

AboutScreen::AboutScreen(std::function<void(Event)> callback):
  mCallback(std::move(callback)) {
  mLabelArray[0].setText("Space Game");
  mLabelArray[1].setText("Copyright 2024, Levi Rizki Saputra");
  mLabelArray[2].setText("Thanks to:");
  mLabelArray[3].setText("Faiz Muhammad A");
  mLabelArray[4].setText("Raymond Agung R");
  mLabelArray[5].setText("Amanda Latifah");
  mLabelArray[6].setText("Isyraf Fajar A");

  for (Label &label : mLabelArray) {
    mColumn.viewList.push_back(&label);
  }

  mColumn.viewList.push_back(&mCloseButton);
}

void AboutScreen::onSizeChanged(const Vec2 &size) {
  mColumn.layout(size);
}

void AboutScreen::onSDLEvent(const SDL_Event &event) {
  if (event.type == SDL_MOUSEBUTTONDOWN &&
      event.button.button == SDL_BUTTON_LEFT) {
    View *clickedView = mColumn.findByPoint({event.button.x, event.button.y});
    if (clickedView == &mCloseButton) {
      mCallback(Event::Close);
    }
  }
}

void AboutScreen::onUpdate() {
  mColumn.update();
}

void AboutScreen::onDraw(SDL_Renderer *renderer) {
  mColumn.draw(renderer);
}

void AboutScreen::onPostDraw() {
}