#include "AboutScreen.hpp"

AboutScreen::AboutScreen(std::function<void(Event)> callback)
    : mCallback(std::move(callback)) {
  mLabelArray[0].setText("Copyright 2024, Levi Rizki Saputra");
  mLabelArray[1].setText("Thanks to:");
  mLabelArray[2].setText("Faiz Muhammad A");
  mLabelArray[3].setText("Raymond Agung R");
  mLabelArray[4].setText("Amanda Latifah");
  mLabelArray[5].setText("Isyraf Fajar A");

  mColumn.viewList.push_back(&mTitle);
  mColumn.viewList.push_back(&mPanel);

  for (Label &label : mLabelArray) {
    mInnerColumn.viewList.push_back(&label);
  }

  mInnerColumn.viewList.push_back(&mCloseButton);

  mCloseButton.onClickHandler = [this](Button *button) {
    mCallback(Event::Close);
    return true;
  };
}

void AboutScreen::onSizeChanged(const Vec2 &size) { mColumn.layout(size); }

void AboutScreen::onSDLEvent(const SDL_Event &event) {
  mColumn.handleSDLEvent(event);
}

void AboutScreen::onUpdate() { mColumn.update(); }

void AboutScreen::onDraw(SDL_Renderer *renderer) { mColumn.draw(renderer); }

void AboutScreen::onPostDraw() {}