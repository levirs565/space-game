#include "AboutScreen.hpp"

AboutScreen::AboutScreen(std::function<void(Event)> callback)
    : mCallback(std::move(callback)) {
  mLabelArray[0].setText("Levi Rizki Saputra (123230127)");
  mLabelArray[1].setText("Farhannivta Ramadhana (123230139)");
  mLabelArray[2].setText("Muhammad Aditya N (123230146)");
  mLabelArray[3].setText("Muhammad Adel Harits (123230156)");
  mLabelArray[4].setText("Taura Kaka Arissa (123230217)");
  mLabelArray[5].setText("");

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