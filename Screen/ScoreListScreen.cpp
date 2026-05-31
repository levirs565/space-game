#include "ScoreListScreen.hpp"
#include "../ScoreListManager.hpp"

ScoreListScreen::ScoreListScreen(std::function<void(Event)> callback)
    : mCallback(std::move(callback)) {
  mColumn.viewList.push_back(&mTitle);
  mColumn.viewList.push_back(&mPanel);

  auto list = ScoreListManager::getList();

  for (int i = 0; LRLabel &label : mScoreLabel) {
    mInnerColumn.viewList.push_back(&label);
    if (i < list.size()) {
      label.setText(list[i].name, std::to_string(list[i].score));
    } else {
      label.setText("", "");
    }
    i++;
  }

  mInnerColumn.viewList.push_back(&mBackButton);

  mBackButton.onClickHandler = [this](Button * button) {
    mCallback(Event::Back);
    return true;
  };
}
void ScoreListScreen::onSizeChanged(const Vec2 &size) {
  mSize = size;
  mColumn.layout(size);
  for (LRLabel &label : mScoreLabel) {
    label.layout();
  }
}
void ScoreListScreen::onSDLEvent(const SDL_Event &event) {
  mColumn.handleSDLEvent(event);
}
void ScoreListScreen::onUpdate() { mColumn.update(); }
void ScoreListScreen::onDraw(SDL_Renderer *renderer) { mColumn.draw(renderer); }

void ScoreListScreen::onPostDraw() {}
