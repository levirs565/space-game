#include "ScoreListScreen.hpp"
#include "../ScoreListManager.hpp"

ScoreListScreen::ScoreListScreen(std::function<void(Event)> callback)
    : mCallback(std::move(callback)) {
  mColumn.viewList.push_back(&mTitle);

  auto list = ScoreListManager::getList();

  for (int i = 0; LRLabel & label : mScoreLabel) {
    mColumn.viewList.push_back(&label);
    if (i < list.size()) {
      label.setText(list[i].name, std::to_string(list[i].score));
    } else {
      label.setText("", "");
    }
    i++;
  }

  mColumn.viewList.push_back(&mBackButton);
}
void ScoreListScreen::onSizeChanged(const Vec2 &size) {
  mSize = size;
  mColumn.layout(size);
  for (LRLabel &label : mScoreLabel) {
    label.layout();
  }
}
void ScoreListScreen::onSDLEvent(const SDL_Event &event) {
  if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
    View* view = mColumn.findByPoint({event.button.x, event.button.y});
    if (view == &mBackButton) {
      mCallback(Event::Back);
    }
  }
}
void ScoreListScreen::onUpdate() { mColumn.update(); }
void ScoreListScreen::onDraw(SDL_Renderer *renderer) { mColumn.draw(renderer); }

void ScoreListScreen::onPostDraw() {}
