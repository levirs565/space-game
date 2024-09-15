#ifndef SPACE_ABOUTSCREEN_H
#define SPACE_ABOUTSCREEN_H

#include <functional>
#include <array>
#include "IScreen.hpp"
#include "../UI/Column.hpp"
#include "../UI/Label.hpp"
#include "../UI/Button.hpp"


class AboutScreen : public IScreen {
public:
  enum class Event { Close };
private:
  Column mColumn;
  Button mCloseButton{"Close"};
  std::array<Label, 7> mLabelArray;
  std::function<void(Event)> mCallback;
public:
  explicit AboutScreen(std::function<void(Event)> callback);

  void onSizeChanged(const Vec2& size) override;
  void onSDLEvent(const SDL_Event& event) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer* renderer) override;
  void onPostDraw() override;
};


#endif //ABOUTSCREEN_H
