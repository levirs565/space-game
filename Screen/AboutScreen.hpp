#ifndef SPACE_ABOUTSCREEN_H
#define SPACE_ABOUTSCREEN_H

#include "../UI/Button.hpp"
#include "../UI/Column.hpp"
#include "../UI/Label.hpp"
#include "../UI/Panel.hpp"
#include "IScreen.hpp"
#include <array>
#include <functional>

class AboutScreen : public IScreen {
public:
  enum class Event { Close };
private:
  Column mColumn, mInnerColumn;
  Panel mPanel{&mInnerColumn};
  Label mTitle{"Space Game", 24};
  Button mCloseButton{"Close", {.topLeft = 10, .bottomRight = 10}};
  std::array<Label, 6> mLabelArray;
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
