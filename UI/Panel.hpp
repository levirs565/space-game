#ifndef SPACE_PANEL_HPP
#define SPACE_PANEL_HPP
#include "View.hpp"

class Panel : public View {
  SDL_Texture *mOutlineTexture = nullptr;
  SDL_Texture *mBackgroundTexture = nullptr;
  View *mChild = nullptr;

  Vec2 mPadding = {10, 10};

public:
  explicit Panel(View *child);
  Vec2 getLayoutSize() override;
  SDL_FRect getRect() override;
  void update() override;
  void draw(SDL_Renderer *renderer) override;
  void setCenterPosition(const Vec2 &centerPosition) override;
  bool onClick(SDL_FPoint point) override;
};

#endif // SPACE_PANEL_HPP
