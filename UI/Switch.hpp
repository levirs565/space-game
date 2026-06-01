#ifndef SPACE_SWITCH_HPP
#define SPACE_SWITCH_HPP
#include "Label.hpp"
#include "View.hpp"

class SwitchButton : public View {
  SDL_Texture *mBackgroundTexture = nullptr;
  SDL_Texture *mOutlineTexture = nullptr;
  SDL_Texture *mCircleTexture = nullptr;

  SDL_Texture *mActiveOutlineTexture = nullptr;
  SDL_Texture *mActiveCircleTexture = nullptr;

  Vec2 mCircleCenter;
  double mActiveOpacity = 0;
  bool mFirstUpdate = true;

  static constexpr double mCircleSize = 28;

public:
  Vec2 getLayoutSize() override;
  SDL_FRect getRect() override;
  void update() override;
  void draw(SDL_Renderer *renderer) override;
  bool value = false;
  bool onClick(SDL_FPoint point) override;
};
#endif // SPACE_SWITCH_HPP
