#ifndef SPACE_MAINSCREEN_HPP
#define SPACE_MAINSCREEN_HPP

#include "IScreen.hpp"
#include <SDL.h>
#include <SDL_ttf.h>
#include <array>
#include <functional>
#include <string>

class MainButton {
  std::string mText;
  double mScale = 1;
  bool mFocus = false;

  SDL_Texture *mButtonTexture;

  TTF_Font *mFont;
  SDL_Surface *mTextSurface;
  SDL_Texture *mTextTexture = nullptr;

  Vec2 mCenterPosition;

  SDL_Rect calculateTextureRect(SDL_Texture *texture);

  static constexpr double mFocusScale = 1.05;

public:
  explicit MainButton(std::string text);
  ~MainButton();

  void setCenter(const Vec2 &center) { mCenterPosition = center; }
  void setFocus(bool focus) { mFocus = focus; }

  void update();
  void draw(SDL_Renderer *renderer);

  Vec2 getFocusSize();

  SDL_Rect getRect() { return calculateTextureRect(mButtonTexture); }
};

class MainScreen : public IScreen {
public:
  enum class Event { Exit, Start };

private:
  Vec2 mSize;
  MainButton mStartButton{"Start"};
  MainButton mExitButton{"Exit"};
  std::array<std::pair<MainButton &, Event>, 2> mButtonArray = {
      {{mStartButton, Event::Start}, {mExitButton, Event::Exit}}};

  std::function<void(Event)> mEventHandler;

public:
  explicit MainScreen(std::function<void(Event)> eventHandler);

  void onSizeChanged(const Vec2 &size) override;
  void onSDLEvent(const SDL_Event &event) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer *renderer) override;
  void onPostDraw() override;
};

#endif // SPACE_MAINSCREEN_HPP
