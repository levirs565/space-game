#include "MainScreen.hpp"

#include <utility>
#include "../AssetManager.hpp"

MainButton::MainButton(std::string text) : mText(std::move(text)) {
  mButtonTexture = TextureManager::getInstance()->load("PNG/UI/buttonBlue.png");
  mFont = FontManager::getInstance()->load("Bonus/kenvector_future.ttf", 16);
  SDL_Color color{.r = 0, .g = 0, .b = 0, .a = 255};
  mTextSurface = TTF_RenderText_Solid(mFont, mText.c_str(), color);
}
Vec2 MainButton::getFocusSize() {
  int buttonWidth, buttonHeight;
  SDL_QueryTexture(mButtonTexture, nullptr, nullptr, &buttonWidth, &buttonHeight);
  return {double(buttonWidth) * mFocusScale, double(buttonHeight) * mFocusScale};
}

void MainButton::draw(SDL_Renderer *renderer) {
  if (mTextTexture == nullptr) {
    mTextTexture = SDL_CreateTextureFromSurface(renderer, mTextSurface);
  }

  SDL_Rect rect = calculateTextureRect(mButtonTexture);
  SDL_RenderCopy(renderer, mButtonTexture, nullptr, &rect);
  rect = calculateTextureRect(mTextTexture);
  SDL_RenderCopy(renderer, mTextTexture, nullptr, &rect);
}

MainButton::~MainButton() {
  if (mTextTexture != nullptr) {
    SDL_DestroyTexture(mTextTexture);
  }
  SDL_FreeSurface(mTextSurface);
}
void MainButton::update() {
  double targetScale = mFocus ? mFocusScale : 1;
  mScale += (targetScale - mScale) * (1.0  - std::exp(- 0.5));
  mScale = std::clamp(mScale, 1.0, mFocusScale);
}

SDL_Rect MainButton::calculateTextureRect(SDL_Texture *texture) {
  int textureWidth, textureHeight;
  SDL_QueryTexture(texture, nullptr, nullptr, &textureWidth, &textureHeight);

  Vec2 size{double(textureWidth), double(textureHeight)};
  size.scale(mScale);

  Vec2 halfSize{size};
  halfSize.scale(0.5);

  Vec2 topLeft{mCenterPosition};
  topLeft.substract(halfSize);

  return {.x = int(std::floor(topLeft.x)),
          .y = int(std::floor(topLeft.y)),
          .w = int(std::ceil(size.x)),
          .h = int(std::ceil(size.y))};
}

MainScreen::MainScreen(std::function<void(Event)> eventHandler)
  : mEventHandler(std::move(eventHandler)){

}

void MainScreen::onSizeChanged(const Vec2 &size) {
  mSize = size;

  constexpr double margin = 25;
  Vec2 focusSize = mButtonArray[0].first.getFocusSize();
  Vec2 allSize{focusSize.x, focusSize.y * double(mButtonArray.size()) + margin * (int(mButtonArray.size()) - 1)};

  Vec2 centerPosition = mSize;
  centerPosition.scale(0.5);
  centerPosition.add(allSize, -0.5);
  centerPosition.add(focusSize, 0.5);

  for (auto& [button, event] : mButtonArray) {
    button.setCenter(centerPosition);
    centerPosition.y += margin + focusSize.y;
  }
}
void MainScreen::onSDLEvent(const SDL_Event &event) {
  if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
    SDL_Point point{
      event.button.x,
          event.button.y
    };

    for (auto& [button, event] : mButtonArray) {
      SDL_Rect rect = button.getRect();
      if (SDL_PointInRect(&point, &rect) == SDL_TRUE) {
        mEventHandler(event);
        break;
      }
    }
  }
}

void MainScreen::onUpdate() {
  SDL_Point mouse;
  SDL_GetMouseState(&mouse.x, &mouse.y);
  for (auto& [button, _] : mButtonArray) {
    SDL_Rect rect = button.getRect();
    button.setFocus(SDL_PointInRect(&mouse, &rect) == SDL_TRUE);
    button.update();
  }
}

void MainScreen::onDraw(SDL_Renderer *renderer) {
  for (auto& [button, _] : mButtonArray) {
    button.draw(renderer);
  }
}
void MainScreen::onPostDraw() {}
