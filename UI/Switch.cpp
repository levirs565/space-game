#include "Switch.hpp"

#include "../AppSettings.hpp"
#include "../SDLHelper.hpp"
#include "../Screen/GameStageScreen.hpp"

#include <algorithm>

SwitchButton::~SwitchButton() {
  SDL_DestroyTexture(mOutlineTexture);
  SDL_DestroyTexture(mActiveOutlineTexture);
  SDL_DestroyTexture(mCircleTexture);
  SDL_DestroyTexture(mBackgroundTexture);
  SDL_DestroyTexture(mActiveCircleTexture);
}
Vec2 SwitchButton::getLayoutSize() { return {75, 39}; }

SDL_FRect SwitchButton::getRect() { return calculateRect(getLayoutSize()); }
void SwitchButton::update() {
  double targetOpacity = value ? 1.0 : 0.0;
  mActiveOpacity += (targetOpacity - mActiveOpacity) * (1.0 - std::exp(-0.5));
  mActiveOpacity = std::clamp(mActiveOpacity, 0.0, 1.0);

  const double circleOffset = mCircleSize / 2 + 5;
  const double circleCenterOn = getLayoutSize().x - circleOffset;
  const double circleCenterOff = circleOffset;
  double targetCircleCenter = value ? circleCenterOn : circleCenterOff;
  mCircleCenter.x +=
      (targetCircleCenter - mCircleCenter.x) * (1.0 - std::exp(-0.5));
  mCircleCenter.x =
      std::clamp(mCircleCenter.x, circleCenterOff, circleCenterOn);

  if (mFirstUpdate) {
    mActiveOpacity = targetOpacity;
    mCircleCenter.x = targetCircleCenter;
    mFirstUpdate = false;
  }

  mCircleCenter.y = getLayoutSize().y / 2;
}

bool SaveTextureToPNG(SDL_Renderer *renderer, SDL_Texture *texture,
                      const char *filename) {
  if (!renderer || !texture || !filename) {
    return false;
  }

  float width, height;
  if (!SDL_GetTextureSize(texture, &width, &height)) {
    SDL_Log("Failed to get texture size: %s", SDL_GetError());
    return false;
  }

  int w = (int)width;
  int h = (int)height;

  SDL_Texture *oldTarget = SDL_GetRenderTarget(renderer);

  SDL_Texture *renderTexture = SDL_CreateTexture(
      renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, w, h);

  if (!SDL_SetRenderTarget(renderer, renderTexture)) {
    SDL_Log("Texture must be created with SDL_TEXTUREACCESS_TARGET: %s",
            SDL_GetError());
    return false;
  }

  SDL_RenderTexture(renderer, texture, nullptr, nullptr);
  SDL_Surface *surface = SDL_RenderReadPixels(renderer, nullptr);

  SDL_SetRenderTarget(renderer, oldTarget);

  if (!surface) {
    SDL_Log("Failed to read pixels from texture: %s", SDL_GetError());
    return false;
  }

  bool success = SDL_SavePNG(surface, filename);
  if (!success) {
    SDL_Log("Failed to save PNG: %s", SDL_GetError());
  }

  SDL_DestroyTexture(renderTexture);
  SDL_DestroySurface(surface);

  return success;
}

void SwitchButton::draw(SDL_Renderer *renderer) {
  AppSettings *settings = getAppSettings();
  SDLHelper::Radius radius = {.topLeft = 10, .bottomRight = 10};
  int thickness = 2;
  Uint32 backgroundColor = 0x000000FF, outlineColor = 0x004c69FF,
         activeOutlineColor = 0x37bbf5ff;

  if (mOutlineTexture == nullptr && !settings->uiHardwareRendering) {
    Vec2 size = getLayoutSize();
    mBackgroundTexture = SDLHelper::createBeveledRectTexture(
        renderer, size.x, size.y, radius, backgroundColor);

    mOutlineTexture = SDLHelper::createBeveledRectTextureOutline(
        renderer, size.x, size.y, thickness, radius, outlineColor);

    mActiveOutlineTexture = SDLHelper::createBeveledRectTextureOutline(
        renderer, size.x, size.y, thickness, radius, activeOutlineColor);

    mCircleTexture = SDLHelper::createCircleTextureOutline(
        renderer, mCircleSize, thickness, outlineColor);
    mActiveCircleTexture = SDLHelper::createCircleTexture(renderer, mCircleSize,
                                                          activeOutlineColor);
  }

  Vec2 circlePosition =
      getCenterPosition() - 0.5 * getLayoutSize() + mCircleCenter;
  SDL_FRect circleRect = SDLHelper::calculateTextureRectByCenter(
      mCircleTexture, circlePosition, 1.0);
  if (!settings->uiHardwareRendering) {
    SDL_FRect backgroundRect = calculateTextureRect(mBackgroundTexture, 1.0);
    SDL_RenderTexture(renderer, mBackgroundTexture, nullptr, &backgroundRect);

    SDL_FRect rect = calculateTextureRect(mOutlineTexture, 1.0);
    SDL_RenderTexture(renderer, mOutlineTexture, nullptr, &rect);

    SDL_RenderTexture(renderer, mCircleTexture, nullptr, &circleRect);

    SDL_SetTextureBlendMode(mActiveOutlineTexture, SDL_BLENDMODE_BLEND);
    SDL_SetTextureAlphaModFloat(mActiveOutlineTexture, mActiveOpacity);

    SDL_FRect activeRect = calculateTextureRect(mActiveOutlineTexture, 1.0);
    SDL_RenderTexture(renderer, mActiveOutlineTexture, nullptr, &activeRect);

    SDL_SetTextureBlendMode(mActiveCircleTexture, SDL_BLENDMODE_BLEND);
    SDL_SetTextureAlphaModFloat(mActiveCircleTexture, mActiveOpacity);

    SDL_FRect activeCircleRect = SDLHelper::calculateTextureRectByCenter(
        mActiveCircleTexture, circlePosition, 1.0);
    SDL_RenderTexture(renderer, mActiveCircleTexture, nullptr,
                      &activeCircleRect);
  } else {
    SDL_FRect rect = getRect();
    SDLHelper::drawBeveledRect(renderer, rect, radius,
                               SDLHelper::hexToFColor(backgroundColor));
    SDL_FColor fColor = SDLHelper::hexToFColor(outlineColor);
    SDLHelper::drawBeveledRectOutline(renderer, rect, thickness, radius,
                                      fColor);

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_FColor fActiveColor = SDLHelper::hexToFColor(activeOutlineColor);
    fActiveColor.a = mActiveOpacity;
    SDLHelper::drawBeveledRectOutline(renderer, rect, thickness, radius,
                                      fActiveColor);

    SDLHelper::drawCircleOutlineGPU(renderer, circlePosition,
                                    mCircleSize / 2.0f, thickness, fColor, 32);

    SDLHelper::drawCircleGPU(renderer, circlePosition, mCircleSize / 2.0f,
                             fActiveColor, 32);
  }
}
bool SwitchButton::onClick(SDL_FPoint point) {
  value = !value;
  return true;
}