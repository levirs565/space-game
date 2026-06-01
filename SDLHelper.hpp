#ifndef SPACE_SDLHELPER_HPP
#define SPACE_SDLHELPER_HPP

#include "Math/Vec2.hpp"
#include <SDL3/SDL.h>

#include <functional>

namespace SDLHelper {
struct Radius {
  int topLeft = 0, topRight = 0, bottomLeft = 0, bottomRight = 0;
};

SDL_FRect calculateRect(const Vec2 &center, const Vec2 &size);
SDL_FRect calculateTextureRectByCenter(SDL_Texture *texture, const Vec2 &center,
                                       double scale);
SDL_Texture *
createRoundedRectTexture(SDL_Renderer *renderer, int width, int height,
                         int radius,
                         const std::function<uint32_t(int x, int y)> &fillFunc);
SDL_Texture *createBeveledRectTexture(SDL_Renderer *renderer, int width,
                                      int height, const Radius &targetRadius,
                                      Uint32 color);
SDL_Texture *createBeveledRectTextureOutline(SDL_Renderer *renderer, int width,
                                             int height, int thickness,
                                             const Radius &targetRadius,
                                             Uint32 color);
SDL_Texture *createCircleTexture(SDL_Renderer *renderer, int size,
                                 Uint32 color);
SDL_Texture *createCircleTextureOutline(SDL_Renderer *renderer, int size,
                                        int thickness, Uint32 color);

SDL_FColor hexToFColor(Uint32 hex);
void drawBeveledRect(SDL_Renderer *renderer, const SDL_FRect &rect,
                     const Radius &targetRadius, const SDL_FColor &color);
void drawBeveledRectOutline(SDL_Renderer *renderer, const SDL_FRect &rect,
                            float thickness, const Radius &targetRadius,
                            const SDL_FColor &color);
void drawCircleGPU(SDL_Renderer *renderer, const Vec2 &center, float radius,
                   const SDL_FColor &color, int segments);
void drawCircleOutlineGPU(SDL_Renderer *renderer, const Vec2 &center,
                          float radius, float thickness,
                          const SDL_FColor &color, int segments);
} // namespace SDLHelper

#endif // SPACE_SDLHELPER_HPP
