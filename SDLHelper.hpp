#ifndef SPACE_SDLHELPER_HPP
#define SPACE_SDLHELPER_HPP

#include <SDL3/SDL.h>
#include "Math/Vec2.hpp"

namespace SDLHelper {
SDL_FRect calculateRect(const Vec2& center, const Vec2& size);
SDL_FRect calculateTextureRectByCenter(SDL_Texture *texture, const Vec2 &center,
                                      double scale);
}

#endif // SPACE_SDLHELPER_HPP
