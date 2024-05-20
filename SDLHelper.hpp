#ifndef SPACE_SDLHELPER_HPP
#define SPACE_SDLHELPER_HPP

#include <SDL.h>
#include "Math/Vec2.hpp"

namespace SDLHelper {
SDL_Rect calculateRect(const Vec2& center, const Vec2& size);
SDL_Rect calculateTextureRectByCenter(SDL_Texture *texture, const Vec2 &center,
                                      double scale);
}

#endif // SPACE_SDLHELPER_HPP
