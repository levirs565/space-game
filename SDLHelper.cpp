#include "SDLHelper.hpp"
#include <array>

SDL_FRect SDLHelper::calculateTextureRectByCenter(SDL_Texture *texture,
                                                  const Vec2 &center,
                                                  double scale) {
  float textureWidth, textureHeight;
  SDL_GetTextureSize(texture, &textureWidth, &textureHeight);

  Vec2 size{double(textureWidth), double(textureHeight)};
  size.scale(scale);

  return calculateRect(center, size);
}
SDL_Texture *SDLHelper::createRoundedRectTexture(
    SDL_Renderer *renderer, int width, int height, int radius,
    const std::function<uint32_t(int x, int y)> &fillFunc) {
  radius = std::min({radius, width / 2, height / 2});

  std::vector<int> sliceX(radius + 1, 0);
  int cx = 0;
  int cy = radius;
  int cd = 3 - 2 * radius;

  while (cx <= cy) {
    sliceX[cy] = cx;
    sliceX[cx] = cy;

    if (cd < 0) {
      cd += 4 * cx + 6;
    } else {
      cd += 4 * (cx - cy) + 10;
      cy--;
    }
    cx++;
  }

  SDL_Texture *texture =
      SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                        SDL_TEXTUREACCESS_STREAMING, width, height);
  if (texture == nullptr) {
    SDL_Log("Create rounded rect texture fail: %s", SDL_GetError());
    return nullptr;
  }

  SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);

  void *pixels = nullptr;
  int pitch = 0;
  if (!SDL_LockTexture(texture, nullptr, &pixels, &pitch)) {
    SDL_Log("SDL_LockTexture failed: %s", SDL_GetError());
    SDL_DestroyTexture(texture);
    return nullptr;
  }

  uint32_t *pixelPtr = reinterpret_cast<uint32_t *>(pixels);
  int pitchPixels = pitch / sizeof(uint32_t);

  SDL_memset(pixels, 0, pitch * height);

  for (int y = 0; y < height; y++) {
    int startX = 0;
    int endX = width - 1;

    if (y < radius) {
      int dy = radius - y;
      int dx = sliceX[dy];
      startX = radius - dx;
      endX = width - 1 - (radius - dx);
    } else if (y >= height - radius) {
      int dy = y - (height - 1 - radius);
      int dx = sliceX[dy];
      startX = radius - dx;
      endX = width - 1 - (radius - dx);
    }

    for (int x = startX; x <= endX; x++) {
      pixelPtr[y * pitchPixels + x] = fillFunc(x, y);
    }
  }

  SDL_UnlockTexture(texture);
  return texture;
}

void drawLineThickness(int x0, int y0, int x1, int y1, int wd,
                       const std::function<void(int x, int y)> &draw) {
  int dx = std::abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = std::abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  float ed = dx + dy == 0 ? 1 : std::sqrt((float)dx * dx + (float)dy * dy);

  wd = (wd + 1) / 2;
  while (true) {
    draw(x0, y0);
    int e2 = err;
    int x2 = x0;
    if (2 * e2 >= -dx) {
      e2 += dy;
      int y2 = y0;
      while (e2 < ed * wd && (y1 != y2 || dx > dy)) {
        draw(x0, y2 += sy);
        e2 += dx;
      }
      if (x0 == x1)
        break;
      e2 = err;
      err -= dy;
      x0 += sx;
    }
    if (2 * e2 <= dy) {
      e2 = dx - e2;
      while (e2 < ed * wd && (x1 != x2 || dx < dy)) {
        draw(x2 += sx, y0);
        e2 += dy;
      }
      if (y0 == y1)
        break;
      err += dx;
      y0 += sy;
    }
  }
}

void drawLineThickness2(int x0, int y0, int x1, int y1, int wd,
                        bool rightDirection,
                        const std::function<void(int x, int y)> &draw) {
  int dx = std::abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = std::abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  float ed = dx + dy == 0 ? 1 : std::sqrt((float)dx * dx + (float)dy * dy);

  while (true) {
    draw(x0, y0);
    int e2 = err;
    int x2 = x0;
    if (2 * e2 >= -dx) {
      int ty = rightDirection ? sx : -sx;

      e2 += dy;
      int y2 = y0;
      while (e2 < ed * wd && (y1 != y2 || dx > dy)) {
        draw(x0, y2 += ty);
        e2 += dx;
      }
      if (x0 == x1)
        break;
      e2 = err;
      err -= dy;
      x0 += sx;
    }
    if (2 * e2 <= dy) {
      int tx = rightDirection ? -sy : sy;

      e2 = dx - e2;
      while (e2 < ed * wd && (x1 != x2 || dx < dy)) {
        draw(x2 += tx, y0);
        e2 += dy;
      }
      if (y0 == y1)
        break;
      err += dx;
      y0 += sy;
    }
  }
}

std::vector<int> generateBresenhamX(int x0, int y0, int x1, int y1,
                                    bool findMin) {
  int dx = std::abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  int x = x0;
  int y = y0;

  std::vector<std::pair<int, int>> points;

  while (true) {
    points.emplace_back(x, y);
    if (x == x1 && y == y1)
      break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }

  std::vector<int> slicesX(std::abs(y1 - y0) + 1,
                           findMin ? std::numeric_limits<int>::max()
                                   : std::numeric_limits<int>::min());

  for (const auto [x, y] : points) {
    int index = std::abs(y - y0);
    slicesX[index] =
        findMin ? std::min(slicesX[index], x) : std::max(slicesX[index], x);
  }

  return slicesX;
}

SDL_Texture *SDLHelper::createBeveledRectTexture(
    SDL_Renderer *renderer, int width, int height, const Radius &targetRadius,
    const std::function<uint32_t(int x, int y)> &fillFunc) {
  Radius radius = targetRadius;
  int maxRadius = std::min(width - 1, height - 1);
  radius.topLeft = std::min(radius.topLeft, maxRadius);
  radius.topRight = std::min(radius.topRight, maxRadius);
  radius.bottomLeft = std::min(radius.bottomLeft, maxRadius);
  radius.bottomRight = std::min(radius.bottomRight, maxRadius);

  int bottomLeftStart = height - 1 - radius.bottomLeft;
  int bottomRightStart = height - 1 - radius.bottomRight;

  auto topLeftSlices =
      generateBresenhamX(radius.topLeft, 0, 0, radius.topLeft, true);
  auto topRightSlices = generateBresenhamX(width - 1 - radius.topRight, 0,
                                           width - 1, radius.topRight, false);
  auto bottomLeftSlices = generateBresenhamX(
      0, bottomLeftStart, radius.bottomLeft, height - 1, true);
  auto bottomRightSlices =
      generateBresenhamX(width - 1, bottomRightStart,
                         width - 1 - radius.bottomRight, height - 1, false);

  SDL_Texture *texture =
      SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                        SDL_TEXTUREACCESS_STREAMING, width, height);
  if (texture == nullptr) {
    SDL_Log("Create rounded rect texture fail: %s", SDL_GetError());
    return nullptr;
  }

  SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);

  void *pixels = nullptr;
  int pitch = 0;
  if (!SDL_LockTexture(texture, nullptr, &pixels, &pitch)) {
    SDL_Log("SDL_LockTexture failed: %s", SDL_GetError());
    SDL_DestroyTexture(texture);
    return nullptr;
  }

  uint32_t *pixelPtr = reinterpret_cast<uint32_t *>(pixels);
  int pitchPixels = pitch / sizeof(uint32_t);

  SDL_memset(pixels, 0, pitch * height);

  for (int y = 0; y < height; y++) {
    int startX = 0;
    int endX = width - 1;

    if (y <= radius.topLeft) {
      startX = topLeftSlices[y];
    }
    if (y <= radius.topRight) {
      endX = topRightSlices[y];
    }
    if (y >= bottomLeftStart) {
      startX = bottomLeftSlices[y - bottomLeftStart];
    }
    if (y >= bottomRightStart) {
      endX = bottomRightSlices[y - bottomRightStart];
    }

    for (int x = startX; x <= endX; x++) {
      pixelPtr[y * pitchPixels + x] = fillFunc(x, y);
    }
  }

  SDL_UnlockTexture(texture);
  return texture;
}
SDL_Texture *SDLHelper::createBeveledRectTextureOutline(
    SDL_Renderer *renderer, int width, int height, int thickness,
    const Radius &targetRadius,
    const std::function<uint32_t(int x, int y)> &fillFunc) {
  Radius radius = targetRadius;
  int maxRadius = std::min(width - 1, height - 1);
  radius.topLeft = std::min(radius.topLeft, maxRadius);
  radius.topRight = std::min(radius.topRight, maxRadius);
  radius.bottomLeft = std::min(radius.bottomLeft, maxRadius);
  radius.bottomRight = std::min(radius.bottomRight, maxRadius);

  int bottomLeftStart = height - 1 - radius.bottomLeft;
  int bottomRightStart = height - 1 - radius.bottomRight;

  SDL_Texture *texture =
      SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                        SDL_TEXTUREACCESS_STREAMING, width, height);
  if (texture == nullptr) {
    SDL_Log("Create rounded rect texture fail: %s", SDL_GetError());
    return nullptr;
  }

  SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);

  void *pixels = nullptr;
  int pitch = 0;
  if (!SDL_LockTexture(texture, nullptr, &pixels, &pitch)) {
    SDL_Log("SDL_LockTexture failed: %s", SDL_GetError());
    SDL_DestroyTexture(texture);
    return nullptr;
  }

  uint32_t *pixelPtr = reinterpret_cast<uint32_t *>(pixels);
  int pitchPixels = pitch / sizeof(uint32_t);

  SDL_memset(pixels, 0, pitch * height);

  auto draw = [&](int x, int y) {
    if (x < 0 || x >= width || y < 0 || y >= height) {
      // SDL_Log("draw out of bound %d, %d", x, y);
      return;
    }

    pixelPtr[y * pitchPixels + x] = fillFunc(x, y);
  };

  std::vector<std::tuple<int, int>> points{
      {0, radius.topLeft},
      {radius.topLeft, 0},
      {width - 1 - radius.topRight, 0},
      {width - 1, radius.topRight},
      {width - 1, bottomRightStart},
      {width - 1 - radius.bottomRight, height - 1},
      {radius.bottomLeft, height - 1},
      {0, bottomLeftStart}};

  for (size_t i = 0; i < points.size(); i++) {
    auto [x0, y0] = points[i];
    auto [x1, y1] = points[(i + 1) % points.size()];

    drawLineThickness2(x0, y0, x1, y1, thickness, true, draw);
  }

  SDL_UnlockTexture(texture);
  return texture;
}

SDL_FRect SDLHelper::calculateRect(const Vec2 &center, const Vec2 &size) {
  Vec2 halfSize{size};
  halfSize.scale(0.5);

  Vec2 topLeft{center};
  topLeft.substract(halfSize);

  return {.x = float(topLeft.x),
          .y = float(topLeft.y),
          .w = float(size.x),
          .h = float(size.y)};
}
