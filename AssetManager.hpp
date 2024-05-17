#ifndef SPACE_ASSETMANAGER_HPP
#define SPACE_ASSETMANAGER_HPP

#include <string>
#include <unordered_map>

#include <SDL.h>
#include <SDL_image.h>

class AssetManager {};

class TextureLoader {
public:
  explicit TextureLoader(SDL_Renderer *renderer) : mRenderer(renderer) {}

  TextureLoader(const TextureLoader &other) = delete;

  SDL_Texture *load(const std::string &name) {
    if (mCache.count(name) > 0)
      return mCache[name];

    SDL_LogMessage(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO,
                   "Loading texture %s", name.c_str());
    SDL_Texture *texture = IMG_LoadTexture(mRenderer, name.c_str());
    mCache[name] = texture;
    return texture;
  }

private:
  SDL_Renderer *mRenderer;
  std::unordered_map<std::string, SDL_Texture *> mCache;
};

#endif // SPACE_ASSETMANAGER_HPP
