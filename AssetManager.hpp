#ifndef SPACE_ASSETMANAGER_HPP
#define SPACE_ASSETMANAGER_HPP

#include <string>
#include <unordered_map>

#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <filesystem>
#include <utility>

class AssetNotFoundException : public std::runtime_error {
public:
  AssetNotFoundException(const std::filesystem::path &rootPath,
                         const std::filesystem::path &name)
    : std::runtime_error("Asset " + name.string() + " not found in " + rootPath.string()) {
  }
};

class AssetManager {
  std::filesystem::path mRootPath;
  bool mIsInitialized = false;

  AssetManager() = default;

public:
  AssetManager(AssetManager const &) = delete;
  AssetManager(AssetManager const &&) = delete;
  void operator=(AssetManager const &) = delete;

  static AssetManager *getInstance();

  void setRootPath(std::filesystem::path path);
  std::filesystem::path getAsset(const std::filesystem::path &name);
};

class TextureManager {
  struct TextureDeleter {
    void operator()(SDL_Texture* texture) const {
      SDL_DestroyTexture(texture);
    }
  };

  bool mIsInitialized = false;
  SDL_Renderer *mRenderer;
  std::unordered_map<std::string, std::unique_ptr<SDL_Texture, TextureDeleter>> mCache;

  TextureManager() = default;
public:
  TextureManager(TextureManager const &) = delete;
  TextureManager(TextureManager const &&) = delete;
  void operator=(TextureManager const &) = delete;

  static TextureManager* getInstance();

  void init(SDL_Renderer* renderer);
  void clear();

  SDL_Texture *load(const std::string &name);
};

class FontManager {
  struct FontDeleter {
    void operator()(TTF_Font *font) const {
      TTF_CloseFont(font);
    }
  };

  std::unordered_map<std::string, std::unique_ptr<TTF_Font, FontDeleter>> mCache;

  FontManager() = default;
public:
  FontManager(FontManager const&) = delete;
  FontManager(FontManager const&&) = delete;
  void operator=(FontManager const&) = delete;

  static FontManager* getInstance();

  TTF_Font* load(const std::string &name, int size);
};

#endif // SPACE_ASSETMANAGER_HPP
