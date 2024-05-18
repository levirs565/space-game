#include "AssetManager.hpp"

AssetManager *AssetManager::getInstance() {
  static AssetManager instance;
  return &instance;
}

void AssetManager::setRootPath(std::filesystem::path path) {
  if (!std::filesystem::is_directory(path))
    throw std::invalid_argument(path.string() + " is not directory");

  mRootPath = std::move(path);
  mIsInitialized = true;
}
std::filesystem::path
AssetManager::getAsset(const std::filesystem::path &name) {
  if (!mIsInitialized)
    throw std::runtime_error("AssetManager not initialized");

  std::filesystem::path path = mRootPath / name;

  if (!std::filesystem::is_regular_file(path))
    throw AssetNotFoundException(mRootPath, name);

  return path;
}

SDL_Texture *TextureManager::load(const std::string &name) {
  if (!mIsInitialized)
      throw std::runtime_error("TextureManager is not initialized");

  if (mCache.count(name) > 0)
    return mCache[name].get();

  SDL_LogMessage(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO,
                 "Loading texture %s", name.c_str());
  std::filesystem::path path = AssetManager::getInstance()->getAsset(name);
  SDL_Texture *texture = IMG_LoadTexture(mRenderer, path.c_str());
  mCache[name] = std::unique_ptr<SDL_Texture, TextureDeleter>(texture);
  return texture;
}
void TextureManager::init(SDL_Renderer *renderer) {
  mRenderer = renderer;
  mIsInitialized = true;
}
void TextureManager::clear() {
  mRenderer = nullptr;
  mIsInitialized = false;
  mCache.clear();
}

TextureManager *TextureManager::getInstance() {
  static TextureManager instance;
  return &instance;
}
