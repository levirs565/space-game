#include <SDL.h>
#include <SDL_image.h>
#include <SDL_mixer.h>
#include <SDL_ttf.h>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <vector>

#include "AI/FlowField.hpp"
#include "Entity/Enemy.hpp"
#include "Entity/PlayerShip.hpp"
#include "Math/Polygon.hpp"
#include "SAP.hpp"
#include "Screen/GameOverScreen.hpp"
#include "Screen/GamePauseScreen.hpp"
#include "Screen/ScoreListScreen.hpp"
#include "Screen/GameScreen.hpp"
#include "Screen/MainScreen.hpp"

class App {
public:
  App() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
      std::cout << "Initializing SDL failed" << std::endl;
      exit(1);
    }
    IMG_Init(IMG_INIT_JPG | IMG_INIT_PNG);
    if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 1024) == 1) {
      std::cout << "Initializing mixer failed" << std::endl;
      exit(1);
    }

    mWindow = SDL_CreateWindow("Space", SDL_WINDOWPOS_UNDEFINED,
                               SDL_WINDOWPOS_UNDEFINED, mWindowSize.x,
                               mWindowSize.y, 0);

    if (!mWindow) {
      std::cout << "Initializing window failed" << std::endl;
      exit(1);
    }

    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");

    mRenderer = SDL_CreateRenderer(mWindow, -1, SDL_RENDERER_ACCELERATED);

    if (!mRenderer) {
      std::cout << "Initializing renderer failed" << std::endl;
      exit(1);
    }

    if (TTF_Init() < 0) {
      std::cout << "TTF Init failed" << std::endl;
      exit(1);
    }

    TextureManager::getInstance()->init(mRenderer);

    mScreen = createMain();
    mScreen->onSizeChanged(mWindowSize);
  }

  std::unique_ptr<IScreen> createMain() {
    return std::make_unique<MainScreen>([this] (auto event) {
        if (event == MainScreen::Event::Exit) {
          this->mIsExit = true;
        } else if (event == MainScreen::Event::Start) {
          mNextScreen = createGameScreen();
        } else if (event == MainScreen::Event::ScoreList) {
          mNextScreen = createScoreListScreen();
        }
      });
  }

  std::unique_ptr<IScreen> createScoreListScreen() {
    return std::make_unique<ScoreListScreen>([this] (auto event) {
      if (event == ScoreListScreen::Event::Back) {
        mNextScreen = createMain();
      }
    });
  }

  std::unique_ptr<IScreen> createGameScreen() {
    return std::make_unique<GameScreen>([this] (auto event) {
      if (event == GameScreen::Event::Quit)
        mNextScreen = createMain();
    });
  }

  void prepareScene() {
    SDL_SetRenderDrawColor(mRenderer, 7, 6, 7, 255);
    SDL_RenderClear(mRenderer);
  }

  void processInput() {
    SDL_Event event;

    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        mIsExit = true;
        return;
      }

      mScreen->onSDLEvent(event);
    }
  }

  void presentScene() { SDL_RenderPresent(mRenderer); }

  void run() {
    while (!mIsExit) {
      prepareScene();
      processInput();

      mScreen->onUpdate();
      mScreen->onDraw(mRenderer);
      presentScene();
      mScreen->onPostDraw();

      if (mNextScreen) {
        std::swap(mNextScreen, mScreen);
        mScreen->onSizeChanged(mWindowSize);
        mNextScreen.reset();
      }

      SDL_Delay(16);
    }
  }

private:
  SDL_Renderer *mRenderer;
  SDL_Window *mWindow;
  std::unique_ptr<IScreen> mScreen;
  std::unique_ptr<IScreen> mNextScreen;
  bool mIsExit = false;
  Vec2 mWindowSize{800, 600};
};

int main(int argc, char **argv) {
  std::string relativeAssetPath = "Data";

  for (int argIndex = 0; argIndex < argc; argIndex++) {
    if (strcmp(*(argv + argIndex), "--data-dir") == 0 && argIndex + 1 < argc) {
      relativeAssetPath = *(argv + argIndex + 1);
    }
  }

  std::filesystem::path assetPath = std::filesystem::current_path();
  assetPath /= relativeAssetPath;
  AssetManager::getInstance()->setRootPath(assetPath);

  SDL_CaptureMouse(SDL_FALSE);

  App app;
  app.run();
  return 0;
}