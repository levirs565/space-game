#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include <SDL3_mixer/SDL_mixer.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <vector>

#include "AI/FlowField.hpp"
#include "Entity/Enemy.hpp"
#include "Entity/PlayerShip.hpp"
#include "Screen/AboutScreen.hpp"
#include "Screen/GamePauseScreen.hpp"
#include "Screen/GameScreen.hpp"
#include "Screen/MainScreen.hpp"
#include "Screen/ScoreListScreen.hpp"
#include "Screen/SettingsScreen.hpp"
#include "Screen/StartGameScreen.hpp"

class App {
public:
  App() {
    if (SDL_Init(SDL_INIT_VIDEO) == false) {
      std::cout << "Initializing SDL failed" << std::endl;
      exit(1);
    }

    if (MIX_Init() == false) {
      std::cout << "Initializing SDL_mixer failed" << std::endl;
      exit(1);
    }
    mMixer = MIX_CreateMixerDevice(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, nullptr);
    if (!mMixer) {
      std::cout << "Failed to create Mixer device" << std::endl;
      exit(1);
    }

    float mainScale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
    mWindow = SDL_CreateWindow("Space", mWindowSize.x * mainScale,
                               mWindowSize.y * mainScale,
                               SDL_WINDOW_HIGH_PIXEL_DENSITY);

    if (!mWindow) {
      std::cout << "Initializing window failed" << std::endl;
      exit(1);
    }

    mRenderer = SDL_CreateRenderer(mWindow, nullptr);

    if (!mRenderer) {
      std::cout << "Initializing renderer failed" << std::endl;
      exit(1);
    }

    SDL_SetRenderLogicalPresentation(mRenderer, mWindowSize.x, mWindowSize.y,
                                     SDL_LOGICAL_PRESENTATION_LETTERBOX);

    if (TTF_Init() == false) {
      std::cout << "TTF Init failed" << std::endl;
      exit(1);
    }

    TextureManager::getInstance()->init(mRenderer);

    mScreen = createMain();
    mScreen->onSizeChanged(mWindowSize);
  }

  std::unique_ptr<IScreen> createMain() {
    return std::make_unique<MainScreen>([this](auto event) {
      if (event == MainScreen::Event::Exit) {
        this->mIsExit = true;
      } else if (event == MainScreen::Event::Start) {
        mNextScreen = createStartGameScreen();
      } else if (event == MainScreen::Event::ScoreList) {
        mNextScreen = createScoreListScreen();
      } else if (event == MainScreen::Event::Settings) {
        mNextScreen = createSettingsScreen();
      } else if (event == MainScreen::Event::About) {
        mNextScreen = createAboutScreen();
      }
    });
  }

  std::unique_ptr<IScreen> createScoreListScreen() {
    return std::make_unique<ScoreListScreen>([this](auto event) {
      if (event == ScoreListScreen::Event::Back) {
        mNextScreen = createMain();
      }
    });
  }

  std::unique_ptr<IScreen> createStartGameScreen() {
    return std::make_unique<StartGameScreen>(
        [this](StartGameScreen *screen, auto event) {
          if (event == StartGameScreen::Event::Back) {
            mNextScreen = createMain();
          } else if (event == StartGameScreen::Event::Start) {
            mNextScreen = createGameScreen(screen->getGameParams());
          }
        });
  }

  std::unique_ptr<IScreen> createGameScreen(GameParams params) {
    return std::make_unique<GameScreen>(params, mMixer, mWindow,
                                        [this](auto event) {
                                          if (event == GameScreen::Event::Quit)
                                            mNextScreen = createMain();
                                        });
  }

  std::unique_ptr<IScreen> createSettingsScreen() {
    return std::make_unique<SettingsScreen>(
        [this](auto event) { mNextScreen = createMain(); });
  }

  std::unique_ptr<IScreen> createAboutScreen() {
    return std::make_unique<AboutScreen>([this](auto event) {
      if (event == AboutScreen::Event::Close) {
        mNextScreen = createMain();
      }
    });
  }

  void prepareScene() {
    SDL_SetRenderDrawColor(mRenderer, 7, 6, 7, 255);
    SDL_RenderClear(mRenderer);
  }

  void processInput() {
    SDL_Event event;

    while (SDL_PollEvent(&event)) {
      SDL_ConvertEventToRenderCoordinates(mRenderer, &event);
      if (event.type == SDL_EVENT_QUIT) {
        mIsExit = true;
        return;
      }

      mScreen->onSDLEvent(event);
    }
  }

  void presentScene() { SDL_RenderPresent(mRenderer); }

  void run() {
    while (!mIsExit) {
      Uint32 nextFrameTick = SDL_GetTicks() + 16;
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

      Uint32 currentTick = SDL_GetTicks();
      if (currentTick < nextFrameTick)
        SDL_Delay(nextFrameTick - currentTick);
      else
        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Frame drop");
    }
  }

private:
  SDL_Renderer *mRenderer;
  SDL_Window *mWindow;
  MIX_Mixer *mMixer;
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

  SDL_CaptureMouse(false);

  App app;
  app.run();
  return 0;
}