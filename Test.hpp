#ifndef SPACE_TEST_HPP
#define SPACE_TEST_HPP

#include "SDLHelper.hpp"

#include <SDL3/SDL.h>
#include <chrono>
#include <iostream>
#include <vector>

using TimeType = std::chrono::microseconds;

class Timer {
  std::chrono::steady_clock::time_point mStart;

public:
  Timer() : mStart(std::chrono::steady_clock::now()) {}

  TimeType getElapsed() {
    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<TimeType>(end - mStart);
    return elapsed;
  }
};

class DrawTesting {
  bool mIsStart = true;

  int mTestCount = 0;

  std::vector<TimeType> mStartTime;
  std::vector<double> mAvgRenderTimes;
  std::vector<TimeType> mRenderTimes;

  SDL_Window *mWindow = nullptr;
  Vec2 mWindowSize{800, 600};
  bool mIsExit = false;

public:
  SDL_Renderer *renderer = nullptr;

  DrawTesting() {
    if (SDL_Init(SDL_INIT_VIDEO) == false) {
      std::cout << "Initializing SDL failed" << std::endl;
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

    renderer = SDL_CreateRenderer(mWindow, "opengl");

    if (!renderer) {
      std::cout << "Initializing renderer failed" << std::endl;
      exit(1);
    }

    SDL_SetRenderVSync(renderer, SDL_RENDERER_VSYNC_DISABLED);
    SDL_SetRenderLogicalPresentation(renderer, mWindowSize.x, mWindowSize.y,
                                     SDL_LOGICAL_PRESENTATION_LETTERBOX);
  }
  virtual ~DrawTesting() = default;

  void clearScene() {
    SDL_SetRenderDrawColor(renderer, 7, 6, 7, 255);
    SDL_RenderClear(renderer);
  }

  void presentScene() { SDL_RenderPresent(renderer); }

  void run() {
    while (!mIsExit) {
      SDL_Event event;
      while (SDL_PollEvent(&event)) {
        SDL_ConvertEventToRenderCoordinates(renderer, &event);
        if (event.type == SDL_EVENT_QUIT) {
          mIsExit = true;
          return;
        }
      }

      onUpdate();
    }
  }

  void onUpdate() {
    if (mIsStart) {
      mStartTime.push_back(onStart());
      mIsStart = false;
    }

    if (mRenderTimes.size() < 10) {
      mRenderTimes.push_back(onRenderFrame(1000));
    } else if (mTestCount < 10) {
      std::cout << "Start Time: " << *mStartTime.rbegin() << std::endl;
      std::cout << "Render Times: ";
      double all = 0;
      for (size_t i = 0; auto time : mRenderTimes) {
        if (i > 0)
          std::cout << ", ";
        std::cout << time;
        all += time.count();
        i++;
      }
      std::cout << std::endl;

      all /= mRenderTimes.size();
      std::cout << "Avg Render Time: " << all << std::endl;

      mAvgRenderTimes.push_back(all);
      mRenderTimes.clear();
      mIsStart = true;
      mTestCount++;
    } else if (mTestCount == 10) {
      double all = 0;
      for (auto time : mStartTime) {
        all += time.count();
      }
      all /= mStartTime.size();
      std::cout << "Avg Start Time: " << all << std::endl;

      all = 0;
      for (auto time : mAvgRenderTimes) {
        all += time;
      }
      all /= mAvgRenderTimes.size();

      std::cout << "Avg All Render Time: " << all << std::endl;
      mTestCount++;
    }
  }
  virtual TimeType onStart() { return {}; }
  virtual TimeType onRenderFrame(int count) { return {}; }
};

class TextureBevelOutlineTesting : public DrawTesting {
  SDL_Texture *mTexture = nullptr;

public:
  TimeType onStart() override {
    Timer timer;
    mTexture =
        SDLHelper::createBeveledRectTextureOutline(renderer, 400, 400, 25,
                                                   {.topLeft = 100,
                                                    .topRight = 100,
                                                    .bottomLeft = 100,
                                                    .bottomRight = 100},
                                                   0xFFFFFFFF);
    return timer.getElapsed();
  }

  TimeType onRenderFrame(int count) override {
    Timer timer;
    clearScene();
    SDL_FRect rect =
        SDLHelper::calculateTextureRectByCenter(mTexture, {250, 250}, 1.0);
    for (int i = 0; i < count; i++) {
      SDL_RenderTexture(renderer, mTexture, nullptr, &rect);
    }
    presentScene();
    return timer.getElapsed();
  }
};

class VertexBevelOutlineTesting : public DrawTesting {
public:
  TimeType onRenderFrame(int count) override {
    Timer timer;
    clearScene();
    for (int i = 0; i < count; i++) {
      SDLHelper::drawBeveledRectOutline(renderer, {50, 50, 400, 400}, 25,
                                        {.topLeft = 100,
                                         .topRight = 100,
                                         .bottomLeft = 100,
                                         .bottomRight = 100},
                                        {1.0f, 1.0f, 1.0f, 1.0f});
    }
    presentScene();
    return timer.getElapsed();
  }
};

class TextureBevelFillTesting : public DrawTesting {
  SDL_Texture *mTexture = nullptr;

public:
  TimeType onStart() override {
    Timer timer;
    mTexture = SDLHelper::createBeveledRectTexture(renderer, 400, 400,
                                                   {.topLeft = 100,
                                                    .topRight = 100,
                                                    .bottomLeft = 100,
                                                    .bottomRight = 100},
                                                   0xFFFFFFFF);
    return timer.getElapsed();
  }

  TimeType onRenderFrame(int count) override {
    Timer timer;
    clearScene();
    SDL_FRect rect =
        SDLHelper::calculateTextureRectByCenter(mTexture, {250, 250}, 1.0);
    for (int i = 0; i < count; i++) {
      SDL_RenderTexture(renderer, mTexture, nullptr, &rect);
    }
    presentScene();
    return timer.getElapsed();
  }
};

class VertexBevelFillTesting : public DrawTesting {
public:
  TimeType onRenderFrame(int count) override {
    Timer timer;
    clearScene();
    for (int i = 0; i < count; i++) {
      SDLHelper::drawBeveledRect(renderer, {50, 50, 400, 400},
                                 {.topLeft = 100,
                                  .topRight = 100,
                                  .bottomLeft = 100,
                                  .bottomRight = 100},
                                 {1.0f, 1.0f, 1.0f, 1.0f});
    }
    presentScene();
    return timer.getElapsed();
  }
};

#endif // SPACE_TEST_HPP
