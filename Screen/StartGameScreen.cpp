#include "StartGameScreen.hpp"

#include "../AppSettings.hpp"
#include "../AssetManager.hpp"
StartGameScreen::StartGameScreen(
    std::function<void(StartGameScreen *, Event)> eventHandler)
    : mEventHandler(std::move(eventHandler)) {
  AppSettings *appSettings = getAppSettings();
  selectedColor = appSettings->lastShipColor;
  selectedShip = appSettings->lastShip;
  selectedMode = appSettings->lastMode;

  mColumn.viewList.push_back(&mTitle);
  mColumn.viewList.push_back(&mHorizontalSlide);

  mHorizontalSlide.viewList.push_back(&mFirstPanel);
  mHorizontalSlide.viewList.push_back(&mSecondPanel);

  mFirstColumn.viewList.push_back(&mColorLabel);
  mFirstColumn.viewList.push_back(&mColorsRow);
  mFirstColumn.viewList.push_back(&mShipLabel);
  mFirstColumn.viewList.push_back(&mShipsRow);
  mFirstColumn.viewList.push_back(&mFirstRow);

  mFirstRow.viewList.push_back(&mBackToMainButton);
  mFirstRow.viewList.push_back(&mNextButton);

  auto updateColor = [this]() {
    for (size_t index = 0; auto &button : mColorsButton) {
      button.isSelected = mColorsKeys[index] == selectedColor;
      index++;
    }
    for (size_t index = 0; auto &button : mShipsButton) {
      std::string path =
          std::format("PNG/playerShip{}_{}.png", index + 1, selectedColor);
      button.textureOverride = TextureManager::getInstance()->load(path);
      index++;
    }
  };

  for (size_t index = 0; auto &button : mColorsButton) {
    mColorsRow.viewList.push_back(&button);
    button.width = 125;
    button.isSelectable = true;
    button.scaleWhenHovered = false;
    button.onClickHandler = [=, this](Button *button) {
      selectedColor = mColorsKeys[index];
      updateColor();
      return true;
    };
    index++;
  }
  auto updateShip = [this]() {
    for (size_t index = 0; auto &button : mShipsButton) {
      button.isSelected = selectedShip == index + 1;
      index++;
    }
  };

  for (size_t index = 0; auto &button : mShipsButton) {
    mShipsRow.viewList.push_back(&button);
    button.width = 150;
    button.height = 150;
    button.isSelectable = true;
    button.scaleWhenHovered = false;
    button.onClickHandler = [=, this](Button *button) {
      selectedShip = index + 1;
      updateShip();
      return true;
    };
    index++;
  }

  updateColor();
  updateShip();

  mBackToMainButton.onClickHandler = [this](Button *button) {
    mEventHandler(this, Back);
    return true;
  };

  mNextButton.onClickHandler = [this](Button *button) {
    mHorizontalSlide.currentIndex = 1;
    return true;
  };

  mSecondColumn.viewList.push_back(&mModeLabel);

  auto updateMode = [this]() {
    for (size_t index = 0; auto &button : mModeButton) {
      button.isSelected = selectedMode == index;
      index++;
    }
  };

  for (size_t index = 0; auto &button : mModeButton) {
    button.isSelectable = true;
    mSecondColumn.viewList.push_back(&button);
    button.onClickHandler = [=, this](Button *button) {
      selectedMode = index;
      updateMode();
      return true;
    };
    index++;
  }

  updateMode();

  mSecondColumn.viewList.push_back(&mSecondRow);

  mSecondRow.viewList.push_back(&mBackTo0Button);
  mSecondRow.viewList.push_back(&mStartButton);

  mBackTo0Button.onClickHandler = [this](Button *button) {
    mHorizontalSlide.currentIndex = 0;
    return true;
  };

  mStartButton.onClickHandler = [this](Button *button) {
    AppSettings *appSettings = getAppSettings();
    appSettings->lastShipColor = selectedColor;
    appSettings->lastShip = selectedShip;
    appSettings->lastMode = selectedMode;
    saveAppSettings();
    mEventHandler(this, Start);
    return true;
  };
}
void StartGameScreen::onSizeChanged(const Vec2 &size) {
  mHorizontalSlide.layoutWidth = size.x;
  mColumn.layout(size);
}
void StartGameScreen::onSDLEvent(const SDL_Event &event) {
  mColumn.handleSDLEvent(event);
}
void StartGameScreen::onUpdate() { mColumn.update(); }
void StartGameScreen::onDraw(SDL_Renderer *renderer) { mColumn.draw(renderer); }
void StartGameScreen::onPostDraw() {}
GameParams StartGameScreen::getGameParams() {
  return {
      .ship = selectedShip,
      .color = selectedColor,
      .enemyWithMissile = selectedMode == 1 || selectedMode == 2,
      .enemyWithLaser = selectedMode == 0 || selectedMode == 2,
  };
}