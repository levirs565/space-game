#include "SettingsScreen.hpp"

#include "../AppSettings.hpp"
SettingsScreen::SettingsScreen(std::function<void(Event)> eventHandler)
    : mPanel{&mPanelColumn}, mEventHandler(std::move(eventHandler)) {
  mColumn.viewList.push_back(&mTitle);
  mColumn.viewList.push_back(&mPanel);

  for (size_t index = 0; auto &row : mSettingRows) {
    row.spaceBetweenWidth = 500;
    row.viewList.push_back(&mSettingLabels[index]);
    mPanelColumn.viewList.push_back(&row);
    index++;
  }

  mPanelColumn.viewList.push_back(&mButtonsRow);

  mButtonsRow.viewList.push_back(&mCancelButton);
  mButtonsRow.viewList.push_back(&mSaveButton);

  mSettingRows[0].viewList.push_back(&mDebugMissile);
  mSettingRows[1].viewList.push_back(&mDebugContextSteering);
  mSettingRows[2].viewList.push_back(&mDebugBoundingBox);
  mSettingRows[3].viewList.push_back(&mDebugFlowField);
  mSettingRows[4].viewList.push_back(&mScaleButtonRow);

  auto updateScaleButtons = [this]() {
    for (size_t index = 0; auto &button : mScaleButtons) {
      button.isSelected =
          mScaleButtonsValues[index] == mScaleButtonsSelectedValue;
      index++;
    }
  };

  for (size_t index = 0; auto &button : mScaleButtons) {
    mScaleButtonRow.viewList.push_back(&button);
    button.width = 75;
    button.scaleWhenHovered = false;
    button.isSelectable = true;
    button.onClickHandler = [=, this](Button *button) {
      mScaleButtonsSelectedValue = mScaleButtonsValues[index];
      updateScaleButtons();
      return true;
    };
    index++;
  }

  mCancelButton.onClickHandler = [this](Button *button) {
    mEventHandler(Close);
    return true;
  };

  AppSettings *settings = getAppSettings();
  mDebugMissile.value = settings->debugMissile;
  mDebugContextSteering.value = settings->debugContextSteering;
  mDebugBoundingBox.value = settings->debugBoundingBox;
  mDebugFlowField.value = settings->debugFlowField;
  mScaleButtonsSelectedValue = settings->cameraScale;
  updateScaleButtons();

  mSaveButton.onClickHandler = [this](Button *button) {
    AppSettings *settings = getAppSettings();
    settings->debugMissile = mDebugMissile.value;
    settings->debugContextSteering = mDebugContextSteering.value;
    settings->debugBoundingBox = mDebugBoundingBox.value;
    settings->debugFlowField = mDebugFlowField.value;
    settings->cameraScale = mScaleButtonsSelectedValue;
    mEventHandler(Close);
    return true;
  };
}

void SettingsScreen::onSizeChanged(const Vec2 &size) { mColumn.layout(size); }

void SettingsScreen::onSDLEvent(const SDL_Event &event) {
  mColumn.handleSDLEvent(event);
}

void SettingsScreen::onUpdate() { mColumn.update(); }

void SettingsScreen::onDraw(SDL_Renderer *renderer) { mColumn.draw(renderer); }

void SettingsScreen::onPostDraw() {}