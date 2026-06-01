#ifndef SPACE_SETTINGSSCREEN_HPP
#define SPACE_SETTINGSSCREEN_HPP

#include "../UI/Button.hpp"
#include "../UI/Column.hpp"
#include "../UI/Label.hpp"
#include "../UI/Panel.hpp"
#include "../UI/Row.hpp"
#include "../UI/Switch.hpp"
#include "IScreen.hpp"
#include <array>

class SettingsScreen : public IScreen {
public:
  enum Event { Close };

private:
  Column mColumn, mPanelColumn;
  Row mButtonsRow{10};
  Panel mPanel;
  Label mTitle{"Settings", 24};
  Button mCancelButton{"Cancel", {.topLeft = 10}, 0x060e20FF, 0xff0000FF};
  Button mSaveButton{"Save", {.bottomRight = 10}};

  Row mScaleButtonRow{0};

  double mScaleButtonsSelectedValue = 1.0;
  std::array<double, 3> mScaleButtonsValues{0.5, 0.75, 1.0};
  std::array<Button, 3> mScaleButtons{Button{"0.5", {.topLeft = 10}},
                                      Button{"0.75"},
                                      Button{"1.0", {.bottomRight = 10}}};

  SwitchButton mDebugFlowField, mDebugContextSteering, mDebugBoundingBox,
      mDebugMissile, mShowAimLine, mUIHardwareRendering;
  std::array<Label, 7> mSettingLabels{
      Label{"Debug Missile"}, Label{"Debug Context Steering"},
      Label{"Debug Bounding Box"}, Label{"Debug Flow Field"}, Label{"Use Hardware Rendering for UI"}, Label{"Show Aim Line"},
      Label{"Camera Scale"}};
  std::array<Row, 7> mSettingRows;

  std::function<void(Event)> mEventHandler;

  bool mOriginalUIHardwareRendering;

public:
  SettingsScreen(std::function<void(Event)> eventHandler);
  void onSizeChanged(const Vec2 &size) override;
  void onSDLEvent(const SDL_Event &event) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer *renderer) override;
  void onPostDraw() override;
};

#endif // SPACE_SETTINGSSCREEN_HPP
