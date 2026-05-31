#ifndef SPACE_APPSETTINGS_HPP
#define SPACE_APPSETTINGS_HPP
#include <string>

struct AppSettings {
  bool debugFlowField;
  bool debugContextSteering;
  bool debugBoundingBox;
  bool debugMissile;
  double cameraScale;
  std::string lastShipColor;
  int lastShip;
  int lastMode;
};

AppSettings* getAppSettings();
void saveAppSettings();

#endif //SPACE_APPSETTINGS_HPP
